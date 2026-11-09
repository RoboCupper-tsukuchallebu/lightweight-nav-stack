"""Shared closed-loop launch and assertions."""
import time
import unittest
from pathlib import Path

import launch_testing.actions
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from lifecycle_msgs.msg import TransitionEvent
from visualization_msgs.msg import MarkerArray
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.action import ActionClient

from lightnav_msgs.action import FollowWaypoints
from lightnav_msgs.msg import FollowStatus, ObstacleStatus, Waypoint
from lightnav_msgs.srv import PlanPath

MAX_PATH_DEVIATION_M = 1.5


def description(scenario, base_frame):
    source = Path(get_package_share_directory("lightnav_bringup")) / "launch" / "sim_closed_loop.launch.py"
    launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(str(source)),
                                      launch_arguments={"scenario": scenario, "base_frame": base_frame,
                                                        "max_path_deviation_m": str(MAX_PATH_DEVIATION_M)}.items())
    return LaunchDescription([launch, launch_testing.actions.ReadyToTest()])


class ClosedLoopTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("closed_loop_test")
        cls.feedback = []
        cls.poses = []
        cls.status = []
        cls.obstacles = []
        cls.obstacle_receipts = []
        cls.commands = []
        cls.lifecycle_recoveries = []
        cls.trace = []
        cls.selected_arc = None
        cls.grids = {}
        cls.lifecycle_events = []
        cls.node.create_subscription(PoseStamped, "/lightnav/pose", cls.poses.append, 10)
        cls.node.create_subscription(FollowStatus, "/lightnav/follow_status", cls.record_status, 10)
        cls.node.create_subscription(MarkerArray, "/lightnav/controller/arc_candidates", cls.record_arcs, 10)
        cls.node.create_subscription(ObstacleStatus, "/lightnav/obstacle_status", cls.record_obstacle, 10)
        cls.node.create_subscription(Twist, "/cmd_vel", cls.record_command, 10)
        for topic in ("/lightnav/local_grid", "/lightnav/local_grid_raw"):
            cls.node.create_subscription(OccupancyGrid, topic,
                                         lambda msg, name=topic: cls.record_grid(name, msg), 10)
        for name in ("obstacle_monitor", "pose_adapter_node", "planner_node", "lightnav_mission",
                     "controller_node", "safety_gate_node"):
            cls.node.create_subscription(TransitionEvent, f"/{name}/transition_event",
                                         lambda msg, node=name: cls.record_transition(node, msg), 10)
        cls.action = ActionClient(cls.node, FollowWaypoints, "/lightnav/mission/follow_waypoints")

    @classmethod
    def record_grid(cls, name, message):
        stamp = (message.header.stamp.sec, message.header.stamp.nanosec)
        cls.grids[name] = (stamp, sum(value > 0 for value in message.data))

    @classmethod
    def record_transition(cls, name, message):
        row = (time.monotonic(), name, message.start_state.label, message.goal_state.label)
        cls.lifecycle_events.append(row)
        print("LIFECYCLE", *row, flush=True)

    @classmethod
    def record_arcs(cls, message):
        selected = [marker for marker in message.markers if marker.ns == "feasible" and marker.scale.x > 0.05]
        cls.selected_arc = selected[0].id if selected else None

    @classmethod
    def record_status(cls, message):
        cls.status.append(message)
        now = time.monotonic()
        if cls.poses:
            pose = cls.poses[-1].pose.position
            inflated = cls.grids.get("/lightnav/local_grid")
            raw = cls.grids.get("/lightnav/local_grid_raw")
            obstacle = cls.obstacles[-1].is_stop_zone_occupied if cls.obstacles else None
            command = cls.commands[-1][1] if cls.commands else None
            cls.trace.append((now, pose.x, pose.y, message.state, message.active_layer, cls.selected_arc,
                              obstacle, command.linear.x if command else None,
                              command.angular.z if command else None, inflated, raw,
                              bool(inflated and raw and inflated[0] == raw[0])))

    @classmethod
    def record_obstacle(cls, message):
        cls.obstacles.append(message)
        cls.obstacle_receipts.append((time.monotonic(), message))

    @classmethod
    def record_command(cls, message):
        cls.commands.append((time.monotonic(), message))

    @classmethod
    def tearDownClass(cls):
        print("TRACE monotonic_s x_m y_m state layer selected_arc_id stop_occupied linear angular inflated_stamp_count raw_stamp_count latest_stamps_match", flush=True)
        for row in cls.trace:
            print("TRACE", *row, flush=True)
        cls.action.destroy()
        cls.node.destroy_node()
        rclpy.shutdown()

    def wait_for(self, condition, timeout=180.0):
        deadline = time.monotonic() + timeout
        while not condition() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertTrue(
            condition(),
            f"condition timed out; last follow status: {self.status[-1] if self.status else None}; "
            f"last obstacle status: {self.obstacles[-1] if self.obstacles else None}; "
            f"last pose: {self.poses[-1] if self.poses else None}",
        )

    def start_mission(self):
        self.assertTrue(self.action.wait_for_server(timeout_sec=60.0))
        for name in ("obstacle_monitor", "pose_adapter_node", "planner_node", "lightnav_mission",
                     "controller_node", "safety_gate_node"):
            client = self.node.create_client(GetState, f"/{name}/get_state")
            self.assertTrue(client.wait_for_service(timeout_sec=60.0))
            deadline = time.monotonic() + 60.0
            state = None
            recovery_after = time.monotonic() + 5.0
            change_client = self.node.create_client(ChangeState, f"/{name}/change_state")
            while time.monotonic() < deadline:
                future = client.call_async(GetState.Request())
                self.wait_for(future.done, 10.0)
                state = future.result().current_state
                print(f"LIFECYCLE_POLL {time.monotonic():.6f} {name} {state.label}", flush=True)
                if state.id == State.PRIMARY_STATE_ACTIVE:
                    break
                if time.monotonic() >= recovery_after and state.id in (
                    State.PRIMARY_STATE_UNCONFIGURED, State.PRIMARY_STATE_INACTIVE
                ):
                    # robot.launch.py bring-up can stall (open bug). Recovery only keeps
                    # the test running; every warning must be investigated.
                    self.assertTrue(change_client.wait_for_service(timeout_sec=10.0))
                    request = ChangeState.Request()
                    request.transition.id = (
                        Transition.TRANSITION_CONFIGURE
                        if state.id == State.PRIMARY_STATE_UNCONFIGURED
                        else Transition.TRANSITION_ACTIVATE
                    )
                    transition_label = (
                        "CONFIGURE" if state.id == State.PRIMARY_STATE_UNCONFIGURED else "ACTIVATE"
                    )
                    self.lifecycle_recoveries.append((name, state.label, transition_label))
                    print(
                        f"WARNING: lifecycle recovery: node={name}, state={state.label}, "
                        f"transition={transition_label}",
                        flush=True,
                    )
                    transition = change_client.call_async(request)
                    self.wait_for(transition.done, 10.0)
                    recovery_after = time.monotonic() + 5.0
                time.sleep(0.2)
            else:
                self.node.destroy_client(client)
                self.node.destroy_client(change_client)
                self.fail(f"{name} did not become active; last state: {state}")
            self.node.destroy_client(client)
            self.node.destroy_client(change_client)
        self.assertFalse(self.lifecycle_recoveries, f"lifecycle recovery required: {self.lifecycle_recoveries}")
        planner = self.node.create_client(PlanPath, "/lightnav/planner/plan_path")
        self.assertTrue(planner.wait_for_service(timeout_sec=60.0))
        self.node.destroy_client(planner)
        self.wait_for(lambda: len(self.poses) >= 3, 60.0)
        self.wait_for(lambda: len(self.obstacles) >= 3 and any(not item.is_input_stale for item in self.obstacles), 60.0)
        goal = FollowWaypoints.Goal()
        for distance in (3.0, 6.0, 10.0):
            waypoint = Waypoint()
            waypoint.x = distance
            goal.waypoints.append(waypoint)
        future = self.action.send_goal_async(goal, feedback_callback=lambda message: self.feedback.append(message.feedback))
        self.wait_for(future.done, 60.0)
        handle = future.result()
        self.assertTrue(handle.accepted)
        return handle.get_result_async()

    def assert_success(self, result):
        self.wait_for(result.done)
        self.assertTrue(result.result().result.success, result.result().result.message)
        self.assertEqual(result.result().result.reached_count, 3)

    def run_avoidance(self):
        """Use a generous condition-based timeout; real-time launch tests vary with load (#32, #40).

        This does not enforce the 60-second limit in scenario A.
        """
        result = self.start_mission()
        # BLOCKED is already a failure for H; stop waiting as soon as it occurs.
        self.wait_for(lambda: result.done() or any(item.state == FollowStatus.BLOCKED for item in self.status))
        self.assertFalse(any(item.state == FollowStatus.BLOCKED for item in self.status))
        self.assert_success(result)
        self.assertTrue(any(item.active_layer == FollowStatus.LAYER_AVOID for item in self.feedback))
        self.assertTrue(self.status)
        self.assertLessEqual(max(item.path_deviation_m for item in self.status), MAX_PATH_DEVIATION_M)
        self.assertFalse(any(item.is_stop_zone_occupied for item in self.obstacles))
        self.assertFalse(any(item.state == FollowStatus.BLOCKED for item in self.status))

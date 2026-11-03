"""mission_node を fake の PlanPath サーバ・FollowStatus・LocalizationStatus で動かす単体試験。"""

import contextlib
import threading
import time

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from lightnav_mission.mission_node import MissionNode
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

from lightnav_msgs.action import FollowWaypoints
from lightnav_msgs.msg import FollowStatus, LocalizationStatus, PathWithId, Waypoint
from lightnav_msgs.srv import PlanPath

WAIT_TIMEOUT_S = 5.0
TRANSIENT_LOCAL_QOS = QoSProfile(
    depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def wait_until(predicate, timeout_s=WAIT_TIMEOUT_S):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return False


class Harness(Node):
    """fake planner / 姿勢 / FollowStatus / LocalizationStatus と、経路・結果の記録。"""

    def __init__(self):
        super().__init__("mission_test_harness")
        group = ReentrantCallbackGroup()
        self.planner_behaviors = {}  # 呼び出し番号 -> {"wait": Event, "fail": bool, "bad_indices": bool}
        self.planner_requests = []
        self.received_paths = []
        self.pose_x = 0.0
        self.is_pose_published = True
        self.create_service(PlanPath, "/lightnav/planner/plan_path", self._plan, callback_group=group)
        self.create_subscription(
            PathWithId, "/lightnav/path", self.received_paths.append,
            QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL),
            callback_group=group)
        self.pose_publisher = self.create_publisher(PoseStamped, "/lightnav/pose", 10)
        self.status_publisher = self.create_publisher(FollowStatus, "/lightnav/follow_status", 10)
        self.localization_publisher = self.create_publisher(
            LocalizationStatus, "/lightnav/localization/status", TRANSIENT_LOCAL_QOS)
        self.create_timer(0.05, self._publish_pose, callback_group=group)
        self.action_client = ActionClient(
            self, FollowWaypoints, "/lightnav/mission/follow_waypoints", callback_group=group)
        self.pause_client = self.create_client(Trigger, "/lightnav/mission/pause", callback_group=group)
        self.resume_client = self.create_client(Trigger, "/lightnav/mission/resume", callback_group=group)
        self.change_state_client = self.create_client(
            ChangeState, "/lightnav_mission/change_state", callback_group=group)

    def _publish_pose(self):
        if not self.is_pose_published:
            return
        message = PoseStamped()
        message.header.frame_id = "map"
        message.pose.position.x = self.pose_x
        message.pose.orientation.w = 1.0
        self.pose_publisher.publish(message)

    def _plan(self, request, response):
        call_index = len(self.planner_requests)
        self.planner_requests.append(request)
        behavior = self.planner_behaviors.get(call_index, {})
        if "wait" in behavior:
            behavior["wait"].wait(10.0)
        if behavior.get("fail"):
            response.success, response.message = False, "fake failure"
            return response
        response.success = True
        response.path = Path()
        for x_position in [request.start.pose.position.x] + [w.x for w in request.waypoints]:
            pose = PoseStamped()
            pose.pose.position.x = x_position
            response.path.poses.append(pose)
        count = len(request.waypoints)
        response.waypoint_indices = list(range(1, count + 1))
        if behavior.get("bad_indices"):
            response.waypoint_indices = []
        return response

    def publish_follow_status(self, session_id, path_id, closest_index=0):
        message = FollowStatus()
        message.session_id, message.path_id, message.closest_index = session_id, path_id, closest_index
        self.status_publisher.publish(message)

    def non_empty_paths(self):
        return [p for p in list(self.received_paths) if p.path.poses]

    def send_goal(self, waypoint_x_positions):
        goal = FollowWaypoints.Goal()
        goal.waypoints = [Waypoint(x=float(x), y=0.0) for x in waypoint_x_positions]
        future = self.action_client.send_goal_async(goal)
        assert wait_until(future.done)
        return future.result()

    def result_of(self, goal_handle, timeout_s=WAIT_TIMEOUT_S):
        future = goal_handle.get_result_async()
        assert wait_until(future.done, timeout_s)
        return future.result()


@pytest.fixture
def unconfigured_rig():
    if not rclpy.ok():
        rclpy.init()
    overrides = [
        Parameter("plan_timeout_s", value=2.0),
        Parameter("tick_hz", value=20.0),
    ]
    observer = rclpy.create_node("mission_cleanup_observer")
    planner_client = None
    try:
        planner_client = observer.create_client(PlanPath, "/lightnav/planner/plan_path")
        with contextlib.ExitStack() as cleanup:
            mission = MissionNode(parameter_overrides=overrides)
            cleanup.callback(mission.destroy_node)
            harness = Harness()
            cleanup.callback(harness.destroy_node)
            executor = MultiThreadedExecutor(num_threads=8)
            executor.add_node(mission)
            executor.add_node(harness)
            thread = threading.Thread(target=executor.spin, daemon=True)
            thread.start()
            try:
                observed_planner_service = wait_until(planner_client.service_is_ready)
                yield mission, harness
            finally:
                with contextlib.suppress(Exception):  # 試験内で deactivate 済みなら失敗するので無視
                    mission.trigger_deactivate()
                for node in (mission, harness):
                    for timer in node.timers:
                        timer.cancel()
                executor.shutdown()
                thread.join(timeout=2.0)
            # destroy_node は試験本体で追加した timer も含めて破棄する。
        # ponytail: ノード名はグラフに残るのでサービスで確認する。
        assert observed_planner_service and wait_until(
            lambda: not planner_client.service_is_ready()), (
                "planner サービスを観測できないか、後片付け後も残っている")
    finally:
        try:
            if planner_client is not None:
                observer.destroy_client(planner_client)
        finally:
            observer.destroy_node()


@pytest.fixture
def rig(unconfigured_rig):
    mission, harness = unconfigured_rig
    mission.trigger_configure()
    mission.trigger_activate()
    assert harness.action_client.wait_for_server(timeout_sec=5.0)
    return mission, harness


def call_trigger(harness, client):
    future = client.call_async(Trigger.Request())
    assert wait_until(future.done)
    return future.result()


def test_goal_publishes_path_then_stops_on_completion(rig):
    _mission, harness = rig
    goal_handle = harness.send_goal([0.0])
    result = harness.result_of(goal_handle).result
    assert result.success and result.reached_count == 1
    assert harness.non_empty_paths()
    assert wait_until(lambda: harness.received_paths[-1].path.poses == [])


def test_pause_twice_keeps_robot_stopped(rig):
    mission, harness = rig
    harness.send_goal([50.0])
    assert wait_until(lambda: harness.non_empty_paths())
    assert call_trigger(harness, harness.pause_client).success
    assert wait_until(lambda: harness.received_paths[-1].path.poses == [])
    path_count = len(harness.received_paths)
    assert call_trigger(harness, harness.pause_client).success  # 念押しの再送は no-op
    time.sleep(1.0)
    assert len(harness.received_paths) == path_count
    assert mission._state.value == "PAUSED"


def test_cancel_during_planning_is_prompt(rig):
    _mission, harness = rig
    gate = threading.Event()
    harness.planner_behaviors[0] = {"wait": gate}
    goal_handle = harness.send_goal([5.0])
    assert wait_until(lambda: harness.planner_requests)
    started = time.monotonic()
    cancel_future = goal_handle.cancel_goal_async()
    assert wait_until(cancel_future.done)
    result = harness.result_of(goal_handle, timeout_s=1.0)
    assert time.monotonic() - started < 0.5
    assert not result.result.success
    gate.set()


def test_preempted_plan_does_not_override_new_goal(rig):
    mission, harness = rig
    gate = threading.Event()
    harness.planner_behaviors[0] = {"wait": gate}
    harness.send_goal([5.0])
    assert wait_until(lambda: harness.planner_requests)
    second = harness.send_goal([9.0])
    assert wait_until(lambda: harness.non_empty_paths())
    gate.set()  # 古い goal の応答を後から返す
    time.sleep(0.5)
    assert harness.non_empty_paths()[-1].path.poses[-1].pose.position.x == 9.0
    assert mission._state.value == "FOLLOWING"
    del second


def test_preempted_plan_failure_does_not_reset_state(rig):
    mission, harness = rig
    gate = threading.Event()
    harness.planner_behaviors[0] = {"wait": gate, "fail": True}
    harness.send_goal([5.0])
    assert wait_until(lambda: harness.planner_requests)
    harness.send_goal([50.0])
    assert wait_until(lambda: mission._state.value == "FOLLOWING")
    gate.set()
    time.sleep(0.5)
    assert mission._state.value == "FOLLOWING"
    assert call_trigger(harness, harness.pause_client).success


def test_deactivate_publishes_stop_and_returns_to_idle(rig):
    mission, harness = rig
    goal_handle = harness.send_goal([50.0])
    assert wait_until(lambda: harness.non_empty_paths())
    mission.trigger_deactivate()
    assert wait_until(lambda: harness.received_paths[-1].path.poses == [])
    assert mission._state.value == "IDLE"
    result = harness.result_of(goal_handle).result
    assert not result.success


def test_malformed_plan_response_aborts_with_stop(rig):
    mission, harness = rig
    harness.planner_behaviors[0] = {"bad_indices": True}
    goal_handle = harness.send_goal([5.0, 6.0])
    result = harness.result_of(goal_handle).result
    assert not result.success
    assert mission._state.value == "IDLE"
    assert not harness.non_empty_paths() or harness.received_paths[-1].path.poses == []


def test_large_correction_replans_only_on_rising_edge(rig):
    mission, harness = rig
    harness.send_goal([50.0])
    assert wait_until(lambda: harness.non_empty_paths())
    for _ in range(4):
        message = LocalizationStatus()
        message.is_large_correction = True
        harness.localization_publisher.publish(message)
        time.sleep(0.2)
    time.sleep(0.5)
    assert mission._was_large_correction, 'status not received'
    assert len(harness.planner_requests) == 2, (mission._state, mission._is_large_correction_pending)


def change_state(harness, transition_id):
    request = ChangeState.Request()
    request.transition.id = transition_id
    future = harness.change_state_client.call_async(request)
    assert wait_until(future.done)
    return future.result().success


def test_lifecycle_transitions_work_through_change_state_service(unconfigured_rig):
    _mission, harness = unconfigured_rig
    assert harness.change_state_client.wait_for_service(timeout_sec=5.0)
    assert change_state(harness, Transition.TRANSITION_CONFIGURE)
    assert change_state(harness, Transition.TRANSITION_ACTIVATE)
    assert harness.action_client.wait_for_server(timeout_sec=5.0)


def test_reconfigure_after_cleanup_succeeds(rig):
    mission, harness = rig
    for trigger in (mission.trigger_deactivate, mission.trigger_cleanup,
                    mission.trigger_configure, mission.trigger_activate):
        assert trigger() == TransitionCallbackReturn.SUCCESS
    assert harness.action_client.wait_for_server(timeout_sec=5.0)
    assert harness.result_of(harness.send_goal([0.0])).result.success


def test_preempt_stops_old_path_and_accepts_pause_while_planning(rig):
    _mission, harness = rig
    harness.send_goal([50.0])
    assert wait_until(lambda: harness.non_empty_paths())
    gate = threading.Event()
    harness.planner_behaviors[1] = {"wait": gate}
    harness.send_goal([60.0])
    assert wait_until(lambda: len(harness.planner_requests) == 2)
    assert wait_until(lambda: harness.received_paths[-1].path.poses == [])
    assert call_trigger(harness, harness.pause_client).success
    gate.set()


def test_pause_right_after_resume_is_honoured(rig):
    mission, harness = rig
    harness.send_goal([50.0])
    assert wait_until(lambda: harness.non_empty_paths())
    assert call_trigger(harness, harness.pause_client).success
    assert wait_until(lambda: mission._state.value == "PAUSED")
    assert call_trigger(harness, harness.resume_client).success
    assert call_trigger(harness, harness.pause_client).success
    time.sleep(1.0)
    assert mission._state.value == "PAUSED"
    assert harness.received_paths[-1].path.poses == []


def test_large_correction_with_lost_pose_defers_instead_of_aborting(rig):
    mission, harness = rig
    harness.send_goal([50.0])
    assert wait_until(lambda: harness.non_empty_paths())
    harness.create_timer(
        0.2, lambda: harness.publish_follow_status(mission._session_id, mission._path_id))
    harness.is_pose_published = False
    time.sleep(0.8)
    message = LocalizationStatus()
    message.is_large_correction = True
    harness.localization_publisher.publish(message)
    time.sleep(3.0)  # plan_timeout_s (2 s) を超えて待つ
    assert mission._state.value != "IDLE"
    harness.is_pose_published = True
    assert wait_until(lambda: len(harness.planner_requests) == 2)

"""lightnav_mission のノード（lifecycle）。判定ロジックは mission_logic の純関数に委譲する。"""

import math
import random
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import Node as LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

from lightnav_mission.mission_logic import (
    MissionEvent,
    MissionState,
    NoMotionTracker,
    advance_waypoint_index,
    effective_arrival_radius_m,
    next_state,
    should_replan_on_large_correction,
    update_no_motion,
)
from lightnav_msgs.action import FollowWaypoints
from lightnav_msgs.msg import FollowStatus, LocalizationStatus, PathWithId
from lightnav_msgs.srv import PlanPath

GLOBAL_FRAME_ID = "map"
LOCALIZATION_STATE_UNKNOWN = 255
MAX_SESSION_ID = 2**32 - 1

PARAMETER_DEFAULTS = {
    "pose_topic": "/lightnav/pose",
    "localization_status_topic": "/lightnav/localization/status",
    "replan_on_large_correction": True,
    "default_arrival_radius_m": 1.0,
    "final_arrival_radius_m": 0.5,
    "pass_index_margin": 5,
    "tick_hz": 10.0,
    "pose_timeout_s": 0.5,
    "plan_timeout_s": 5.0,
    "v_progress_min": 0.1,
    "no_motion_timeout_s": 5.0,
    "no_motion_distance_m": 0.1,
}
POSITIVE_PARAMETERS = (
    "default_arrival_radius_m", "final_arrival_radius_m", "tick_hz",
    "pose_timeout_s", "plan_timeout_s", "no_motion_timeout_s", "no_motion_distance_m",
)


def is_goal_valid(goal: FollowWaypoints.Goal) -> bool:
    waypoint_count = len(goal.waypoints)
    if waypoint_count == 0 or goal.start_index >= waypoint_count:
        return False
    return all(
        math.isfinite(waypoint.x) and math.isfinite(waypoint.y)
        and math.isfinite(waypoint.arrival_radius_m) and waypoint.arrival_radius_m >= 0.0
        for waypoint in goal.waypoints
    )


class MissionNode(LifecycleNode):
    def __init__(self):
        super().__init__("lightnav_mission")
        self._session_id = random.randint(1, MAX_SESSION_ID)  # 0 は無効
        self._path_id = 0
        self._state = MissionState.IDLE
        self._goal_generation = 0
        self._generation_lock = threading.Lock()
        self._is_pause_requested = False
        self._latest_pose: PoseStamped | None = None
        self._latest_pose_time_s = 0.0
        self._latest_follow_status: FollowStatus | None = None
        self._localization_state = LOCALIZATION_STATE_UNKNOWN
        self._is_large_correction_pending = False
        self._parameters = {}
        self._last_path_message: PathWithId | None = None

    # ---- lifecycle ----

    def on_configure(self, previous_state) -> TransitionCallbackReturn:
        for name, default in PARAMETER_DEFAULTS.items():
            self.declare_parameter(name, default)
        self._parameters = {name: self.get_parameter(name).value for name in PARAMETER_DEFAULTS}
        if any(self._parameters[name] <= 0 for name in POSITIVE_PARAMETERS):
            self.get_logger().error("パラメータ不正: 正でなければならない値がある")
            return TransitionCallbackReturn.FAILURE

        group = ReentrantCallbackGroup()
        path_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        status_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._path_publisher = self.create_lifecycle_publisher(
            PathWithId, "/lightnav/path", path_qos)
        self.create_subscription(
            PoseStamped, self._parameters["pose_topic"], self._on_pose, 10,
            callback_group=group)
        self.create_subscription(
            FollowStatus, "/lightnav/follow_status", self._on_follow_status, 10,
            callback_group=group)
        if self._parameters["localization_status_topic"]:
            self.create_subscription(
                LocalizationStatus, self._parameters["localization_status_topic"],
                self._on_localization_status, status_qos, callback_group=group)
        self._plan_client = self.create_client(
            PlanPath, "/lightnav/planner/plan_path", callback_group=group)
        self.create_service(
            Trigger, "/lightnav/mission/pause", self._on_pause, callback_group=group)
        self.create_service(
            Trigger, "/lightnav/mission/resume", self._on_resume, callback_group=group)
        self._action_server = ActionServer(
            self, FollowWaypoints, "/lightnav/mission/follow_waypoints",
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=lambda _handle: CancelResponse.ACCEPT,
            callback_group=group)
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, previous_state) -> TransitionCallbackReturn:
        self._bump_generation()  # 実行中の goal を打ち切る
        return super().on_deactivate(previous_state)

    # ---- 入力 ----

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_pose(self, message: PoseStamped) -> None:
        position = message.pose.position
        if math.isfinite(position.x) and math.isfinite(position.y):
            self._latest_pose = message
            self._latest_pose_time_s = self._now_s()

    def _on_follow_status(self, message: FollowStatus) -> None:
        if message.session_id == self._session_id and message.path_id == self._path_id:
            self._latest_follow_status = message

    def _on_localization_status(self, message: LocalizationStatus) -> None:
        self._localization_state = message.state
        if message.is_large_correction:
            self._is_large_correction_pending = True

    def _on_pause(self, request, response):
        self._is_pause_requested = self._state is MissionState.FOLLOWING
        response.success = self._is_pause_requested
        response.message = "" if response.success else "FOLLOWING 中のみ pause できる"
        return response

    def _on_resume(self, request, response):
        response.success = self._state is MissionState.PAUSED
        if response.success:
            self._is_pause_requested = False
        else:
            response.message = "PAUSED 中のみ resume できる"
        return response

    def _on_goal(self, goal: FollowWaypoints.Goal) -> GoalResponse:
        is_accepted = self._is_active() and is_goal_valid(goal)
        if not is_accepted:
            self.get_logger().warn("goal を reject（非 active または不正な goal）")
        return GoalResponse.ACCEPT if is_accepted else GoalResponse.REJECT

    def _is_active(self) -> bool:
        return self._path_publisher.is_activated

    # ---- goal 実行 ----

    def _bump_generation(self) -> int:
        with self._generation_lock:
            self._goal_generation += 1
            return self._goal_generation

    def _is_current(self, generation: int) -> bool:
        return generation == self._goal_generation

    def _publish_path(self, path: Path | None = None, indices=()) -> None:
        self._path_id += 1
        self._latest_follow_status = None
        message = PathWithId()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = GLOBAL_FRAME_ID
        message.session_id = self._session_id
        message.path_id = self._path_id
        message.path = path if path is not None else Path()
        message.path.header.frame_id = GLOBAL_FRAME_ID
        message.waypoint_indices = list(indices)
        self._path_publisher.publish(message)
        self._last_path_message = message

    def _stop_path(self) -> None:
        self._publish_path()

    def _request_plan(self, waypoints, start_index):
        """成功なら (path, 絶対インデックス対応の経路 index 配列)、失敗なら (None, 理由)。"""
        if self._latest_pose is None:
            return None, "姿勢が未受信"
        request = PlanPath.Request()
        request.frame_id = GLOBAL_FRAME_ID
        request.start = self._latest_pose
        request.waypoints = list(waypoints[start_index:])
        if not self._plan_client.wait_for_service(timeout_sec=self._parameters["plan_timeout_s"]):
            return None, "PlanPath サービスが無い"
        future = self._plan_client.call_async(request)
        deadline = time.monotonic() + self._parameters["plan_timeout_s"]
        while not future.done():
            if time.monotonic() > deadline:
                future.cancel()
                return None, "PlanPath がタイムアウト"
            time.sleep(0.01)
        response = future.result()
        if not response.success:
            return None, f"PlanPath 失敗: {response.message}"
        return response.path, list(response.waypoint_indices)

    def _plan_and_publish(self, waypoints, start_index):
        path, indices_or_reason = self._request_plan(waypoints, start_index)
        if path is None:
            return None, indices_or_reason
        self._publish_path(path, indices_or_reason)
        return indices_or_reason, ""

    def _transition(self, event: MissionEvent) -> None:
        self._state = next_state(self._state, event)

    def _finish(self, goal_handle, generation, is_success, message, reached_count):
        """終了処理。preempt 済みなら新しい goal の経路を壊さないよう stop を出さない。"""
        if self._is_current(generation):
            self._stop_path()
            self._transition(MissionEvent.CANCEL)
        result = FollowWaypoints.Result(
            success=is_success, message=message, reached_count=reached_count)
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        elif is_success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def _build_feedback(self, index, distance_m, is_not_moving):
        status = self._latest_follow_status
        feedback = FollowWaypoints.Feedback()
        feedback.state = self._state.value
        feedback.current_waypoint_index = index
        feedback.distance_to_waypoint_m = float(distance_m)
        feedback.path_id = self._path_id
        feedback.localization_state = self._localization_state
        feedback.is_not_moving = is_not_moving
        if status is not None:
            feedback.follow_state = status.state
            feedback.active_layer = status.active_layer
            feedback.is_blocked = status.state == FollowStatus.BLOCKED
            feedback.is_stuck = status.is_stuck
        return feedback

    def _execute(self, goal_handle):
        generation = self._bump_generation()
        goal = goal_handle.request
        waypoints = list(goal.waypoints)
        index = goal.start_index
        self._is_pause_requested = False
        self._is_large_correction_pending = False
        self._transition(MissionEvent.GOAL_ACCEPTED)

        path_indices, reason = self._plan_and_publish(waypoints, index)
        if path_indices is None:
            self._transition(MissionEvent.PLAN_FAILED)
            return self._finish(goal_handle, generation, False, reason, index)
        self._transition(MissionEvent.PLAN_SUCCEEDED)
        plan_offset = index  # path_indices[k] は waypoints[plan_offset + k] に対応

        parameters = self._parameters
        tracker = NoMotionTracker()
        plan_sent_s = self._now_s()
        has_resent = False
        period_s = 1.0 / parameters["tick_hz"]
        while rclpy.ok():
            time.sleep(period_s)
            if not self._is_current(generation):
                return self._finish(goal_handle, generation, False, "新しい goal に置き換えられた", index)
            if goal_handle.is_cancel_requested:
                return self._finish(goal_handle, generation, False, "キャンセルされた", index)

            if self._state is MissionState.PAUSED:
                if not self._is_pause_requested:
                    self._transition(MissionEvent.RESUME)
                    path_indices, reason = self._plan_and_publish(waypoints, index)
                    if path_indices is None:
                        return self._finish(goal_handle, generation, False, reason, index)
                    self._transition(MissionEvent.PLAN_SUCCEEDED)
                    plan_offset, plan_sent_s, has_resent = index, self._now_s(), False
                goal_handle.publish_feedback(self._build_feedback(index, 0.0, False))
                continue
            if self._is_pause_requested:
                self._transition(MissionEvent.PAUSE)
                self._stop_path()
                continue

            if should_replan_on_large_correction(
                self._state, self._is_large_correction_pending,
                parameters["replan_on_large_correction"],
            ):
                self._is_large_correction_pending = False
                path_indices, reason = self._plan_and_publish(waypoints, index)
                if path_indices is None:
                    return self._finish(goal_handle, generation, False, reason, index)
                plan_offset, plan_sent_s, has_resent = index, self._now_s(), False
            self._is_large_correction_pending = False

            pose = self._latest_pose
            is_pose_fresh = (
                pose is not None
                and self._now_s() - self._latest_pose_time_s <= parameters["pose_timeout_s"]
            )
            status = self._latest_follow_status
            if status is None:
                elapsed_s = self._now_s() - plan_sent_s
                if elapsed_s > parameters["plan_timeout_s"]:
                    if has_resent:
                        return self._finish(
                            goal_handle, generation, False, "FollowStatus が一致しない", index)
                    self._path_publisher.publish(self._last_path_message)
                    plan_sent_s, has_resent = self._now_s(), True

            if not is_pose_fresh:
                # 到達判定は保留（abort しない）
                goal_handle.publish_feedback(self._build_feedback(index, 0.0, False))
                continue

            target = waypoints[index]
            distance_m = math.hypot(
                pose.pose.position.x - target.x, pose.pose.position.y - target.y)
            is_final = index == len(waypoints) - 1
            radius_m = effective_arrival_radius_m(
                target.arrival_radius_m, is_final,
                parameters["default_arrival_radius_m"], parameters["final_arrival_radius_m"])
            new_index = advance_waypoint_index(
                index, len(waypoints), distance_m, radius_m,
                status.closest_index if status is not None else None,
                path_indices[index - plan_offset], parameters["pass_index_margin"])
            if new_index == len(waypoints):
                self._transition(MissionEvent.FINAL_REACHED)
                return self._finish(goal_handle, generation, True, "全点到達", new_index)
            index = new_index

            tracker, is_not_moving = update_no_motion(
                tracker, self._now_s(), pose.pose.position.x, pose.pose.position.y,
                status.commanded_linear_mps if status is not None else 0.0,
                parameters["v_progress_min"], parameters["no_motion_timeout_s"],
                parameters["no_motion_distance_m"])
            if is_not_moving:
                self.get_logger().warn("指令しているのに動いていない（モーター無効・空転の可能性）")
            goal_handle.publish_feedback(self._build_feedback(index, distance_m, is_not_moving))
        return self._finish(goal_handle, generation, False, "シャットダウン", index)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

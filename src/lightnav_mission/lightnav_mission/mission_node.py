"""lightnav_mission のノード（lifecycle）。判定ロジックは mission_logic の純関数に委譲する。"""

import math
import random
import threading
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
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
    validate_plan_response,
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
READ_ONLY_PARAMETERS = {"pose_topic", "localization_status_topic", "tick_hz"}
NON_NEGATIVE_PARAMETERS = ("pass_index_margin", "v_progress_min")
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
    def __init__(self, **node_kwargs):
        super().__init__("lightnav_mission", **node_kwargs)
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
        self._mission_parameters = {}
        self._mission_services, self._mission_subscriptions = [], []
        self._follow_status_time_s = 0.0
        self._was_large_correction = False
        for name, default in PARAMETER_DEFAULTS.items():
            descriptor = ParameterDescriptor(read_only=name in READ_ONLY_PARAMETERS)
            self.declare_parameter(name, default, descriptor)
        self._mission_parameters = {name: self.get_parameter(name).value for name in PARAMETER_DEFAULTS}
        self._pose_timeout_baseline = self._mission_parameters["pose_timeout_s"]
        self.add_pre_set_parameters_callback(self._log_read_only_attempts)
        self.add_on_set_parameters_callback(self._validate_parameter_changes)
        self.add_post_set_parameters_callback(self._apply_parameter_changes)
        self._last_path_message: PathWithId | None = None

    def _log_read_only_attempts(self, parameters):
        # descriptor による拒否は on_set より先に起こる。
        for parameter in parameters:
            if parameter.name in READ_ONLY_PARAMETERS:
                self.get_logger().warn(
                    f"parameter change rejected: {parameter.name}: read-only")
        return parameters

    def _parameter_error(self, values):
        for name in POSITIVE_PARAMETERS + NON_NEGATIVE_PARAMETERS:
            value = values[name]
            if not math.isfinite(value):
                return f"{name}: value must be finite"
            if value < 0 or (name in POSITIVE_PARAMETERS and value == 0):
                return f"{name}: value is outside the allowed range"
        if not values["pose_topic"]:
            return "pose_topic: must not be empty"
        return ""

    def _validate_parameter_changes(self, parameters):
        values = dict(self._mission_parameters)
        reason = ""
        for parameter in parameters:
            name, value = parameter.name, parameter.value
            if name == "use_sim_time" or name in READ_ONLY_PARAMETERS:
                reason = f"{name}: runtime changes are read-only"
            elif name in PARAMETER_DEFAULTS:
                if type(value) is not type(PARAMETER_DEFAULTS[name]):
                    reason = f"{name}: incorrect type"
                elif name == "pose_timeout_s" and not (
                    math.isfinite(self._pose_timeout_baseline)
                    and math.isfinite(value) and value <= self._pose_timeout_baseline
                ):
                    reason = f"{name}: must be finite and no greater than the startup value"
                else:
                    values[name] = value
            if reason:
                break
        if not reason:
            reason = self._parameter_error(values)
        if reason:
            self.get_logger().warn(f"parameter change rejected: {reason}")
        return SetParametersResult(successful=not bool(reason), reason=reason)

    def _apply_parameter_changes(self, parameters):
        old_values = self._mission_parameters
        values = dict(old_values)
        for parameter in parameters:
            if parameter.name in PARAMETER_DEFAULTS:
                values[parameter.name] = parameter.value
        self._mission_parameters = values
        for parameter in parameters:
            if parameter.name in old_values and old_values[parameter.name] != parameter.value:
                self.get_logger().info(
                    f"parameter changed: {parameter.name} "
                    f"{old_values[parameter.name]} -> {parameter.value}")

    # ---- lifecycle ----

    def on_configure(self, previous_state) -> TransitionCallbackReturn:
        self._mission_parameters = {name: self.get_parameter(name).value for name in PARAMETER_DEFAULTS}
        reason = self._parameter_error(self._mission_parameters)
        if reason:
            self.get_logger().error(f"パラメータ不正: {reason}")
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
        self._mission_subscriptions.append(self.create_subscription(
            PoseStamped, self._mission_parameters["pose_topic"], self._on_pose, 10,
            callback_group=group))
        self._mission_subscriptions.append(self.create_subscription(
            FollowStatus, "/lightnav/follow_status", self._on_follow_status, 10,
            callback_group=group))
        if self._mission_parameters["localization_status_topic"]:
            self._mission_subscriptions.append(self.create_subscription(
                LocalizationStatus, self._mission_parameters["localization_status_topic"],
                self._on_localization_status, status_qos, callback_group=group))
        self._plan_client = self.create_client(
            PlanPath, "/lightnav/planner/plan_path", callback_group=group)
        self._mission_services.append(self.create_service(
            Trigger, "/lightnav/mission/pause", self._on_pause, callback_group=group))
        self._mission_services.append(self.create_service(
            Trigger, "/lightnav/mission/resume", self._on_resume, callback_group=group))
        self._action_server = ActionServer(
            self, FollowWaypoints, "/lightnav/mission/follow_waypoints",
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=lambda _handle: CancelResponse.ACCEPT,
            callback_group=group)
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, previous_state) -> TransitionCallbackReturn:
        # 実行中の goal を打ち切り、publisher が無効になる前に停止経路を出して IDLE に戻す
        with self._generation_lock:
            self._goal_generation += 1
            self._publish_path_locked()
            self._state = MissionState.IDLE
            self._is_pause_requested = False
        return super().on_deactivate(previous_state)

    def on_cleanup(self, previous_state) -> TransitionCallbackReturn:
        self._action_server.destroy()
        for service in list(self._mission_services):
            self.destroy_service(service)
        for subscription in list(self._mission_subscriptions):
            self.destroy_subscription(subscription)
        self.destroy_client(self._plan_client)
        self.destroy_publisher(self._path_publisher)
        self._mission_services, self._mission_subscriptions = [], []
        return TransitionCallbackReturn.SUCCESS

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
            self._follow_status_time_s = self._now_s()

    def _on_localization_status(self, message: LocalizationStatus) -> None:
        self._localization_state = message.state
        if message.is_large_correction and not self._was_large_correction:
            self._is_large_correction_pending = True  # 立ち上がりだけで再計画する
        self._was_large_correction = message.is_large_correction

    def _on_pause(self, request, response):
        # PAUSED 中の再送や resume 直後の pause でも要求を立てる（再開させない）
        response.success = self._state in (
            MissionState.FOLLOWING, MissionState.PLANNING, MissionState.PAUSED)
        if response.success:
            self._is_pause_requested = True
        response.message = "" if response.success else "実行中の goal が無い"
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

    def _publish_path_locked(self, path: Path | None = None, indices=()) -> None:
        """_generation_lock を持って呼ぶ。path_id の採番と出版を排他する。"""
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
        if self._path_publisher.is_activated:
            self._path_publisher.publish(message)
        self._last_path_message = message

    def _publish_if_current(self, generation, path=None, indices=()) -> bool:
        with self._generation_lock:
            if not self._is_current(generation):
                return False
            self._publish_path_locked(path, indices)
            return True

    def _transition_if_current(self, generation: int, event: MissionEvent) -> None:
        with self._generation_lock:
            if self._is_current(generation):
                self._state = next_state(self._state, event)

    def _finish(self, goal_handle, generation, is_success, message, reached_count):
        """終了処理。現行 goal のときだけ stop を出して IDLE に戻す（preempt 先を壊さない）。"""
        with self._generation_lock:
            if self._is_current(generation):
                self._publish_path_locked()
                self._state = next_state(self._state, MissionEvent.ABORT)
        result = FollowWaypoints.Result(
            success=is_success, message=message, reached_count=reached_count)
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        elif is_success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def _build_feedback(self, index, distance_m, is_not_moving, is_planning=False):
        status = self._latest_follow_status
        feedback = FollowWaypoints.Feedback()
        feedback.state = MissionState.PLANNING.value if is_planning else self._state.value
        feedback.current_waypoint_index = index
        feedback.distance_to_waypoint_m = float(distance_m)
        feedback.path_id = self._path_id
        feedback.localization_state = self._localization_state
        feedback.is_not_moving = is_not_moving
        feedback.follow_state = FollowStatus.NO_PATH
        feedback.active_layer = FollowStatus.LAYER_NONE
        if status is not None:
            feedback.follow_state = status.state
            feedback.active_layer = status.active_layer
            feedback.is_blocked = status.state == FollowStatus.BLOCKED
            feedback.is_stuck = status.is_stuck
        return feedback

    def _is_pose_fresh(self, parameters) -> bool:
        return (
            self._latest_pose is not None
            and self._now_s() - self._latest_pose_time_s <= parameters["pose_timeout_s"]
        )

    def _begin_plan(self, waypoints, index, parameters) -> "PendingPlan":
        """計画要求を作る。応答待ちは tick ごとに _poll_plan で行い、スレッドをブロックしない。"""
        return PendingPlan(
            waypoints=waypoints, index=index,
            deadline_s=self._now_s() + parameters["plan_timeout_s"],
            timeout_s=parameters["plan_timeout_s"])

    def _poll_plan(self, pending: "PendingPlan", parameters):
        """戻り値: ("waiting"|"failed"|"done", 理由, 経路, 経路 index 列)。"""
        if pending.future is None:
            if not self._is_pose_fresh(parameters):
                # 姿勢途絶中は計画を保留する（abort しない）。復帰後に改めて期限を数える
                pending.deadline_s = self._now_s() + pending.timeout_s
                return "waiting", "", None, None
            if self._now_s() > pending.deadline_s:
                return "failed", "姿勢またはPlanPathサービスが得られずタイムアウト", None, None
            if not (self._is_pose_fresh(parameters) and self._plan_client.service_is_ready()):
                return "waiting", "", None, None
            request = PlanPath.Request()
            request.frame_id = GLOBAL_FRAME_ID
            request.start = self._latest_pose
            request.waypoints = list(pending.waypoints[pending.index:])
            pending.future = self._plan_client.call_async(request)
        if not pending.future.done():
            if self._now_s() > pending.deadline_s:
                pending.future.cancel()
                return "failed", "PlanPath がタイムアウト", None, None
            return "waiting", "", None, None
        response = pending.future.result()
        if response is None or not response.success:
            reason = response.message if response is not None else "応答なし"
            return "failed", f"PlanPath 失敗: {reason}", None, None
        problem = validate_plan_response(
            len(response.path.poses), response.waypoint_indices,
            len(pending.waypoints) - pending.index)
        if problem:
            return "failed", f"PlanPath 応答が不正: {problem}", None, None
        return "done", "", response.path, list(response.waypoint_indices)

    def _execute(self, goal_handle):
        generation = self._bump_generation()
        try:
            return self._run_goal(goal_handle, generation)
        except Exception as error:  # noqa: BLE001 例外でも必ず停止経路を出して abort する
            self.get_logger().error(f"mission 実行中の例外: {error!r}")
            return self._finish(goal_handle, generation, False, f"内部エラー: {error!r}", 0)

    def _run_goal(self, goal_handle, generation):
        waypoints = list(goal_handle.request.waypoints)
        index = goal_handle.request.start_index
        parameters = self._mission_parameters
        with self._generation_lock:
            is_superseded = not self._is_current(generation)
            if not is_superseded:
                if self._state is not MissionState.IDLE:
                    self._publish_path_locked()  # preempt: 古い経路で走らせたまま計画を待たない
                self._is_pause_requested = False
                self._is_large_correction_pending = False
                self._state = next_state(self._state, MissionEvent.GOAL_ACCEPTED)
        if is_superseded:
            return self._finish(goal_handle, generation, False, "開始前に打ち切られた", index)
        pending = self._begin_plan(waypoints, index, parameters)
        path_indices, plan_offset = None, index
        tracker = NoMotionTracker()
        plan_sent_s, has_resent = self._now_s(), False
        period_s = 1.0 / parameters["tick_hz"]
        while rclpy.ok():
            time.sleep(period_s)
            parameters = self._mission_parameters
            if not self._is_current(generation):
                return self._finish(
                    goal_handle, generation, False, "preempt または deactivate で打ち切られた", index)
            if goal_handle.is_cancel_requested:
                return self._finish(goal_handle, generation, False, "キャンセルされた", index)

            if self._state is MissionState.PAUSED:
                if not self._is_pause_requested:
                    self._transition_if_current(generation, MissionEvent.RESUME)
                    self._is_large_correction_pending = False
                    pending = self._begin_plan(waypoints, index, parameters)
                goal_handle.publish_feedback(self._build_feedback(index, 0.0, False))
                if self._state is MissionState.PAUSED:
                    continue
            elif self._is_pause_requested:
                pending = None  # 計画待ちも破棄して停止する
                if self._publish_if_current(generation):
                    self._transition_if_current(generation, MissionEvent.PAUSE)
                continue
            if (
                pending is None
                and should_replan_on_large_correction(
                    self._state, self._is_large_correction_pending,
                    parameters["replan_on_large_correction"])
            ):
                pending = self._begin_plan(waypoints, index, parameters)
                self._is_large_correction_pending = False

            if pending is not None:
                outcome, reason, path, indices = self._poll_plan(pending, parameters)
                if outcome == "failed":
                    self._transition_if_current(generation, MissionEvent.PLAN_FAILED)
                    return self._finish(goal_handle, generation, False, reason, index)
                if outcome == "waiting":
                    goal_handle.publish_feedback(
                        self._build_feedback(index, 0.0, False, is_planning=True))
                    continue
                self._is_large_correction_pending = False  # 新しい経路は現在位置から計画済み
                if not self._publish_if_current(generation, path, indices):
                    continue  # 次の tick で打ち切りを検出する
                self._transition_if_current(generation, MissionEvent.PLAN_SUCCEEDED)
                path_indices, plan_offset = indices, pending.index
                plan_sent_s, has_resent = self._now_s(), False
                pending = None
            if path_indices is None:
                continue

            status = self._latest_follow_status
            if status is not None and self._follow_status_time_s > plan_sent_s:
                has_resent = False
            last_alive_s = max(plan_sent_s, self._follow_status_time_s) if status else plan_sent_s
            if self._now_s() - last_alive_s > parameters["plan_timeout_s"]:
                if has_resent:
                    return self._finish(
                        goal_handle, generation, False, "FollowStatus が途絶または不一致", index)
                with self._generation_lock:
                    if self._is_current(generation) and self._last_path_message is not None:
                        self._path_publisher.publish(self._last_path_message)
                plan_sent_s, has_resent = self._now_s(), True

            if not self._is_pose_fresh(parameters):
                goal_handle.publish_feedback(self._build_feedback(index, 0.0, False))
                continue  # 到達判定は保留（abort しない）
            pose = self._latest_pose
            target = waypoints[index]
            distance_m = math.hypot(
                pose.pose.position.x - target.x, pose.pose.position.y - target.y)
            radius_m = effective_arrival_radius_m(
                target.arrival_radius_m, index == len(waypoints) - 1,
                parameters["default_arrival_radius_m"], parameters["final_arrival_radius_m"])
            new_index = advance_waypoint_index(
                index, len(waypoints), distance_m, radius_m,
                status.closest_index if status is not None else None,
                path_indices[index - plan_offset], parameters["pass_index_margin"])
            if new_index == len(waypoints):
                self._transition_if_current(generation, MissionEvent.FINAL_REACHED)
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


@dataclass
class PendingPlan:
    """応答待ちの PlanPath 要求。deadline はサービス待ちと応答待ちで共有する。"""

    waypoints: list
    index: int
    deadline_s: float
    timeout_s: float
    future: object = None


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

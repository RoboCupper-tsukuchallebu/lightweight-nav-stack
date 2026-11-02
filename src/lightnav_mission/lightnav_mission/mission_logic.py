"""ミッション状態機械・到達判定・不動検知（ROS 非依存の純関数）。"""

import math
from dataclasses import dataclass
from enum import Enum
from itertools import pairwise


class MissionState(str, Enum):
    IDLE = "IDLE"
    PLANNING = "PLANNING"
    FOLLOWING = "FOLLOWING"
    PAUSED = "PAUSED"


class MissionEvent(str, Enum):
    GOAL_ACCEPTED = "GOAL_ACCEPTED"
    PLAN_SUCCEEDED = "PLAN_SUCCEEDED"
    PLAN_FAILED = "PLAN_FAILED"
    PAUSE = "PAUSE"
    RESUME = "RESUME"
    FINAL_REACHED = "FINAL_REACHED"
    CANCEL = "CANCEL"
    ABORT = "ABORT"


_TRANSITIONS = {
    (MissionState.IDLE, MissionEvent.GOAL_ACCEPTED): MissionState.PLANNING,
    # 2 つ目の goal は 1 つ目を preempt する
    (MissionState.PLANNING, MissionEvent.GOAL_ACCEPTED): MissionState.PLANNING,
    (MissionState.FOLLOWING, MissionEvent.GOAL_ACCEPTED): MissionState.PLANNING,
    (MissionState.PAUSED, MissionEvent.GOAL_ACCEPTED): MissionState.PLANNING,
    (MissionState.PLANNING, MissionEvent.PLAN_SUCCEEDED): MissionState.FOLLOWING,
    (MissionState.PLANNING, MissionEvent.PLAN_FAILED): MissionState.IDLE,
    (MissionState.FOLLOWING, MissionEvent.PAUSE): MissionState.PAUSED,
    (MissionState.PAUSED, MissionEvent.RESUME): MissionState.PLANNING,
    (MissionState.FOLLOWING, MissionEvent.FINAL_REACHED): MissionState.IDLE,
}
_TERMINATING_EVENTS = (MissionEvent.CANCEL, MissionEvent.ABORT)


def next_state(state: MissionState, event: MissionEvent) -> MissionState:
    """遷移表に無い組み合わせは現状維持（無視）。CANCEL / ABORT はどこからでも IDLE。"""
    if event in _TERMINATING_EVENTS:
        return MissionState.IDLE
    return _TRANSITIONS.get((state, event), state)


def should_replan_on_large_correction(
    state: MissionState, is_large_correction: bool, is_replan_enabled: bool
) -> bool:
    return is_replan_enabled and is_large_correction and state is MissionState.FOLLOWING


def effective_arrival_radius_m(
    waypoint_radius_m: float,
    is_final: bool,
    default_arrival_radius_m: float,
    final_arrival_radius_m: float,
) -> float:
    if waypoint_radius_m > 0.0:
        return waypoint_radius_m
    return final_arrival_radius_m if is_final else default_arrival_radius_m


def advance_waypoint_index(
    current_index: int,
    waypoint_count: int,
    distance_to_current_m: float,
    arrival_radius_m: float,
    closest_index: int | None,
    current_path_index: int,
    pass_index_margin: int,
) -> int:
    """到達判定。距離条件、または経路インデックス通過条件で 1 つ進める。戻らない。

    最終ウェイポイントは距離条件のみ。進めた結果が waypoint_count なら全点到達。
    """
    is_final = current_index == waypoint_count - 1
    is_within_radius = distance_to_current_m < arrival_radius_m
    has_passed = (
        not is_final
        and closest_index is not None
        and closest_index > current_path_index + pass_index_margin
    )
    if is_within_radius or has_passed:
        return current_index + 1
    return current_index


@dataclass(frozen=True)
class NoMotionTracker:
    """指令中の不動検知の状態（不変。update が新しい値を返す）。"""

    start_stamp_s: float | None = None
    start_x: float = 0.0
    start_y: float = 0.0


def update_no_motion(
    tracker: NoMotionTracker,
    now_s: float,
    x: float,
    y: float,
    commanded_linear_mps: float,
    v_progress_min: float,
    no_motion_timeout_s: float,
    no_motion_distance_m: float,
) -> tuple[NoMotionTracker, bool]:
    """commanded_linear_mps > v_progress_min が timeout 続き、移動が閾値未満なら True。"""
    if commanded_linear_mps <= v_progress_min:
        return NoMotionTracker(), False
    if tracker.start_stamp_s is None:
        return NoMotionTracker(now_s, x, y), False
    if math.hypot(x - tracker.start_x, y - tracker.start_y) >= no_motion_distance_m:
        # 動いているので窓を張り直す
        return NoMotionTracker(now_s, x, y), False
    return tracker, now_s - tracker.start_stamp_s >= no_motion_timeout_s


def validate_plan_response(
    pose_count: int, waypoint_indices, expected_waypoint_count: int
) -> str | None:
    """PlanPath 応答の整合性を検証する。問題があれば理由、無ければ None。"""
    if pose_count == 0:
        return "経路が空"
    if len(waypoint_indices) != expected_waypoint_count:
        return f"waypoint_indices の長さ {len(waypoint_indices)} が waypoint 数 {expected_waypoint_count} と違う"
    if any(not 0 <= path_index < pose_count for path_index in waypoint_indices):
        return "waypoint_indices が経路の範囲外"
    if any(later < earlier for earlier, later in pairwise(waypoint_indices)):
        return "waypoint_indices が単調でない"
    return None

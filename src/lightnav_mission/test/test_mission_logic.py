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


def test_normal_flow_reaches_idle():
    state = MissionState.IDLE
    for event in (MissionEvent.GOAL_ACCEPTED, MissionEvent.PLAN_SUCCEEDED, MissionEvent.FINAL_REACHED):
        state = next_state(state, event)
    assert state is MissionState.IDLE


def test_plan_failure_returns_to_idle():
    assert next_state(MissionState.PLANNING, MissionEvent.PLAN_FAILED) is MissionState.IDLE


def test_pause_and_resume():
    paused = next_state(MissionState.FOLLOWING, MissionEvent.PAUSE)
    assert paused is MissionState.PAUSED
    assert next_state(paused, MissionEvent.RESUME) is MissionState.PLANNING


def test_second_goal_preempts_following():
    assert next_state(MissionState.FOLLOWING, MissionEvent.GOAL_ACCEPTED) is MissionState.PLANNING


def test_invalid_event_is_ignored_and_cancel_always_idles():
    assert next_state(MissionState.IDLE, MissionEvent.PAUSE) is MissionState.IDLE
    assert next_state(MissionState.PAUSED, MissionEvent.CANCEL) is MissionState.IDLE


def test_replan_only_while_following():
    assert should_replan_on_large_correction(MissionState.FOLLOWING, True, True)
    assert not should_replan_on_large_correction(MissionState.PAUSED, True, True)
    assert not should_replan_on_large_correction(MissionState.FOLLOWING, True, False)
    assert not should_replan_on_large_correction(MissionState.FOLLOWING, False, True)


def test_arrival_radius_selection():
    assert effective_arrival_radius_m(0.0, False, 1.0, 0.5) == 1.0
    assert effective_arrival_radius_m(0.0, True, 1.0, 0.5) == 0.5
    assert effective_arrival_radius_m(2.0, True, 1.0, 0.5) == 2.0


def test_advance_by_distance():
    assert advance_waypoint_index(0, 3, 0.9, 1.0, None, 10, 5) == 1
    assert advance_waypoint_index(0, 3, 1.0, 1.0, None, 10, 5) == 0


def test_advance_by_passed_path_index_even_outside_radius():
    assert advance_waypoint_index(0, 3, 3.0, 1.0, 16, 10, 5) == 1
    assert advance_waypoint_index(0, 3, 3.0, 1.0, 15, 10, 5) == 0


def test_final_waypoint_uses_distance_only():
    assert advance_waypoint_index(2, 3, 3.0, 0.5, 999, 10, 5) == 2
    assert advance_waypoint_index(2, 3, 0.4, 0.5, None, 10, 5) == 3


def _run_tracker(samples, **overrides):
    parameters = {"v_progress_min": 0.1, "no_motion_timeout_s": 5.0, "no_motion_distance_m": 0.1}
    parameters.update(overrides)
    tracker, flag = NoMotionTracker(), False
    for now_s, x, commanded in samples:
        tracker, flag = update_no_motion(tracker, now_s, x, 0.0, commanded, **parameters)
    return flag


def test_not_moving_detected_after_timeout():
    assert _run_tracker([(0.0, 0.0, 0.3), (5.0, 0.05, 0.3)])


def test_not_moving_not_flagged_before_timeout_or_when_moving_or_idle():
    assert not _run_tracker([(0.0, 0.0, 0.3), (4.9, 0.0, 0.3)])
    assert not _run_tracker([(0.0, 0.0, 0.3), (5.0, 0.5, 0.3)])
    assert not _run_tracker([(0.0, 0.0, 0.05), (9.0, 0.0, 0.05)])
    assert not _run_tracker([(0.0, 0.0, 0.3), (3.0, 0.0, 0.0), (6.0, 0.0, 0.3)])

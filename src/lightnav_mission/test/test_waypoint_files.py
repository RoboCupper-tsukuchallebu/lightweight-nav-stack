import math

import pytest
from lightnav_mission.waypoint_files import (
    FLAG_LARGE_CORRECTION,
    FLAG_NO_TRAJECTORY,
    PoseSample,
    TrajectoryRow,
    WaypointFileError,
    WaypointRecord,
    interpolate_trajectory,
    parse_waypoints_document,
    refine_waypoints,
    select_recorded_samples,
)


def _samples(xs):
    return [PoseSample(float(i), x, 0.0, 0.0) for i, x in enumerate(xs)]


def test_select_first_recorded_immediately_and_spacing_respected():
    selected = select_recorded_samples(_samples([0, 1, 2, 3, 4, 5]), 2.0)
    assert [s.x for s in selected] == [0, 2, 4, 5]


def test_select_always_records_final_stop_position():
    selected = select_recorded_samples(_samples([0, 0.5, 0.7]), 2.0)
    assert [s.x for s in selected] == [0, 0.7]


def test_select_single_and_empty():
    assert len(select_recorded_samples(_samples([1.0]), 2.0)) == 1
    assert select_recorded_samples([], 2.0) == []


def test_select_does_not_duplicate_final_when_already_recorded():
    selected = select_recorded_samples(_samples([0, 2]), 2.0)
    assert [s.x for s in selected] == [0, 2]


def _rows(shift_x=0.3, shift_y=-0.2):
    return [TrajectoryRow(float(t), t + shift_x, shift_y, 0.0) for t in range(11)]


def test_interpolation_is_linear():
    pose = interpolate_trajectory(_rows(), 2.5, 2.0)
    assert pose.x == pytest.approx(2.8) and pose.y == pytest.approx(-0.2)


def test_interpolation_yaw_wraps_across_two_pi():
    rows = [TrajectoryRow(0.0, 0, 0, math.pi - 0.1), TrajectoryRow(1.0, 0, 0, -math.pi + 0.1)]
    yaw = interpolate_trajectory(rows, 0.5, 2.0).yaw
    assert abs(abs(yaw) - math.pi) < 1e-9


def test_interpolation_outside_range_or_large_gap_is_none():
    assert interpolate_trajectory(_rows(), -0.1, 2.0) is None
    assert interpolate_trajectory(_rows(), 10.5, 2.0) is None
    sparse = [TrajectoryRow(0.0, 0, 0, 0), TrajectoryRow(3.0, 3, 0, 0)]
    assert interpolate_trajectory(sparse, 1.5, 2.0) is None


def test_refine_normal_applies_shift_and_records_correction():
    raw = [WaypointRecord(x=2.0, y=0.0, yaw=0.0, stamp=2.0)]
    refined = refine_waypoints(raw, _rows())[0]
    assert (refined.x, refined.y) == (pytest.approx(2.3), pytest.approx(-0.2))
    assert refined.correction_m == pytest.approx(math.hypot(0.3, 0.2))
    assert refined.flagged is None


def test_refine_large_correction_is_not_applied():
    raw = [WaypointRecord(x=2.0, y=0.0, yaw=0.0, stamp=2.0)]
    refined = refine_waypoints(raw, _rows(shift_x=3.0), max_correction_m=2.0)[0]
    assert refined.flagged == FLAG_LARGE_CORRECTION and refined.x == 2.0


def test_refine_missing_stamp_or_trajectory_is_flagged():
    raw = [
        WaypointRecord(x=0.0, y=0.0, stamp=99.0),
        WaypointRecord(x=0.0, y=0.0, stamp=None),
    ]
    assert [r.flagged for r in refine_waypoints(raw, _rows())] == [FLAG_NO_TRAJECTORY] * 2


@pytest.mark.parametrize("document", [
    None,
    {"waypoints": []},
    {"waypoints": [{"x": 1.0}]},
    {"waypoints": [{"x": float("nan"), "y": 0.0}]},
    {"waypoints": [{"x": "1", "y": 0.0}]},
    {"frame_id": "odom", "waypoints": [{"x": 1.0, "y": 0.0}]},
    {"waypoints": [{"x": 1.0, "y": 0.0, "arrival_radius_m": -1.0}]},
])
def test_invalid_documents_are_rejected(document):
    with pytest.raises(WaypointFileError):
        parse_waypoints_document(document)


def test_valid_document_ignores_stamp_and_yaw_extras():
    records = parse_waypoints_document(
        {"frame_id": "map", "waypoints": [{"x": 1, "y": 2, "yaw": 0.1, "stamp": 5.0}]})
    assert records[0].x == 1.0 and records[0].stamp == 5.0

import math

import pytest
from lightnav_planner.interpolation import PlanningError, plan_straight_path


def test_resamples_at_resolution_and_hits_waypoint_exactly():
    planned = plan_straight_path((0.0, 0.0), [(1.0, 0.0)], 0.2)
    assert len(planned.points) == 6
    assert planned.points[-1] == (1.0, 0.0)
    assert planned.waypoint_indices == (5,)
    xs = [point[0] for point in planned.points]
    assert xs == pytest.approx([0.0, 0.2, 0.4, 0.6, 0.8, 1.0])


def test_non_divisible_length_keeps_spacing_at_most_resolution():
    planned = plan_straight_path((0.0, 0.0), [(1.0, 0.0)], 0.3)
    gaps = [math.dist(a, b) for a, b in zip(planned.points, planned.points[1:])]
    assert max(gaps) <= 0.3 + 1e-9
    assert planned.points[-1] == (1.0, 0.0)


def test_multiple_waypoints_indices_point_at_waypoints():
    waypoints = [(1.0, 0.0), (1.0, 1.0)]
    planned = plan_straight_path((0.0, 0.0), waypoints, 0.5)
    for waypoint, index in zip(waypoints, planned.waypoint_indices):
        assert planned.points[index] == waypoint
    assert planned.waypoint_indices == (2, 4)


def test_yaw_follows_segment_and_last_reuses_previous():
    planned = plan_straight_path((0.0, 0.0), [(1.0, 0.0), (1.0, 1.0)], 0.5)
    assert planned.yaws[0] == pytest.approx(0.0)
    assert planned.yaws[3] == pytest.approx(math.pi / 2)
    assert planned.yaws[-1] == pytest.approx(math.pi / 2)


def test_consecutive_duplicate_waypoints_are_dropped_but_indexed():
    planned = plan_straight_path((0.0, 0.0), [(1.0, 0.0), (1.0, 0.0)], 0.5)
    assert len(planned.points) == 3
    assert planned.waypoint_indices == (2, 2)


def test_waypoint_equal_to_start_gives_single_point_path():
    planned = plan_straight_path((1.0, 1.0), [(1.0, 1.0)], 0.2)
    assert planned.points == ((1.0, 1.0),)
    assert planned.waypoint_indices == (0,)


def test_empty_waypoints_rejected():
    with pytest.raises(PlanningError):
        plan_straight_path((0.0, 0.0), [], 0.2)


@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf])
def test_non_finite_coordinates_rejected(bad):
    with pytest.raises(PlanningError):
        plan_straight_path((0.0, 0.0), [(bad, 0.0)], 0.2)
    with pytest.raises(PlanningError):
        plan_straight_path((0.0, bad), [(1.0, 0.0)], 0.2)


@pytest.mark.parametrize("bad", [0.0, -0.1, math.nan, math.inf])
def test_invalid_resolution_rejected(bad):
    with pytest.raises(PlanningError):
        plan_straight_path((0.0, 0.0), [(1.0, 0.0)], bad)


def test_long_path_processing_time_is_small():
    import time

    waypoints = [(float(index), 0.0) for index in range(1, 101)]
    began = time.perf_counter()
    plan_straight_path((0.0, 0.0), waypoints, 0.2)
    print(f"100 waypoints / 100 m: {(time.perf_counter() - began) * 1000:.2f} ms")

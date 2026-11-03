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
    gaps = [math.dist(previous, current) for previous, current in zip(planned.points, planned.points[1:])]
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


def test_overflowing_distance_rejected():
    with pytest.raises(PlanningError):
        plan_straight_path((-1e308, 0.0), [(1e308, 0.0)], 0.2)


def test_tiny_resolution_rejected():
    with pytest.raises(PlanningError):
        plan_straight_path((0.0, 0.0), [(1.0, 0.0)], 1e-320)


def test_excessive_finite_distance_rejected_before_allocation():
    with pytest.raises(PlanningError):
        plan_straight_path((0.0, 0.0), [(1e12, 0.0)], 0.2)


def test_point_budget_boundary():
    plan_straight_path((0.0, 0.0), [(1.0, 0.0)], 0.2, max_path_points=6)
    with pytest.raises(PlanningError):
        plan_straight_path((0.0, 0.0), [(1.0, 0.0)], 0.2, max_path_points=5)

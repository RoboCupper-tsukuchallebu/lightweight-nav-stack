import math

import pytest
from lightnav_sim.sim_math import (
    Circle,
    Pose2D,
    Wall,
    apply_command_timeout,
    cast_ray,
    cloud_points,
    integrate_unicycle,
    offset_point,
    scan_ranges,
    world_to_sensor,
)

HEIGHT_M = 0.3
FAR_M = 100.0


def test_straight_motion_advances_along_heading():
    pose = integrate_unicycle(Pose2D(0.0, 0.0, math.pi / 2), 1.0, 0.0, 0.5)
    assert pose.x == pytest.approx(0.0, abs=1e-9)
    assert pose.y == pytest.approx(0.5)


def test_rotation_in_place_wraps_yaw():
    pose = integrate_unicycle(Pose2D(0.0, 0.0, math.pi - 0.1), 0.0, 1.0, 0.2)
    assert pose.yaw == pytest.approx(-math.pi + 0.1)
    assert (pose.x, pose.y) == (0.0, 0.0)


def test_circular_arc_returns_to_start_after_full_turn():
    pose = Pose2D(0.0, 0.0, 0.0)
    for _ in range(1000):
        pose = integrate_unicycle(pose, 1.0, 2.0 * math.pi, 0.001)
    assert pose.x == pytest.approx(0.0, abs=1e-2)
    assert pose.y == pytest.approx(0.0, abs=1e-2)


def test_offset_point_rotates_with_yaw():
    x, y = offset_point(Pose2D(1.0, 2.0, math.pi / 2), 0.1, 0.0)
    assert (x, y) == pytest.approx((1.0, 2.1))


def test_world_to_sensor_is_inverse_of_offset_point():
    pose = Pose2D(1.0, 2.0, 0.7)
    world = offset_point(pose, 0.1, 0.0)
    assert world_to_sensor(pose, 0.1, 0.0, *world) == pytest.approx((0.0, 0.0), abs=1e-9)


def test_circle_hit_distance():
    assert cast_ray(0.0, 0.0, [Circle(5.0, 0.0, 0.5)], [], HEIGHT_M, 0.0, FAR_M) == pytest.approx(4.5)


def test_circle_miss_returns_none():
    assert cast_ray(0.0, 0.0, [Circle(5.0, 2.0, 0.5)], [], HEIGHT_M, 0.0, FAR_M) is None


def test_wall_hit_distance_and_parallel_miss():
    wall = Wall(3.0, -1.5, 3.0, 1.5)
    assert cast_ray(0.0, 0.0, [], [wall], HEIGHT_M, 0.0, FAR_M) == pytest.approx(3.0)
    assert cast_ray(0.0, 0.0, [], [Wall(0.0, 1.0, 5.0, 1.0)], HEIGHT_M, 0.0, FAR_M) is None


def test_flat_ground_distance():
    elevation = -math.atan2(HEIGHT_M, 3.0)
    distance = cast_ray(0.0, elevation, [], [], HEIGHT_M, 0.0, FAR_M)
    assert distance * math.cos(elevation) == pytest.approx(3.0)


def test_ray_beyond_max_range_is_dropped():
    assert cast_ray(0.0, 0.0, [Circle(5.0, 0.0, 0.5)], [], HEIGHT_M, 0.0, 4.0) is None


def test_slope_ground_is_higher_ahead_and_lower_behind():
    slope = math.radians(10.0)
    elevation = math.radians(-2.0)
    ahead = cast_ray(0.0, elevation, [], [], HEIGHT_M, slope, FAR_M)
    flat_ahead = cast_ray(0.0, elevation, [], [], HEIGHT_M, 0.0, FAR_M)
    assert ahead < flat_ahead


def test_slope_ground_points_stay_within_small_height_difference_per_cell():
    slope = math.radians(10.0)
    points = cloud_points(360, [], [], HEIGHT_M, slope, 30.0)
    assert points
    for x, _, z in points:
        assert z == pytest.approx(-HEIGHT_M + math.tan(slope) * x, abs=1e-6)


def test_obstacle_behind_ground_is_hidden():
    below_ground = math.radians(-30.0)
    distance = cast_ray(0.0, below_ground, [Circle(10.0, 0.0, 0.5)], [], HEIGHT_M, 0.0, FAR_M)
    assert distance == pytest.approx(HEIGHT_M / math.sin(-below_ground))


def test_cloud_contains_points_on_obstacle_face():
    points = cloud_points(360, [Circle(4.0, 0.0, 0.5)], [], HEIGHT_M, 0.0, 30.0)
    assert any(abs(x - 3.5) < 0.05 and abs(y) < 0.05 and z > -HEIGHT_M + 0.05 for x, y, z in points)


def test_scan_sees_circle_ahead_and_inf_elsewhere():
    ranges = scan_ranges(4, [Circle(3.0, 0.0, 1.0)], [], 10.0)
    assert ranges[2] == pytest.approx(2.0)
    assert ranges[0] == math.inf


def test_command_timeout_zeroes_stale_command():
    assert apply_command_timeout(0.5, 0.2, 0.6, 0.5) == (0.0, 0.0)


def test_command_timeout_keeps_fresh_command():
    assert apply_command_timeout(0.5, 0.2, 0.4, 0.5) == (0.5, 0.2)

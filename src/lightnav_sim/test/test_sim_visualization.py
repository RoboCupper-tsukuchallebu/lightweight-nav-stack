import math

import pytest
from builtin_interfaces.msg import Time
from lightnav_sim.sim_math import Circle, Pose2D, Wall
from lightnav_sim.sim_visualization import (
    build_markers,
    robot_transform,
    static_transforms,
    yaw_to_quaternion,
)
from visualization_msgs.msg import Marker


def test_yaw_to_quaternion_quarter_turn():
    x, y, z, w = yaw_to_quaternion(math.pi / 2.0)
    assert (x, y) == (0.0, 0.0)
    assert z == pytest.approx(math.sqrt(0.5))
    assert w == pytest.approx(math.sqrt(0.5))


def test_robot_transform_places_base_link_on_the_ground():
    transform = robot_transform(Time(), Pose2D(1.0, 2.0, 0.0))
    assert (transform.header.frame_id, transform.child_frame_id) == ("world", "base_link")
    assert transform.transform.translation.x == 1.0
    assert transform.transform.translation.y == 2.0
    assert transform.transform.translation.z == 0.0
    assert transform.transform.rotation.w == 1.0


def test_static_transforms_use_sensor_offset_and_identity_map():
    transforms = {t.child_frame_id: t for t in static_transforms(Time(), [0.1, 0.0, 0.7])}
    assert set(transforms) == {"world", "lidar_3d", "laser"}
    assert (transforms["world"].header.frame_id, transforms["world"].transform.translation.x) == ("map", 0.0)
    for child in ("lidar_3d", "laser"):
        assert transforms[child].header.frame_id == "base_link"
        assert transforms[child].transform.translation.x == pytest.approx(0.1)
        assert transforms[child].transform.translation.z == pytest.approx(0.7)


def test_markers_describe_circle_wall_and_robot():
    array = build_markers(Time(), Pose2D(0.0, 0.0, 0.0), [Circle(2.0, 1.0, 0.5)], [Wall(0.0, 3.0, 4.0, 3.0)])
    assert array.markers[0].action == Marker.DELETEALL
    by_namespace = {m.ns: m for m in array.markers[1:] if m.type != Marker.ARROW}
    circle = by_namespace["obstacles"]
    assert circle.type == Marker.CYLINDER
    assert circle.scale.x == pytest.approx(1.0)
    wall = by_namespace["walls"]
    assert wall.pose.position.x == pytest.approx(2.0)
    assert wall.scale.x == pytest.approx(4.0)
    assert by_namespace["robot"].type == Marker.CUBE


def test_markers_without_obstacles_only_hold_robot():
    array = build_markers(Time(), Pose2D(0.0, 0.0, 0.0), [], [])
    assert {m.ns for m in array.markers[1:]} == {"robot"}

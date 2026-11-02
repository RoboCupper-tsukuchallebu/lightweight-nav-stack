import math

import numpy
import pytest

from lightnav_pose_adapter import transform

IDENTITY = numpy.eye(4)


def project(map_from_odom=IDENTITY, odom_from_body=IDENTITY, body_to_base=IDENTITY):
    return transform.project_pose_to_2d(map_from_odom, odom_from_body, body_to_base)


def test_identity_returns_origin():
    assert project() == transform.Pose2D(0.0, 0.0, 0.0)


def test_translation_only():
    odom_from_body = transform.homogeneous(numpy.eye(3), (1.0, 2.0, 3.0))
    pose = project(odom_from_body=odom_from_body)
    assert (pose.x, pose.y, pose.yaw) == pytest.approx((1.0, 2.0, 0.0))


def test_rotation_and_translation_compose_in_order():
    map_from_odom = transform.map_from_odom_matrix([1.0, 0.0, math.pi / 2])
    odom_from_body = transform.homogeneous(numpy.eye(3), (2.0, 0.0, 0.0))
    body_to_base = transform.body_to_base_matrix([1.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    pose = project(map_from_odom, odom_from_body, body_to_base)
    # odom x=2, base offset +1 along x -> odom (3, 0); rotated 90 deg -> map (1, 3)
    assert (pose.x, pose.y, pose.yaw) == pytest.approx((1.0, 3.0, math.pi / 2))


def test_yaw_extracted_from_quaternion_with_roll_and_pitch():
    rotation = transform.rotation_from_rpy(0.2, -0.1, 1.0)
    pose = project(odom_from_body=transform.homogeneous(rotation, (0, 0, 0)))
    assert pose.yaw == pytest.approx(1.0)


def test_quaternion_matches_rpy():
    half = 0.7 / 2
    rotation = transform.rotation_from_quaternion(0.0, 0.0, math.sin(half), math.cos(half))
    assert rotation == pytest.approx(transform.rotation_from_rpy(0.0, 0.0, 0.7))


def test_quaternion_zero_norm_rejected():
    with pytest.raises(ValueError):
        transform.rotation_from_quaternion(0.0, 0.0, 0.0, 0.0)


def test_sensor_offset_is_absorbed_by_inverse_body_to_base():
    # reference point is 0.1 m ahead of base_link; robot base at (5, 0) facing +x
    odom_from_body = transform.homogeneous(numpy.eye(3), (5.1, 0.0, 0.3))
    body_to_base = transform.body_to_base_matrix([-0.1, 0.0, -0.3, 0.0, 0.0, 0.0])
    pose = project(odom_from_body=odom_from_body, body_to_base=body_to_base)
    assert pose.x == pytest.approx(5.0)


def test_variance_limit_checks_x_and_y_diagonals_and_nan():
    covariance = [0.0] * 36
    assert not transform.exceeds_position_variance(covariance, 4.0)
    covariance[0] = 5.0
    assert transform.exceeds_position_variance(covariance, 4.0)
    covariance[0], covariance[7] = 0.0, 5.0
    assert transform.exceeds_position_variance(covariance, 4.0)
    covariance[7] = float("nan")
    assert transform.exceeds_position_variance(covariance, 4.0)


@pytest.mark.parametrize(
    "frame_id, expected, values, expected_reason_part",
    [
        ("odom", "map", [0.0], "frame_id"),
        ("map", "map", [float("inf")], "non-finite"),
        ("map", "map", [0.0], None),
        ("anything", "", [0.0], None),
    ],
)
def test_reject_reason(frame_id, expected, values, expected_reason_part):
    reason = transform.reject_reason(frame_id, expected, values, [0.0] * 36, 4.0)
    if expected_reason_part is None:
        assert reason is None
    else:
        assert expected_reason_part in reason

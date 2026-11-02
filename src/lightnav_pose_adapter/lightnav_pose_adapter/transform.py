"""Pure pose math for lightnav_pose_adapter (no ROS dependency)."""
import math
from collections.abc import Sequence
from dataclasses import dataclass

import numpy


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


def rotation_from_rpy(roll: float, pitch: float, yaw: float) -> numpy.ndarray:
    """Rotation matrix R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    cos_r, sin_r = math.cos(roll), math.sin(roll)
    cos_p, sin_p = math.cos(pitch), math.sin(pitch)
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    rotation_z = numpy.array([[cos_y, -sin_y, 0], [sin_y, cos_y, 0], [0, 0, 1]])
    rotation_y = numpy.array([[cos_p, 0, sin_p], [0, 1, 0], [-sin_p, 0, cos_p]])
    rotation_x = numpy.array([[1, 0, 0], [0, cos_r, -sin_r], [0, sin_r, cos_r]])
    return rotation_z @ rotation_y @ rotation_x


def rotation_from_quaternion(x: float, y: float, z: float, w: float) -> numpy.ndarray:
    """Rotation matrix from a quaternion; raises ValueError if it is not normalizable."""
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError("quaternion has zero or non-finite norm")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return numpy.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def homogeneous(rotation: numpy.ndarray, translation: Sequence[float]) -> numpy.ndarray:
    matrix = numpy.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = translation
    return matrix


def map_from_odom_matrix(map_from_odom_xy_yaw: Sequence[float]) -> numpy.ndarray:
    x, y, yaw = map_from_odom_xy_yaw
    return homogeneous(rotation_from_rpy(0.0, 0.0, yaw), (x, y, 0.0))


def body_to_base_matrix(body_to_base_xyz_rpy: Sequence[float]) -> numpy.ndarray:
    x, y, z, roll, pitch, yaw = body_to_base_xyz_rpy
    return homogeneous(rotation_from_rpy(roll, pitch, yaw), (x, y, z))


def project_pose_to_2d(
    map_from_odom: numpy.ndarray,
    odom_from_body: numpy.ndarray,
    body_to_base: numpy.ndarray,
) -> Pose2D:
    """Multiply T_map_odom * T_odom_body * T_body_base and take (x, y, yaw)."""
    map_from_base = map_from_odom @ odom_from_body @ body_to_base
    yaw = math.atan2(map_from_base[1, 0], map_from_base[0, 0])
    return Pose2D(x=float(map_from_base[0, 3]), y=float(map_from_base[1, 3]), yaw=yaw)


def is_finite_sequence(values: Sequence[float]) -> bool:
    return all(math.isfinite(value) for value in values)


def exceeds_position_variance(
    covariance: Sequence[float], max_position_variance_m2: float
) -> bool:
    """True when the x or y diagonal of the 6x6 pose covariance is above the limit.

    A NaN covariance also counts as exceeding (rejected on the safe side).
    """
    for diagonal_index in (0, 7):
        variance = covariance[diagonal_index]
        if not math.isfinite(variance) or variance > max_position_variance_m2:
            return True
    return False


def reject_reason(
    frame_id: str,
    expected_frame_id: str,
    pose_values: Sequence[float],
    covariance: Sequence[float],
    max_position_variance_m2: float,
) -> str | None:
    """Return why an input should be dropped, or None when it is usable."""
    if expected_frame_id and frame_id != expected_frame_id:
        return f"frame_id '{frame_id}' != expected '{expected_frame_id}'"
    if not is_finite_sequence(pose_values):
        return "non-finite pose value"
    if exceeds_position_variance(covariance, max_position_variance_m2):
        return "position variance above limit"
    return None

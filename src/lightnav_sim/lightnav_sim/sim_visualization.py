"""Builders for the TF tree and RViz markers of the simulator (no node state)."""
import math

from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker, MarkerArray

MAP_FRAME_ID = "map"
WORLD_FRAME_ID = "world"
BASE_FRAME_ID = "base_link"
OBSTACLE_HEIGHT_M = 1.0
WALL_THICKNESS_M = 0.1
# Visual-only robot box; the real chassis size is not specified yet.
ROBOT_LENGTH_M = 0.6
ROBOT_WIDTH_M = 0.4
ROBOT_HEIGHT_M = 0.5
HEADING_ARROW_LENGTH_M = 0.5
HEADING_ARROW_THICKNESS_M = 0.05
OBSTACLE_COLOR = (0.9, 0.4, 0.1, 0.8)
WALL_COLOR = (0.5, 0.5, 0.5, 0.9)
ROBOT_COLOR = (0.1, 0.5, 0.9, 0.8)
HEADING_COLOR = (1.0, 1.0, 0.0, 1.0)


def yaw_to_quaternion(yaw):
    """Quaternion (x, y, z, w) for a rotation about +z."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def make_transform(stamp, parent_frame_id, child_frame_id, translation_xyz, yaw=0.0):
    transform = TransformStamped()
    transform.header.stamp = stamp
    transform.header.frame_id = parent_frame_id
    transform.child_frame_id = child_frame_id
    transform.transform.translation.x = float(translation_xyz[0])
    transform.transform.translation.y = float(translation_xyz[1])
    transform.transform.translation.z = float(translation_xyz[2])
    quaternion = yaw_to_quaternion(yaw)
    transform.transform.rotation.x, transform.transform.rotation.y = quaternion[0], quaternion[1]
    transform.transform.rotation.z, transform.transform.rotation.w = quaternion[2], quaternion[3]
    return transform


def static_transforms(stamp, sensor_offset_xyz):
    """map->world identity, and base_link->lidar_3d / laser at the sensor offset."""
    return [
        make_transform(stamp, MAP_FRAME_ID, WORLD_FRAME_ID, (0.0, 0.0, 0.0)),
        make_transform(stamp, BASE_FRAME_ID, "lidar_3d", sensor_offset_xyz),
        make_transform(stamp, BASE_FRAME_ID, "laser", sensor_offset_xyz),
    ]


def robot_transform(stamp, pose):
    """world->base_link at the robot origin on the ground."""
    return make_transform(stamp, WORLD_FRAME_ID, BASE_FRAME_ID, (pose.x, pose.y, 0.0), pose.yaw)


def _marker(stamp, marker_id, namespace, marker_type, color):
    marker = Marker()
    marker.header.stamp = stamp
    marker.header.frame_id = WORLD_FRAME_ID
    marker.ns = namespace
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
    return marker


def _set_pose(marker, x, y, z, yaw):
    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = x, y, z
    quaternion = yaw_to_quaternion(yaw)
    marker.pose.orientation.z, marker.pose.orientation.w = quaternion[2], quaternion[3]


def build_markers(stamp, pose, circles, walls):
    """DELETEALL first so that cleared obstacles disappear, then the current scene."""
    delete_all = Marker()
    delete_all.action = Marker.DELETEALL
    markers = [delete_all]
    for index, circle in enumerate(circles):
        marker = _marker(stamp, index, "obstacles", Marker.CYLINDER, OBSTACLE_COLOR)
        _set_pose(marker, circle.x, circle.y, OBSTACLE_HEIGHT_M / 2.0, 0.0)
        marker.scale.x = marker.scale.y = 2.0 * circle.radius
        marker.scale.z = OBSTACLE_HEIGHT_M
        markers.append(marker)
    for index, wall in enumerate(walls):
        marker = _marker(stamp, index, "walls", Marker.CUBE, WALL_COLOR)
        _set_pose(
            marker,
            (wall.x1 + wall.x2) / 2.0,
            (wall.y1 + wall.y2) / 2.0,
            OBSTACLE_HEIGHT_M / 2.0,
            math.atan2(wall.y2 - wall.y1, wall.x2 - wall.x1),
        )
        marker.scale.x = math.hypot(wall.x2 - wall.x1, wall.y2 - wall.y1)
        marker.scale.y = WALL_THICKNESS_M
        marker.scale.z = OBSTACLE_HEIGHT_M
        markers.append(marker)
    robot = _marker(stamp, 0, "robot", Marker.CUBE, ROBOT_COLOR)
    _set_pose(robot, pose.x, pose.y, ROBOT_HEIGHT_M / 2.0, pose.yaw)
    robot.scale.x, robot.scale.y, robot.scale.z = ROBOT_LENGTH_M, ROBOT_WIDTH_M, ROBOT_HEIGHT_M
    heading = _marker(stamp, 1, "robot", Marker.ARROW, HEADING_COLOR)
    _set_pose(heading, pose.x, pose.y, ROBOT_HEIGHT_M, pose.yaw)
    heading.scale.x = HEADING_ARROW_LENGTH_M
    heading.scale.y = heading.scale.z = HEADING_ARROW_THICKNESS_M
    markers.extend([robot, heading])
    return MarkerArray(markers=markers)

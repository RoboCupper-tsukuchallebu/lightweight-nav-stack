"""Kinematic simulator node: integrates /cmd_vel_nav and publishes odometry and synthetic LiDAR."""
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from std_srvs.srv import SetBool, Trigger
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from visualization_msgs.msg import MarkerArray

from lightnav_msgs.msg import PathWithId
from lightnav_sim.sim_math import (
    Circle,
    Pose2D,
    Wall,
    apply_command_timeout,
    cloud_points,
    integrate_unicycle,
    offset_point,
    scan_ranges,
    world_to_sensor,
)
from lightnav_sim.sim_visualization import (
    build_markers,
    map_world_transform,
    robot_transform,
    sensor_transforms,
)

CIRCLE_STRIDE = 3
WALL_STRIDE = 4
POSITION_VARIANCE_M2 = 0.01
CLOUD_AZIMUTH_COUNT = 360
CLOUD_RANGE_MAX_M = 30.0
SERVICE_PREFIX = "/lightnav/sim/"
MARKER_TOPIC = "/lightnav/sim/markers"
PATH_TOPIC = "/lightnav/path"
PATH_VIEW_TOPIC = "/lightnav/sim/path_view"
# Same QoS as the mission publisher so a late-started RViz still gets the last path.
PATH_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
# Reliable also connects to best-effort subscribers; obstacle_monitor defaults to reliable points.
POINTS_QOS = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)


def _chunks(flat_values, stride, name):
    if len(flat_values) % stride != 0:
        raise ValueError(f"{name} length must be a multiple of {stride}")
    return [tuple(flat_values[i : i + stride]) for i in range(0, len(flat_values), stride)]


class KinematicSimNode(Node):
    def __init__(self, **node_kwargs):
        super().__init__("kinematic_sim_node", **node_kwargs)
        self.declare_parameter("initial_x", 0.0)
        self.declare_parameter("initial_y", 0.0)
        self.declare_parameter("initial_yaw", 0.0)
        self.declare_parameter("integration_hz", 50.0)
        self.declare_parameter("odom_hz", 50.0)
        self.declare_parameter("sensor_hz", 10.0)
        self.declare_parameter("cmd_timeout_s", 0.5)
        self.declare_parameter("odom_topic", "/sim/odom")
        self.declare_parameter("odom_frame_id", "world")
        self.declare_parameter("sensor_offset_xyz", [0.1, 0.0, 0.7])
        # Flat arrays because ROS parameters cannot nest: [x, y, radius, ...] and [x1, y1, x2, y2, ...].
        self.declare_parameter("obstacles", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("walls", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("ground_slope_deg", 0.0)
        self.declare_parameter("scan_range_max_m", 10.0)
        self.declare_parameter("scan_beam_count", 360)
        self.declare_parameter("points_topic", "/velodyne_points")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("publish_tf", True)

        self._pose = Pose2D(*(self._number(n) for n in ("initial_x", "initial_y", "initial_yaw")))
        self._sensor_offset = tuple(self._number_array("sensor_offset_xyz"))
        if len(self._sensor_offset) != 3:
            raise ValueError("sensor_offset_xyz must have 3 elements")
        self._circles = [Circle(*c) for c in _chunks(self._number_array("obstacles"), CIRCLE_STRIDE, "obstacles")]
        self._walls = [Wall(*w) for w in _chunks(self._number_array("walls"), WALL_STRIDE, "walls")]
        self._slope_rad = math.radians(self._number("ground_slope_deg"))
        self._linear_mps = 0.0
        self._angular_radps = 0.0
        self._last_cmd_ns = self.get_clock().now().nanoseconds
        self._is_pose_publishing = True
        self._is_motion_enabled = True
        self._last_integration_ns = self.get_clock().now().nanoseconds

        self._odom_publisher = self.create_publisher(Odometry, self.get_parameter("odom_topic").value, 10)
        self._cloud_publisher = self.create_publisher(
            PointCloud2, self.get_parameter("points_topic").value, POINTS_QOS
        )
        scan_topic = self.get_parameter("scan_topic").value
        self._scan_publisher = (
            self.create_publisher(LaserScan, scan_topic, qos_profile_sensor_data) if scan_topic else None
        )
        self._marker_publisher = self.create_publisher(MarkerArray, MARKER_TOPIC, 10)
        self._is_tf_publishing = bool(self.get_parameter("publish_tf").value)
        stamp = self.get_clock().now().to_msg()
        # Sensor frames are always published; publish_tf only gates world->base_link and map->world.
        static_transforms = sensor_transforms(stamp, self._sensor_offset)
        if self._is_tf_publishing:
            self._tf_broadcaster = TransformBroadcaster(self)
            static_transforms.append(map_world_transform(stamp))
        self._static_tf_broadcaster = StaticTransformBroadcaster(self)
        self._static_tf_broadcaster.sendTransform(static_transforms)
        self._path_view_publisher = self.create_publisher(Path, PATH_VIEW_TOPIC, PATH_QOS)
        self.create_subscription(PathWithId, PATH_TOPIC, self._on_path, PATH_QOS)
        self.create_subscription(Twist, "/cmd_vel_nav", self._on_cmd_vel, 10)
        self.create_service(SetBool, SERVICE_PREFIX + "set_pose_publishing", self._on_set_pose_publishing)
        self.create_service(SetBool, SERVICE_PREFIX + "set_motion_enabled", self._on_set_motion_enabled)
        self.create_service(Trigger, SERVICE_PREFIX + "clear_obstacles", self._on_clear_obstacles)
        self.create_timer(1.0 / self._positive("integration_hz"), self._integrate)
        self.create_timer(1.0 / self._positive("odom_hz"), self._publish_odom)
        self.create_timer(1.0 / self._positive("sensor_hz"), self._publish_sensors)

    def _number(self, name):
        return float(self.get_parameter(name).value)

    def _number_array(self, name):
        return [float(value) for value in self.get_parameter_or(name, Parameter(name, Parameter.Type.DOUBLE_ARRAY, [])).value]

    def _positive(self, name):
        value = self._number(name)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be positive and finite")
        return value

    def _on_cmd_vel(self, message):
        if not self._is_motion_enabled:
            return
        self._last_cmd_ns = self.get_clock().now().nanoseconds
        self._linear_mps = message.linear.x
        self._angular_radps = message.angular.z

    def _on_path(self, message):
        self._path_view_publisher.publish(message.path)

    def _on_set_pose_publishing(self, request, response):
        self._is_pose_publishing = request.data
        response.success = True
        return response

    def _on_set_motion_enabled(self, request, response):
        self._is_motion_enabled = request.data
        if not request.data:
            self._linear_mps = 0.0
            self._angular_radps = 0.0
        response.success = True
        return response

    def _on_clear_obstacles(self, request, response):
        self._circles = []
        self._walls = []
        response.success = True
        return response

    def _integrate(self):
        now_ns = self.get_clock().now().nanoseconds
        delta_s = (now_ns - self._last_integration_ns) * 1e-9
        self._last_integration_ns = now_ns
        self._linear_mps, self._angular_radps = apply_command_timeout(
            self._linear_mps,
            self._angular_radps,
            (now_ns - self._last_cmd_ns) * 1e-9,
            self._positive("cmd_timeout_s"),
        )
        if delta_s > 0.0:
            self._pose = integrate_unicycle(self._pose, self._linear_mps, self._angular_radps, delta_s)
        if self._is_tf_publishing:
            self._tf_broadcaster.sendTransform(robot_transform(self.get_clock().now().to_msg(), self._pose))

    def _header(self, frame_id):
        return Header(stamp=self.get_clock().now().to_msg(), frame_id=frame_id)

    def _publish_odom(self):
        if not self._is_pose_publishing:
            return
        offset_x, offset_y, offset_z = self._sensor_offset
        reference_x, reference_y = offset_point(self._pose, offset_x, offset_y)
        message = Odometry(header=self._header(self.get_parameter("odom_frame_id").value))
        message.child_frame_id = "base_link"
        message.pose.pose.position.x = reference_x
        message.pose.pose.position.y = reference_y
        message.pose.pose.position.z = offset_z
        message.pose.pose.orientation.z = math.sin(self._pose.yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(self._pose.yaw / 2.0)
        message.pose.covariance[0] = message.pose.covariance[7] = POSITION_VARIANCE_M2
        message.pose.covariance[35] = POSITION_VARIANCE_M2
        message.twist.twist.linear.x = self._linear_mps
        message.twist.twist.angular.z = self._angular_radps
        self._odom_publisher.publish(message)

    def _obstacles_in_sensor_frame(self):
        offset_x, offset_y, _ = self._sensor_offset
        to_sensor = lambda x, y: world_to_sensor(self._pose, offset_x, offset_y, x, y)
        circles = [Circle(*to_sensor(c.x, c.y), c.radius) for c in self._circles]
        walls = [Wall(*to_sensor(w.x1, w.y1), *to_sensor(w.x2, w.y2)) for w in self._walls]
        return circles, walls

    def _publish_sensors(self):
        self._marker_publisher.publish(
            build_markers(self.get_clock().now().to_msg(), self._pose, self._circles, self._walls)
        )
        circles, walls = self._obstacles_in_sensor_frame()
        height_m = self._sensor_offset[2]
        points = cloud_points(CLOUD_AZIMUTH_COUNT, circles, walls, height_m, self._slope_rad, CLOUD_RANGE_MAX_M)
        self._cloud_publisher.publish(point_cloud2.create_cloud_xyz32(self._header("lidar_3d"), points))
        if self._scan_publisher is None:
            return
        beam_count = int(self.get_parameter("scan_beam_count").value)
        range_max = self._number("scan_range_max_m")
        scan = LaserScan(header=self._header("laser"))
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = 2.0 * math.pi / beam_count
        scan.range_min = 0.05
        scan.range_max = range_max
        scan.ranges = scan_ranges(beam_count, circles, walls, range_max)
        self._scan_publisher.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = KinematicSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

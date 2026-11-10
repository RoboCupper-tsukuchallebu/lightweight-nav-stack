"""Lifecycle node: nav_msgs/Odometry -> /lightnav/pose (2D, map frame)."""
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.lifecycle import Node, State, TransitionCallbackReturn
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from lightnav_pose_adapter import transform

GLOBAL_FRAME_ID = "map"
WARN_THROTTLE_SECONDS = 1.0
QUEUE_DEPTH = 10


class PoseAdapterNode(Node):
    def __init__(self, node_name: str = "pose_adapter_node", **kwargs) -> None:
        super().__init__(node_name, **kwargs)
        self.declare_parameter("odom_topic", "/lightnav/localization/odom", descriptor=ParameterDescriptor(read_only=True))
        self.declare_parameter("output_topic", "/lightnav/pose", descriptor=ParameterDescriptor(read_only=True))
        self.declare_parameter("expected_frame_id", GLOBAL_FRAME_ID, descriptor=ParameterDescriptor(read_only=True))
        self.declare_parameter("body_to_base_xyz_rpy", [0.0] * 6, descriptor=ParameterDescriptor(read_only=True))
        self.declare_parameter("map_from_odom_xy_yaw", [0.0] * 3, descriptor=ParameterDescriptor(read_only=True))
        self.declare_parameter("max_position_variance_m2", 4.0, descriptor=ParameterDescriptor(read_only=True))
        self.add_on_set_parameters_callback(self._reject_use_sim_time)
        self._subscription = None
        self._publisher = None

    def _reject_use_sim_time(self, parameters):
        for parameter in parameters:
            if parameter.name == "use_sim_time":
                reason = "use_sim_time is read-only after construction"
                self.get_logger().warn(reason)
                return SetParametersResult(successful=False, reason=reason)
        return SetParametersResult(successful=True)

    def on_configure(self, previous_state: State) -> TransitionCallbackReturn:
        body_to_base_xyz_rpy = list(self.get_parameter("body_to_base_xyz_rpy").value)
        map_from_odom_xy_yaw = list(self.get_parameter("map_from_odom_xy_yaw").value)
        self._max_position_variance_m2 = float(
            self.get_parameter("max_position_variance_m2").value
        )
        self._expected_frame_id = self.get_parameter("expected_frame_id").value

        error = self._validate_parameters(
            body_to_base_xyz_rpy, map_from_odom_xy_yaw, self._max_position_variance_m2
        )
        if error is not None:
            self.get_logger().error(f"invalid parameter: {error}")
            return TransitionCallbackReturn.FAILURE

        self._body_to_base = transform.body_to_base_matrix(body_to_base_xyz_rpy)
        self._map_from_odom = transform.map_from_odom_matrix(map_from_odom_xy_yaw)

        qos_profile = QoSProfile(
            depth=QUEUE_DEPTH,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._publisher = self.create_lifecycle_publisher(
            PoseStamped, self.get_parameter("output_topic").value, qos_profile
        )
        self._subscription = self.create_subscription(
            Odometry,
            self.get_parameter("odom_topic").value,
            self._on_odometry,
            qos_profile,
        )
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, previous_state: State) -> TransitionCallbackReturn:
        self.destroy_subscription(self._subscription)
        self.destroy_lifecycle_publisher(self._publisher)
        self._subscription = None
        self._publisher = None
        return TransitionCallbackReturn.SUCCESS

    @staticmethod
    def _validate_parameters(body_to_base, map_from_odom, max_variance):
        if len(body_to_base) != 6 or not transform.is_finite_sequence(body_to_base):
            return "body_to_base_xyz_rpy must be 6 finite values"
        if len(map_from_odom) != 3 or not transform.is_finite_sequence(map_from_odom):
            return "map_from_odom_xy_yaw must be 3 finite values"
        if not math.isfinite(max_variance) or max_variance <= 0.0:
            return "max_position_variance_m2 must be positive and finite"
        return None

    def _on_odometry(self, message: Odometry) -> None:
        if self._publisher is None or not self._publisher.is_activated:
            return
        pose = message.pose.pose
        orientation = pose.orientation
        pose_values = (
            pose.position.x, pose.position.y, pose.position.z,
            orientation.x, orientation.y, orientation.z, orientation.w,
        )
        reason = transform.reject_reason(
            message.header.frame_id,
            self._expected_frame_id,
            pose_values,
            message.pose.covariance,
            self._max_position_variance_m2,
        )
        if reason is None:
            try:
                odom_from_body = transform.homogeneous(
                    transform.rotation_from_quaternion(*pose_values[3:]),
                    pose_values[:3],
                )
            except ValueError as error:
                reason = str(error)
        if reason is not None:
            self.get_logger().warn(
                f"dropping odometry: {reason}", throttle_duration_sec=WARN_THROTTLE_SECONDS
            )
            return

        pose_2d = transform.project_pose_to_2d(
            self._map_from_odom, odom_from_body, self._body_to_base
        )
        self._publisher.publish(self._build_pose_message(message, pose_2d))

    @staticmethod
    def _build_pose_message(source: Odometry, pose_2d: transform.Pose2D) -> PoseStamped:
        output = PoseStamped()
        output.header.stamp = source.header.stamp
        output.header.frame_id = GLOBAL_FRAME_ID
        output.pose.position.x = pose_2d.x
        output.pose.position.y = pose_2d.y
        output.pose.orientation.z = math.sin(pose_2d.yaw / 2.0)
        output.pose.orientation.w = math.cos(pose_2d.yaw / 2.0)
        return output


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PoseAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

"""Check that lightnav_pose_adapter's default configuration accepts the localization output."""

import math
import time

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from lightnav_pose_adapter.pose_adapter_node import PoseAdapterNode
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.qos import QoSProfile

SPIN_TIMEOUT_SECONDS = 5.0


@pytest.fixture(scope="module")
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def build_odometry():
    message = Odometry()
    message.header.frame_id = "map"
    message.header.stamp.sec = 7
    message.child_frame_id = "base_footprint"
    message.pose.pose.position.x = 1.25
    message.pose.pose.position.y = -0.5
    message.pose.pose.orientation.z = math.sin(0.3)
    message.pose.pose.orientation.w = math.cos(0.3)
    # Diagonal positive covariance as published by localization_node. The adapter rejects
    # position variances above max_position_variance_m2 (default 4.0).
    message.pose.covariance[0] = 0.01
    message.pose.covariance[7] = 0.01
    message.pose.covariance[35] = 0.0025
    return message


def test_default_adapter_accepts_localization_contract(ros_context):
    adapter = PoseAdapterNode("test_localization_adapter")
    helper = rclpy.create_node("test_localization_adapter_helper")
    received = []
    publisher = helper.create_publisher(
        Odometry, "/lightnav/localization/odom", QoSProfile(depth=10)
    )
    helper.create_subscription(
        PoseStamped, "/lightnav/pose", received.append, QoSProfile(depth=10)
    )
    executor = SingleThreadedExecutor()
    executor.add_node(adapter)
    executor.add_node(helper)
    try:
        assert adapter.get_parameter("expected_frame_id").value == "map"
        assert adapter.trigger_configure() == TransitionCallbackReturn.SUCCESS
        assert adapter.trigger_activate() == TransitionCallbackReturn.SUCCESS
        message = build_odometry()
        deadline = time.monotonic() + SPIN_TIMEOUT_SECONDS
        while time.monotonic() < deadline and not received:
            publisher.publish(message)
            executor.spin_once(timeout_sec=0.05)
        assert received, "the default pose adapter did not accept the localization output"
        pose = received[0]
        assert pose.header.frame_id == "map"
        assert pose.header.stamp.sec == 7
        assert pose.pose.position.x == pytest.approx(1.25)
        assert pose.pose.position.y == pytest.approx(-0.5)
    finally:
        executor.shutdown()
        adapter.destroy_node()
        helper.destroy_node()

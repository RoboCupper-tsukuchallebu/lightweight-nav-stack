import math
import time

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from lightnav_pose_adapter.pose_adapter_node import PoseAdapterNode
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile

SPIN_TIMEOUT_SECONDS = 3.0


@pytest.fixture(scope="module")
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def build_odometry(frame_id="map", x=1.0, yaw=0.0, variance=0.01, stamp_sec=7):
    message = Odometry()
    message.header.frame_id = frame_id
    message.header.stamp.sec = stamp_sec
    message.pose.pose.position.x = x
    message.pose.pose.orientation.z = math.sin(yaw / 2)
    message.pose.pose.orientation.w = math.cos(yaw / 2)
    message.pose.covariance[0] = variance
    message.pose.covariance[7] = variance
    return message


class Harness:
    def __init__(self, parameters):
        self.adapter = PoseAdapterNode(
            "test_pose_adapter",
            parameter_overrides=[Parameter(k, value=v) for k, v in parameters.items()],
        )
        self.helper = rclpy.create_node("test_pose_adapter_helper")
        self.received = []
        self.publisher = self.helper.create_publisher(
            Odometry, "/lightnav/localization/odom", QoSProfile(depth=10)
        )
        self.helper.create_subscription(
            PoseStamped, "/lightnav/pose", self.received.append, QoSProfile(depth=10)
        )
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.adapter)
        self.executor.add_node(self.helper)

    def spin_until(self, condition):
        deadline = time.monotonic() + SPIN_TIMEOUT_SECONDS
        while time.monotonic() < deadline and not condition():
            self.executor.spin_once(timeout_sec=0.05)
        return condition()

    def spin_briefly(self):
        end = time.monotonic() + 0.5
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=0.05)

    def close(self):
        self.executor.shutdown()
        self.adapter.destroy_node()
        self.helper.destroy_node()


@pytest.fixture
def make_harness(ros_context):
    harnesses = []

    def factory(parameters=None):
        harness = Harness(parameters or {})
        harnesses.append(harness)
        return harness

    yield factory
    for harness in harnesses:
        harness.close()


def activate(harness):
    assert harness.adapter.trigger_configure() == TransitionCallbackReturn.SUCCESS
    assert harness.adapter.trigger_activate() == TransitionCallbackReturn.SUCCESS


def publish_until_received(harness, message, expected_count):
    deadline = time.monotonic() + SPIN_TIMEOUT_SECONDS
    while time.monotonic() < deadline and len(harness.received) < expected_count:
        harness.publisher.publish(message)
        harness.executor.spin_once(timeout_sec=0.05)
    return len(harness.received) >= expected_count


def test_active_node_publishes_pose_with_input_stamp(make_harness):
    harness = make_harness()
    activate(harness)
    assert publish_until_received(harness, build_odometry(x=2.5, yaw=0.5), 1)
    pose = harness.received[0]
    assert pose.header.frame_id == "map"
    assert pose.header.stamp.sec == 7
    assert pose.pose.position.x == pytest.approx(2.5)
    assert 2 * math.atan2(pose.pose.orientation.z, pose.pose.orientation.w) == pytest.approx(0.5)


def test_parameters_apply_map_from_odom_offset(make_harness):
    harness = make_harness(
        {"expected_frame_id": "odom", "map_from_odom_xy_yaw": [10.0, 0.0, 0.0]}
    )
    activate(harness)
    assert publish_until_received(harness, build_odometry(frame_id="odom", x=1.0), 1)
    assert harness.received[0].pose.position.x == pytest.approx(11.0)


def test_inactive_node_is_silent_until_activated(make_harness):
    harness = make_harness()
    harness.adapter.trigger_configure()
    # wait for the connection (positive control for the silence below)
    assert harness.spin_until(
        lambda: harness.publisher.get_subscription_count() > 0
        and harness.adapter.count_publishers("/lightnav/localization/odom") > 0
    )
    for _ in range(5):
        harness.publisher.publish(build_odometry())
        harness.spin_briefly()
    assert harness.received == []
    harness.adapter.trigger_activate()
    assert publish_until_received(harness, build_odometry(), 1)


def test_invalid_inputs_are_dropped_and_valid_one_passes(make_harness):
    harness = make_harness()
    activate(harness)
    assert harness.spin_until(
        lambda: harness.publisher.get_subscription_count() > 0
    )
    for bad in (
        build_odometry(frame_id="odom"),
        build_odometry(variance=9.0),
        build_odometry(x=float("nan")),
    ):
        harness.publisher.publish(bad)
    harness.spin_briefly()
    assert harness.received == []
    harness.publisher.publish(build_odometry(x=3.0))
    assert harness.spin_until(lambda: len(harness.received) >= 1)
    harness.spin_briefly()
    assert len(harness.received) == 1
    assert harness.received[0].pose.position.x == pytest.approx(3.0)


def test_invalid_parameter_fails_configure(make_harness):
    harness = make_harness({"max_position_variance_m2": -1.0})
    result = harness.adapter.trigger_configure()
    assert result == TransitionCallbackReturn.FAILURE
    assert harness.adapter._publisher is None


@pytest.mark.parametrize("name,value", [
    ("odom_topic", "/other_odom"), ("output_topic", "/other_pose"),
    ("expected_frame_id", "odom"), ("body_to_base_xyz_rpy", [1.0] * 6),
    ("map_from_odom_xy_yaw", [1.0] * 3), ("max_position_variance_m2", 2.0),
])
def test_configuration_parameters_are_read_only(make_harness, name, value):
    node = make_harness().adapter
    original = node.get_parameter(name).value
    result = node.set_parameters([Parameter(name, value=value)])[0]
    assert not result.successful
    assert "read-only" in result.reason.lower()
    assert node.get_parameter(name).value == original


def test_use_sim_time_is_rejected_in_every_lifecycle_state(make_harness):
    node = make_harness().adapter
    for transition in (None, node.trigger_configure, node.trigger_activate):
        if transition is not None:
            assert transition() == TransitionCallbackReturn.SUCCESS
        result = node.set_parameters([Parameter("use_sim_time", value=True)])[0]
        assert not result.successful
        assert "use_sim_time" in result.reason and "read-only" in result.reason
        assert node.get_parameter("use_sim_time").value is False

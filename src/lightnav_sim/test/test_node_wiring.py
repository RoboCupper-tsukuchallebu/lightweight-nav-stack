import time

import pytest
import rclpy
from geometry_msgs.msg import Twist
from lightnav_sim.kinematic_sim_node import KinematicSimNode
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile
from std_srvs.srv import SetBool, Trigger

SPIN_TIMEOUT_SECONDS = 3.0


@pytest.fixture(scope="module")
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


class Harness:
    def __init__(self, parameters):
        self.sim = KinematicSimNode(
            parameter_overrides=[Parameter(k, value=v) for k, v in parameters.items()]
        )
        self.helper = rclpy.create_node("test_sim_helper")
        self.odometry = []
        self.helper.create_subscription(Odometry, "/sim/odom", self.odometry.append, QoSProfile(depth=10))
        self.command_publisher = self.helper.create_publisher(Twist, "/cmd_vel", QoSProfile(depth=10))
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.sim)
        self.executor.add_node(self.helper)

    def spin_until(self, condition):
        deadline = time.monotonic() + SPIN_TIMEOUT_SECONDS
        while time.monotonic() < deadline and not condition():
            self.executor.spin_once(timeout_sec=0.02)
        return condition()

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=0.02)

    def send_command(self, linear_mps):
        message = Twist()
        message.linear.x = linear_mps
        self.command_publisher.publish(message)

    def call(self, service_type, name, **fields):
        client = self.helper.create_client(service_type, name)
        assert client.wait_for_service(timeout_sec=SPIN_TIMEOUT_SECONDS)
        future = client.call_async(service_type.Request(**fields))
        assert self.spin_until(future.done)
        return future.result()

    def latest_x(self):
        return self.odometry[-1].pose.pose.position.x

    def close(self):
        self.executor.shutdown()
        self.sim.destroy_node()
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


def drive_forward_for(harness, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        harness.send_command(1.0)
        harness.spin_for(0.05)


def test_odometry_is_offset_in_world_frame(make_harness):
    harness = make_harness({"initial_x": 2.0})
    assert harness.spin_until(lambda: harness.odometry)
    message = harness.odometry[-1]
    assert message.header.frame_id == "world"
    assert message.pose.pose.position.x == pytest.approx(2.1)
    assert message.pose.pose.position.z == pytest.approx(0.3)


def test_streamed_command_moves_the_sim(make_harness):
    harness = make_harness()
    assert harness.spin_until(lambda: harness.odometry)
    start_x = harness.latest_x()
    drive_forward_for(harness, 0.5)
    assert harness.latest_x() > start_x + 0.2


def test_single_command_stops_after_timeout(make_harness):
    harness = make_harness({"cmd_timeout_s": 0.2})
    assert harness.spin_until(lambda: harness.odometry)
    harness.send_command(1.0)
    harness.spin_for(0.6)
    settled_x = harness.latest_x()
    harness.spin_for(0.3)
    assert harness.latest_x() == pytest.approx(settled_x)
    assert settled_x < 0.1 + 0.4


def test_set_motion_enabled_false_freezes_pose(make_harness):
    harness = make_harness()
    assert harness.spin_until(lambda: harness.odometry)
    assert harness.call(SetBool, "/lightnav/sim/set_motion_enabled", data=False).success
    start_x = harness.latest_x()
    drive_forward_for(harness, 0.4)
    assert harness.latest_x() == pytest.approx(start_x)


def test_set_pose_publishing_false_stops_odometry(make_harness):
    harness = make_harness()
    assert harness.spin_until(lambda: harness.odometry)
    assert harness.call(SetBool, "/lightnav/sim/set_pose_publishing", data=False).success
    harness.spin_for(0.1)
    count = len(harness.odometry)
    harness.spin_for(0.4)
    assert len(harness.odometry) == count
    assert harness.call(SetBool, "/lightnav/sim/set_pose_publishing", data=True).success
    assert harness.spin_until(lambda: len(harness.odometry) > count)


def test_clear_obstacles_empties_world(make_harness):
    harness = make_harness({"obstacles": [3.0, 0.0, 0.5]})
    assert len(harness.sim._circles) == 1
    assert harness.call(Trigger, "/lightnav/sim/clear_obstacles").success
    assert harness.sim._circles == []


def test_odometry_topic_parameter_is_honored(make_harness):
    harness = make_harness({"odom_frame_id": "odom_test"})
    assert harness.spin_until(lambda: harness.odometry)
    assert harness.odometry[-1].header.frame_id == "odom_test"

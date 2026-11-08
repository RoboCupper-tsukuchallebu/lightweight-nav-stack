"""robot.launch.py with safety_gate:=false starts the nodes but not safety_gate_node (no /cmd_vel publisher)."""

import time
from pathlib import Path

import launch_testing.actions
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState

# <repository>/src/lightnav_bringup/test/<this file>
ROBOT_CONFIG_PATH = Path(__file__).resolve().parents[3] / "docker" / "config" / "robot.yaml"
ACTIVE_NODE_NAMES = (
    "obstacle_monitor",
    "pose_adapter_node",
    "planner_node",
    "lightnav_mission",
    "controller_node",
)
ACTIVATION_TIMEOUT_SECONDS = 60.0
SETTLE_SECONDS = 2.0
SERVICE_TIMEOUT_SECONDS = 1.0


@pytest.mark.launch_test
def generate_test_description():
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory("lightnav_bringup")) / "launch" / "robot.launch.py")
        ),
        launch_arguments={"safety_gate": "false", "robot_config": str(ROBOT_CONFIG_PATH)}.items(),
    )
    return LaunchDescription([robot_launch, launch_testing.actions.ReadyToTest()])


@pytest.fixture(scope="module")
def helper_node():
    rclpy.init()
    node = rclpy.create_node("test_robot_launch_helper")
    yield node
    node.destroy_node()
    rclpy.shutdown()


def read_lifecycle_state_id(node, node_name):
    """The lifecycle state id of `node_name`, or None while its get_state service does not answer."""
    client = node.create_client(GetState, f"/{node_name}/get_state")
    try:
        if not client.wait_for_service(timeout_sec=SERVICE_TIMEOUT_SECONDS):
            return None
        future = client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=SERVICE_TIMEOUT_SECONDS)
        return future.result().current_state.id if future.done() else None
    finally:
        node.destroy_client(client)


def wait_until_active(node, node_name):
    deadline = time.monotonic() + ACTIVATION_TIMEOUT_SECONDS
    while time.monotonic() < deadline:
        if read_lifecycle_state_id(node, node_name) == State.PRIMARY_STATE_ACTIVE:
            return True
    return False


def test_nodes_become_active_and_safety_gate_is_not_started(helper_node):
    # Positive control first: if the launch had failed as a whole, "no gate" would pass for the wrong reason.
    for node_name in ACTIVE_NODE_NAMES:
        assert wait_until_active(helper_node, node_name), f"{node_name} did not become active"

    time.sleep(SETTLE_SECONDS)
    node_names = helper_node.get_node_names()
    assert "safety_gate_node" not in node_names, node_names
    assert helper_node.count_publishers("/cmd_vel") == 0

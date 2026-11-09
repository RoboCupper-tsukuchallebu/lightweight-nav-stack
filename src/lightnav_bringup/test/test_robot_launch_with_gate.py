"""robot.launch.py with the safety gate enabled publishes only the nav output topic.

launch_testing runs the unittest.TestCase classes of this module while the launch is up. A plain `def test_...`
function is not run (the item reports "Ran 0 tests" and still passes), so the checks must be in a TestCase.
"""

import time
import unittest
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
        launch_arguments={"safety_gate": "true", "robot_config": str(ROBOT_CONFIG_PATH)}.items(),
    )
    return LaunchDescription([robot_launch, launch_testing.actions.ReadyToTest()])


class TestRobotLaunchWithGate(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.helper_node = rclpy.create_node("test_robot_launch_helper")

    @classmethod
    def tearDownClass(cls):
        cls.helper_node.destroy_node()
        rclpy.shutdown()

    def read_lifecycle_state_id(self, node_name):
        """The lifecycle state id of `node_name`, or None while its get_state service does not answer."""
        client = self.helper_node.create_client(GetState, f"/{node_name}/get_state")
        try:
            if not client.wait_for_service(timeout_sec=SERVICE_TIMEOUT_SECONDS):
                return None
            future = client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(self.helper_node, future, timeout_sec=SERVICE_TIMEOUT_SECONDS)
            return future.result().current_state.id if future.done() else None
        finally:
            self.helper_node.destroy_client(client)

    def wait_until_active(self, node_name):
        deadline = time.monotonic() + ACTIVATION_TIMEOUT_SECONDS
        while time.monotonic() < deadline:
            if self.read_lifecycle_state_id(node_name) == State.PRIMARY_STATE_ACTIVE:
                return True
        return False

    def test_gate_publishes_nav_topic_only(self):
        # Verify all lifecycle nodes activated before checking their topic wiring.
        for node_name in (*ACTIVE_NODE_NAMES, "safety_gate_node"):
            self.assertTrue(self.wait_until_active(node_name), f"{node_name} did not become active")

        time.sleep(SETTLE_SECONDS)
        node_names = self.helper_node.get_node_names()
        self.assertIn("safety_gate_node", node_names)
        self.assertEqual(self.helper_node.count_publishers("/cmd_vel_nav"), 1)
        self.assertEqual(self.helper_node.count_publishers("/cmd_vel"), 0)

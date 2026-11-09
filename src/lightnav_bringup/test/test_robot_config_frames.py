"""docker/config/robot.yaml must give the controller the frame that obstacle_monitor stamps on the local grid.

The controller drops every local_grid whose frame_id differs from its grid_frame, so a mismatch silently turns
obstacle avoidance off (Issue #1). Both nodes fall back to base_link when the key is absent.
"""

from pathlib import Path

import yaml

# <repository>/src/lightnav_bringup/test/<this file>
ROBOT_CONFIG_PATH = Path(__file__).resolve().parents[3] / "docker" / "config" / "robot.yaml"
CODE_DEFAULT_FRAME = "base_link"


def read_node_parameters(node_name):
    with ROBOT_CONFIG_PATH.open() as config_file:
        return yaml.safe_load(config_file)[node_name]["ros__parameters"]


def test_controller_grid_frame_equals_obstacle_monitor_base_frame():
    grid_frame = read_node_parameters("controller_node").get("grid_frame", CODE_DEFAULT_FRAME)
    base_frame = read_node_parameters("obstacle_monitor").get("base_frame", CODE_DEFAULT_FRAME)
    assert grid_frame == base_frame


def test_robot_config_uses_the_description_container_frame():
    assert read_node_parameters("controller_node")["grid_frame"] == "base_footprint"

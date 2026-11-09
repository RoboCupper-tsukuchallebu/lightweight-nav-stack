"""docker/config/robot.yaml must give the controller the frame that obstacle_monitor stamps on the local grid.

The controller drops every local_grid whose frame_id differs from its grid_frame, so a mismatch silently turns
obstacle avoidance off (Issue #1). Both nodes fall back to base_link when the key is absent.
"""

import importlib.util
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


def test_controller_uses_obstacle_monitor_geometry():
    launch_path = Path(__file__).resolve().parents[1] / "launch" / "robot.launch.py"
    specification = importlib.util.spec_from_file_location("robot_launch_geometry", launch_path)
    robot_launch = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(robot_launch)
    assert robot_launch.controller_obstacle_parameters({}) == {
        "stop_zone_x_m": 1.0, "stop_zone_y_half_m": 0.4, "inflation_radius_m": 0.5,
    }
    configured = {"stop_zone_x_m": 0.3, "stop_zone_y_half_m": 0.6, "inflation_radius_m": 0.3}
    assert robot_launch.controller_obstacle_parameters(configured) == configured

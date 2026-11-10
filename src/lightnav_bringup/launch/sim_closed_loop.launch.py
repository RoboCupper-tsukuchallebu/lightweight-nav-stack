"""Isolated simulation launch for closed-loop obstacle tests."""
import importlib.util
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    specification = importlib.util.spec_from_file_location("robot_launch", Path(__file__).with_name("robot.launch.py"))
    robot_launch = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(robot_launch)

    scenario = LaunchConfiguration("scenario").perform(context)
    frame = LaunchConfiguration("base_frame").perform(context)
    if scenario not in ("avoid", "blocked") or frame not in ("base_link", "base_footprint"):
        raise ValueError("invalid simulation scenario or base frame")
    candidates = [parent / "docker" / "config" / "robot.yaml" for parent in Path(__file__).resolve().parents]
    robot_config = next((path for path in candidates if path.is_file()), None)
    if robot_config is None:
        raise FileNotFoundError(f"robot.yaml not found in repository ancestors of {__file__}")
    with robot_config.open() as config_file:
        production = yaml.safe_load(config_file)
    parameters = {
        "obstacle_monitor": {**production["obstacle_monitor"]["ros__parameters"], "base_frame": frame, "memory_frame": "world"},
        "pose_adapter_node": {"odom_topic": "/sim/odom", "expected_frame_id": "world",
                              "body_to_base_xyz_rpy": [-0.1, 0.0, -0.7, 0.0, 0.0, 0.0]},
        "controller_node": {**production["controller_node"]["ros__parameters"], **robot_launch.controller_obstacle_parameters(production["obstacle_monitor"]["ros__parameters"]), "grid_frame": frame, "grid_diagnostic_enabled": True, "max_path_deviation_m": float(LaunchConfiguration("max_path_deviation_m").perform(context))},
    }
    scene = {"obstacles": [4.0, 0.3, 0.5]} if scenario == "avoid" else {"walls": [4.0, -1.5, 4.0, 1.5]}
    entities = [Node(package="lightnav_sim", executable="kinematic_sim_node", output="screen",
                     parameters=[scene])]
    if frame == "base_footprint":
        entities.append(Node(package="tf2_ros", executable="static_transform_publisher",
                             arguments=["0", "0", "0", "0", "0", "0", "base_link", "base_footprint"]))
    for package, executable, name in (*robot_launch.NAV_NODES, robot_launch.SAFETY_GATE_NODE):
        entities.extend(robot_launch.build_lifecycle_entities(package, executable, name, parameters.get(name, {}), trace=True))
    return entities


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("scenario", default_value="avoid"),
        DeclareLaunchArgument("max_path_deviation_m", default_value="1.5"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        OpaqueFunction(function=launch_setup),
    ])

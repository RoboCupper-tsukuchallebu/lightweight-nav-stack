"""Start the lightnav nodes of the robot and drive every lifecycle node through configure -> activate.

Launch arguments:
  robot_config  YAML with the parameters of the nodes (the top-level keys are the node names below).
  safety_gate   "true" (default) also starts safety_gate_node, the only node of the stack that publishes /cmd_vel.
                "false" leaves it out: only for teaching drives (record_waypoints with the gamepad), never for
                following waypoints.
"""

import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.descriptions import ParameterFile
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

# (package, executable, node name). The node name is the key in robot.yaml and what `ros2 lifecycle` uses.
NAV_NODES = (
    ("lightnav_obstacle_monitor", "obstacle_monitor_node", "obstacle_monitor"),
    ("lightnav_pose_adapter", "pose_adapter_node", "pose_adapter_node"),
    ("lightnav_planner", "planner_node", "planner_node"),
    ("lightnav_mission", "mission_node", "lightnav_mission"),
    ("lightnav_controller", "controller_node", "controller_node"),
)
SAFETY_GATE_NODE = ("lightnav_safety_gate", "safety_gate_node", "safety_gate_node")


def build_lifecycle_entities(package, executable, node_name, robot_config):
    node = LifecycleNode(
        package=package,
        executable=executable,
        name=node_name,
        namespace="",
        output="screen",
        parameters=robot_config if isinstance(robot_config, list) else [robot_config],
    )
    activate_after_configure = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=node,
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )
    configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(node), transition_id=Transition.TRANSITION_CONFIGURE
        )
    )
    return [node, activate_after_configure, configure]


def controller_obstacle_parameters(monitor_parameters):
    # Pass only explicit monitor configuration; the controller owns fallback values.
    return {name: float(monitor_parameters[name]) for name in (
        "stop_zone_x_m", "stop_zone_y_half_m") if name in monitor_parameters}


def launch_setup(context, *args, **kwargs):
    safety_gate_value = LaunchConfiguration("safety_gate").perform(context).strip().lower()
    if safety_gate_value not in ("true", "false"):
        raise RuntimeError(f"safety_gate must be 'true' or 'false', got '{safety_gate_value}'")
    config_path = LaunchConfiguration("robot_config").perform(context)
    with open(config_path) as config_file:
        config = yaml.safe_load(config_file) or {}
    monitor_parameters = config.get("obstacle_monitor", {}).get("ros__parameters", {})
    controller_parameters = controller_obstacle_parameters(monitor_parameters)
    robot_config = ParameterFile(LaunchConfiguration("robot_config"), allow_substs=False)
    nodes = list(NAV_NODES)
    if safety_gate_value == "true":
        nodes.append(SAFETY_GATE_NODE)
    entities = []
    for package, executable, node_name in nodes:
        parameters = [robot_config, controller_parameters] if node_name == "controller_node" else robot_config
        entities.extend(build_lifecycle_entities(package, executable, node_name, parameters))
    return entities


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_config", default_value="/config/robot.yaml"),
            DeclareLaunchArgument("safety_gate", default_value="true"),
            OpaqueFunction(function=launch_setup),
        ]
    )

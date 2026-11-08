"""Start localization_node and drive it through configure -> activate.

Launch arguments:
  mode        "localization" (default) or "mapping". Passed to the node's `mode` parameter.
  map_file    Prior map PCD. The node needs it in localization mode (here or in params_file); mapping ignores it.
  map_yaml    map.yaml of the prior map; without it GPS is disabled (localization mode).
  params_file Parameter file for localization_node (everything else is set there).
"""

import os

from ament_index_python.packages import get_package_share_directory
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
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

SUPPORTED_MODES = ("localization", "mapping")


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("lightnav_localization")
    mode = LaunchConfiguration("mode")
    params_file = LaunchConfiguration("params_file")
    map_file = LaunchConfiguration("map_file")
    map_yaml = LaunchConfiguration("map_yaml")

    def launch_setup(context, *args, **kwargs):
        parameters = [params_file.perform(context)]
        mode_value = mode.perform(context).strip()
        if mode_value not in SUPPORTED_MODES:
            raise RuntimeError(f"mode must be one of {SUPPORTED_MODES}, got '{mode_value}'")
        overrides = {"mode": mode_value}
        map_file_value = map_file.perform(context)
        if map_file_value:
            overrides["map_file"] = map_file_value
        map_yaml_value = map_yaml.perform(context)
        if map_yaml_value:
            overrides["map_yaml"] = map_yaml_value
        parameters.append(overrides)

        localization_node = LifecycleNode(
            package="lightnav_localization",
            executable="localization_node",
            name="localization_node",
            namespace="",
            output="screen",
            parameters=parameters,
        )
        configure = EmitEvent(
            event=ChangeState(
                lifecycle_node_matcher=matches_action(localization_node),
                transition_id=Transition.TRANSITION_CONFIGURE,
            )
        )
        activate_after_configure = RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=localization_node,
                goal_state="inactive",
                entities=[
                    EmitEvent(
                        event=ChangeState(
                            lifecycle_node_matcher=matches_action(localization_node),
                            transition_id=Transition.TRANSITION_ACTIVATE,
                        )
                    )
                ],
            )
        )
        return [localization_node, activate_after_configure, configure]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                default_value="localization",
                description="localization | mapping (mapping does not need map_file)",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(package_share, "config", "localization.yaml"),
                description="Parameter file for localization_node",
            ),
            DeclareLaunchArgument(
                "map_file",
                default_value="",
                description="Absolute path to the prior map PCD (required in localization mode)",
            ),
            DeclareLaunchArgument(
                "map_yaml",
                default_value="",
                description="Absolute path to map.yaml; without it GPS is disabled",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )

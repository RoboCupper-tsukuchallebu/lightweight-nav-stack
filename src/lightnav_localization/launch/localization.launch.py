"""Start localization_node and drive it through configure -> activate."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("lightnav_localization")
    params_file = LaunchConfiguration("params_file")
    map_file = LaunchConfiguration("map_file")
    map_yaml = LaunchConfiguration("map_yaml")

    localization_node = LifecycleNode(
        package="lightnav_localization",
        executable="localization_node",
        name="localization_node",
        namespace="",
        output="screen",
        parameters=[
            params_file,
            {
                "map_file": map_file,
                "map_yaml": map_yaml,
            },
        ],
    )

    configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=localization_node,
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
                        lifecycle_node_matcher=localization_node,
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(package_share, "config", "localization.yaml"),
                description="Parameter file for localization_node",
            ),
            DeclareLaunchArgument(
                "map_file",
                default_value="",
                description="Absolute path to the prior map PCD (required)",
            ),
            DeclareLaunchArgument(
                "map_yaml",
                default_value="",
                description="Absolute path to map.yaml; without it GPS is disabled",
            ),
            localization_node,
            activate_after_configure,
            configure,
        ]
    )

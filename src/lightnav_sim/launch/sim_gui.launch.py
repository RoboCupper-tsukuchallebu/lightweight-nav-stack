"""Start kinematic_sim_node with the sample scene and RViz2."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_directory = get_package_share_directory("lightnav_sim")
    return LaunchDescription(
        [
            DeclareLaunchArgument("scene", default_value=os.path.join(share_directory, "config", "sample_scene.yaml")),
            DeclareLaunchArgument("rviz_config", default_value=os.path.join(share_directory, "rviz", "sim.rviz")),
            Node(
                package="lightnav_sim",
                executable="kinematic_sim_node",
                parameters=[LaunchConfiguration("scene")],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", LaunchConfiguration("rviz_config")],
                output="screen",
            ),
        ]
    )

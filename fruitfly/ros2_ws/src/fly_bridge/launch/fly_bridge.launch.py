"""Launch the fly bridge with its default configuration."""
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    share = get_package_share_directory("fly_bridge")
    params = os.path.join(share, "config", "fly_bridge.yaml")
    return LaunchDescription([
        Node(
            package="fly_bridge",
            executable="fly_bridge",
            name="fly_bridge",
            output="screen",
            parameters=[params],
        ),
    ])

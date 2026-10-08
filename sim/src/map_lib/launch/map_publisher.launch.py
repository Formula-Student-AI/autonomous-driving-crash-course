from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    map_path = DeclareLaunchArgument(
        name="map_path",
        default_value="",
        description="Absolute path to the map as a csv file in EUFS format."
    )

    frame_id = DeclareLaunchArgument(
        name="frame_id",
        default_value="map",
        description="Frame to publish the map in."
    )

    map_publisher = Node(
        package='map_lib',
        executable='map_publisher',
        name='map_publisher',
        parameters=[
            {"map_path": LaunchConfiguration("map_path")},
            {"frame_id": LaunchConfiguration("frame_id")}
        ],
        output='both',
    )

    ld = LaunchDescription()
    ld.add_action(map_path)
    ld.add_action(frame_id)
    ld.add_action(map_publisher)
    return ld

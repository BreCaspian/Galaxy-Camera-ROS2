import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("galaxy_camera_ros2"), "config", "camera.yaml"
    )
    params_file = LaunchConfiguration("params_file")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Path to camera parameter YAML file.",
            ),
            Node(
                package="galaxy_camera_ros2",
                executable="galaxy_camera_node",
                name="galaxy_camera_ros2",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )

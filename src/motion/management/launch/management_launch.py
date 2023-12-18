import os
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # config_file_name = "aliengo/config.yaml"
    config_file_name = "a1/config.yaml"
    config_file = os.path.join(get_package_share_path("asserts"), config_file_name)
    # print(config_file)
    return LaunchDescription(
        [
            Node(
                package="management",
                executable="management",
                name="a1_management",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {"/config_file": config_file},
                ],
            ),
            Node(
                package="joy",
                executable="joy_node",
                name="a1_joy_node",
                output="screen",
                emulate_tty=True,
            ),
        ]
    )
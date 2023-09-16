import os
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    xml_file_name = "aliengo/config.yaml"
    xml_file = os.path.join(get_package_share_path("asserts"), xml_file_name)
    # print(xml_file)
    return LaunchDescription(
        [
            Node(
                package="estimation",
                executable="estimator",
                name="aliengo_estimator",
                output="screen",
                emulate_tty=True,
                arguments=[xml_file, ("__log_level:=debug")],
            ),
        ]
    )
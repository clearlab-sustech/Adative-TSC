import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    urdfFile = os.path.join(
        get_package_share_directory("piano_robotic_assets"),
        "resources/anymal_c/urdf/anymal.urdf",
    )
    taskFile = os.path.join(
        get_package_share_directory("piano_legged_robot"),
        "config/mpc/task.info",
    )
    referenceFile = os.path.join(
        get_package_share_directory("piano_legged_robot"),
        "config/command/reference.info",
    )
    gaitCommandFile = os.path.join(
        get_package_share_directory("piano_legged_robot"),
        "config/command/gait.info",
    )
    rviz_config = os.path.join(
        get_package_share_directory("piano_legged_robot_ros"),
        "rviz/legged_robot.rviz",
    )
    
    print(urdfFile)

    return LaunchDescription(
        [
            TimerAction(
                period=2.0,
                actions=[
                    Node(
                        package="piano_legged_robot_ros",
                        executable="legged_robot_dummy",
                        name="legged_robot_dummy",
                        output="screen",
                        parameters=[
                            {"/urdfFile": urdfFile},
                            {"/referenceFile": referenceFile},
                            {"/taskFile": taskFile},
                        ],
                    )
                ]),

            
        ]
    )

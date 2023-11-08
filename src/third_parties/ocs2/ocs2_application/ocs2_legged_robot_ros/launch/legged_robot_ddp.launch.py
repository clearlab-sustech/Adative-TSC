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
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (Gazebo) clock if true",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"robot_description": open(urdfFile).read()},
                ],
            ),

            TimerAction(
                period=0.5,
                actions=[
                    Node(
                        package="piano_legged_robot_ros",
                        executable="legged_robot_ddp_mpc",
                        name="legged_robot_ddp_mpc",
                        output="screen",
                        parameters=[
                            {"/urdfFile": urdfFile},
                            {"/referenceFile": referenceFile},
                            {"/taskFile": taskFile},
                        ],
                    )
                ]),

            TimerAction(
                period=0.0,
                actions=[
                    Node(
                        package="piano_legged_robot_ros",
                        executable="legged_robot_dummy",
                        name="legged_robot_dummy",
                        output="screen",
                        prefix="gnome-terminal --",
                        parameters=[
                            {"/urdfFile": urdfFile},
                            {"/referenceFile": referenceFile},
                            {"/taskFile": taskFile},
                        ],
                    )
                ]),

            TimerAction(
                period=1.0,
                actions=[
                    Node(
                        package="piano_legged_robot_ros",
                        executable="legged_robot_target",
                        name="legged_robot_target",
                        output="screen",
                        prefix="gnome-terminal --",
                        parameters=[
                            {"/referenceFile": referenceFile},
                        ],
                        emulate_tty=True,
                        arguments=[("__log_level:=debug")],
                    )
                ]),

            TimerAction(
                period=1.00,
                actions=[
                    Node(
                        package="piano_legged_robot_ros",
                        executable="legged_robot_gait_command",
                        name="legged_robot_gait_command",
                        output="screen",
                        prefix="gnome-terminal --",
                        parameters=[
                            {"/gaitCommandFile": gaitCommandFile},
                        ],
                        emulate_tty=True,
                        arguments=[("__log_level:=debug")],
                    )
                ]),

            TimerAction(
                period=4.00,
                actions=[
                    Node(
                        package="rviz2",
                        executable="rviz2",
                        name="rviz2",
                        output="screen",
                        arguments=["-d", rviz_config],
                    ),
                ]),
        ]
    )

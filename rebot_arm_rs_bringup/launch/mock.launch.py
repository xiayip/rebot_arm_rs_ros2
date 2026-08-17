from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("rebot_arm_rs_description"), "rviz", "view.rviz"]
    )
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("rebot_arm_rs_bringup"), "launch", "bringup.launch.py"]
                    )
                ),
                launch_arguments={
                    "use_mock_hardware": "true",
                    "activate_trajectory_controller": "true",
                }.items(),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config_file],
                output="screen",
            ),
        ]
    )

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_file = LaunchConfiguration("description_file")
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("rebot_arm_rs_description"), "rviz", "view.rviz"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "description_file",
                default_value="rebot_arm_rs.urdf.xacro",
                description="Xacro filename from rebot_arm_rs_description/urdf.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("rebot_arm_rs_bringup"), "launch", "bringup.launch.py"]
                    )
                ),
                launch_arguments={
                    "use_mock_hardware": "true",
                    "activate_trajectory_controller": "true",
                    "description_file": description_file,
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

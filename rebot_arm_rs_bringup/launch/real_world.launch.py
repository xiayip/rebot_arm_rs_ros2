from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    can_interface = LaunchConfiguration("can_interface")
    auto_enable = LaunchConfiguration("auto_enable_on_activate")
    return LaunchDescription(
        [
            DeclareLaunchArgument("can_interface", default_value="can1"),
            DeclareLaunchArgument("auto_enable_on_activate", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("rebot_arm_rs_bringup"), "launch", "bringup.launch.py"]
                    )
                ),
                launch_arguments={
                    "use_mock_hardware": "false",
                    "can_interface": can_interface,
                    "auto_enable_on_activate": auto_enable,
                    "activate_trajectory_controller": "false",
                }.items(),
            ),
        ]
    )

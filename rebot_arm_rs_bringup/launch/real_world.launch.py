from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    can_interface = LaunchConfiguration("can_interface")
    activate_trajectory = LaunchConfiguration("activate_trajectory_controller")
    description_file = LaunchConfiguration("description_file")
    return LaunchDescription(
        [
            DeclareLaunchArgument("can_interface", default_value="can1"),
            DeclareLaunchArgument("activate_trajectory_controller", default_value="true"),
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
                    "use_mock_hardware": "false",
                    "can_interface": can_interface,
                    "activate_trajectory_controller": activate_trajectory,
                    "description_file": description_file,
                }.items(),
            ),
        ]
    )

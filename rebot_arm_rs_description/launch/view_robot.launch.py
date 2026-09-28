from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_file = LaunchConfiguration("description_file")
    xacro_file = PathJoinSubstitution(
        [FindPackageShare("rebot_arm_rs_description"), "urdf", description_file]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("rebot_arm_rs_description"), "rviz", "view.rviz"]
    )
    robot_description = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", xacro_file, " use_mock_hardware:=true"]),
            value_type=str,
        )
    }
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "description_file",
                default_value="rebot_arm_rs.urdf.xacro",
                description="Xacro filename from rebot_arm_rs_description/urdf.",
            ),
            Node(package="robot_state_publisher", executable="robot_state_publisher", parameters=[robot_description]),
            Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui"),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config_file],
                output="screen",
            ),
        ]
    )

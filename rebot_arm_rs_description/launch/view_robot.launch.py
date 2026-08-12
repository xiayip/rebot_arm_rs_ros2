from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    xacro_file = PathJoinSubstitution(
        [FindPackageShare("rebot_arm_rs_description"), "urdf", "rebot_arm_rs.urdf.xacro"]
    )
    robot_description = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", xacro_file, " use_mock_hardware:=true"]),
            value_type=str,
        )
    }
    return LaunchDescription(
        [
            Node(package="robot_state_publisher", executable="robot_state_publisher", parameters=[robot_description]),
            Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui"),
            Node(package="rviz2", executable="rviz2", output="screen"),
        ]
    )


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    can_interface = LaunchConfiguration("can_interface")
    auto_enable = LaunchConfiguration("auto_enable_on_activate")
    activate_trajectory = LaunchConfiguration("activate_trajectory_controller")

    xacro_file = PathJoinSubstitution(
        [FindPackageShare("rebot_arm_rs_description"), "urdf", "rebot_arm_rs.urdf.xacro"]
    )
    controller_file = PathJoinSubstitution(
        [FindPackageShare("rebot_arm_rs_bringup"), "config", "controllers.yaml"]
    )
    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    FindExecutable(name="xacro"),
                    " ", xacro_file,
                    " use_mock_hardware:=", use_mock_hardware,
                    " can_interface:=", can_interface,
                    " auto_enable_on_activate:=", auto_enable,
                ]
            ),
            value_type=str,
        )
    }

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[controller_file],
        remappings=[("~/robot_description", "/robot_description")],
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
    )
    active_trajectory_spawner = Node(
        condition=IfCondition(activate_trajectory),
        package="controller_manager",
        executable="spawner",
        arguments=["rebot_arm_rs_trajectory_controller", "-c", "/controller_manager"],
        output="screen",
    )
    inactive_trajectory_spawner = Node(
        condition=UnlessCondition(activate_trajectory),
        package="controller_manager",
        executable="spawner",
        arguments=["rebot_arm_rs_trajectory_controller", "-c", "/controller_manager", "--inactive"],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_mock_hardware", default_value="false"),
            DeclareLaunchArgument("can_interface", default_value="can1"),
            DeclareLaunchArgument("auto_enable_on_activate", default_value="false"),
            DeclareLaunchArgument("activate_trajectory_controller", default_value="false"),
            robot_state_publisher,
            control_node,
            joint_state_spawner,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_spawner,
                    on_exit=[active_trajectory_spawner, inactive_trajectory_spawner],
                )
            ),
        ]
    )

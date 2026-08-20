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
    mit_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rebot_arm_rs_mit_controller", "-c", "/controller_manager"],
        output="screen",
    )
    active_upstream_chain_spawner = Node(
        condition=IfCondition(activate_trajectory),
        package="controller_manager",
        executable="spawner",
        arguments=[
            "rebot_arm_rs_position_adapter",
            "rebot_arm_rs_trajectory_controller",
            "-c", "/controller_manager",
            "--activate-as-group",
        ],
        output="screen",
    )
    inactive_upstream_chain_spawner = Node(
        condition=UnlessCondition(activate_trajectory),
        package="controller_manager",
        executable="spawner",
        arguments=[
            "rebot_arm_rs_position_adapter",
            "rebot_arm_rs_trajectory_controller",
            "-c", "/controller_manager",
            "--inactive",
        ],
        output="screen",
    )
    damping_spawner_after_active_chain = Node(
        condition=IfCondition(activate_trajectory),
        package="controller_manager",
        executable="spawner",
        arguments=[
            "rebot_arm_rs_damping_controller",
            "-c", "/controller_manager",
            "--inactive",
        ],
        output="screen",
    )
    damping_spawner_after_inactive_chain = Node(
        condition=UnlessCondition(activate_trajectory),
        package="controller_manager",
        executable="spawner",
        arguments=[
            "rebot_arm_rs_damping_controller",
            "-c", "/controller_manager",
            "--inactive",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_mock_hardware", default_value="false"),
            DeclareLaunchArgument("can_interface", default_value="can1"),
            DeclareLaunchArgument("activate_trajectory_controller", default_value="false"),
            robot_state_publisher,
            control_node,
            joint_state_spawner,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_spawner,
                    on_exit=[mit_controller_spawner],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=mit_controller_spawner,
                    on_exit=[active_upstream_chain_spawner, inactive_upstream_chain_spawner],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=active_upstream_chain_spawner,
                    on_exit=[damping_spawner_after_active_chain],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=inactive_upstream_chain_spawner,
                    on_exit=[damping_spawner_after_inactive_chain],
                )
            ),
        ]
    )

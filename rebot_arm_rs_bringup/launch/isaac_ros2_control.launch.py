from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    xacro_file = PathJoinSubstitution(
        [
            FindPackageShare('rebot_arm_rs_description'),
            'urdf',
            'rebot_arm_rs.urdf.xacro',
        ]
    )
    controller_file = PathJoinSubstitution(
        [
            FindPackageShare('rebot_arm_rs_bringup'),
            'config',
            'isaac_controllers.yaml',
        ]
    )
    robot_description = {
        'robot_description': ParameterValue(
            Command(
                [
                    FindExecutable(name='xacro'),
                    ' ',
                    xacro_file,
                    ' use_isaac_sim:=true',
                ]
            ),
            value_type=str,
        )
    }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='both',
        parameters=[controller_file],
        remappings=[('~/robot_description', '/robot_description')],
    )
    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen',
    )
    trajectory_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'rebot_arm_rs_isaac_trajectory_controller',
            '-c',
            '/controller_manager',
        ],
        output='screen',
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            control_node,
            joint_state_spawner,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_spawner,
                    on_exit=[trajectory_spawner],
                )
            ),
        ]
    )

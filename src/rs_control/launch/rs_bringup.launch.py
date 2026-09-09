"""Bring up the RobStride ros2_control hardware and standard controllers."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('rs_control'), 'config', 'robstride.yaml',
    ])
    controller_config = PathJoinSubstitution([
        FindPackageShare('rs_control'), 'config', 'ros2_controllers.yaml',
    ])
    model = PathJoinSubstitution([
        FindPackageShare('rs_control'), 'urdf', 'nxp_rs_ros2_control.xacro',
    ])

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ', LaunchConfiguration('model'),
            ' motor_config:=', LaunchConfiguration('config_file'),
            ' can_interface:=', LaunchConfiguration('can_interface'),
            ' bitrate:=', LaunchConfiguration('bitrate'),
            ' host_id:=', LaunchConfiguration('host_id'),
            ' response_timeout_ms:=', LaunchConfiguration('response_timeout_ms'),
            ' poll_rate_hz:=', LaunchConfiguration('poll_rate_hz'),
            ' read_only:=', LaunchConfiguration('read_only'),
        ]),
        value_type=str,
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controller_config,
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=config),
        DeclareLaunchArgument('model', default_value=model),
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument('bitrate', default_value='1000000'),
        DeclareLaunchArgument('host_id', default_value='255'),
        DeclareLaunchArgument('response_timeout_ms', default_value='20'),
        DeclareLaunchArgument('poll_rate_hz', default_value='20.0'),
        DeclareLaunchArgument(
            'read_only', default_value='true',
            description='Keep motors disabled and ignore position commands.'),
        DeclareLaunchArgument(
            'enable_control', default_value='false',
            description='Start the position trajectory controller.'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        control_node,
        # A failed hardware activation terminates ros2_control_node.  Stop the
        # spawners as well so they do not wait forever on list_controllers.
        RegisterEventHandler(
            OnProcessExit(
                target_action=control_node,
                on_exit=[Shutdown(reason='ros2_control_node exited')],
            )
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint_state_broadcaster',
                '--controller-manager', '/controller_manager',
            ],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'arm_controller',
                '--controller-manager', '/controller_manager',
            ],
            condition=IfCondition(LaunchConfiguration('enable_control')),
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'gripper_controller',
                '--controller-manager', '/controller_manager',
            ],
            condition=IfCondition(LaunchConfiguration('enable_control')),
            output='screen',
        ),
    ])

"""Read RobStride encoders without enabling or commanding any motor."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('rs_control'), 'config', 'robstride.yaml',
    ])
    model = PathJoinSubstitution([
        FindPackageShare('nxp_rs_description'), 'urdf', 'nxp_rs.xacro',
    ])

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ', LaunchConfiguration('model'),
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=config),
        DeclareLaunchArgument('model', default_value=model),
        DeclareLaunchArgument('publish_rate_hz', default_value='20.0'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='rs_control',
            executable='rs_encoder_reader',
            name='rs_encoder_reader',
            parameters=[{
                'config_file': LaunchConfiguration('config_file'),
                'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
            }],
            output='screen',
        ),
    ])

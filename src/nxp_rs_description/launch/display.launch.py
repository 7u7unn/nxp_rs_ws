"""Display the current robot description and its TF frames in RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Launch the robot model, joint sliders, and RViz."""
    package_share = get_package_share_directory('nxp_rs_description')
    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value=os.path.join(package_share, 'urdf', 'nxp_rs.xacro'),
            description='Absolute path to the robot Xacro file',
        ),
        DeclareLaunchArgument(
            'rvizconfig',
            default_value=os.path.join(package_share, 'rviz', 'urdf_config.rviz'),
            description='Absolute path to the RViz configuration',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(
                    Command([FindExecutable(name='xacro'), ' ', LaunchConfiguration('model')]),
                    value_type=str,
                ),
            }],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', LaunchConfiguration('rvizconfig')],
            output='screen',
        ),
    ])

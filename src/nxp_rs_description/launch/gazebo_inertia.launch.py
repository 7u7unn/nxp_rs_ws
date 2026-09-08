"""Inspect the physical model in Gazebo Classic, paused and bench mounted."""

import os

import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('nxp_rs_description')
    gazebo_share = get_package_share_directory('gazebo_ros')
    robot_description = xacro.process_file(
        os.path.join(package_share, 'urdf', 'nxp_rs_gazebo.xacro'),
    ).toxml().replace('package://nxp_rs_description/', 'file://' + package_share + '/')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('pause', default_value='true'),
        DeclareLaunchArgument('gazebo_port', default_value='11346'),
        DeclareLaunchArgument('ros_domain_id', default_value='46'),
        SetEnvironmentVariable('GAZEBO_MASTER_URI', [
            'http://127.0.0.1:', LaunchConfiguration('gazebo_port'),
        ]),
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('ros_domain_id')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')),
            launch_arguments={
                'world': os.path.join(package_share, 'worlds', 'inertia.world'),
                'gui': LaunchConfiguration('gui'),
                'pause': LaunchConfiguration('pause'),
                'verbose': 'true',
                'server_required': 'true',
            }.items(),
        ),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            parameters=[{
                'use_sim_time': True,
                'robot_description': robot_description,
            }],
            output='screen',
        ),
        Node(
            package='gazebo_ros', executable='spawn_entity.py',
            arguments=['-entity', 'nxp_rs', '-topic', 'robot_description', '-timeout', '60'],
            output='screen',
        ),
    ])

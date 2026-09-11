"""Bring up the Python RobStride reader/controller."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare("rs_control"), "config", "robstride.yaml",
    ])
    model = PathJoinSubstitution([
        FindPackageShare("nxp_rs_description"), "urdf", "nxp_rs.xacro",
    ])
    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", model]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=config),
        DeclareLaunchArgument("can_interface", default_value="can0"),
        DeclareLaunchArgument("bitrate", default_value="1000000"),
        DeclareLaunchArgument("host_id", default_value="255"),
        DeclareLaunchArgument("response_timeout_ms", default_value="20"),
        DeclareLaunchArgument(
            "poll_rate_hz",
            default_value="100.0",
            description="RobStride operation/readback loop rate.",
        ),
        DeclareLaunchArgument(
            "read_only",
            default_value="true",
            description="Read parameters without enabling or commanding torque.",
        ),
        DeclareLaunchArgument("joint_state_topic", default_value="/joint_states"),
        DeclareLaunchArgument(
            "command_topic", default_value="/rs_control/joint_trajectory"
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
        Node(
            package="rs_control",
            executable="rs_python_control",
            name="rs_python_control",
            parameters=[{
                "config_file": LaunchConfiguration("config_file"),
                "can_interface": LaunchConfiguration("can_interface"),
                "bitrate": LaunchConfiguration("bitrate"),
                "host_id": LaunchConfiguration("host_id"),
                "response_timeout_ms": LaunchConfiguration("response_timeout_ms"),
                "poll_rate_hz": LaunchConfiguration("poll_rate_hz"),
                "read_only": LaunchConfiguration("read_only"),
                "joint_state_topic": LaunchConfiguration("joint_state_topic"),
                "command_topic": LaunchConfiguration("command_topic"),
            }],
            output="screen",
        ),
    ])

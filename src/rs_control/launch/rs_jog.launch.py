"""Launch the standalone interactive RobStride jog controller."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    limits_file = PathJoinSubstitution(
        [FindPackageShare("rs_control"), "config", "joints.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("limits_file", default_value=limits_file),
            DeclareLaunchArgument("state_topic", default_value="/joint_states"),
            DeclareLaunchArgument(
                "arm_command_topic",
                default_value="/arm_controller/joint_trajectory",
            ),
            DeclareLaunchArgument(
                "gripper_command_topic",
                default_value="/gripper_controller/joint_trajectory",
            ),
            DeclareLaunchArgument("selected_joint", default_value="joint-1"),
            DeclareLaunchArgument("arm_step", default_value="0.05"),
            DeclareLaunchArgument("gripper_step", default_value="0.001"),
            DeclareLaunchArgument(
                "command_duration_s",
                default_value="0.2",
                description="Short trajectory horizon used for continuous retargeting.",
            ),
            DeclareLaunchArgument("command_rate_hz", default_value="20.0"),
            DeclareLaunchArgument("feedback_timeout_s", default_value="0.25"),
            DeclareLaunchArgument("max_arm_velocity", default_value="0.4"),
            DeclareLaunchArgument("max_gripper_velocity", default_value="0.02"),
            DeclareLaunchArgument("max_arm_acceleration", default_value="0.8"),
            DeclareLaunchArgument("max_gripper_acceleration", default_value="0.04"),
            DeclareLaunchArgument("max_arm_jerk", default_value="4.0"),
            DeclareLaunchArgument("max_gripper_jerk", default_value="0.2"),
            DeclareLaunchArgument("max_arm_target_lead", default_value="0.25"),
            DeclareLaunchArgument("max_gripper_target_lead", default_value="0.01"),
            DeclareLaunchArgument("arm_limit_margin", default_value="0.02"),
            DeclareLaunchArgument("gripper_limit_margin", default_value="0.001"),
            Node(
                package="rs_control",
                executable="rs_jog_controller",
                name="rs_jog_controller",
                parameters=[
                    {
                        "limits_file": LaunchConfiguration("limits_file"),
                        "state_topic": LaunchConfiguration("state_topic"),
                        "arm_command_topic": LaunchConfiguration("arm_command_topic"),
                        "gripper_command_topic": LaunchConfiguration(
                            "gripper_command_topic"
                        ),
                        "selected_joint": LaunchConfiguration("selected_joint"),
                        "arm_step": LaunchConfiguration("arm_step"),
                        "gripper_step": LaunchConfiguration("gripper_step"),
                        "command_duration_s": LaunchConfiguration("command_duration_s"),
                        "command_rate_hz": LaunchConfiguration("command_rate_hz"),
                        "feedback_timeout_s": LaunchConfiguration("feedback_timeout_s"),
                        "max_arm_velocity": LaunchConfiguration("max_arm_velocity"),
                        "max_gripper_velocity": LaunchConfiguration(
                            "max_gripper_velocity"
                        ),
                        "max_arm_acceleration": LaunchConfiguration(
                            "max_arm_acceleration"
                        ),
                        "max_gripper_acceleration": LaunchConfiguration(
                            "max_gripper_acceleration"
                        ),
                        "max_arm_jerk": LaunchConfiguration("max_arm_jerk"),
                        "max_gripper_jerk": LaunchConfiguration("max_gripper_jerk"),
                        "max_arm_target_lead": LaunchConfiguration(
                            "max_arm_target_lead"
                        ),
                        "max_gripper_target_lead": LaunchConfiguration(
                            "max_gripper_target_lead"
                        ),
                        "arm_limit_margin": LaunchConfiguration("arm_limit_margin"),
                        "gripper_limit_margin": LaunchConfiguration(
                            "gripper_limit_margin"
                        ),
                    }
                ],
                output="screen",
            ),
        ]
    )

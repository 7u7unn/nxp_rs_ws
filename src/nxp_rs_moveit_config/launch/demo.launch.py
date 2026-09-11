"""Launch MoveIt 2 with the NXP RS URDF and a simulation-only mock controller."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    package_share = get_package_share_directory("nxp_rs_moveit_config")

    moveit_config = (
        MoveItConfigsBuilder("nxp_rs", package_name="nxp_rs_moveit_config")
        .robot_description(file_path="config/nxp_rs_moveit.urdf.xacro")
        .robot_description_semantic(file_path="config/nxp_rs.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    rviz = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Start RViz with the MoveIt MotionPlanning display.",
    )
    use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use a simulated /clock if another simulator provides one.",
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    floor_scene = Node(
        package="nxp_rs_moveit_config",
        executable="floor_scene",
        name="floor_scene",
        output="screen",
        parameters=[os.path.join(package_share, "config", "floor_scene.yaml")],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(package_share, "rviz", "moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            moveit_config.planning_pipelines,
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=[
            "--frame-id",
            "world",
            "--child-frame-id",
            "base_link",
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    controllers_path = os.path.join(package_share, "config", "ros2_controllers.yaml")
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )
    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager"],
        output="screen",
    )
    gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "-c", "/controller_manager"],
        output="screen",
    )

    return LaunchDescription(
        [
            rviz,
            use_sim_time,
            static_tf,
            robot_state_publisher,
            move_group,
            floor_scene,
            ros2_control_node,
            joint_state_broadcaster,
            arm_controller,
            gripper_controller,
            rviz_node,
        ]
    )

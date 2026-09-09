"""Launch MoveIt 2 against the physical RobStride ros2_control system.

The launch is read-only by default.  Set both ``read_only:=false`` and
``enable_control:=true`` only after the CAN bus, calibration, limits, and
emergency-stop path have been checked.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_share = get_package_share_directory("nxp_rs_moveit_config")
    rs_control_share = get_package_share_directory("rs_control")

    default_config = PathJoinSubstitution(
        [FindPackageShare("rs_control"), "config", "robstride.yaml"]
    )
    controller_config = PathJoinSubstitution(
        [FindPackageShare("rs_control"), "config", "ros2_controllers.yaml"]
    )
    hardware_model = os.path.join(
        rs_control_share, "urdf", "nxp_rs_ros2_control.xacro"
    )

    config_file = DeclareLaunchArgument(
        "config_file",
        default_value=default_config,
        description="RobStride motor YAML used by the physical hardware plugin.",
    )
    can_interface = DeclareLaunchArgument("can_interface", default_value="can0")
    bitrate = DeclareLaunchArgument("bitrate", default_value="1000000")
    host_id = DeclareLaunchArgument("host_id", default_value="255")
    response_timeout_ms = DeclareLaunchArgument(
        "response_timeout_ms", default_value="20"
    )
    poll_rate_hz = DeclareLaunchArgument("poll_rate_hz", default_value="20.0")
    read_only = DeclareLaunchArgument(
        "read_only",
        default_value="true",
        description="Keep motors disabled and ignore trajectory commands.",
    )
    enable_control = DeclareLaunchArgument(
        "enable_control",
        default_value="false",
        description="Spawn physical arm and gripper trajectory controllers.",
    )
    rviz = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Start RViz with the MoveIt MotionPlanning display.",
    )
    use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="false")

    # Use the exact hardware URDF for robot_state_publisher, ros2_control, and
    # MoveIt.  Passing LaunchConfigurations keeps the xacro expansion at launch
    # time, so config_file and the CAN settings remain overridable.
    moveit_config = (
        MoveItConfigsBuilder("nxp_rs", package_name="nxp_rs_moveit_config")
        .robot_description(
            file_path=hardware_model,
            mappings={
                "motor_config": LaunchConfiguration("config_file"),
                "can_interface": LaunchConfiguration("can_interface"),
                "bitrate": LaunchConfiguration("bitrate"),
                "host_id": LaunchConfiguration("host_id"),
                "response_timeout_ms": LaunchConfiguration("response_timeout_ms"),
                "poll_rate_hz": LaunchConfiguration("poll_rate_hz"),
                "read_only": LaunchConfiguration("read_only"),
            },
        )
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

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, controller_config],
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
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        condition=IfCondition(LaunchConfiguration("enable_control")),
        output="screen",
    )
    gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        condition=IfCondition(LaunchConfiguration("enable_control")),
        output="screen",
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

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(moveit_share, "rviz", "moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            moveit_config.planning_pipelines,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    )

    return LaunchDescription(
        [
            config_file,
            can_interface,
            bitrate,
            host_id,
            response_timeout_ms,
            poll_rate_hz,
            read_only,
            enable_control,
            rviz,
            use_sim_time,
            static_tf,
            robot_state_publisher,
            ros2_control_node,
            joint_state_broadcaster,
            arm_controller,
            gripper_controller,
            move_group,
            rviz_node,
        ]
    )

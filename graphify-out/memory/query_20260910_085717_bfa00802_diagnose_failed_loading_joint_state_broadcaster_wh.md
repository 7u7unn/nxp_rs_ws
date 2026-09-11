---
type: "query"
date: "2026-09-10T08:57:17.431244+00:00"
question: "Diagnose failed loading joint_state_broadcaster when launching hardware MoveIt read_only"
contributor: "graphify"
outcome: "useful"
source_nodes: ["src/nxp_rs_moveit_config/launch/hardware_moveit.launch.py", "src/rs_control/launch/rs_bringup.launch.py", "src/rs_control/config/ros2_controllers.yaml", "joint_state_broadcaster/JointStateBroadcaster", "RobstrideSystem::on_activate()"]
---

# Q: Diagnose failed loading joint_state_broadcaster when launching hardware MoveIt read_only

## Answer

The failure is caused by launching hardware_moveit while rs_bringup is still running. The live ROS graph shows duplicate /controller_manager, /joint_state_broadcaster, /robot_state_publisher, and /rviz2 nodes. ros2_control successfully initializes, configures, and activates robstride_system; the JointStateBroadcaster plugin is discoverable. Stop both launch sessions, verify only one controller manager remains, then launch hardware_moveit as the sole owner of hardware, robot_state_publisher, controllers, MoveIt, and RViz.

## Outcome

- Signal: useful

## Source Nodes

- src/nxp_rs_moveit_config/launch/hardware_moveit.launch.py
- src/rs_control/launch/rs_bringup.launch.py
- src/rs_control/config/ros2_controllers.yaml
- joint_state_broadcaster/JointStateBroadcaster
- RobstrideSystem::on_activate()
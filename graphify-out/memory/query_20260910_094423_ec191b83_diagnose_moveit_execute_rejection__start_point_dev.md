---
type: "query"
date: "2026-09-10T09:44:23.145573+00:00"
question: "Diagnose MoveIt execute rejection: start point deviates from current robot state"
contributor: "graphify"
outcome: "useful"
source_nodes: ["src/rs_control/config/ros2_controllers.yaml", "RobstrideSystem::read_all_initial_states()", "RobstrideSystem::io_loop()", "src/nxp_rs_moveit_config/config/moveit_controllers.yaml", "src/nxp_rs_moveit_config/README.md"]
---

# Q: Diagnose MoveIt execute rejection: start point deviates from current robot state

## Answer

The controllers and hardware are running, but trajectory execution was rejected safely because the first trajectory point for joint-2 was 0.0602089 rad while current feedback was 0.144963 rad, a 0.0847541 rad (~4.86 degree) mismatch above MoveIt's 0.01 rad start tolerance. Replan from the Current start state after the robot settles; do not increase tolerance. If joint-2 drifts while stationary, investigate holding/calibration/feedback and the physical open_loop_control setting before retrying.

## Outcome

- Signal: useful

## Source Nodes

- src/rs_control/config/ros2_controllers.yaml
- RobstrideSystem::read_all_initial_states()
- RobstrideSystem::io_loop()
- src/nxp_rs_moveit_config/config/moveit_controllers.yaml
- src/nxp_rs_moveit_config/README.md
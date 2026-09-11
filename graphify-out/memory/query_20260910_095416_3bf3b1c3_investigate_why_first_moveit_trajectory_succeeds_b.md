---
type: "query"
date: "2026-09-10T09:54:16.133665+00:00"
question: "Investigate why first MoveIt trajectory succeeds but the next plan starts in collision"
contributor: "graphify"
outcome: "useful"
source_nodes: ["src/rs_control/config/ros2_controllers.yaml", "src/rs_control/src/robstride_system.cpp", "src/nxp_rs_moveit_config/config/nxp_rs.srdf", "src/nxp_rs_description/urdf/nxp_rs.xacro", "RobstrideSystem::io_loop()", "Move the physical robot from MoveIt"]
---

# Q: Investigate why first MoveIt trajectory succeeds but the next plan starts in collision

## Answer

The first arm trajectory action reported SUCCEEDED, but live arm_controller state showed final reference positions joint-2=-0.05327 and joint-4=1.18714 while measured feedback remained joint-2=0.01304 and joint-4=1.26728, with errors -0.06631 and -0.08014 rad. open_loop_control is true and all configured JTC position/goal tolerances are zero, so action completion did not prove measured pose reached the reference. MoveIt state-validity check showed the measured endpoint invalid with base_link/right-finger penetration 0.01163 m, while the controller desired endpoint was valid. The second request correctly detected the measured post-motion collision and refused execution. Root issue is physical tracking/holding or calibration plus open-loop success semantics; do not disable the collision or increase MoveIt tolerance.

## Outcome

- Signal: useful

## Source Nodes

- src/rs_control/config/ros2_controllers.yaml
- src/rs_control/src/robstride_system.cpp
- src/nxp_rs_moveit_config/config/nxp_rs.srdf
- src/nxp_rs_description/urdf/nxp_rs.xacro
- RobstrideSystem::io_loop()
- Move the physical robot from MoveIt
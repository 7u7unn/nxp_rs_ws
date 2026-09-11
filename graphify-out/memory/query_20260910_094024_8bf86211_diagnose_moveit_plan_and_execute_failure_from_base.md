---
type: "query"
date: "2026-09-10T09:40:24.018479+00:00"
question: "Diagnose MoveIt plan-and-execute failure from base_link/right-finger collision"
contributor: "graphify"
outcome: "useful"
source_nodes: ["src/nxp_rs_moveit_config/config/nxp_rs.srdf", "src/nxp_rs_description/urdf/nxp_rs.xacro", "src/nxp_rs_moveit_config/launch/hardware_moveit.launch.py", "src/rs_control/config/ros2_controllers.yaml", "Move the physical robot from MoveIt"]
---

# Q: Diagnose MoveIt plan-and-execute failure from base_link/right-finger collision

## Answer

Hardware and all three controllers loaded and activated successfully. MoveIt rejected the request because the current start state is in self-collision: base_link contacts right-finger. The fix-start-state adapter tried to move the start state, but the generated path still contained collision states, so MoveIt explicitly did not execute it. The warnings about trajectory-end velocity, RViz plugin namespace, missing recognize_objects, and root-link inertia are unrelated. SRDF disables palm/finger and adjacent link collisions but intentionally does not disable base_link/right-finger; verify the physical pose and collision mesh/joint calibration before changing SRDF.

## Outcome

- Signal: useful

## Source Nodes

- src/nxp_rs_moveit_config/config/nxp_rs.srdf
- src/nxp_rs_description/urdf/nxp_rs.xacro
- src/nxp_rs_moveit_config/launch/hardware_moveit.launch.py
- src/rs_control/config/ros2_controllers.yaml
- Move the physical robot from MoveIt
---
type: "query"
date: "2026-09-10T08:44:50.667524+00:00"
question: "Audit all requirements and setup before final MoveIt deployment to the real robot"
contributor: "graphify"
outcome: "useful"
source_nodes: ["src/nxp_rs_moveit_config/launch/hardware_moveit.launch.py", "src/nxp_rs_moveit_config/launch/demo.launch.py", "src/rs_control/launch/rs_bringup.launch.py", "rs_control/RobstrideSystem", "RobstrideSystem::on_activate", "src/rs_control/config/robstride.yaml", "src/rs_control/config/ros2_controllers.yaml", "src/nxp_rs_moveit_config/config/nxp_rs.srdf"]
---

# Q: Audit all requirements and setup before final MoveIt deployment to the real robot

## Answer

Static software checks pass: build, URDF parsing, joint/SRDF/controller consistency, rs_control tests, and simulation MoveIt plan-only smoke test. Deployment is not yet go: live can0 is ERROR-PASSIVE with prior bus-off/error-pass events, and the live robstride_system component reports unconfigured with unavailable interfaces despite controllers appearing active. The current rs_bringup writable process must be stopped before launching hardware_moveit. Also verify persistent CAN bring-up, rosdep dependencies, direct robot_description deprecation, hardware launch shutdown behavior, open_loop_control, trajectory-end velocity policy, limits/calibration/safety, and collision-scene requirements.

## Outcome

- Signal: useful

## Source Nodes

- src/nxp_rs_moveit_config/launch/hardware_moveit.launch.py
- src/nxp_rs_moveit_config/launch/demo.launch.py
- src/rs_control/launch/rs_bringup.launch.py
- rs_control/RobstrideSystem
- RobstrideSystem::on_activate
- src/rs_control/config/robstride.yaml
- src/rs_control/config/ros2_controllers.yaml
- src/nxp_rs_moveit_config/config/nxp_rs.srdf
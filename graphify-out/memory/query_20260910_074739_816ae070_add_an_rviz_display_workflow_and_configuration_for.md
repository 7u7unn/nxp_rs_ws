---
type: "query"
date: "2026-09-10T07:47:39.535506+00:00"
question: "Add an RViz display workflow and configuration for physical command-line jogging"
contributor: "graphify"
outcome: "useful"
source_nodes: ["src/rs_control/README.md", "src/rs_control/launch/rs_jog_rviz.launch.py", "src/rs_control/rviz/jog.rviz", "src/rs_control/launch/rs_bringup.launch.py", "src/nxp_rs_description/launch/display.launch.py"]
---

# Q: Add an RViz display workflow and configuration for physical command-line jogging

## Answer

The project graph linked rs_bringup.launch.py, robot_state_publisher, /joint_states, /robot_description, base_link, and the existing standalone display launch. Added rs_jog_rviz.launch.py to start only RViz, added rs_control/rviz/jog.rviz with RobotModel, TF, and Grid, installed rviz assets from CMake, declared rviz2, and documented the third-terminal workflow and why display.launch.py is unsuitable for hardware jogging.

## Outcome

- Signal: useful

## Source Nodes

- src/rs_control/README.md
- src/rs_control/launch/rs_jog_rviz.launch.py
- src/rs_control/rviz/jog.rviz
- src/rs_control/launch/rs_bringup.launch.py
- src/nxp_rs_description/launch/display.launch.py
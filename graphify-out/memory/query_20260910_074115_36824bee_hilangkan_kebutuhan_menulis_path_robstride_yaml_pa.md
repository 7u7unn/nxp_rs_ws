---
type: "query"
date: "2026-09-10T07:41:15.573920+00:00"
question: "Hilangkan kebutuhan menulis path robstride.yaml pada command launch bringup dan MoveIt"
contributor: "graphify"
outcome: "useful"
source_nodes: ["rs_control/README.md", "rs_bringup.launch.py", "Encoder-only bringup", "nxp_rs_moveit_config/README.md"]
---

# Q: Hilangkan kebutuhan menulis path robstride.yaml pada command launch bringup dan MoveIt

## Answer

Expanded from original query via graph vocabulary: [config, launch, bringup, encoder, hardware, control, package, path, robot, source]. The hardware bringup launch files already default config_file to FindPackageShare(rs_control)/config/robstride.yaml, and hardware_moveit.launch.py does the same. Removed explicit config_file paths from the default commands in src/rs_control/README.md and src/nxp_rs_moveit_config/README.md. Added documentation that config_file is optional and only needed for a deployment or calibration YAML. Kept the launch override available.

## Outcome

- Signal: useful

## Source Nodes

- rs_control/README.md
- rs_bringup.launch.py
- Encoder-only bringup
- nxp_rs_moveit_config/README.md
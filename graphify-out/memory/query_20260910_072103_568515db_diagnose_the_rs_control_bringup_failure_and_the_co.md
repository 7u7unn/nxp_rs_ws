---
type: "query"
date: "2026-09-10T07:21:03.892676+00:00"
question: "Diagnose the rs_control bringup failure and the controller argument spelling"
contributor: "graphify"
outcome: "useful"
source_nodes: ["RobstrideSystem::on_activate()", "RobstrideBus::receive_status()", "rs_control/README.md"]
---

# Q: Diagnose the rs_control bringup failure and the controller argument spelling

## Answer

Expanded from original query via graph vocabulary: [robstride, can, hardware, activate, initial, status, frame, socket, device, motor, joint, control]. The bringup fails because can0 is present but DOWN/STOPPED. RobstrideSystem on_activate performs an initial status read when read_only is false; RobstrideBus writes the request through SocketCAN and the kernel returns Network is down. Bring can0 up at the configured 1000000 bitrate, verify the motors respond with the read-only rs_scan tool, then relaunch. The launch argument is enable_control, not eneble_control; the typo leaves arm_controller and gripper_controller disabled, as shown by only the joint-state spawner in the log. No source change is required.

## Outcome

- Signal: useful

## Source Nodes

- RobstrideSystem::on_activate()
- RobstrideBus::receive_status()
- rs_control/README.md
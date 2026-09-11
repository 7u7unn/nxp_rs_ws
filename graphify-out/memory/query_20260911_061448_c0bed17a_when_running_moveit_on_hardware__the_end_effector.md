---
type: "query"
date: "2026-09-11T06:14:48.291174+00:00"
question: "When running MoveIt on hardware, the end effector is shaky despite reaching the goal. Could Kp/Kd cause it?"
contributor: "graphify"
outcome: "useful"
source_nodes: ["RobstrideSystem::io_loop()", "RobstrideSystem::export_command_interfaces()"]
---

# Q: When running MoveIt on hardware, the end effector is shaky despite reaching the goal. Could Kp/Kd cause it?

## Answer

Expanded graph vocabulary: robstride system motor command position velocity control. Verified current source after traversal: all seven motors use kp=500 and kd=5 in src/rs_control/config/robstride.yaml; the working-tree diff changes these from 10/1. Both gains are at driver and official RobStride Python_Sample table maxima. YAML validation rejects higher gains. RobstrideSystem::io_loop sends position, zero velocity, zero torque feedforward at nominal 100 Hz through sequential CAN transactions. Maximum stiffness is a plausible vibration contributor; missing velocity feedforward and asynchronous reference timing can contribute during travel. No current hardware processes or oscillation recording were available, so physical cause remains unconfirmed. Hardware launch omits hardware_execution.yaml and base JTC YAML has no position tolerances. Installed JTC 2.53.1 forwards position commands and does not compute velocity error for this position-only interface. Success therefore is not evidence of settling. RS05 protocol ranges match official SDK; a suspected scaling mismatch was not established. No robot configuration or gains changed. Next diagnostic is desired/actual position and velocity plus joint effort during travel and a stationary hold, before per-joint tuning.

## Outcome

- Signal: useful

## Source Nodes

- RobstrideSystem::io_loop()
- RobstrideSystem::export_command_interfaces()
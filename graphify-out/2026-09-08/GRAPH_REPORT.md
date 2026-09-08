# Graph Report - nxp_rs_ws  (2026-09-08)

## Corpus Check
- 30 files · ~20,853 words
- Verdict: corpus is large enough that graph structure adds value.

## Summary
- 299 nodes · 443 edges · 26 communities (16 shown, 10 thin omitted)
- Extraction: 85% EXTRACTED · 15% INFERRED · 0% AMBIGUOUS · INFERRED: 66 edges (avg confidence: 0.85)
- Token cost: 0 input · 0 output

## Graph Freshness
- Built from commit: `4684e6ba`
- Run `git rev-parse HEAD` and compare to check if the graph is stale.
- Run `graphify update .` after code changes (no API cost).

## Community Hubs (Navigation)
- display.launch.py
- check_inertia.py
- gazebo_inertia.launch.py
- Graphify Trigger
- ament_cmake dependency
- Gazebo inertia inspection
- robstride_bus.cpp
- RobstrideSystem
- MotorConfig
- robstride_system.cpp
- robstride_bus.hpp
- RobstrideBus
- EncoderReaderNode
- graphify reference: extra exports and benchmark
- OperationCommand
- graphify reference: query, path, explain
- rs_control
- graphify reference: add a URL and watch a folder
- graphify reference: commit hook and native CLAUDE.md integration
- graphify reference: incremental update and cluster-only
- graphify reference: GitHub clone and cross-repo merge
- graphify reference: transcribe video and audio
- rs_bringup.launch.py
- rs_encoder_bringup.launch.py
- CLAUDE.md
- extraction-spec.md

## God Nodes (most connected - your core abstractions)
1. `RobstrideSystem` - 43 edges
2. `RobstrideBus` - 22 edges
3. `MotorConfig` - 17 edges
4. `MotorState` - 13 edges
5. `EncoderReaderNode` - 13 edges
6. `TEST()` - 13 edges
7. `RobotConfig` - 12 edges
8. `BusConfig` - 10 edges
9. `TEST()` - 10 edges
10. `RobstrideBus::send_operation_command()` - 9 edges

## Surprising Connections (you probably didn't know these)
- `RobstrideBus::connect()` --calls--> `disconnect`  [INFERRED]
  src/rs_control/src/robstride_bus.cpp → src/rs_control/include/rs_control/robstride_bus.hpp
- `RobstrideBus::scan()` --calls--> `ping`  [INFERRED]
  src/rs_control/src/robstride_bus.cpp → src/rs_control/include/rs_control/robstride_bus.hpp
- `RobstrideBus::read_encoder()` --calls--> `read_parameter`  [INFERRED]
  src/rs_control/src/robstride_bus.cpp → src/rs_control/include/rs_control/robstride_bus.hpp
- `TEST()` --calls--> `on_init`  [INFERRED]
  src/rs_control/test/test_hardware_plugin.cpp → src/rs_control/include/rs_control/robstride_system.hpp
- `TEST()` --calls--> `export_state_interfaces`  [INFERRED]
  src/rs_control/test/test_hardware_plugin.cpp → src/rs_control/include/rs_control/robstride_system.hpp

## Import Cycles
- None detected.

## Communities (26 total, 10 thin omitted)

### Community 0 - "display.launch.py"
Cohesion: 0.50
Nodes (3): generate_launch_description(), Display the current robot description and its TF frames in RViz., Launch the robot model, joint sliders, and RViz.

### Community 1 - "check_inertia.py"
Cohesion: 0.67
Nodes (3): main(), Check this robot's CAD-to-URDF inertia transfer and Gazebo SDF conversion. Run…, tensor()

### Community 6 - "robstride_bus.cpp"
Cohesion: 0.09
Nodes (42): ComposesRobStrideExtendedCanId, EncodesAndDecodesUnsignedParameters, ProtocolTest, MotorState, position_rad, status_flags, temperature_c, torque_nm (+34 more)

### Community 7 - "RobstrideSystem"
Cohesion: 0.06
Nodes (41): atomic, InitializesAndExportsConfiguredInterfaces, mutex, RobstrideSystemTest, string, unique_ptr, vector, RobstrideSystem (+33 more)

### Community 8 - "MotorConfig"
Cohesion: 0.10
Nodes (29): ConvertsPositionVelocityAndEffort, Duration, MotorConfigTest, RejectsDuplicateCanIds, RejectsUnassignedCanId, return_type, MotorModel, MotorConfig (+21 more)

### Community 9 - "robstride_system.cpp"
Cohesion: 0.10
Nodes (28): CallbackReturn, CommandInterface, disable_all_motors, load_hardware_config, read_all_initial_states, start_io_thread, stop_io_thread, validate_hardware_joints (+20 more)

### Community 10 - "robstride_bus.hpp"
Cohesion: 0.13
Nodes (18): array, optional, vector, DiscoveredMotor, id, uuid, string, MotorLimits (+10 more)

### Community 11 - "RobstrideBus"
Cohesion: 0.08
Nodes (25): milliseconds, BusConfig, bitrate, host_id, interface_name, response_timeout, ReceivedFrame, communication_type (+17 more)

### Community 12 - "EncoderReaderNode"
Cohesion: 0.09
Nodes (24): DiagnosticStatus, SharedPtr, RobotConfig, bus, motors, poll_rate_hz, read_only, Node (+16 more)

### Community 13 - "graphify reference: extra exports and benchmark"
Cohesion: 0.22
Nodes (8): graphify reference: extra exports and benchmark, Step 6b - Wiki (only if --wiki flag), Step 7 - Neo4j export (only if --neo4j or --neo4j-push flag), Step 7a - FalkorDB export (only if --falkordb or --falkordb-push flag), Step 7b - SVG export (only if --svg flag), Step 7c - GraphML export (only if --graphml flag), Step 7d - MCP server (only if --mcp flag), Step 8 - Token reduction benchmark (only if total_words > 5000)

### Community 14 - "OperationCommand"
Cohesion: 0.33
Nodes (6): OperationCommand, kd, kp, position_rad, torque_nm, velocity_rad_s

### Community 15 - "graphify reference: query, path, explain"
Cohesion: 0.33
Nodes (5): For /graphify explain, For /graphify path, graphify reference: query, path, explain, Step 0 — Constrained query expansion (REQUIRED before traversal), Step 1 — Traversal

### Community 16 - "rs_control"
Cohesion: 0.33
Nodes (5): Build and test, Configure the physical robot, Encoder-only bringup, ros2_control / MoveIt / RL boundary, rs_control

### Community 17 - "graphify reference: add a URL and watch a folder"
Cohesion: 0.50
Nodes (3): For /graphify add, For --watch, graphify reference: add a URL and watch a folder

### Community 18 - "graphify reference: commit hook and native CLAUDE.md integration"
Cohesion: 0.50
Nodes (3): For git commit hook, For native CLAUDE.md integration, graphify reference: commit hook and native CLAUDE.md integration

### Community 19 - "graphify reference: incremental update and cluster-only"
Cohesion: 0.50
Nodes (3): For --cluster-only, For --update (incremental re-extraction), graphify reference: incremental update and cluster-only

## Knowledge Gaps
- **103 isolated node(s):** `bus`, `read_only`, `poll_rate_hz`, `motors`, `position_rad` (+98 more)
  These have ≤1 connection - possible missing edges or undocumented components. (Counts symbols only; 161 node(s) total have ≤1 connection when file, concept and rationale nodes are included.)
- **10 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `RobstrideSystem` connect `RobstrideSystem` to `robstride_bus.cpp`, `MotorConfig`, `robstride_system.cpp`, `RobstrideBus`, `EncoderReaderNode`?**
  _High betweenness centrality (0.227) - this node is a cross-community bridge._
- **Why does `RobstrideBus` connect `RobstrideBus` to `robstride_bus.hpp`, `EncoderReaderNode`, `robstride_bus.cpp`, `RobstrideSystem`?**
  _High betweenness centrality (0.131) - this node is a cross-community bridge._
- **Why does `RobotConfig` connect `EncoderReaderNode` to `MotorConfig`, `robstride_bus.hpp`, `RobstrideBus`, `RobstrideSystem`?**
  _High betweenness centrality (0.091) - this node is a cross-community bridge._
- **What connects `bus`, `read_only`, `poll_rate_hz` to the rest of the system?**
  _103 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `robstride_bus.cpp` be split into smaller, more focused modules?**
  _Cohesion score 0.08888888888888889 - nodes in this community are weakly interconnected._
- **Should `RobstrideSystem` be split into smaller, more focused modules?**
  _Cohesion score 0.05813953488372093 - nodes in this community are weakly interconnected._
- **Should `MotorConfig` be split into smaller, more focused modules?**
  _Cohesion score 0.0989247311827957 - nodes in this community are weakly interconnected._
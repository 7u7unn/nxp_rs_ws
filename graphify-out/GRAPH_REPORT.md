# Graph Report - nxp_rs_ws  (2026-09-08)

## Corpus Check
- 32 files · ~21,461 words
- Verdict: corpus is large enough that graph structure adds value.

## Summary
- 306 nodes · 448 edges · 28 communities (17 shown, 11 thin omitted)
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
- RobotConfig
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
- nxp_rs_moveit_config
- demo.launch.py

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
- `main()` --references--> `DiscoveredMotor`  [INFERRED]
  src/rs_control/src/rs_scan.cpp → src/rs_control/include/rs_control/robstride_bus.hpp
- `RobstrideBus::connect()` --calls--> `disconnect`  [INFERRED]
  src/rs_control/src/robstride_bus.cpp → src/rs_control/include/rs_control/robstride_bus.hpp
- `RobstrideBus::scan()` --calls--> `ping`  [INFERRED]
  src/rs_control/src/robstride_bus.cpp → src/rs_control/include/rs_control/robstride_bus.hpp
- `TEST()` --calls--> `on_init`  [INFERRED]
  src/rs_control/test/test_hardware_plugin.cpp → src/rs_control/include/rs_control/robstride_system.hpp
- `TEST()` --calls--> `export_state_interfaces`  [INFERRED]
  src/rs_control/test/test_hardware_plugin.cpp → src/rs_control/include/rs_control/robstride_system.hpp

## Import Cycles
- None detected.

## Communities (28 total, 11 thin omitted)

### Community 0 - "display.launch.py"
Cohesion: 0.50
Nodes (3): generate_launch_description(), Display the current robot description and its TF frames in RViz., Launch the robot model, joint sliders, and RViz.

### Community 1 - "check_inertia.py"
Cohesion: 0.67
Nodes (3): main(), Check this robot's CAD-to-URDF inertia transfer and Gazebo SDF conversion. Run…, tensor()

### Community 6 - "robstride_bus.cpp"
Cohesion: 0.08
Nodes (44): ComposesRobStrideExtendedCanId, EncodesAndDecodesUnsignedParameters, optional, ProtocolTest, MotorState, position_rad, status_flags, temperature_c (+36 more)

### Community 7 - "RobstrideSystem"
Cohesion: 0.06
Nodes (41): atomic, InitializesAndExportsConfiguredInterfaces, mutex, RobstrideSystemTest, string, unique_ptr, vector, RobstrideSystem (+33 more)

### Community 8 - "MotorConfig"
Cohesion: 0.09
Nodes (31): ConvertsPositionVelocityAndEffort, MotorConfigTest, RejectsDuplicateCanIds, RejectsUnassignedCanId, MotorModel, MotorConfig, direction, id (+23 more)

### Community 9 - "robstride_system.cpp"
Cohesion: 0.09
Nodes (33): CallbackReturn, CommandInterface, Duration, return_type, disable_all_motors, load_hardware_config, read_all_initial_states, start_io_thread (+25 more)

### Community 10 - "robstride_bus.hpp"
Cohesion: 0.09
Nodes (22): array, DiscoveredMotor, id, uuid, MotorLimits, kd, kp, position_rad (+14 more)

### Community 11 - "RobstrideBus"
Cohesion: 0.11
Nodes (18): milliseconds, BusConfig, bitrate, host_id, interface_name, response_timeout, RobstrideBus, config_ (+10 more)

### Community 12 - "EncoderReaderNode"
Cohesion: 0.13
Nodes (12): DiagnosticStatus, SharedPtr, add_diagnostic_value(), Node, string, unique_ptr, EncoderReaderNode, bus_ (+4 more)

### Community 13 - "graphify reference: extra exports and benchmark"
Cohesion: 0.22
Nodes (8): graphify reference: extra exports and benchmark, Step 6b - Wiki (only if --wiki flag), Step 7 - Neo4j export (only if --neo4j or --neo4j-push flag), Step 7a - FalkorDB export (only if --falkordb or --falkordb-push flag), Step 7b - SVG export (only if --svg flag), Step 7c - GraphML export (only if --graphml flag), Step 7d - MCP server (only if --mcp flag), Step 8 - Token reduction benchmark (only if total_words > 5000)

### Community 14 - "RobotConfig"
Cohesion: 0.19
Nodes (12): vector, RobotConfig, bus, motors, poll_rate_hz, read_only, string, string (+4 more)

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

### Community 26 - "nxp_rs_moveit_config"
Cohesion: 0.50
Nodes (3): Install and build, nxp_rs_moveit_config, Run the simulation-only MoveIt demo

## Knowledge Gaps
- **105 isolated node(s):** `bus`, `read_only`, `poll_rate_hz`, `motors`, `position_rad` (+100 more)
  These have ≤1 connection - possible missing edges or undocumented components. (Counts symbols only; 166 node(s) total have ≤1 connection when file, concept and rationale nodes are included.)
- **11 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `RobstrideSystem` connect `RobstrideSystem` to `robstride_bus.cpp`, `MotorConfig`, `robstride_system.cpp`, `RobstrideBus`, `RobotConfig`?**
  _High betweenness centrality (0.217) - this node is a cross-community bridge._
- **Why does `RobstrideBus` connect `RobstrideBus` to `robstride_bus.hpp`, `EncoderReaderNode`, `robstride_bus.cpp`, `RobstrideSystem`?**
  _High betweenness centrality (0.125) - this node is a cross-community bridge._
- **Why does `RobotConfig` connect `RobotConfig` to `MotorConfig`, `RobstrideBus`, `EncoderReaderNode`, `RobstrideSystem`?**
  _High betweenness centrality (0.087) - this node is a cross-community bridge._
- **What connects `bus`, `read_only`, `poll_rate_hz` to the rest of the system?**
  _105 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `robstride_bus.cpp` be split into smaller, more focused modules?**
  _Cohesion score 0.08333333333333333 - nodes in this community are weakly interconnected._
- **Should `RobstrideSystem` be split into smaller, more focused modules?**
  _Cohesion score 0.05813953488372093 - nodes in this community are weakly interconnected._
- **Should `MotorConfig` be split into smaller, more focused modules?**
  _Cohesion score 0.0946969696969697 - nodes in this community are weakly interconnected._
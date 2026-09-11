# Graph Report - nxp_rs_ws  (2026-09-10)

## Corpus Check
- 63 files · ~41,534 words
- Verdict: corpus is large enough that graph structure adds value.

## Summary
- 742 nodes · 1225 edges · 60 communities (45 shown, 15 thin omitted)
- Extraction: 89% EXTRACTED · 11% INFERRED · 0% AMBIGUOUS · INFERRED: 137 edges (avg confidence: 0.87)
- Token cost: 0 input · 0 output

## Graph Freshness
- Built from commit: `4898d9b4`
- Run `git rev-parse HEAD` and compare to check if the graph is stale.
- Run `graphify update .` after code changes (no API cost).

## Community Hubs (Navigation)
- display.launch.py
- robstride_bus.hpp
- gazebo_inertia.launch.py
- Graphify Trigger
- ament_cmake dependency
- Gazebo inertia inspection
- robstride_bus.cpp
- RobstrideSystem
- MotorConfig
- robstride_system.cpp
- MotorLimits
- BusConfig
- TEST
- graphify reference: extra exports and benchmark
- rs_scan.cpp
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
- Move the physical robot from MoveIt
- demo.launch.py
- bus.py
- RobstrideBus
- PythonRobstrideNode
- JogTargetState
- JogControllerNode
- JogMotionProfile
- RobstrideSystem::on_activate
- TEST
- Findings, in priority order
- JointLimit
- ProtocolFrame
- MotorState
- rs_jog_controller.py
- OperationCommand
- ReceivedFrame
- Q: Diagnose the rs_control bringup failure and the controller argument spelling
- Q: Why did can0 need setup again, and can udev make the SocketCAN setup persistent?
- Q: Rewrite rs_control README to clearly document the command line for jogging
- Q: Hilangkan kebutuhan menulis path robstride.yaml pada command launch bringup dan MoveIt
- Q: Add an RViz display workflow and configuration for physical command-line jogging
- Q: Audit all requirements and setup before final MoveIt deployment to the real robot
- Q: Diagnose failed loading joint_state_broadcaster when launching hardware MoveIt read_only
- HardwareExecutionTest
- hardware_moveit.launch.py
- rs_jog.launch.py
- rs_jog_rviz.launch.py
- rs_python_bringup.launch.py
- TEST
- rs_encoder_reader.cpp
- Q: Diagnose MoveIt plan-and-execute failure from base_link/right-finger collision
- Q: Diagnose MoveIt execute rejection: start point deviates from current robot state
- Q: Investigate why first MoveIt trajectory succeeds but the next plan starts in collision

## God Nodes (most connected - your core abstractions)
1. `RobstrideSystem` - 44 edges
2. `RobstrideBus` - 41 edges
3. `HardwareExecutionTest` - 26 edges
4. `JogMotionProfile` - 22 edges
5. `PythonRobstrideNode` - 22 edges
6. `RobstrideBus` - 21 edges
7. `JogControllerNode` - 20 edges
8. `TEST()` - 20 edges
9. `RobstrideError` - 18 edges
10. `Motor` - 17 edges

## Surprising Connections (you probably didn't know these)
- `RobstrideBus::scan()` --calls--> `ping`  [INFERRED]
  src/rs_control/src/robstride_bus.cpp → src/rs_control/include/rs_control/robstride_bus.hpp
- `TEST()` --calls--> `on_init`  [INFERRED]
  src/rs_control/test/test_hardware_plugin.cpp → src/rs_control/include/rs_control/robstride_system.hpp
- `TEST()` --calls--> `export_state_interfaces`  [INFERRED]
  src/rs_control/test/test_hardware_plugin.cpp → src/rs_control/include/rs_control/robstride_system.hpp
- `TEST()` --calls--> `export_command_interfaces`  [INFERRED]
  src/rs_control/test/test_hardware_plugin.cpp → src/rs_control/include/rs_control/robstride_system.hpp
- `RobstrideBus::receive_frame()` --calls--> `read`  [INFERRED]
  src/rs_control/src/robstride_bus.cpp → src/rs_control/include/rs_control/robstride_system.hpp

## Import Cycles
- None detected.

## Communities (60 total, 15 thin omitted)

### Community 0 - "display.launch.py"
Cohesion: 0.50
Nodes (3): generate_launch_description(), Display the current robot description and its TF frames in RViz., Launch the robot model, joint sliders, and RViz.

### Community 1 - "robstride_bus.hpp"
Cohesion: 0.47
Nodes (3): optional, vector, string

### Community 6 - "robstride_bus.cpp"
Cohesion: 0.06
Nodes (74): BuildsExactFeedbackAndParameterFrames, BuildsExactOperationControlFrame, BuildsExactTorqueControlFrames, ComposesRobStrideExtendedCanId, DescribesStatusAndFaultBits, EncodesAndDecodesUnsignedParameters, N, ProtocolTest (+66 more)

### Community 7 - "RobstrideSystem"
Cohesion: 0.07
Nodes (31): atomic, mutex, MotorState, RobotConfig, RobstrideBus, string, unique_ptr, vector (+23 more)

### Community 8 - "MotorConfig"
Cohesion: 0.15
Nodes (13): MotorModel, MotorConfig, direction, has_position_limits, id, joint_name, kd, kp (+5 more)

### Community 9 - "robstride_system.cpp"
Cohesion: 0.18
Nodes (15): CommandInterface, InterfaceInfo, StateInterface, string, T, vector, has_interface(), parse_bool() (+7 more)

### Community 10 - "MotorLimits"
Cohesion: 0.33
Nodes (6): MotorLimits, kd, kp, position_rad, torque_nm, velocity_rad_s

### Community 11 - "BusConfig"
Cohesion: 0.17
Nodes (12): milliseconds, MotorConfig, RobotConfig, bus, motors, poll_rate_hz, read_only, BusConfig (+4 more)

### Community 12 - "TEST"
Cohesion: 0.07
Nodes (39): ConvertsPositionVelocityAndEffort, Duration, MapsAndClampsFingerMotorLimits, MotorConfigTest, RejectsDuplicateCanIds, RejectsInvalidMotorPositionLimits, RejectsMotorIdEqualToHostId, RejectsUnassignedCanId (+31 more)

### Community 13 - "graphify reference: extra exports and benchmark"
Cohesion: 0.22
Nodes (8): graphify reference: extra exports and benchmark, Step 6b - Wiki (only if --wiki flag), Step 7 - Neo4j export (only if --neo4j or --neo4j-push flag), Step 7a - FalkorDB export (only if --falkordb or --falkordb-push flag), Step 7b - SVG export (only if --svg flag), Step 7c - GraphML export (only if --graphml flag), Step 7d - MCP server (only if --mcp flag), Step 8 - Token reduction benchmark (only if total_words > 5000)

### Community 14 - "rs_scan.cpp"
Cohesion: 0.60
Nodes (5): string, main(), parse_device_id(), print_usage(), read_option()

### Community 15 - "graphify reference: query, path, explain"
Cohesion: 0.33
Nodes (5): For /graphify explain, For /graphify path, graphify reference: query, path, explain, Step 0 — Constrained query expansion (REQUIRED before traversal), Step 1 — Traversal

### Community 16 - "rs_control"
Cohesion: 0.15
Nodes (12): Build and test, Command-line jogging, Configure one motor CAN ID, Configure the physical robot, Display the physical robot in RViz, Encoder-only bringup, Jog keys, Python RobStride bringup (+4 more)

### Community 17 - "graphify reference: add a URL and watch a folder"
Cohesion: 0.50
Nodes (3): For /graphify add, For --watch, graphify reference: add a URL and watch a folder

### Community 18 - "graphify reference: commit hook and native CLAUDE.md integration"
Cohesion: 0.50
Nodes (3): For git commit hook, For native CLAUDE.md integration, graphify reference: commit hook and native CLAUDE.md integration

### Community 19 - "graphify reference: incremental update and cluster-only"
Cohesion: 0.50
Nodes (3): For --cluster-only, For --update (incremental re-extraction), graphify reference: incremental update and cluster-only

### Community 26 - "Move the physical robot from MoveIt"
Cohesion: 0.11
Nodes (17): Controller success was insufficiently checked, Findings, Hardware execution investigation — 2026-09-10, Independent gripper bounds issue, Simplified base geometry caused a CAD-relative false contact, Validation and deployment, 1. Stop other physical launch files, 2. Check the CAN interface (+9 more)

### Community 28 - "bus.py"
Cohesion: 0.07
Nodes (44): IntEnum, SocketCAN RobStride bus based on the official Python sample's API flow., CommunicationType, compose_extended_id(), decode_fault_report(), decode_operation_status(), decode_parameter_response(), decode_symmetric() (+36 more)

### Community 29 - "RobstrideBus"
Cohesion: 0.10
Nodes (22): DiscoveredMotor, Motor, MotorState, Receive one matching extended frame, ignoring unrelated traffic., Read safe state parameters without enabling or commanding torque., Raised for a failed or invalid RobStride transaction., Change one motor's ID using the reference type-7 transaction., One shared SocketCAN connection for any number of motors. Transactions are… (+14 more)

### Community 30 - "PythonRobstrideNode"
Cohesion: 0.08
Nodes (25): Any, _as_bool(), _as_finite(), ConfigurationError, load_robot_config(), MotorConfig, YAML configuration and joint/motor calibration for the Python driver., Load and validate the shared ``robstride.yaml`` format. (+17 more)

### Community 31 - "JogTargetState"
Cohesion: 0.10
Nodes (8): RuntimeError, JogTargetState, Track a complete command pose while changing one joint at a time., Return a copy so callers cannot alter the held pose accidentally., Capture a complete feedback pose, including positions at a stop., Set a target explicitly and clamp it to the configured joint limit., Apply one increment and return old value, new value, and clamp flag., JogTargetStateTest

### Community 32 - "JogControllerNode"
Cohesion: 0.21
Nodes (5): JointState, JointTrajectory, JogControllerNode, Node, Read feedback and publish bounded, continuous jog trajectories.

### Community 33 - "JogMotionProfile"
Cohesion: 0.12
Nodes (9): JogMotionProfile, Jerk-limited one-dimensional profile used by the interactive jogger. The…, Initialize a profile from a feedback sample., Set a bounded target and return ``(applied_target, was_clamped)``. The target…, Request a stop at the reachable braking point for current velocity., Return a profile copy for previewing a future state., Return the position and velocity after ``duration`` seconds., Advance the profile and return its new position and velocity. (+1 more)

### Community 34 - "RobstrideSystem::on_activate"
Cohesion: 0.15
Nodes (16): CallbackReturn, disable_all_motors, load_hardware_config, read_all_initial_states, report_io_error, start_io_thread, stop_io_thread, validate_hardware_joints (+8 more)

### Community 35 - "TEST"
Cohesion: 0.18
Nodes (14): InitializesAndExportsConfiguredInterfaces, RobstrideSystemTest, export_command_interfaces, export_state_interfaces, on_init, read, write, HardwareInfo (+6 more)

### Community 36 - "Findings, in priority order"
Cohesion: 0.14
Nodes (13): 1. Trajectory replacement can briefly remove holding torque, 2. Unchanged targets and keyboard repeat repeatedly restart trajectories, 3. Sparse, linear position references cannot provide smooth starts/stops, 4. Motor gains and feedforward need load-aware treatment, 5. Timing and motor fault visibility are incomplete, 6. Other issues to resolve before relying on MoveIt, Evidence from the last jog session, Findings, in priority order (+5 more)

### Community 38 - "JointLimit"
Cohesion: 0.14
Nodes (5): JointLimit, Pure target bookkeeping for the interactive RobStride jogger., Allowed command interval in the ROS joint coordinate., JogMotionProfileTest, No-ROS tests for complete-pose jog target bookkeeping.

### Community 39 - "ProtocolFrame"
Cohesion: 0.50
Nodes (4): ProtocolFrame, data, id, length

### Community 40 - "MotorState"
Cohesion: 0.22
Nodes (9): MotorState, fault_code, position_rad, status_flags, temperature_c, torque_nm, valid, velocity_rad_s (+1 more)

### Community 41 - "rs_jog_controller.py"
Cohesion: 0.24
Nodes (8): Path, main(), Check this robot's CAD-to-URDF inertia transfer and Gazebo SDF conversion. Run…, tensor(), _default_limits_file(), _load_limits(), main(), Interactive single-joint position jogger for the physical ROS 2 controllers.

### Community 42 - "OperationCommand"
Cohesion: 0.33
Nodes (6): OperationCommand, kd, kp, position_rad, torque_nm, velocity_rad_s

### Community 43 - "ReceivedFrame"
Cohesion: 0.18
Nodes (10): DiscoveredMotor, id, uuid, array, ReceivedFrame, communication_type, data, extra_data (+2 more)

### Community 44 - "Q: Diagnose the rs_control bringup failure and the controller argument spelling"
Cohesion: 0.40
Nodes (4): Answer, Outcome, Q: Diagnose the rs_control bringup failure and the controller argument spelling, Source Nodes

### Community 45 - "Q: Why did can0 need setup again, and can udev make the SocketCAN setup persistent?"
Cohesion: 0.40
Nodes (4): Answer, Outcome, Q: Why did can0 need setup again, and can udev make the SocketCAN setup persistent?, Source Nodes

### Community 46 - "Q: Rewrite rs_control README to clearly document the command line for jogging"
Cohesion: 0.40
Nodes (4): Answer, Outcome, Q: Rewrite rs_control README to clearly document the command line for jogging, Source Nodes

### Community 47 - "Q: Hilangkan kebutuhan menulis path robstride.yaml pada command launch bringup dan MoveIt"
Cohesion: 0.40
Nodes (4): Answer, Outcome, Q: Hilangkan kebutuhan menulis path robstride.yaml pada command launch bringup dan MoveIt, Source Nodes

### Community 48 - "Q: Add an RViz display workflow and configuration for physical command-line jogging"
Cohesion: 0.40
Nodes (4): Answer, Outcome, Q: Add an RViz display workflow and configuration for physical command-line jogging, Source Nodes

### Community 49 - "Q: Audit all requirements and setup before final MoveIt deployment to the real robot"
Cohesion: 0.40
Nodes (4): Answer, Outcome, Q: Audit all requirements and setup before final MoveIt deployment to the real robot, Source Nodes

### Community 50 - "Q: Diagnose failed loading joint_state_broadcaster when launching hardware MoveIt read_only"
Cohesion: 0.40
Nodes (4): Answer, Outcome, Q: Diagnose failed loading joint_state_broadcaster when launching hardware MoveIt read_only, Source Nodes

### Community 51 - "HardwareExecutionTest"
Cohesion: 0.07
Nodes (29): Future, Handle, JointTrajectoryController, NonzeroFinalVelocityRejected, RecordedShoulderAndElbowErrorsAbort, shared_ptr, SingleThreadedExecutor, SmallPersistentErrorFailsGoalDeadline (+21 more)

### Community 58 - "TEST"
Cohesion: 0.17
Nodes (14): CollisionGeometry, CollisionResult, GenuineBaseFingerIntersectionStillRejected, GraspFrameIsTheArmTipBetweenFingerRoots, PlanningScene, RecordedFingerBoundsViolationRemainsVisible, RecordedPoseReproducesLegacyMeshFalsePositive, check() (+6 more)

### Community 62 - "rs_encoder_reader.cpp"
Cohesion: 0.40
Nodes (3): DiagnosticStatus, add_diagnostic_value(), string

### Community 63 - "Q: Diagnose MoveIt plan-and-execute failure from base_link/right-finger collision"
Cohesion: 0.40
Nodes (4): Answer, Outcome, Q: Diagnose MoveIt plan-and-execute failure from base_link/right-finger collision, Source Nodes

### Community 64 - "Q: Diagnose MoveIt execute rejection: start point deviates from current robot state"
Cohesion: 0.40
Nodes (4): Answer, Outcome, Q: Diagnose MoveIt execute rejection: start point deviates from current robot state, Source Nodes

### Community 65 - "Q: Investigate why first MoveIt trajectory succeeds but the next plan starts in collision"
Cohesion: 0.40
Nodes (4): Answer, Outcome, Q: Investigate why first MoveIt trajectory succeeds but the next plan starts in collision, Source Nodes

## Knowledge Gaps
- **185 isolated node(s):** `command_`, `position_`, `velocity_`, `bias_`, `commands_` (+180 more)
  These have ≤1 connection - possible missing edges or undocumented components. (Counts symbols only; 391 node(s) total have ≤1 connection when file, concept and rationale nodes are included.)
- **15 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Work-memory lessons

**Preferred sources** — corroborated by past sessions; start here.
- `rs_control/README.md` (4× useful, score=3.997022706)
- `rs_bringup.launch.py` (2× useful, score=1.99874975) _(code changed — re-verify)_

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `RobstrideSystem::read()` connect `TEST` to `robstride_system.cpp`?**
  _High betweenness centrality (0.230) - this node is a cross-community bridge._
- **Why does `TEST()` connect `TEST` to `robstride_bus.hpp`, `robstride_bus.cpp`?**
  _High betweenness centrality (0.099) - this node is a cross-community bridge._
- **Why does `RobstrideSystem` connect `RobstrideSystem` to `RobstrideSystem::on_activate`, `TEST`?**
  _High betweenness centrality (0.076) - this node is a cross-community bridge._
- **Are the 7 inferred relationships involving `RobstrideBus` (e.g. with `CommunicationType` and `ParameterSpec`) actually correct?**
  _`RobstrideBus` has 7 INFERRED edges - model-reasoned connections that need verification._
- **Are the 25 inferred relationships involving `ValueError` (e.g. with `.__init__()` and `.receive()`) actually correct?**
  _`ValueError` has 25 INFERRED edges - model-reasoned connections that need verification._
- **Are the 2 inferred relationships involving `JogMotionProfile` (e.g. with `JogControllerNode` and `JogMotionProfileTest`) actually correct?**
  _`JogMotionProfile` has 2 INFERRED edges - model-reasoned connections that need verification._
- **What connects `command_`, `position_`, `velocity_` to the rest of the system?**
  _185 weakly-connected nodes found - possible documentation gaps or missing edges._
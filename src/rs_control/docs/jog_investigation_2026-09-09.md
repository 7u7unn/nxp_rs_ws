# Jogging investigation — 2026-09-09

The strongest explanation for the reported brief weakness after each jog is
trajectory replacement removing the position error that generates holding
torque. This mechanism was reproduced using the installed ROS Humble
`joint_trajectory_controller` 2.53.1 trajectory library. It remains a likely
physical cause, not a measured torque diagnosis: no synchronized joint-state,
controller-state, or CAN recording of the event was available.

This investigation changed no motor gains, controller configuration, or runtime
control code and sent no hardware commands. The existing graph was used for
orientation; recent uncommitted source and runtime logs supplied the current
evidence.

## Evidence from the last jog session

- `/home/jundi/.ros/log/ros2_control_node_86448_1788943420091.log`
  confirms position-only arm/gripper command interfaces, 20 Hz controller
  updates, spline interpolation, and failure to obtain FIFO scheduling.
  Activation completed at 16:43; shutdown was requested by SIGINT at 16:49.
- `/home/jundi/.ros/log/python3_86602_1788943449348.log` identifies the
  standalone jogger and the two JTC command topics. It records joint-4
  increments of 0.05 rad followed by repeated clamping at 2.066 rad.
  Repeated keys at the limit arrive 20–40 ms apart.
- The installed controller and motor YAML files resolve to the source files
  inspected here. No CAN timeout or fault is recorded in that controller log
  or its launch log; absence of a logged fault does not establish clean power,
  clean timing, or fault-free motor feedback.

## Findings, in priority order

### 1. Trajectory replacement can briefly remove holding torque

`python/rs_jog_controller.py:174` sends a single position endpoint 0.5 seconds
in the future. `config/ros2_controllers.yaml` does not set `open_loop_control`;
the installed parameter header defaults it to false.

In the [matching upstream controller source](https://github.com/ros-controls/ros2_controllers/blob/2.53.1/joint_trajectory_controller/src/joint_trajectory_controller.cpp#L187),
the first sample of each new trajectory starts from measured state under this
default; the alternative uses the last commanded state. Position commands are
forwarded to hardware. This position-only configuration does not use the
controller's PID adapter.

`src/robstride_system.cpp:406` sends operation mode commands with configured
Kp/Kd, zero velocity reference, and zero feedforward torque. Consequently, the
approximate external position-loop torque law is:

```text
torque = Kp * (position_reference - measured_position)
       + Kd * (0 - measured_velocity)
```

At rest, holding a load requires a position error when feedforward is zero.
Restarting from measured position erases that proportional contribution on the
first sample, then rebuilds it as the reference approaches the endpoint. A
loaded joint can sag or hesitate before recovering. A stationary unselected
joint can experience the same effect when its trajectory is replaced.

The complete chain is:

```text
key press → replace arm and gripper trajectories → start from feedback
          → position error collapses → holding contribution drops
          → reference ramps toward target → holding contribution returns
```

### 2. Unchanged targets and keyboard repeat repeatedly restart trajectories

`python/rs_jog_controller.py:193` republishes both groups for every jog.
`_jog()` publishes even when clamping leaves the target unchanged. Space also
republishes both trajectories. The six arm joints share one trajectory, so
even publishing only the arm group still requires continuity for its
unselected joints.

The observed key repeats can replace a 0.5-second trajectory faster than the
20 Hz controller can advance it. Persistent replacement can repeatedly reset
the reference close to feedback. After releasing the key, replacement stops
and the trajectory finally has time to complete. This closely matches the
reported delayed recovery.

Targets accumulate independently of actual motion. At an illustrative 30
repeats/s, a 0.05-rad increment advances the requested endpoint at 1.5 rad/s
until clamped. There is no bounded lead over actual position, acceleration
limit, or jerk limit in the jogger. The `h` key intentionally recaptures
feedback and can also discard the error supporting the load.

### 3. Sparse, linear position references cannot provide smooth starts/stops

The controller manager and CAN loop both default to 20 Hz. A nominal
0.5-second jog therefore has only about ten reference intervals, with an
additional independent CAN-loop phase delay and sequential motor transactions.
The motor's internal loop is faster; 20 Hz is the host reference rate, not
the motor's internal servo rate.

The jog endpoint contains no velocity or acceleration. Despite the logged
name `splines`, this input selects linear interpolation with abrupt reference
velocity changes at start/end. This is documented by the installed trajectory
header and the [ROS trajectory documentation](https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/trajectory.html).

Increasing only `poll_rate_hz` does not increase the controller's 20 Hz
reference-generation rate. The launch file also overrides the YAML poll rate
with its own default, so editing motor YAML alone is insufficient.

### 4. Motor gains and feedforward need load-aware treatment

All seven motors use Kp=10 and Kd=1 in `config/robstride.yaml`, including joints
with very different loads. There is no outer integral term or gravity
compensation in this path. At rest, an illustrative 1 Nm load needs 0.1 rad
(5.73 degrees) of error with Kp=10. Actual load and resulting error need
measurement. Kd with a zero velocity reference also resists desired motion;
increasing Kd indiscriminately can increase tracking lag.

The [manufacturer's operation example](https://github.com/RobStride/Python_Sample/blob/main/examples/operation_control.py)
uses velocity feedforward alongside the position reference. Carrying a
consistent desired velocity through this hardware interface is a possible
improvement, but must be coupled to a bounded motion profile. Adding arbitrary
velocity or torque feedforward would not be justified.

The configured RS-00/RS-05 position, velocity, torque, Kp, and Kd packing ranges
match the [manufacturer's scaling table](https://github.com/RobStride/Python_Sample/blob/main/robstride_dynamics/table.py).
The ±17 Nm range is protocol scaling, not proof of the installed motor's
continuous capability or an appropriate operating torque limit.

### 5. Timing and motor fault visibility are incomplete

`src/robstride_system.cpp:385` serializes seven send/response transactions.
Slow replies delay later motors. A 20 ms response timeout is not a mandatory
20 ms sleep per healthy response. A failed transaction disables motors and
stops the I/O loop; that path does not automatically recover after each jog,
so it is a weaker fit for a repeatable, self-recovering hesitation.

The FIFO scheduling warning is real evidence of missing realtime scheduling,
but not proof that scheduling jitter caused the symptom. Measure actual cycle
and per-motor intervals before selecting a higher rate.

`src/robstride_bus.cpp:370` records type-2 status fault bits, but the C++
hardware path neither exposes them as diagnostics nor rejects such feedback
solely because the bits are nonzero. It rejects an explicit type-21 fault
frame. Motor mode bits are discarded. Supply voltage, configured current/
torque limits, and motor communication timeout are not read in normal bringup.
The [manufacturer parameter definitions](https://github.com/RobStride/Python_Sample/blob/main/robstride_dynamics/protocol.py)
identify these settings, but their applicability and units need checking
against the actual firmware before changing anything.

### 6. Other issues to resolve before relying on MoveIt

- The standalone jogger clamps position only. It does not enforce MoveIt's
  velocity/acceleration settings, monitor following error, or reject jogging
  after joint feedback becomes stale. Position samples are accumulated
  without checking age or whether they came from one coherent snapshot.
- Space means resend the existing destination; it is not a controlled stop.
  Quitting leaves the hardware controller pursuing/holding its last target.
  Key repeat can leave substantial pending travel when the key is released.
- URDF effort limits of 1.0 are not enforced as motor torque caps by this
  position-only hardware implementation. They do not explain a momentary
  1 Nm cap. The finger effort conversion also omits inverse position scaling,
  so its reported effort is motor torque rather than the force conjugate to
  its metre-based position. This is separate from the arm hesitation.
- The alternative Python control node takes only the last trajectory point
  and ignores timing/derivatives (`python/rs_control_py/node.py:169`). It is
  not an interchangeable smooth trajectory executor. Logs identify the C++
  JTC path for the session inspected here.
- Mechanical stiction/backlash, brake behavior if fitted, payload, voltage
  droop, thermal/current limiting, and competing CAN writers remain physical
  possibilities. There is no measurement here to confirm or exclude them.

## Offline reproduction

A standalone C++ probe was compiled against the installed trajectory library.
It created no ROS nodes and accessed no CAN device. It samples both first-point
choices used by the controller, with measured position held at an illustrative
0.95 rad, previous reference 1.00 rad, Kp=10, and measured velocity zero.
This isolates interpolation; it is not a simulated or measured arm response.

For an unchanged endpoint of 1.00 rad:

| Time after replacement | P torque, measured-state start | P torque, previous-command start |
| --- | ---: | ---: |
| 0 ms | 0.00 Nm | 0.50 Nm |
| 50 ms | 0.05 Nm | 0.50 Nm |
| 100 ms | 0.10 Nm | 0.50 Nm |
| 250 ms | 0.25 Nm | 0.50 Nm |
| 500 ms | 0.50 Nm | 0.50 Nm |

The probe also checked an actual +0.05-rad jog: the first proportional torque
was again zero versus 0.50 Nm. Assertions passed for all four scenarios.
Temporary source, CMake project, and executable are in
`/tmp/rs-jog-investigation-btNMLt/` for this session.

## Proposed correction and validation order

1. Preserve the commanded reference through trajectory replacement. In this
   installed Humble version, `open_loop_control: true` under each JTC is the
   available mechanism. The motor PD loop remains active; controller source
   still reads hardware state and checks configured tracking tolerances.
   Test activation, replacement, reversal, tolerance failure, and restart
   behavior together. This option preserves continuity but does not solve
   weak static holding or unbounded target lead.
2. Suppress unchanged/clamped publications and leave the unrelated controller
   group running its existing trajectory. Replace keyboard-driven trajectory
   resets with a bounded profile that preserves position and velocity on
   retargeting, limits acceleration/jerk and target lead, and provides an
   explicit controlled stop. Simply adding zero derivatives to every new
   endpoint is insufficient to guarantee continuous repeated jogging.
3. Add feedback-age and following-error monitoring and make motor status,
   transaction latency, and loop overruns visible. Establish an appropriate
   supported-arm response to communication loss; disabling a gravity-loaded
   arm is not equivalent to holding it safely.
4. Measure CAN capacity and timing, then increase both controller and I/O rates
   together. A measured 100 Hz starting target is reasonable to evaluate;
   200 Hz is a later option if all seven motors and the adapter sustain it.
   These are trial rates, not verified settings for this machine. Keep
   diagnostic publication separate from the control rate.
5. After continuity/timing are established, tune Kp/Kd individually against
   measured load and tracking response, and evaluate consistent velocity
   feedforward. Validate mass/COM, joint directions, and payload before adding
   model-based gravity feedforward. Do not blindly raise all gains or add I.
6. Verify isolated positive/negative jogs, repeated keys, reversal, release,
   limit clamping, and unselected-joint holding at representative poses. Only
   move to MoveIt after this behavior is repeatable within agreed limits.

For the physical verification, record arm/gripper `controller_state`,
`joint_states`, incoming trajectories, and passive CAN timestamps/status.
Resolve the actual topic names first. Check that a command does not pull the
desired position back to measured position, holding effort remains continuous,
unselected joints stay within a chosen error bound, and command intervals have
no watchdog-threatening gaps. Observe motor supply voltage under load if torque
drops while reference/error remain continuous. Capture fault/mode bits and
actual limit settings as well as temperatures.

Smooth physical jogging is not yet certified by this investigation. The next
implementation can address the identified software mechanisms; physical
confirmation requires the above measurements with the arm supported and an
operator controlling motion.

## Follow-up joint-direction calibration

During RViz verification, the operator reported that joints 1, 2, 3, and 6
and the finger moved opposite to the URDF positive direction. The shared motor
configuration now uses `direction: -1` for those five actuators. Their
revolute limits in `joints.yaml`, the visual URDF, and MoveIt limits are
reflected to match the converted ROS coordinates. For the finger, the mapping
treats raw `0.954 rad` as open and `-0.04 rad` as closed so the ROS convention
remains `0.0 m` open and `0.0403 m` closed; this endpoint assignment is an
assumption from the direction report and must be verified physically.

This calibration must be checked with a small, unloaded jog after restarting
the bringup. If the finger's physical open/closed endpoint labels differ from
that observation, stop and correct only the finger offset/sign before using
MoveIt; do not bypass the configured limits.

## Follow-up activation log

The subsequent writable bringup attempt reached hardware activation but motor
1 returned a communication-type-21 fault report. This is an actuator
protection state, not a PID response. The hardware plugin now reports the raw
fault/warning payload, decodes the documented bits, labels the failing
activation step, rejects nonzero type-2 protection flags, and stops the launch
spawners when `ros2_control_node` exits. Jogging should remain disabled until
the reported physical fault is corrected and a read-only status check is clean.

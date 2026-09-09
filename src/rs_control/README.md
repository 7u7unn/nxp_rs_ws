# rs_control

`rs_control` is the RobStride hardware boundary for the NXP RS robot. It
contains a native C++ SocketCAN protocol layer, a ROS 2 encoder reader, and a
Humble `ros2_control` system plugin.

The configured physical topology is seven motors and eight URDF joints:

- RS-00: `joint-1` through `joint-4`.
- RS-05: `joint-5`, `joint-6`, and `joint_right-finger`.
- `joint_left-finger` is a URDF mimic of the one physical finger actuator and
  is reported with the opposite sign.

## Configure the physical robot

First inspect the bus without enabling a motor:

```bash
ros2 run rs_control rs_scan --interface can0
```

SocketCAN interfaces must already be configured by the operating system, for
example:

```bash
sudo ip link set can0 up type can bitrate 1000000
```

Copy the discovered IDs into
`src/rs_control/config/robstride.yaml` (or a deployment copy). Every motor ID
must be unique, non-zero, and different from `host_id`. Set `direction` and
`position_offset` after checking the physical zero and joint direction. A motor
may define `motor_position_min` and `motor_position_max`; outgoing position
commands are clamped to that range.

The direction-corrected finger mapping treats motor 7 position `0.954 rad`
(open) as `0.0 m`, and `-0.04 rad` (closed) as `0.0403 m`. Its scale is
`0.040543259557 m/rad`, with the open motor position as the offset. The
motor-side encoder zero datum was measured at `0.601 rad`; verify the endpoint
labels with a small, unloaded jog after changing direction calibration.

The default YAML has `read_only: true`. This is deliberate: it allows encoder
bringup and state observation while keeping motors disabled.

## Encoder-only bringup

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rs_control rs_encoder_bringup.launch.py \
  config_file:=/absolute/path/to/robstride.yaml
ros2 topic echo /joint_states
ros2 topic echo /diagnostics
```

The reader requests the RobStride operation-status frame and publishes
position, velocity, effort, and temperature through standard
`sensor_msgs/msg/JointState` plus diagnostics. It does not send enable,
disable, or operation-control frames.

If writable bringup reports `returned a fault report`, stop and resolve the
motor fault before retrying. The driver now prints the raw type-21 fault and
warning masks, names the documented protection bits, identifies the activation
step (`disable`, `set_run_mode`, or `enable`), and refuses to continue with
torque enabled. Capture the evidence without commanding motion:

```bash
candump -tz can0
ros2 launch rs_control rs_encoder_bringup.launch.py \
  config_file:=/absolute/path/to/robstride.yaml
ros2 topic echo /diagnostics
```

Typical causes are an uncalibrated encoder, stall/overload, over/undervoltage,
overtemperature, phase overcurrent, or a driver fault. Check the motor's
mechanics, supply voltage, wiring, and encoder calibration first; do not mask a
fault or raise gains to work around it. The bringup launch also shuts down its
controller spawners when `ros2_control_node` exits, so a failed activation no
longer leaves them waiting indefinitely.

## Python RobStride bringup

The package also contains a Python SocketCAN path based on the
[RobStride Python sample](https://github.com/RobStride/Python_Sample). Install
its runtime dependencies first:

```bash
sudo apt install python3-can python3-yaml
```

Start the safe, read-only Python node with the same seven-motor YAML:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rs_control rs_python_bringup.launch.py \
  config_file:=/absolute/path/to/robstride.yaml
```

In read-only mode it sends the reference sample's type-17 parameter-read
requests for position, velocity, and torque for every configured motor. This
does not enable torque and does not depend on a type-2 status frame being
periodically emitted. Each response is checked before it is assigned to a
motor, so a missing motor produces one diagnostic error without shifting the
remaining six readings.

After checking the IDs, directions, limits, and emergency-stop path, enable
operation control explicitly:

```bash
ros2 launch rs_control rs_python_bringup.launch.py \
  config_file:=/absolute/path/to/robstride.yaml \
  read_only:=false
```

Control mode accepts the latest position point from
`/rs_control/joint_trajectory` (`trajectory_msgs/msg/JointTrajectory`) and
uses one operation-control/status transaction per motor. The node publishes
the seven configured motors plus the URDF-mimic left finger on
`/joint_states`. To inspect IDs without ROS, use:

```bash
ros2 run rs_control rs_python_scan --interface can0 --start-id 1 --end-id 255
```

## Configure one motor CAN ID

Use the guarded utility while only one motor is connected or powered. The
first command scans the bus and prints the current ID and UUID:

```bash
ros2 run rs_control rs_python_set_id --interface can0
```

Preview an ID change without transmitting it:

```bash
ros2 run rs_control rs_python_set_id \
  --interface can0 --new-id 2
```

Apply and verify the type-7 ID change explicitly:

```bash
ros2 run rs_control rs_python_set_id \
  --interface can0 --new-id 2 --yes
```

When `--old-id` is omitted, the utility requires exactly one responding motor
before an ID change. `--old-id` can be used when several motors are connected.
`--new-id` is the desired ID. The utility refuses an already occupied new ID,
sends no enable or motion command, and pings the new ID after the change.
Power-cycle the motor and rescan afterward to confirm that the ID remains
configured.

## ros2_control / MoveIt / RL boundary

The standard bringup exports position commands and position, velocity, effort,
and temperature states through the `rs_control/RobstrideSystem` plugin. The standard
interfaces are:

- `/joint_states` from `joint_state_broadcaster`;
- `/controller_manager` services;
- `arm_controller` as a position `JointTrajectoryController` for joints 1--6
  when enabled;
- `gripper_controller` as a position `JointTrajectoryController` for the
  physical right finger when enabled.

Start read-only ros2_control state publication with:

```bash
ros2 launch rs_control rs_bringup.launch.py \
  config_file:=/absolute/path/to/robstride.yaml
```

Only after validating limits, directions, offsets, and the emergency stop
path, explicitly enable control:

```bash
ros2 launch rs_control rs_bringup.launch.py \
  config_file:=/absolute/path/to/robstride.yaml \
  read_only:=false enable_control:=true
```

MoveIt can target `arm_controller` and `gripper_controller` using its normal
`FollowJointTrajectory` configuration. An RL process can consume `/joint_states`
and send position commands through the controller or directly through a future
controller node;
the SocketCAN protocol and calibration conversions remain isolated in this
package.

## Standalone single-joint jogger

The jogger is independent of MoveIt. It waits for a complete `/joint_states`
message, captures the robot's current pose (so startup does not assume zero),
and then advances a bounded, jerk-limited profile at a fixed rate. A jog key
changes a target; it does not immediately replace a trajectory from measured
feedback. Each affected controller receives a two-point trajectory containing
the current planned position/velocity and a short future horizon. This keeps
trajectory replacement continuous and leaves the unrelated controller group
alone.

Stop the encoder reader and any MoveIt launch first. In one terminal, start the
physical controllers with explicit torque enablement:

```bash
source /opt/ros/humble/setup.bash
source /home/jundi/nxp_rs_ws/install/setup.bash
ros2 launch rs_control rs_bringup.launch.py \
  config_file:=/home/jundi/nxp_rs_ws/src/rs_control/config/robstride.yaml \
  can_interface:=can0 read_only:=false enable_control:=true
```

In a second interactive terminal, start the jogger:

```bash
source /opt/ros/humble/setup.bash
source /home/jundi/nxp_rs_ws/install/setup.bash
ros2 run rs_control rs_jog_controller
```

Keys are `1`--`6` to select an arm joint, `7` for the right finger, `+`/`=`
and `-` to jog, `[`/`]` to halve/double the step, space or `s` to request a
controlled stop, `h` to recapture feedback, `?` for help, and `q` to request a
controlled stop before exiting the jogger. Arm steps default to `0.05 rad`;
finger steps default to `0.001 m`.
The default profile limits are 0.4 rad/s, 0.8 rad/s², and 4 rad/s³ for arm
joints, and 0.02 m/s, 0.04 m/s², and 0.2 m/s³ for the finger. A command can
lead the current planned position by at most 0.25 rad (arm) or 0.01 m
(finger). These values are launch parameters and should be reduced for a
loaded or unfamiliar mechanism. The jogger keeps a small margin inside the
measured hard stops and clamps commands using `config/joints.yaml`.

The arm and gripper controllers set `open_loop_control: true` while jogging.
This makes a trajectory replacement start from the last commanded state,
preserving the position error that may be supporting a load. The controllers
still read hardware state and apply their tracking tolerances. Keep the
controller manager and CAN loop rates aligned with the jogger's
`command_rate_hz` (20 Hz by default); raise them together only after measuring
CAN transaction time and cycle jitter. If `/joint_states` stops arriving for
0.25 seconds, the jogger requests a controlled stop and waits for feedback to
become current before accepting further motion.

Check that `joint_state_broadcaster`, `arm_controller`, and
`gripper_controller` are all active before pressing `+` or `-`:

```bash
ros2 control list_controllers
```

The jogger itself does not disable torque when it exits. Press `Ctrl+C` in the
hardware bringup terminal to stop the hardware and disable all motors.

## Build and test

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select rs_control --symlink-install
colcon test --packages-select rs_control --event-handlers console_direct+
```

The test suite exercises model limits, protocol packing, calibration
conversions, and ros2_control interface export without requiring a live CAN
device.

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

The hardware bringup launch files in this package use the installed
`rs_control/config/robstride.yaml` by default. Therefore, the commands below do
not need a `config_file:=...` argument. Pass `config_file:=/absolute/path/to/robstride.yaml`
only when using a separate deployment or calibration file.

## Encoder-only bringup

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rs_control rs_encoder_bringup.launch.py
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
ros2 launch rs_control rs_encoder_bringup.launch.py
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
ros2 launch rs_control rs_python_bringup.launch.py
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
ros2 launch rs_control rs_bringup.launch.py
```

Only after validating limits, directions, offsets, and the emergency stop
path, explicitly enable control:

```bash
ros2 launch rs_control rs_bringup.launch.py \
  read_only:=false enable_control:=true
```

MoveIt can target `arm_controller` and `gripper_controller` using its normal
`FollowJointTrajectory` configuration. An RL process can consume `/joint_states`
and send position commands through the controller or directly through a future
controller node;
the SocketCAN protocol and calibration conversions remain isolated in this
package.

## Command-line jogging

`rs_jog_controller` is an interactive terminal node for the standard
`ros2_control` bringup. It publishes position trajectories to
`/arm_controller/joint_trajectory` and `/gripper_controller/joint_trajectory`;
it does not start the hardware, configure CAN, or disable motors. It is not the
same command path as `rs_python_bringup.launch.py`, which uses
`/rs_control/joint_trajectory`.

Before jogging:

1. Configure and verify `can0` at the bitrate in `robstride.yaml`.
2. Stop `rs_encoder_bringup`, `rs_python_bringup`, MoveIt, and any other node
   that may command the motors.
3. Make sure the arm is supported, the emergency-stop path is available, and
   the configured limits and directions have been checked.

### Terminal 1: start the physical controllers

Source ROS 2 and this workspace in the terminal that will own the hardware:

```bash
source /opt/ros/humble/setup.bash
source /home/jundi/nxp_rs_ws/install/setup.bash

ros2 launch rs_control rs_bringup.launch.py \
  read_only:=false \
  enable_control:=true
```

`read_only:=false` enables the motor control path during hardware activation.
`enable_control:=true` starts `arm_controller` and `gripper_controller`; the
spelling is `enable_control`, not `eneble_control`. If only state observation
is wanted, use `read_only:=true` and do not start the jogger.

In another terminal, verify that all three required controllers are active:

```bash
source /opt/ros/humble/setup.bash
source /home/jundi/nxp_rs_ws/install/setup.bash
ros2 control list_controllers
```

You should see `joint_state_broadcaster`, `arm_controller`, and
`gripper_controller` in the `active` state before starting the jogger.

### Terminal 2: start the jogger

Run this in a real interactive terminal. The node waits for one complete,
current `/joint_states` snapshot containing `joint-1` through `joint-6` and
`joint_right-finger`; it sends no motion until that pose has been captured.

```bash
source /opt/ros/humble/setup.bash
source /home/jundi/nxp_rs_ws/install/setup.bash
ros2 run rs_control rs_jog_controller
```

The executable has no ordinary argparse options. Set its node parameters with
ROS 2's `--ros-args -p name:=value` syntax, for example:

```bash
ros2 run rs_control rs_jog_controller --ros-args \
  -p selected_joint:=joint-4 \
  -p arm_step:=0.02 \
  -p command_rate_hz:=20.0 \
  -p max_arm_velocity:=0.2
```

### Display the physical robot in RViz

Start RViz in a third terminal while the hardware bringup and jogger are
running:

```bash
source /opt/ros/humble/setup.bash
source /home/jundi/nxp_rs_ws/install/setup.bash
ros2 launch rs_control rs_jog_rviz.launch.py
```

This launch file starts only RViz and automatically loads the installed
`rs_control/rviz/jog.rviz` configuration. The hardware bringup already
publishes `/robot_description` and the TF tree from the current
`/joint_states`, so the robot model follows the real arm as it is jogged. No
additional `robot_state_publisher` or joint-state publisher is needed.

To try a different RViz layout, override the config explicitly:

```bash
ros2 launch rs_control rs_jog_rviz.launch.py \
  rviz_config:=/absolute/path/to/custom.rviz
```

Do not use `nxp_rs_description display.launch.py` for physical jogging: that
standalone model viewer starts `joint_state_publisher_gui` and its own
`robot_state_publisher`, which can publish simulated joint states and conflict
with the hardware bringup. Close RViz at any time without stopping the
jogger; stop the jogger with `q`, then stop the hardware bringup with `Ctrl+C`.

The equivalent launch wrapper is useful when several parameters must be
configured. Launch arguments use `name:=value` directly:

```bash
ros2 launch rs_control rs_jog.launch.py \
  selected_joint:=joint-4 \
  arm_step:=0.02 \
  command_rate_hz:=20.0 \
  max_arm_velocity:=0.2 \
  max_arm_acceleration:=0.4 \
  max_arm_jerk:=2.0 \
  max_arm_target_lead:=0.10
```

The launch wrapper uses the installed `config/joints.yaml` by default. Use an
absolute path when selecting another limits file:

```bash
ros2 launch rs_control rs_jog.launch.py \
  limits_file:=/absolute/path/to/joints.yaml
```

### Jog keys

| Key | Action |
| --- | --- |
| `1`--`6` | Select `joint-1` through `joint-6` |
| `7` | Select `joint_right-finger`; its position and step are in metres |
| `+` or `=` | Jog the selected joint in the positive direction |
| `-` or `_` | Jog the selected joint in the negative direction |
| `[` | Halve both the arm and finger step sizes |
| `]` | Double both the arm and finger step sizes |
| `Space` or `s` | Request a controlled stop; torque remains enabled |
| `h` | Re-capture current feedback and refresh the hold target |
| `?` or `i` | Print the key help |
| `q` | Request a controlled stop and exit the jogger |

The default steps are `0.05 rad` for arm joints and `0.001 m` for the finger.
The default motion limits are:

| Parameter | Arm default | Finger default | Meaning |
| --- | ---: | ---: | --- |
| `max_*_velocity` | `0.4 rad/s` | `0.02 m/s` | Maximum profile velocity |
| `max_*_acceleration` | `0.8 rad/s²` | `0.04 m/s²` | Maximum profile acceleration |
| `max_*_jerk` | `4 rad/s³` | `0.2 m/s³` | Maximum profile jerk |
| `max_*_target_lead` | `0.25 rad` | `0.01 m` | Maximum target distance ahead of the planned position |

The jogger clamps every target to the limits file, after applying the default
`arm_limit_margin:=0.02` rad and `gripper_limit_margin:=0.001` m. Reduce the
step and motion-limit parameters for a loaded or unfamiliar mechanism. Keep
`command_rate_hz` aligned with the controller-manager and hardware CAN loop
rates; increase them together only after measuring CAN transaction time and
cycle jitter.

Other useful parameters are `selected_joint`, `arm_step`, `gripper_step`,
`command_duration_s` (default `0.2` s), `feedback_timeout_s` (default `0.25` s),
`state_topic`, `arm_command_topic`, `gripper_command_topic`, and `limits_file`.
The `keyboard_poll_hz` parameter defaults to `50.0` and can be set with the
direct `ros2 run ... --ros-args -p keyboard_poll_hz:=...` form; it is not
currently exposed as an argument by `rs_jog.launch.py`.

If complete joint feedback stops for `feedback_timeout_s`, the jogger requests
a controlled stop and rejects further jog commands until feedback is current
again. Exiting the jogger does not disable torque: press `Ctrl+C` in the
hardware bringup terminal to stop ros2_control and disable the motors.

## Build and test

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select rs_control --symlink-install
colcon test --packages-select rs_control --event-handlers console_direct+
```

The test suite exercises model limits, protocol packing, calibration
conversions, and ros2_control interface export without requiring a live CAN
device.

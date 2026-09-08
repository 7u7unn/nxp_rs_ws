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
must be unique and non-zero. Set `direction` and `position_offset` after
checking the physical zero and joint direction. For the finger actuator,
`position_scale` is the measured linear travel in metres per motor radian; the
driver rejects the placeholder value until it is configured.

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

The reader queries the RobStride mechanical position (`0x7019`), mechanical
velocity (`0x701B`), and measured torque (`0x302C`) parameters and publishes
standard `sensor_msgs/msg/JointState` plus diagnostics. It does not send
enable, disable, or operation-control frames.

## ros2_control / MoveIt / RL boundary

The standard bringup exports position commands and position, velocity, and
effort states through the `rs_control/RobstrideSystem` plugin. The standard
interfaces are:

- `/joint_states` from `joint_state_broadcaster`;
- `/controller_manager` services;
- `arm_controller` as a position `JointTrajectoryController` when enabled.

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

MoveIt can target `arm_controller` using its normal `FollowJointTrajectory`
configuration. An RL process can consume `/joint_states` and send position
commands through the controller or directly through a future controller node;
the SocketCAN protocol and calibration conversions remain isolated in this
package.

## Build and test

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select rs_control --symlink-install
colcon test --packages-select rs_control --event-handlers console_direct+
```

The test suite exercises model limits, protocol packing, calibration
conversions, and ros2_control interface export without requiring a live CAN
device.

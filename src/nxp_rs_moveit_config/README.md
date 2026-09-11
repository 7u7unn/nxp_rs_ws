# nxp_rs_moveit_config

MoveIt 2 configuration for the NXP RS robot, with separate simulation and
physical-hardware launch paths.

The physical launch uses the `rs_control/RobstrideSystem` ros2_control plugin
and the configured SocketCAN interface. It is read-only by default.

The physical launch automatically uses the installed
`rs_control/config/robstride.yaml`. A `config_file:=/absolute/path/to/robstride.yaml`
argument is optional and is only needed for a separate deployment or
calibration file.

The simulation launch uses:

- the existing `nxp_rs.xacro` model;
- an SRDF `arm` group from `base_link` to the fixed `grasp_frame` between the
  fingers;
- the existing mimic relationship for the left finger;
- `mock_components/GenericSystem` and simulated ROS 2 controllers.

Both MoveIt launch modes also add a `nxp_rs_floor` collision box whose top
surface is at `z=0` in the `world` frame. The existing planning-scene Allowed
Collision Matrix is preserved, with only the fixed `base_link`/floor contact
allowed. Moving links remain forbidden from intersecting the floor. Adjust
`config/floor_scene.yaml` if the physical mounting height or floor frame
changes.

`grasp_frame` is a non-actuated fixed link centered between the right and left
finger CAD envelopes. It is the arm group's tip and the `gripper_eef` parent
frame, so MoveIt target poses are expressed at the grasp center rather than at
the palm/wrist area.

It does not include `rs_control/RobstrideSystem`, open SocketCAN, or send motor
commands. The physical launch uses the real hardware plugin and is documented
below.

## Install and build

```bash
sudo apt install ros-humble-moveit
cd /home/jundi/nxp_rs_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to nxp_rs_moveit_config
source install/setup.bash
```

## Run the simulation-only MoveIt demo

```bash
ros2 launch nxp_rs_moveit_config demo.launch.py
```

For a headless smoke test that starts MoveIt and the fake controllers without
RViz:

```bash
ros2 launch nxp_rs_moveit_config demo.launch.py rviz:=false
```

In RViz, choose the `arm` planning group in the MotionPlanning panel, set a
target pose or joint target, click **Plan**, and then **Execute**. The motion
is sent to the mock `arm_controller`; no physical motor is involved.

After changing the URDF or SRDF, restart MoveIt/RViz. Existing processes keep
the old robot model until they are shut down and relaunched.

## Move the physical robot from MoveIt

Use this procedure for the first real-robot test. The physical MoveIt launch
owns the hardware, controller manager, robot state publisher, controllers,
MoveIt, and RViz. Run only one of these owners at a time:
`hardware_moveit.launch.py`, `rs_bringup.launch.py`, or another physical jog
bringup. Running two of them creates duplicate `/controller_manager` and
`/joint_state_broadcaster` nodes and causes controller loading to fail.

The commands use the installed
`rs_control/config/robstride.yaml` automatically. No
`config_file:=...` argument is required for the normal deployment.

### 1. Stop other physical launch files

Stop any running `rs_bringup`, jogger, physical RViz, or previous
`hardware_moveit` session with `Ctrl+C`. Do not use
`nxp_rs_description display.launch.py` for this procedure because it starts a
separate robot state publisher and slider GUI.
Also stop the MoveIt simulation demo in the same ROS domain: its controller
and joint-state names overlap with hardware.

After starting MoveIt, `ros2 node list` must contain exactly one
`/controller_manager`, `/robot_state_publisher`, and `/joint_state_broadcaster`.

### 2. Check the CAN interface

The ROS launch files expect the operating system to configure SocketCAN before
they start. Check that `can0` is up with the motor bus bitrate:

```bash
ip -details link show can0
```

The expected state is `UP`, `ERROR-ACTIVE`, and `bitrate 1000000`. If the
interface is down and the CAN wiring and power are safe, configure it once for
the current session:

```bash
sudo ip link set can0 up type can bitrate 1000000
```

For deployment, test the udev/systemd configuration after both reboot and USB
adapter reconnect. The ROS launch does not set the CAN bitrate for you.

### 3. Start read-only MoveIt and inspect the real pose

Keep the robot in a safe, supported position and start the read-only launch:

```bash
source /opt/ros/humble/setup.bash
source /home/jundi/nxp_rs_ws/install/setup.bash
ros2 launch nxp_rs_moveit_config hardware_moveit.launch.py \
  can_interface:=can0 \
  read_only:=true \
  enable_control:=false \
  rviz:=true
```

`read_only:=true` does not enable motors or send operation-control commands; it
only reads encoder status. Start it only after stopping any previous physical
bringup so another process is not still holding motor torque.
`enable_control:=false` prevents the physical arm and gripper trajectory
controllers from being spawned. It is a controller-spawn option; it is not a
replacement for `read_only`.

From a second terminal with the same ROS environment, check:

```bash
source /opt/ros/humble/setup.bash
source /home/jundi/nxp_rs_ws/install/setup.bash
ros2 node list
ros2 control list_hardware_components --verbose
ros2 control list_controllers
ros2 topic echo --once /joint_states
```

Before continuing, verify that:

- `robstride_system` is `active` and all state interfaces are `available`;
- `joint_state_broadcaster` is `active`;
- joint positions are updating and stable;
- there is only one controller manager and one state publisher;
- RViz shows the physical robot at the same pose as the real robot.

Inspect RViz's collision geometry as well as the visual robot. During this
commissioning phase, every link uses its visual CAD mesh for collision
checking, including the base and fingers. This avoids the false base/finger
contact from the former simplified geometry, but it still requires verification
against the physical assembly, including brackets, wiring, and obstacles.

### 4. Set the MoveIt start state correctly

In RViz's **MotionPlanning** panel, select the `arm` group and set the start
state to **Current**. The current robot model must match the real joint
positions. The straight-up pose or the SRDF `home` pose is a possible planning
goal; it is not an automatic startup pose and must not be used as the start
state unless the physical robot is actually there.

Use **Plan** only during this read-only stage. Do not execute a trajectory yet.

### 5. Enable physical position control

Before enabling, check the emergency-stop path, clear the workspace, support
the arm if gravity can make it drop, and keep the E-stop reachable. Stop the
read-only launch with `Ctrl+C`, wait for its nodes to exit, and start one fresh
physical MoveIt instance:

```bash
ros2 launch nxp_rs_moveit_config hardware_moveit.launch.py \
  can_interface:=can0 \
  read_only:=false \
  enable_control:=true \
  rviz:=true
```

Warning: `read_only:=false` enables the motors during hardware activation even
if `enable_control:=false`. The driver reads the encoder positions before
enabling and initializes its position commands from those measurements, so the
launch does not command the robot to the straight-up or `home` pose. The
configured reference gains (`kp: 500`, `kd: 5`) are applied immediately,
however. A calibration error or a pose change while enabling can produce a
strong holding correction; treat motor activation as motion-capable and be
ready to press the E-stop.

Confirm the enabled controllers before sending a goal:

```bash
ros2 control list_hardware_components --verbose
ros2 control list_controllers
```

The expected controllers are `joint_state_broadcaster`, `arm_controller`, and
`gripper_controller`, all `active`. MoveIt maps the arm and gripper groups to
their respective `FollowJointTrajectory` action servers.

The reference-style physical launch intentionally does not load the optional
`config/hardware_execution.yaml` strict-tolerance overlay. Check that the
active controller uses the reference-compatible loop settings after restarting:

```bash
ros2 param get /controller_manager update_rate
ros2 param get /arm_controller state_publish_rate
ros2 param get /arm_controller open_loop_control
```

Expected values are `100`, `100.0`, and `true`, respectively. Editing YAML or
rebuilding does not change parameters or robot geometry in an already running
launch. The optional overlay remains available for offline commissioning tests,
but its nonzero path/goal tolerances are deliberately not part of the normal
reference-style hardware launch.

### 6. Execute the first motion

Start with one joint and a target approximately `0.01`--`0.02 rad` from its
measured current position, well inside the configured limits. Use the lowest
available velocity and acceleration scaling. Click **Plan**, inspect the
trajectory and start point in RViz, then click **Execute**.

After **every** execution, inspect feedback before planning again:

```bash
ros2 topic echo --once /arm_controller/controller_state
ros2 topic echo --once /joint_states
```

Compare `desired.positions`, `actual.positions`, and `error.positions` in
joint-name order. Confirm the measured joints are stable over several samples,
then refresh the RViz start state to **Current** and create a new plan. Do not
reuse the previous trajectory. `SUCCEEDED` is not a continuous holding or
collision-safety guarantee.

Stop immediately with the E-stop if the robot pulls toward an unexpected pose,
vibrates, moves more than expected, or if the CAN state leaves `ERROR-ACTIVE`.
After a normal test, cancel/stop the motion and shut down the launch with
`Ctrl+C`; this lets the hardware plugin deactivate and disable the motors.
Arrange mechanical support first: controller cancel/abort can reset its hold
target to the measured pose, reducing load-supporting torque, and shutdown
disables the motors. Software cancellation is not a safety-rated stop.

## Execution checks and troubleshooting

The normal reference-style hardware launch leaves JTC path/goal position
tolerances at their defaults (zero disables those checks). This avoids turning
the position-only driver’s tracking error into an abort/hold-position reset,
but it does **not** certify tracking accuracy. Use the optional
`config/hardware_execution.yaml` overlay only during a supported commissioning
test after motor tuning. It rejects substantial tracking error; it cannot
correct motor tuning or guarantee collision clearance. Jogging and simulation
retain their separate configurations.

- **`PATH_TOLERANCE_VIOLATED` / `GOAL_TOLERANCE_VIOLATED`:** compare measured and
  commanded positions. Do not increase tolerances to hide persistent error.
  With the present position-only driver, low motor stiffness and no gravity
  feedforward can leave a loaded joint away from the requested position.
  Tune only in a supported, controlled commissioning procedure.
- **Start point deviates more than `0.01`:** wait for stable feedback, select
  **Current**, and replan. If it recurs, investigate drift/tracking/calibration;
  do not disable the start-state check.
- **Base/finger contact with no observed contact:** rebuild both description
  and MoveIt config, then restart all model-consuming nodes. The corrected
  base mesh preserves collision checking for this pair. If it still occurs,
  inspect collision geometry, joint zero/sign mapping, and actual clearance;
  do not add an SRDF collision exemption.
- **Finger below its lower limit:** the captured encoder value was
  `-0.0002782 m` although its lower bound is `0`. Verify gripper zero calibration
  and mapping physically. Do not widen limits or clamp feedback to hide it.

In the installed Humble JTC 2.53.1, `open_loop_control: true` preserves the
command reference at trajectory replacement but **still checks measured
position error**. It remains enabled deliberately. This position-only
configuration does not independently enforce measured stopping velocity via
`stopped_velocity_tolerance`; check actual velocity and settling separately.
There is also no continuous position-tolerance enforcement after success.

See [the investigation and evidence](docs/execution_investigation_2026-09-10.md).
Offline regression tests use in-memory joints and CAD geometry, not CAN:

```bash
colcon test --packages-select nxp_rs_moveit_config
colcon test-result --test-result-base build/nxp_rs_moveit_config --verbose
```

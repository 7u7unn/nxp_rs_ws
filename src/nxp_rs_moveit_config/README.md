# nxp_rs_moveit_config

MoveIt 2 configuration for the NXP RS robot, with separate simulation and
physical-hardware launch paths.

The physical launch uses the `rs_control/RobstrideSystem` ros2_control plugin
and the configured SocketCAN interface. It is read-only by default.

The launch file uses:

- the existing `nxp_rs.xacro` model;
- an SRDF `arm` group from `base_link` to `palm`;
- the existing mimic relationship for the left finger;
- `mock_components/GenericSystem` and simulated ROS 2 controllers.

It does not include `rs_control/RobstrideSystem`, open SocketCAN, or send motor
commands.

## Install and build

```bash
sudo apt install ros-humble-moveit
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

## Move the physical robot from MoveIt

Do not run this launch together with `rs_bringup.launch.py` or
`demo.launch.py`; this launch owns the hardware, controllers, MoveIt, and RViz.

First perform a read-only hardware/MoveIt startup:

```bash
source /opt/ros/humble/setup.bash
source /home/jundi/nxp_rs_ws/install/setup.bash
ros2 launch nxp_rs_moveit_config hardware_moveit.launch.py \
  config_file:=/home/jundi/nxp_rs_ws/src/rs_control/config/robstride.yaml \
  can_interface:=can0 read_only:=true enable_control:=false
```

Confirm `/joint_states`, the controller manager, and the displayed robot state.
After checking the emergency-stop path and confirming the robot is clear, stop
that launch and explicitly enable physical position control:

```bash
ros2 launch nxp_rs_moveit_config hardware_moveit.launch.py \
  config_file:=/home/jundi/nxp_rs_ws/src/rs_control/config/robstride.yaml \
  can_interface:=can0 read_only:=false enable_control:=true
```

In RViz, start with a small joint target well inside the measured limits, click
**Plan**, verify the trajectory, and only then click **Execute**. The launch
starts separate `arm_controller` and `gripper_controller` action servers that
MoveIt maps to the physical joints.

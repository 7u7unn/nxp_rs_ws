# nxp_rs_moveit_config

MoveIt 2 configuration for testing the current `nxp_rs_description` model
without connecting to RobStride hardware.

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

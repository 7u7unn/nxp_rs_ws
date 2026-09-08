# Gazebo inertia inspection

Verified on 2026-09-08 using ROS 2 Humble and Gazebo Classic 11.10.2.

All nine physical links have positive masses and positive-definite inertia
tensors whose principal moments satisfy the triangle inequalities. Total mass
is **2.444 kg**. Mass, COM, and inertia agree with the supplied SolidWorks
reports under the frame mapping documented in `urdf/nxp_rs.xacro`. Conversion
to SDF preserves these values. A live query of the inspection scene also
matched all nine links' mass, COM, inertial orientation, and tensor entries.

This checks the transfer and mathematical validity of the reported data.
It does not independently establish the CAD frame mapping or the real robot's
mass distribution. In particular, `link_2` uses an inferred 180-degree rotation
about X; confirm this against SolidWorks Coordinate System1. The reports use
user-overridden masses and rounded inertia entries; higher precision CAD
exports would be useful for the small finger inertias.

## Open the inspection scene

From `/home/jundi/nxp_rs_ws`:

```bash
rtk proxy bash -lc 'source /opt/ros/humble/setup.bash; colcon build --symlink-install --packages-select nxp_rs_description'
rtk proxy bash -lc 'source /opt/ros/humble/setup.bash; source install/setup.bash; ros2 launch nxp_rs_description gazebo_inertia.launch.py'
```

The launch starts paused, with the base fixed to the world at Z = 2 mm.
It uses Gazebo port 11346 and ROS domain 46 to coexist with a normal session
on port 11345. Override `gazebo_port` and `ros_domain_id` if those are occupied.
Use `gui:=false` for a server without a window. Stop this launch with Ctrl+C
before starting another copy on the same port.

The wrapper adds only a world mounting joint. The original model's inertia,
mesh origins, joint limits, and damping are not changed. The launch resolves
the package mesh paths to absolute file URIs for Gazebo.

## Inspect in the GUI

1. Keep simulation paused. Select `nxp_rs` in the model tree and double-click
   the model if needed to bring it into view.
2. Enable **View → Center of Mass** and **View → Inertia**. These options are
   also available through the model's right-click View menu. Toggle one at a
   time if the overlays obscure each other.
3. Enable **View → Transparent** or **Wireframe** to see the COM markers.
   Check that each marker has a plausible position relative to its link's
   mass distribution. A COM can lie in a hollow space; it need not intersect
   the visible material. Pay particular attention to `link_2` and the fingers.
4. The inertia overlay represents an equivalent inertial shape, not a fitted
   collision box. Its center and principal-axis orientation matter; its
   dimensions need not coincide with the mesh boundary.
5. Toggle **View → Collisions** separately and check alignment with the
   visual geometry before using contact behavior as a validation signal.

Gazebo references:
[inertia visualization](https://classic.gazebosim.org/tutorials?tut=inertia) and
[COM visualization](https://get.gazebosim.org/tutorials/?tut=ros_urdf).

## Repeat the numerical audit

From the workspace root (requires Python numpy, ROS xacro, and Gazebo `gz`):

```bash
rtk proxy bash -lc 'source /opt/ros/humble/setup.bash; /usr/bin/python3 src/nxp_rs_description/scripts/check_inertia.py'
```

With the inspection scene running, include the actual loaded model:

```bash
rtk proxy bash -lc 'source /opt/ros/humble/setup.bash; /usr/bin/python3 src/nxp_rs_description/scripts/check_inertia.py --live'
```

For another Gazebo port, add `--gazebo-port PORT`. These are read-only checks.
The checker is specific to this model and its documented zero inertial RPY
and CAD report format.

## Before interpreting motion

There are no motor controllers or joint damping in this description. A fixed
base prevents the whole robot from falling, but does not hold its joints.
If you unpause, gravity can fold the arm. That alone is not evidence of bad
inertia. Use a few single steps for initial observation; relaunch to return
to the initial inspection pose.

The original session on port 11345 was observed with its model approximately
5.8 km below the world origin and without a fixed world joint. Its inertia
entries were present, but its falling state was unsuitable for inspection.
The new scene includes local ground geometry and explicit mesh paths.

Confirm CAD coordinate axes, mass overrides, and higher precision values
before treating the inertia as physically validated. Controlled torque or
pendulum measurements would be a subsequent check, after validating joint
axes, collision geometry, and the intended actuator behavior.

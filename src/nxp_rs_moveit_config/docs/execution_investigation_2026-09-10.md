# Hardware execution investigation — 2026-09-10

## Findings

The first action completed at ROS time `1789033768.69`; the next request at
`1789033778.04` was rejected during planning for `base_link`/`right-finger`
self-collision. No second trajectory was sent to the arm controller. This was
not a physical contact switch or joint-limit alarm.

Read-only inspection of the running system found one authoritative
joint-state publisher and persistent command-versus-feedback differences:

| Joint | Desired rad | Measured rad | Absolute error rad |
| --- | --- | --- | --- |
| joint-2 | -0.0532726 | 0.0130383 | 0.0663109 |
| joint-4 | 1.1871404 | 1.2672792 | 0.0801389 |

Those errors are about 3.8 and 4.6 degrees. Near-zero velocity and holding
effort are consistent with a static load/stiffness offset. This is an
inference, not proof of a particular mechanical cause. The driver uses motor
Kp/Kd position control, with no gravity torque feedforward; ROS position-only
JTC gains do not provide an outer position PID. No live gains were changed.

### Controller success was insufficiently checked

The old configuration supplied no per-joint path/goal position tolerances.
Their zero defaults disable position checks, allowing success despite the
recorded errors. The installed controller is Humble JTC 2.53.1.

Correction to the earlier diagnosis: `open_loop_control: true` does **not**
disable measured position-error checking in this version. It selects the last
command as the interpolation start. Turning it off alone would not repair
the missing checks and can remove a loaded joint's supporting position error
when replacing a trajectory.

The hardware-only overlay now sets nonzero path/goal position tolerances,
a finite settling deadline, and requires zero endpoint velocity. The motor
gains and the deliberate open-loop reference continuity are unchanged.

Additional limitations confirmed in the installed controller's implementation:

- Position-only command interfaces do not calculate measured velocity error
  for tolerance enforcement. The configured stopped-velocity value is not an
  independent guarantee of physical settling.
- A successful action enters holding mode, where path/goal tolerances are
  not continuously enforced.
- On cancellation or tolerance abort, the controller holds the measured
  position. This can reduce gravity-supporting torque from a position offset.
  Mechanical support and a validated stop procedure remain essential.

These behaviors follow the [versioned JTC implementation](https://github.com/ros-controls/ros2_controllers/blob/2.53.1/joint_trajectory_controller/src/joint_trajectory_controller.cpp)
and [tolerance checks](https://github.com/ros-controls/ros2_controllers/blob/2.53.1/joint_trajectory_controller/include/joint_trajectory_controller/tolerances.hpp).

### Simplified base geometry caused a CAD-relative false contact

An offline MoveIt/FCL probe used the same URDF/SRDF and captured joint state.
The base collision mesh was substituted in memory while transforms, finger
mesh, joint positions, and collision exemptions were unchanged.

| Base collision geometry | Measured arm pose | Desired endpoint |
| --- | --- | --- |
| `base-link-col.STL` (old simplified mesh) | Base/right-finger contact, up to 11.8 mm reported depth | No self-collision |
| `base-link.STL` (detailed CAD) | No self-collision | No self-collision |

Thus the feedback deviation moved the model into a region incorrectly
occupied by the simplified mesh relative to the detailed CAD. This explains
why the planned endpoint was accepted but planning from the measured endpoint
failed, and supports the operator's observation of no physical contact.

The shared URDF now uses each link's visual CAD mesh as its collision geometry;
no `-col.STL` mesh is referenced by the Xacro. No link pair was exempted. The
detailed meshes trade collision-processing cost for geometric fidelity.
Physical conformity, brackets/cabling, and all surrounding obstacles must
still be verified; a CAD comparison cannot certify the real workspace.

### Independent gripper bounds issue

At the time of this investigation, the recorded right-finger value was
`-0.0002782029 m`, below the then-URDF minimum of zero. Offline full-robot
bounds checks rejected that state. The base contact reproduced even with the
finger position set to zero, so this was a separate issue. A later calibration
rebased physical open to ROS zero and expanded the configured travel range;
the historical observation is retained here for context.

## Validation and deployment

`test_hardware_execution.cpp` runs the installed JTC with in-memory position
and velocity interfaces on isolated ROS domain 79. It checks sequential
successful trajectories, rejection of the recorded tracking errors, a smaller
goal error exceeding its deadline, and rejection of nonzero endpoint velocity.
No RobStride plugin or CAN device is loaded.

`test_collision_geometry.cpp` reproduces the old mesh contact, verifies the
corrected model at measured/desired poses, verifies a separate genuine modeled
base/finger intersection is still rejected, and exposes the gripper bounds
issue. These are offline geometry checks, not physical motion tests.

Rebuild `nxp_rs_description` and `nxp_rs_moveit_config`, then follow the
[read-only-first README procedure](../README.md). Existing live nodes keep
their old geometry and parameters until restarted. Resolve the persistent
tracking error and gripper calibration before treating this as production
deployment. New tolerance aborts are useful diagnostics, not a reason to relax
the limits or bypass collision checking.

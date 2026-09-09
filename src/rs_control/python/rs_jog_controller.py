#!/usr/bin/env python3
"""Interactive single-joint position jogger for the physical ROS 2 controllers."""

from __future__ import annotations

import math
import os
from pathlib import Path
import select
import sys
import termios
import time
import tty

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from rs_control_py.jog import (
    ARM_JOINTS,
    CONTROL_JOINTS,
    GRIPPER_JOINTS,
    JointLimit,
    JogMotionProfile,
    JogTargetState,
)


def _default_limits_file() -> str:
    from ament_index_python.packages import get_package_share_directory

    return str(
        Path(get_package_share_directory("rs_control")) / "config" / "joints.yaml"
    )


def _load_limits(
    path: str,
    arm_margin: float,
    gripper_margin: float,
) -> dict[str, JointLimit]:
    if arm_margin < 0.0 or gripper_margin < 0.0:
        raise ValueError("limit margins must not be negative")

    try:
        import yaml
    except ImportError as exc:
        raise ValueError("PyYAML is required to load jog limits") from exc

    try:
        root = yaml.safe_load(Path(path).expanduser().read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError(f"unable to load jog limits '{path}': {exc}") from exc

    if not isinstance(root, dict) or not isinstance(root.get("joints"), dict):
        raise ValueError("jog limits must contain a 'joints' mapping")

    limits: dict[str, JointLimit] = {}
    for joint_name in CONTROL_JOINTS:
        entry = root["joints"].get(joint_name)
        if not isinstance(entry, dict) or not isinstance(entry.get("urdf"), dict):
            raise ValueError(f"jog limits are missing URDF limits for '{joint_name}'")
        urdf = entry["urdf"]
        if joint_name in GRIPPER_JOINTS:
            lower_key, upper_key = "lower_m", "upper_m"
            margin = gripper_margin
        else:
            lower_key, upper_key = "lower_rad", "upper_rad"
            margin = arm_margin
        try:
            lower = float(urdf[lower_key]) + margin
            upper = float(urdf[upper_key]) - margin
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"invalid jog limits for '{joint_name}'") from exc
        limits[joint_name] = JointLimit(lower, upper)
    return limits


class JogControllerNode(Node):
    """Read feedback and publish bounded, continuous jog trajectories."""

    def __init__(self) -> None:
        super().__init__("rs_jog_controller")

        limits_file = str(
            self.declare_parameter("limits_file", _default_limits_file()).value
        )
        arm_margin = float(self.declare_parameter("arm_limit_margin", 0.02).value)
        gripper_margin = float(
            self.declare_parameter("gripper_limit_margin", 0.001).value
        )
        self._arm_step = float(self.declare_parameter("arm_step", 0.05).value)
        self._gripper_step = float(
            self.declare_parameter("gripper_step", 0.001).value
        )
        self._command_duration = float(
            self.declare_parameter("command_duration_s", 0.2).value
        )
        command_rate_hz = float(
            self.declare_parameter("command_rate_hz", 20.0).value
        )
        keyboard_poll_hz = float(
            self.declare_parameter("keyboard_poll_hz", 50.0).value
        )
        feedback_timeout_s = float(
            self.declare_parameter("feedback_timeout_s", 0.25).value
        )
        max_arm_velocity = float(
            self.declare_parameter("max_arm_velocity", 0.4).value
        )
        max_gripper_velocity = float(
            self.declare_parameter("max_gripper_velocity", 0.02).value
        )
        max_arm_acceleration = float(
            self.declare_parameter("max_arm_acceleration", 0.8).value
        )
        max_gripper_acceleration = float(
            self.declare_parameter("max_gripper_acceleration", 0.04).value
        )
        max_arm_jerk = float(
            self.declare_parameter("max_arm_jerk", 4.0).value
        )
        max_gripper_jerk = float(
            self.declare_parameter("max_gripper_jerk", 0.2).value
        )
        max_arm_target_lead = float(
            self.declare_parameter("max_arm_target_lead", 0.25).value
        )
        max_gripper_target_lead = float(
            self.declare_parameter("max_gripper_target_lead", 0.01).value
        )
        state_topic = str(self.declare_parameter("state_topic", "/joint_states").value)
        arm_topic = str(
            self.declare_parameter(
                "arm_command_topic", "/arm_controller/joint_trajectory"
            ).value
        )
        gripper_topic = str(
            self.declare_parameter(
                "gripper_command_topic", "/gripper_controller/joint_trajectory"
            ).value
        )
        selected_joint = str(
            self.declare_parameter("selected_joint", "joint-1").value
        )

        if self._arm_step <= 0.0 or self._gripper_step <= 0.0:
            raise ValueError("arm_step and gripper_step must be positive")
        if not math.isfinite(self._command_duration) or self._command_duration <= 0.0:
            raise ValueError("command_duration_s must be positive")
        for name, value in (
            ("command_rate_hz", command_rate_hz),
            ("keyboard_poll_hz", keyboard_poll_hz),
            ("feedback_timeout_s", feedback_timeout_s),
            ("max_arm_velocity", max_arm_velocity),
            ("max_gripper_velocity", max_gripper_velocity),
            ("max_arm_acceleration", max_arm_acceleration),
            ("max_gripper_acceleration", max_gripper_acceleration),
            ("max_arm_jerk", max_arm_jerk),
            ("max_gripper_jerk", max_gripper_jerk),
            ("max_arm_target_lead", max_arm_target_lead),
            ("max_gripper_target_lead", max_gripper_target_lead),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if selected_joint not in CONTROL_JOINTS:
            raise ValueError(
                f"selected_joint must be one of: {', '.join(CONTROL_JOINTS)}"
            )

        limits = _load_limits(limits_file, arm_margin, gripper_margin)
        self._targets = JogTargetState(limits)
        self._profiles: dict[str, JogMotionProfile] = {}
        for name in ARM_JOINTS:
            self._profiles[name] = JogMotionProfile(
                limits[name],
                max_arm_velocity,
                max_arm_acceleration,
                max_arm_jerk,
                max_arm_target_lead,
            )
        for name in GRIPPER_JOINTS:
            self._profiles[name] = JogMotionProfile(
                limits[name],
                max_gripper_velocity,
                max_gripper_acceleration,
                max_gripper_jerk,
                max_gripper_target_lead,
        )
        self._selected_joint = selected_joint
        self._last_complete_positions: dict[str, float] = {}
        self._active_groups: set[str] = set()
        self._last_motion_time = time.monotonic()
        self._feedback_timeout_s = feedback_timeout_s
        self._last_feedback_time: float | None = None
        self._feedback_stale = False
        self._shutdown_stop_sent = False
        self._keyboard_fd = sys.stdin.fileno() if sys.stdin.isatty() else None

        self._arm_publisher = self.create_publisher(JointTrajectory, arm_topic, 10)
        self._gripper_publisher = self.create_publisher(
            JointTrajectory, gripper_topic, 10
        )
        self._state_subscription = self.create_subscription(
            JointState,
            state_topic,
            self._on_joint_state,
            qos_profile_sensor_data,
        )
        self._keyboard_timer = self.create_timer(
            1.0 / keyboard_poll_hz, self._poll_keyboard
        )
        self._motion_timer = self.create_timer(
            1.0 / command_rate_hz, self._advance_motion
        )

        self.get_logger().info(
            "Waiting for a complete current pose on "
            f"{state_topic}; no command is sent until all seven joints arrive."
        )
        self.get_logger().info(
            f"arm topic: {arm_topic}; gripper topic: {gripper_topic}; "
            f"limits: {limits_file}"
        )

    def _on_joint_state(self, message: JointState) -> None:
        snapshot: dict[str, float] = {}
        for name, position in zip(message.name, message.position):
            if name in CONTROL_JOINTS and math.isfinite(float(position)):
                snapshot[name] = float(position)
        if not snapshot:
            return

        complete_snapshot = all(name in snapshot for name in CONTROL_JOINTS)
        if complete_snapshot:
            self._last_complete_positions = dict(snapshot)
            self._last_feedback_time = time.monotonic()
            if self._feedback_stale:
                self._feedback_stale = False
                self.get_logger().info("Joint feedback is current again")

        if not self._targets.initialized and complete_snapshot:
            self._targets.initialize(snapshot)
            for name in CONTROL_JOINTS:
                self._profiles[name].initialize(snapshot[name])
                self._targets.set_target(name, self._profiles[name].position)
            self._last_motion_time = time.monotonic()
            self._publish_all_targets()
            self.get_logger().info(
                "Captured the current robot pose and published a hold target for "
                "all arm and gripper joints."
            )

    def _make_trajectory(self, joint_names: tuple[str, ...]) -> JointTrajectory:
        message = JointTrajectory()
        message.header.stamp = self.get_clock().now().to_msg()
        message.joint_names = list(joint_names)

        start = JointTrajectoryPoint()
        start.positions = [self._profiles[name].position for name in joint_names]
        start.velocities = [self._profiles[name].velocity for name in joint_names]
        start.time_from_start = Duration()

        endpoint = JointTrajectoryPoint()
        endpoint.positions = []
        endpoint.velocities = []
        for name in joint_names:
            position, velocity = self._profiles[name].preview(self._command_duration)
            endpoint.positions.append(position)
            endpoint.velocities.append(velocity)
        seconds = int(self._command_duration)
        nanoseconds = int(round((self._command_duration - seconds) * 1_000_000_000))
        if nanoseconds >= 1_000_000_000:
            seconds += 1
            nanoseconds -= 1_000_000_000
        endpoint_duration = Duration()
        endpoint_duration.sec = seconds
        endpoint_duration.nanosec = nanoseconds
        endpoint.time_from_start = endpoint_duration
        message.points = [start, endpoint]
        return message

    def _publish_group(self, joint_names: tuple[str, ...]) -> None:
        if not self._targets.initialized:
            return
        message = self._make_trajectory(joint_names)
        if joint_names == ARM_JOINTS:
            self._arm_publisher.publish(message)
        else:
            self._gripper_publisher.publish(message)

    def _publish_all_targets(self) -> None:
        self._publish_group(ARM_JOINTS)
        self._publish_group(GRIPPER_JOINTS)

    def _group_for_joint(self, joint_name: str) -> str:
        return "gripper" if joint_name in GRIPPER_JOINTS else "arm"

    def _advance_motion(self) -> None:
        if not self._targets.initialized:
            return
        now = time.monotonic()
        duration = max(0.0, min(now - self._last_motion_time, 0.25))
        self._last_motion_time = now
        if duration <= 0.0:
            return

        if (
            self._last_feedback_time is not None
            and now - self._last_feedback_time > self._feedback_timeout_s
        ):
            if not self._feedback_stale:
                self._feedback_stale = True
                self.get_logger().warning(
                    "Joint feedback is stale; requesting a controlled stop"
                )
                self._stop_motion(publish=False)

        for profile in self._profiles.values():
            if profile.active:
                profile.advance(duration)

        for group, joint_names in (("arm", ARM_JOINTS), ("gripper", GRIPPER_JOINTS)):
            if group not in self._active_groups:
                continue
            self._publish_group(joint_names)
            if not any(self._profiles[name].active for name in joint_names):
                # The just-published message is the final stationary hold.
                self._active_groups.discard(group)

    def _step_for_selected_joint(self) -> float:
        return (
            self._gripper_step
            if self._selected_joint in GRIPPER_JOINTS
            else self._arm_step
        )

    def _jog(self, direction: float) -> None:
        if not self._targets.initialized:
            self.get_logger().warning("Still waiting for the complete joint state")
            return
        if self._feedback_stale:
            self.get_logger().warning(
                "Joint feedback is stale; wait for feedback before jogging"
            )
            return
        old, new, clamped = self._targets.jog(
            self._selected_joint, direction * self._step_for_selected_joint()
        )
        if new == old:
            return

        profile = self._profiles[self._selected_joint]
        previous_profile_target = profile.target
        applied, lead_clamped = profile.set_target(new)
        if lead_clamped and applied != previous_profile_target:
            self.get_logger().warning(
                f"{self._selected_joint} target lead is limited to "
                f"{profile.max_target_lead:g}; release the key to stop smoothly"
            )
        # Keep future increments relative to the bounded profile target rather
        # than accumulating a hidden queue behind the profile.
        self._targets.set_target(self._selected_joint, applied)
        if applied != previous_profile_target or profile.active:
            self._active_groups.add(self._group_for_joint(self._selected_joint))
        if clamped:
            self.get_logger().warning(
                f"{self._selected_joint} is at the jog safety limit: {new:.6f}"
            )
        else:
            self.get_logger().info(
                f"{self._selected_joint}: {old:.6f} -> {applied:.6f}"
            )

    def _stop_motion(self, *, publish: bool = True) -> None:
        if not self._targets.initialized:
            return
        for name, profile in self._profiles.items():
            before = profile.target
            applied, _ = profile.request_stop()
            self._targets.set_target(name, applied)
            if profile.active or applied != before:
                self._active_groups.add(self._group_for_joint(name))
        if publish:
            for group, joint_names in (
                ("arm", ARM_JOINTS),
                ("gripper", GRIPPER_JOINTS),
            ):
                if group in self._active_groups:
                    self._publish_group(joint_names)
        self.get_logger().info("Controlled stop requested; profiles will decelerate")

    def _recapture_current_pose(self) -> None:
        if (
            self._last_feedback_time is None
            or time.monotonic() - self._last_feedback_time > self._feedback_timeout_s
            or not all(name in self._last_complete_positions for name in CONTROL_JOINTS)
        ):
            self.get_logger().warning(
                "Cannot recapture: complete, current joint state is missing"
            )
            return
        self._targets.initialize(self._last_complete_positions)
        for name in CONTROL_JOINTS:
            self._profiles[name].initialize(self._last_complete_positions[name])
            self._targets.set_target(name, self._profiles[name].position)
        self._active_groups.clear()
        self._last_motion_time = time.monotonic()
        self._publish_all_targets()
        self.get_logger().info("Recaptured feedback and refreshed all hold targets")

    def _print_help(self) -> None:
        self.get_logger().info(
            "Jog keys: 1..6 select arm joint, 7 select finger, +/= positive, "
            "- negative, [ / ] change step, space/s controlled stop, h recapture, "
            "? help, q quit"
        )

    def _handle_key(self, key: str) -> None:
        if key in "123456":
            self._selected_joint = f"joint-{key}"
            self.get_logger().info(f"Selected {self._selected_joint}")
        elif key == "7":
            self._selected_joint = "joint_right-finger"
            self.get_logger().info("Selected joint_right-finger (metres)")
        elif key in "+=":
            self._jog(1.0)
        elif key in "-_":
            self._jog(-1.0)
        elif key == "[":
            self._arm_step *= 0.5
            self._gripper_step *= 0.5
            self.get_logger().info(
                f"Steps: arm={self._arm_step:g}, gripper={self._gripper_step:g}"
            )
        elif key == "]":
            self._arm_step *= 2.0
            self._gripper_step *= 2.0
            self.get_logger().info(
                f"Steps: arm={self._arm_step:g}, gripper={self._gripper_step:g}"
            )
        elif key == " ":
            self._stop_motion()
        elif key in "sS":
            self._stop_motion()
        elif key in "hH":
            self._recapture_current_pose()
        elif key in "?iI":
            self._print_help()
        elif key in "qQ\x03":
            self._stop_motion()
            self._shutdown_stop_sent = True
            self.get_logger().info(
                "Jogger stopped after requesting a controlled stop; use Ctrl+C on "
                "hardware bringup to disable motors"
            )
            # Let the exception unwind rclpy.spin() so the terminal is restored
            # by main() in exactly the same way as Ctrl+C.
            raise KeyboardInterrupt

    def _poll_keyboard(self) -> None:
        if self._keyboard_fd is None:
            return
        while True:
            readable, _, _ = select.select([self._keyboard_fd], [], [], 0.0)
            if not readable:
                return
            data = os.read(self._keyboard_fd, 1)
            if not data:
                return
            self._handle_key(data.decode("utf-8", errors="ignore"))


def main(args=None) -> None:
    if not sys.stdin.isatty():
        print(
            "[rs_jog_controller] stdin is not a TTY; "
            "run this node in an interactive terminal"
        )
        return

    rclpy.init(args=args)
    node = None
    terminal_state = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        node = JogControllerNode()
        node._print_help()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node is not None and not node._shutdown_stop_sent:
            node._stop_motion()
    except ExternalShutdownException:
        pass
    except Exception as exc:
        print(f"[rs_jog_controller] {exc}")
        raise SystemExit(1) from exc
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, terminal_state)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

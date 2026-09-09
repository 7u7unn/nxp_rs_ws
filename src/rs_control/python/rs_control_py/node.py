"""ROS 2 node for reading and controlling seven RobStride motors."""

from __future__ import annotations

import math
import threading
import time

from .bus import Motor, MotorState, RobstrideBus, RobstrideError
from .config import (
    ConfigurationError,
    load_robot_config,
    validate_robot_config,
)


def _parameter_bool(value) -> bool:
    """Accept ROS bool parameters and launch substitutions such as 'false'."""

    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"true", "1", "yes", "on"}:
            return True
        if normalized in {"false", "0", "no", "off"}:
            return False
    raise ConfigurationError(f"read_only must be boolean, got {value!r}")


class PythonRobstrideNode:
    """ROS wrapper around one shared :class:`RobstrideBus` instance."""

    def __init__(self) -> None:
        try:
            import rclpy
            from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
            from rclpy.node import Node
            from rclpy.qos import qos_profile_sensor_data
            from sensor_msgs.msg import JointState
            from trajectory_msgs.msg import JointTrajectory
        except ImportError as exc:
            raise RuntimeError(
                "ROS 2 Python dependencies are unavailable; source /opt/ros/humble/setup.bash"
            ) from exc

        # Keep ROS imports inside the constructor. The protocol and YAML code
        # can then be tested on a development machine without ROS installed.
        self._DiagnosticArray = DiagnosticArray
        self._DiagnosticStatus = DiagnosticStatus
        self._KeyValue = KeyValue
        self._JointState = JointState
        self._node = Node("rs_python_control")

        try:
            self._start(qos_profile_sensor_data, JointTrajectory)
        except Exception:
            self.close()
            self._node.destroy_node()
            raise

    @property
    def node(self):
        return self._node

    def _start(self, sensor_data_qos, JointTrajectory) -> None:
        node = self._node
        config_path = node.declare_parameter("config_file", "").value
        if not config_path:
            raise ConfigurationError("parameter 'config_file' is required")
        config = load_robot_config(str(config_path))

        # Defaults come from YAML; launch/ROS parameter overrides still win.
        config.interface_name = str(
            node.declare_parameter("can_interface", config.interface_name).value
        )
        config.bitrate = int(node.declare_parameter("bitrate", config.bitrate).value)
        config.host_id = int(node.declare_parameter("host_id", config.host_id).value)
        config.response_timeout_ms = int(
            node.declare_parameter("response_timeout_ms", config.response_timeout_ms).value
        )
        config.poll_rate_hz = float(
            node.declare_parameter("poll_rate_hz", config.poll_rate_hz).value
        )
        config.read_only = _parameter_bool(
            node.declare_parameter("read_only", config.read_only).value
        )
        validate_robot_config(config)

        joint_state_topic = str(
            node.declare_parameter("joint_state_topic", "/joint_states").value
        )
        command_topic = str(
            node.declare_parameter(
                "command_topic", "/rs_control/joint_trajectory"
            ).value
        )

        self._config = config
        self._read_only = config.read_only
        self._io_lock = threading.Lock()
        self._closed = False
        self._enabled: set[str] = set()
        self._warned_at: dict[str, float] = {}
        self._states: dict[str, MotorState] = {
            motor.joint_name: MotorState() for motor in config.motors
        }
        self._targets: dict[str, float] = {
            motor.joint_name: 0.0 for motor in config.motors
        }

        motors = {
            motor.joint_name: Motor(id=motor.id, model=motor.model)
            for motor in config.motors
        }
        self._bus = RobstrideBus(
            channel=config.interface_name,
            motors=motors,
            host_id=config.host_id,
            bitrate=config.bitrate,
            response_timeout=config.response_timeout_ms / 1000.0,
        )

        self._joint_state_publisher = node.create_publisher(
            self._JointState, joint_state_topic, sensor_data_qos
        )
        self._diagnostics_publisher = node.create_publisher(
            self._DiagnosticArray, "/diagnostics", 10
        )
        self._command_subscription = node.create_subscription(
            JointTrajectory, command_topic, self._on_trajectory, 10
        )

        self._bus.connect()
        self._read_initial_states()
        if not self._read_only:
            self._enable_control()

        period = 1.0 / config.poll_rate_hz
        self._timer = node.create_timer(period, self._poll)
        node.get_logger().info(
            f"Python RobStride control connected to {config.interface_name}: "
            f"{len(config.motors)} motor(s), read_only={config.read_only}"
        )

    def _read_initial_states(self) -> None:
        for motor in self._config.motors:
            try:
                state = self._bus.read_parameter_state(motor.joint_name)
            except RobstrideError as exc:
                self._warn(motor.joint_name, f"initial state read failed: {exc}")
                if not self._read_only:
                    raise RuntimeError(
                        f"cannot enable control without an initial state from {motor.joint_name}"
                    ) from exc
                continue
            self._states[motor.joint_name] = state
            self._targets[motor.joint_name] = motor.to_joint_position(state.position_rad)

    def _enable_control(self) -> None:
        # The reference requires mode changes while disabled. Configure every
        # motor in order, and keep _enabled precise for safe cleanup.
        for motor in self._config.motors:
            self._bus.disable(motor.joint_name)
            self._bus.set_run_mode(motor.joint_name, 0)
            self._bus.enable(motor.joint_name)
            self._enabled.add(motor.joint_name)

    def _on_trajectory(self, message) -> None:
        if self._read_only or not message.points:
            return
        point = message.points[-1]
        positions = {
            name: float(point.positions[index])
            for index, name in enumerate(message.joint_names)
            if index < len(point.positions) and math.isfinite(point.positions[index])
        }
        if "joint_left-finger" in positions and "joint_right-finger" not in positions:
            positions["joint_right-finger"] = -positions["joint_left-finger"]
        for motor in self._config.motors:
            if motor.joint_name in positions:
                self._targets[motor.joint_name] = positions[motor.joint_name]

    def _poll(self) -> None:
        if self._closed:
            return
        diagnostics = self._DiagnosticArray()
        stamp = self._node.get_clock().now().to_msg()
        diagnostics.header.stamp = stamp

        with self._io_lock:
            for motor in self._config.motors:
                status = self._DiagnosticStatus()
                status.name = f"rs_control/{motor.joint_name}"
                status.hardware_id = f"{self._config.interface_name}:{motor.id}"
                self._add_value(status, "model", motor.model)
                self._add_value(status, "can_id", str(motor.id))
                try:
                    if self._read_only:
                        state = self._bus.read_parameter_state(motor.joint_name)
                    else:
                        motor_position = motor.to_motor_position(
                            self._targets[motor.joint_name]
                        )
                        state = self._bus.write_operation_frame(
                            motor.joint_name,
                            position=motor_position,
                            kp=motor.kp,
                            kd=motor.kd,
                        )
                    self._states[motor.joint_name] = state
                    self._fill_state_diagnostic(status, state)
                    status.level = self._DiagnosticStatus.OK
                    status.message = (
                        "parameter state read OK"
                        if self._read_only
                        else "operation control/status OK"
                    )
                except RobstrideError as exc:
                    status.level = self._DiagnosticStatus.ERROR
                    status.message = str(exc)
                    self._warn(motor.joint_name, str(exc))
                diagnostics.status.append(status)

        self._publish_joint_state(stamp)
        self._diagnostics_publisher.publish(diagnostics)

    def _publish_joint_state(self, stamp) -> None:
        message = self._JointState()
        message.header.stamp = stamp
        for motor in self._config.motors:
            state = self._states[motor.joint_name]
            if not state.valid:
                continue
            message.name.append(motor.joint_name)
            message.position.append(motor.to_joint_position(state.position_rad))
            message.velocity.append(motor.to_joint_velocity(state.velocity_rad_s))
            message.effort.append(motor.to_joint_effort(state.torque_nm))

        finger_index = next(
            (
                index
                for index, name in enumerate(message.name)
                if name == "joint_right-finger"
            ),
            -1,
        )
        if finger_index >= 0:
            message.name.append("joint_left-finger")
            message.position.append(-message.position[finger_index])
            message.velocity.append(-message.velocity[finger_index])
            message.effort.append(message.effort[finger_index])
        if message.name:
            self._joint_state_publisher.publish(message)

    def _fill_state_diagnostic(self, status, state: MotorState) -> None:
        self._add_value(status, "motor_position_rad", f"{state.position_rad:.9f}")
        self._add_value(status, "motor_velocity_rad_s", f"{state.velocity_rad_s:.9f}")
        self._add_value(status, "motor_torque_nm", f"{state.torque_nm:.9f}")
        if math.isfinite(state.temperature_c):
            self._add_value(status, "motor_temperature_c", f"{state.temperature_c:.3f}")
        else:
            self._add_value(
                status,
                "motor_temperature_c",
                "unavailable in parameter-read mode",
            )
        self._add_value(status, "status_flags", hex(state.status_flags))
        self._add_value(status, "fault_code", f"0x{state.fault_code:08x}")
        self._add_value(status, "warning_code", f"0x{state.warning_code:08x}")

    def _add_value(self, status, key: str, value: str) -> None:
        item = self._KeyValue()
        item.key = key
        item.value = value
        status.values.append(item)

    def _warn(self, key: str, message: str) -> None:
        now = time.monotonic()
        if now - self._warned_at.get(key, 0.0) >= 5.0:
            self._warned_at[key] = now
            self._node.get_logger().warning(f"{key}: {message}")

    def close(self) -> None:
        if getattr(self, "_closed", True):
            return
        self._closed = True
        timer = getattr(self, "_timer", None)
        if timer is not None:
            timer.cancel()
        bus = getattr(self, "_bus", None)
        if bus is None:
            return

        with self._io_lock:
            if not self._read_only:
                for motor_name in tuple(self._enabled):
                    try:
                        # Stop the operation command before disabling torque.
                        state = self._states[motor_name]
                        self._bus.write_operation_frame(
                            motor_name,
                            position=state.position_rad,
                            kp=0.0,
                            kd=0.0,
                        )
                    except Exception:
                        pass
                    try:
                        self._bus.disable(motor_name)
                    except Exception:
                        pass
            self._enabled.clear()
            try:
                bus.disconnect()
            except Exception:
                pass

    def destroy_node(self) -> None:
        self.close()
        self._node.destroy_node()

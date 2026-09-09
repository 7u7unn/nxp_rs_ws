"""SocketCAN RobStride bus based on the official Python sample's API flow."""

from __future__ import annotations

from dataclasses import dataclass
import math
import struct
import time
from typing import Callable, Iterable

from .protocol import (
    CommunicationType,
    ParameterSpec,
    ParameterType,
    ProtocolFrame,
    decode_operation_status,
    decode_parameter_response,
    make_disable_frame,
    make_enable_frame,
    make_operation_frame,
    make_parameter_read_frame,
    make_parameter_write_frame,
    make_ping_frame,
    make_set_device_id_frame,
    decode_fault_report,
    describe_fault_report,
    describe_status_flags,
)


class RobstrideError(RuntimeError):
    """Raised for a failed or invalid RobStride transaction."""


@dataclass(frozen=True)
class Motor:
    id: int
    model: str


@dataclass(frozen=True)
class DiscoveredMotor:
    id: int
    uuid: bytes


@dataclass
class MotorState:
    valid: bool = False
    position_rad: float = 0.0
    velocity_rad_s: float = 0.0
    torque_nm: float = 0.0
    temperature_c: float = math.nan
    status_flags: int = 0
    fault_code: int = 0
    warning_code: int = 0


@dataclass(frozen=True)
class _ReceivedFrame:
    communication_type: int
    extra_data: int
    source_id: int
    data: bytes


class RobstrideBus:
    """One shared SocketCAN connection for any number of motors.

    Transactions are intentionally synchronous. Each request waits for its
    matching response before the next motor is queried, preventing a slow or
    missing motor from causing responses to be attributed to another motor.
    """

    def __init__(
        self,
        channel: str,
        motors: dict[str, Motor] | None = None,
        *,
        host_id: int = 0xFF,
        bitrate: int = 1_000_000,
        response_timeout: float = 0.02,
    ) -> None:
        self.channel = channel
        self.motors = motors or {}
        self.host_id = int(host_id)
        self.bitrate = int(bitrate)
        self.response_timeout = float(response_timeout)
        self._channel_handler = None

        if not 0 <= self.host_id <= 0xFF:
            raise ValueError("host_id must be in the range 0..255")
        if self.bitrate <= 0 or self.response_timeout <= 0:
            raise ValueError("bitrate and response_timeout must be positive")

    @property
    def is_connected(self) -> bool:
        return self._channel_handler is not None

    def connect(self) -> None:
        if self.is_connected:
            raise RobstrideError(f"'{self.channel}' is already connected")
        try:
            import can
        except ImportError as exc:
            raise RobstrideError(
                "python-can is required; install the python3-can package"
            ) from exc
        try:
            self._channel_handler = can.interface.Bus(
                interface="socketcan",
                channel=self.channel,
                bitrate=self.bitrate,
            )
        except Exception as exc:
            self._channel_handler = None
            raise RobstrideError(
                f"unable to connect to SocketCAN interface '{self.channel}': {exc}"
            ) from exc

    def disconnect(self) -> None:
        handler = self._channel_handler
        self._channel_handler = None
        if handler is not None:
            handler.shutdown()

    def __enter__(self) -> "RobstrideBus":
        self.connect()
        return self

    def __exit__(self, _type, _value, _traceback) -> None:
        self.disconnect()

    def _require_connected(self):
        if self._channel_handler is None:
            raise RobstrideError("CAN bus is not connected")
        return self._channel_handler

    @staticmethod
    def _decode_frame(message) -> _ReceivedFrame | None:
        if not bool(getattr(message, "is_extended_id", False)):
            return None
        arbitration_id = int(message.arbitration_id) & 0x1FFFFFFF
        return _ReceivedFrame(
            communication_type=(arbitration_id >> 24) & 0x1F,
            extra_data=(arbitration_id >> 8) & 0xFFFF,
            source_id=arbitration_id & 0xFF,
            data=bytes(message.data),
        )

    def transmit(self, frame: ProtocolFrame) -> None:
        handler = self._require_connected()
        if len(frame.data) > 8:
            raise ValueError("CAN data length must not exceed eight bytes")
        try:
            import can

            message = can.Message(
                arbitration_id=frame.arbitration_id,
                is_extended_id=True,
                data=frame.data,
            )
            handler.send(message)
        except Exception as exc:
            raise RobstrideError(f"failed to transmit RobStride frame: {exc}") from exc

    @staticmethod
    def _matches(
        frame: _ReceivedFrame,
        expected_types: set[int],
        device_id: int | None,
        matcher: Callable[[_ReceivedFrame], bool] | None,
    ) -> bool:
        if expected_types and frame.communication_type not in expected_types:
            return False
        if device_id is not None and frame.communication_type in {
            CommunicationType.OPERATION_STATUS,
            CommunicationType.FAULT_REPORT,
            CommunicationType.READ_PARAMETER,
            CommunicationType.WRITE_PARAMETER,
        }:
            # Type-2 and parameter responses carry the responding actuator ID
            # in the low byte of extra data.  A few firmware revisions put the
            # type-21 motor ID in the low CAN-ID byte instead, so accept either
            # layout only for fault reports.
            matches_extra_id = (frame.extra_data & 0xFF) == device_id
            matches_source_id = frame.source_id == device_id
            if frame.communication_type == CommunicationType.FAULT_REPORT:
                if not (matches_extra_id or matches_source_id):
                    return False
            elif not matches_extra_id:
                return False
        return matcher is None or matcher(frame)

    def receive(
        self,
        expected_types: Iterable[int],
        *,
        device_id: int | None = None,
        timeout: float | None = None,
        matcher: Callable[[_ReceivedFrame], bool] | None = None,
    ) -> _ReceivedFrame:
        """Receive one matching extended frame, ignoring unrelated traffic."""

        handler = self._require_connected()
        expected = {int(value) for value in expected_types}
        wait_time = self.response_timeout if timeout is None else float(timeout)
        if wait_time <= 0:
            raise ValueError("receive timeout must be positive")
        deadline = time.monotonic() + wait_time

        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            try:
                message = handler.recv(timeout=remaining)
            except Exception as exc:
                raise RobstrideError(f"failed to receive RobStride frame: {exc}") from exc
            if message is None:
                break
            frame = self._decode_frame(message)
            if frame is None:
                continue
            if self._matches(frame, expected, device_id, matcher):
                return frame
            # Discard unrelated/late frames. Only one request is outstanding,
            # so retaining an active-report stream would grow memory without
            # improving correlation.

        types = ",".join(str(value) for value in sorted(expected))
        target = "" if device_id is None else f" from motor {device_id}"
        raise RobstrideError(
            f"timeout waiting for RobStride response{target} (types {types})"
        )

    def _motor(self, motor: str | Motor) -> Motor:
        if isinstance(motor, Motor):
            return motor
        try:
            return self.motors[motor]
        except KeyError as exc:
            raise RobstrideError(f"unknown motor '{motor}'") from exc

    def read(self, motor: str | Motor, parameter: ParameterSpec) -> int | float:
        target = self._motor(motor)
        self.transmit(make_parameter_read_frame(self.host_id, target.id, parameter))

        expected_parameter = struct.pack("<H", parameter.index)

        def matches_parameter(frame: _ReceivedFrame) -> bool:
            # A type-17 response echoes the requested parameter in its first
            # two bytes. Accept a zero echo for older firmware variants, but
            # reject a different non-zero echo so seven sequential reads cannot
            # consume one another's stale replies.
            if len(frame.data) < 2:
                return False
            return frame.data[:2] in {expected_parameter, b"\x00\x00"}

        frame = self.receive(
            {CommunicationType.READ_PARAMETER},
            device_id=target.id,
            matcher=matches_parameter,
        )
        try:
            value = decode_parameter_response(parameter, frame.data)
        except (struct.error, ValueError) as exc:
            raise RobstrideError(
                f"invalid response for {parameter.name} from motor {target.id}: {exc}"
            ) from exc
        return value

    def write(
        self, motor: str | Motor, parameter: ParameterSpec, value: int | float
    ) -> MotorState:
        target = self._motor(motor)
        self.transmit(
            make_parameter_write_frame(self.host_id, target.id, parameter, value)
        )
        return self.receive_status_frame(target)

    def read_parameter_state(self, motor: str | Motor) -> MotorState:
        """Read safe state parameters without enabling or commanding torque."""

        position = float(self.read(motor, ParameterType.MECHANICAL_POSITION))
        velocity = float(self.read(motor, ParameterType.MECHANICAL_VELOCITY))
        torque = float(self.read(motor, ParameterType.MEASURED_TORQUE))
        return MotorState(
            valid=True,
            position_rad=position,
            velocity_rad_s=velocity,
            torque_nm=torque,
            temperature_c=math.nan,
        )

    def receive_status_frame(self, motor: str | Motor) -> MotorState:
        target = self._motor(motor)
        frame = self.receive(
            {CommunicationType.OPERATION_STATUS, CommunicationType.FAULT_REPORT},
            device_id=target.id,
        )
        if frame.communication_type == CommunicationType.FAULT_REPORT:
            try:
                fault_code, warning_code = decode_fault_report(frame.data)
            except ValueError as exc:
                raise RobstrideError(
                    f"motor {target.id} returned a truncated fault report "
                    f"(data={frame.data.hex()}): {exc}"
                ) from exc
            raise RobstrideError(
                f"motor {target.id} returned a fault report "
                f"({describe_fault_report(fault_code, warning_code)}, "
                f"extra_data=0x{frame.extra_data:04x}, "
                f"source_id=0x{frame.source_id:02x}, data={frame.data.hex()})"
            )
        status_flags = frame.extra_data & 0x3F00
        if status_flags:
            raise RobstrideError(
                f"motor {target.id} operation status reports protection flags "
                f"{describe_status_flags(status_flags)} "
                f"(extra_data=0x{frame.extra_data:04x})"
            )
        try:
            position, velocity, torque, temperature = decode_operation_status(
                target.model, frame.data
            )
        except (struct.error, ValueError) as exc:
            raise RobstrideError(
                f"invalid operation status from motor {target.id}: {exc}"
            ) from exc
        return MotorState(
            valid=True,
            position_rad=position,
            velocity_rad_s=velocity,
            torque_nm=torque,
            temperature_c=temperature,
            status_flags=status_flags,
        )

    def write_operation_frame(
        self,
        motor: str | Motor,
        *,
        position: float,
        kp: float,
        kd: float,
        velocity: float = 0.0,
        torque: float = 0.0,
    ) -> MotorState:
        target = self._motor(motor)
        self.transmit(
            make_operation_frame(
                target.id,
                target.model,
                position=position,
                kp=kp,
                kd=kd,
                velocity=velocity,
                torque=torque,
            )
        )
        return self.receive_status_frame(target)

    def enable(self, motor: str | Motor) -> MotorState:
        target = self._motor(motor)
        self.transmit(make_enable_frame(self.host_id, target.id))
        return self.receive_status_frame(target)

    def disable(self, motor: str | Motor) -> MotorState:
        target = self._motor(motor)
        self.transmit(make_disable_frame(self.host_id, target.id))
        return self.receive_status_frame(target)

    def set_run_mode(self, motor: str | Motor, run_mode: int) -> MotorState:
        if run_mode not in {0, 1, 2, 3, 5}:
            raise ValueError("run_mode must be one of 0, 1, 2, 3, or 5")
        return self.write(motor, ParameterType.MODE, run_mode)

    def ping_by_id(self, device_id: int, timeout: float | None = None) -> DiscoveredMotor:
        self.transmit(make_ping_frame(self.host_id, device_id))

        def matches_device(frame: _ReceivedFrame) -> bool:
            # A type-0 reply puts the target motor ID in data-area 2 and the
            # broadcast reply address in the low CAN-ID byte (0xFE).
            return (frame.extra_data & 0xFF) == device_id

        frame = self.receive(
            {CommunicationType.GET_DEVICE_ID},
            timeout=timeout,
            matcher=matches_device,
        )
        return DiscoveredMotor(id=device_id, uuid=frame.data[:8].ljust(8, b"\x00"))

    def set_device_id(
        self,
        old_device_id: int,
        new_device_id: int,
        timeout: float | None = None,
    ) -> DiscoveredMotor:
        """Change one motor's ID using the reference type-7 transaction."""

        self.transmit(
            make_set_device_id_frame(self.host_id, old_device_id, new_device_id)
        )
        # Type-7 acknowledges with a type-0 broadcast response. Firmware
        # versions do not all echo the new ID in the same extra-data bytes, so
        # the caller verifies the new ID with a fresh ping afterward.
        frame = self.receive(
            {CommunicationType.GET_DEVICE_ID},
            timeout=timeout,
        )
        return DiscoveredMotor(
            id=new_device_id,
            uuid=frame.data[:8].ljust(8, b"\x00"),
        )

    def scan_channel(
        self, start_id: int = 1, end_id: int = 255
    ) -> list[DiscoveredMotor]:
        if not 1 <= start_id <= end_id <= 255:
            raise ValueError("scan IDs must satisfy 1 <= start_id <= end_id <= 255")
        discovered: list[DiscoveredMotor] = []
        for device_id in range(start_id, end_id + 1):
            if device_id == self.host_id:
                continue
            try:
                discovered.append(
                    self.ping_by_id(device_id, timeout=self.response_timeout)
                )
            except RobstrideError as exc:
                if "timeout" not in str(exc).lower():
                    raise
        return discovered

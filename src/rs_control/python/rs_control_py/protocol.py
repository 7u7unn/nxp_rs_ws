"""RobStride private CAN protocol helpers.

The layout follows the public RobStride Python sample:

* communication type: extended CAN ID bits 28..24;
* extra data: extended CAN ID bits 23..8;
* device ID: extended CAN ID bits 7..0;
* parameter values are little-endian, while operation status fields are
  big-endian.

Keeping the packing in this small module makes it possible to test the wire
format without a CAN adapter or the ``python-can`` package installed.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import math
import struct
from typing import Final


class CommunicationType(IntEnum):
    """RobStride private communication types."""

    GET_DEVICE_ID = 0
    OPERATION_CONTROL = 1
    OPERATION_STATUS = 2
    ENABLE = 3
    DISABLE = 4
    SET_ZERO_POSITION = 6
    SET_DEVICE_ID = 7
    READ_PARAMETER = 17
    WRITE_PARAMETER = 18
    FAULT_REPORT = 21
    SAVE_PARAMETERS = 22
    SET_BAUDRATE = 23
    ACTIVE_REPORT = 24
    SET_PROTOCOL = 25


# Protection bits exposed in the type-2 CAN-ID status field.  The values are
# kept in their wire positions so diagnostics can be compared directly with
# a candump trace.
STATUS_FLAG_NAMES: Final[dict[int, str]] = {
    0x2000: "encoder uncalibrated",
    0x1000: "stall/overload",
    0x0800: "magnetic encoder fault",
    0x0400: "overtemperature",
    0x0200: "overcurrent",
    0x0100: "undervoltage",
}

FAULT_FLAG_NAMES: Final[dict[int, str]] = {
    1 << 0: "motor overtemperature",
    1 << 1: "driver IC fault",
    1 << 2: "undervoltage",
    1 << 3: "overvoltage",
    1 << 4: "B-phase overcurrent",
    1 << 5: "C-phase overcurrent",
    1 << 7: "encoder uncalibrated",
    1 << 8: "hardware ID fault",
    1 << 9: "position initialization fault",
    1 << 14: "stall/overload",
    1 << 16: "A-phase overcurrent",
}

WARNING_FLAG_NAMES: Final[dict[int, str]] = {
    1 << 0: "motor overtemperature warning",
}


def _describe_bits(value: int, names: dict[int, str], width: int) -> str:
    labels = [name for mask, name in names.items() if value & mask]
    known = 0
    for mask in names:
        known |= mask
    unknown = value & ~known
    if unknown:
        labels.append(f"unknown=0x{unknown:0{width}x}")
    return ", ".join(labels) if labels else "none"


def describe_status_flags(status_flags: int) -> str:
    """Describe the raw type-2 protection flags."""

    value = int(status_flags) & 0xFFFF
    return f"0x{value:04x} ({_describe_bits(value, STATUS_FLAG_NAMES, 4)})"


def describe_fault_report(fault_code: int, warning_code: int) -> str:
    """Describe raw little-endian type-21 fault and warning bitmasks."""

    fault = int(fault_code) & 0xFFFFFFFF
    warning = int(warning_code) & 0xFFFFFFFF
    return (
        f"fault=0x{fault:08x} ({_describe_bits(fault, FAULT_FLAG_NAMES, 8)}), "
        f"warning=0x{warning:08x} ({_describe_bits(warning, WARNING_FLAG_NAMES, 8)})"
    )


def decode_fault_report(data: bytes) -> tuple[int, int]:
    """Decode the eight-byte type-21 payload as little-endian uint32 values."""

    if len(data) < 8:
        raise ValueError("fault report is shorter than eight bytes")
    return struct.unpack("<II", data[:8])


@dataclass(frozen=True)
class ParameterSpec:
    """A parameter index and its four-byte wire representation."""

    index: int
    fmt: str
    name: str


class ParameterType:
    """Parameter indices used by the driver."""

    MEASURED_POSITION: Final = ParameterSpec(0x3016, "<f", "mechPos")
    MEASURED_VELOCITY: Final = ParameterSpec(0x3017, "<f", "mechVel")
    MEASURED_TORQUE: Final = ParameterSpec(0x302C, "<f", "torque_fdb")
    MODE: Final = ParameterSpec(0x7005, "<b", "run_mode")
    IQ_TARGET: Final = ParameterSpec(0x7006, "<f", "iq_ref")
    VELOCITY_TARGET: Final = ParameterSpec(0x700A, "<f", "spd_ref")
    TORQUE_LIMIT: Final = ParameterSpec(0x700B, "<f", "limit_torque")
    CURRENT_KP: Final = ParameterSpec(0x7010, "<f", "cur_kp")
    CURRENT_KI: Final = ParameterSpec(0x7011, "<f", "cur_ki")
    CURRENT_FILTER_GAIN: Final = ParameterSpec(0x7014, "<f", "cur_filter_gain")
    POSITION_TARGET: Final = ParameterSpec(0x7016, "<f", "loc_ref")
    VELOCITY_LIMIT: Final = ParameterSpec(0x7017, "<f", "limit_spd")
    CURRENT_LIMIT: Final = ParameterSpec(0x7018, "<f", "limit_cur")
    MECHANICAL_POSITION: Final = ParameterSpec(0x7019, "<f", "mechPos")
    IQ_FILTERED: Final = ParameterSpec(0x701A, "<f", "iqf")
    MECHANICAL_VELOCITY: Final = ParameterSpec(0x701B, "<f", "mechVel")
    VBUS: Final = ParameterSpec(0x701C, "<f", "VBUS")
    POSITION_KP: Final = ParameterSpec(0x701E, "<f", "loc_kp")
    VELOCITY_KP: Final = ParameterSpec(0x701F, "<f", "spd_kp")
    VELOCITY_KI: Final = ParameterSpec(0x7020, "<f", "spd_ki")
    VELOCITY_FILTER_GAIN: Final = ParameterSpec(0x7021, "<f", "spd_filter_gain")
    VEL_ACCELERATION_TARGET: Final = ParameterSpec(0x7022, "<f", "acc_rad")
    PP_VELOCITY_MAX: Final = ParameterSpec(0x7024, "<f", "vel_max")
    PP_ACCELERATION_TARGET: Final = ParameterSpec(0x7025, "<f", "acc_set")
    EPSCAN_TIME: Final = ParameterSpec(0x7026, "<H", "EPScan_time")
    CAN_TIMEOUT: Final = ParameterSpec(0x7028, "<I", "canTimeout")
    ZERO_STATE: Final = ParameterSpec(0x7029, "<B", "zero_sta")


@dataclass(frozen=True)
class MitLimits:
    position_rad: float
    velocity_rad_s: float
    torque_nm: float
    kp: float
    kd: float


# These are the same model tables used by the RobStride Python sample.
MODEL_MIT_LIMITS: Final[dict[str, MitLimits]] = {
    "rs-00": MitLimits(4.0 * math.pi, 50.0, 17.0, 500.0, 5.0),
    "rs-01": MitLimits(4.0 * math.pi, 44.0, 17.0, 500.0, 5.0),
    "rs-02": MitLimits(4.0 * math.pi, 44.0, 17.0, 500.0, 5.0),
    "rs-03": MitLimits(4.0 * math.pi, 50.0, 60.0, 5000.0, 100.0),
    "rs-04": MitLimits(4.0 * math.pi, 15.0, 120.0, 5000.0, 100.0),
    "rs-05": MitLimits(4.0 * math.pi, 33.0, 17.0, 500.0, 5.0),
    "rs-06": MitLimits(4.0 * math.pi, 20.0, 60.0, 5000.0, 100.0),
}


@dataclass(frozen=True)
class ProtocolFrame:
    """A frame before it is converted to a python-can Message."""

    arbitration_id: int
    data: bytes = b"\x00" * 8


def canonical_model(model: str) -> str:
    """Return the lower-case model spelling used by the scaling tables."""

    normalized = model.strip().lower().replace("_", "-")
    if normalized.startswith("rs") and not normalized.startswith("rs-"):
        normalized = "rs-" + normalized[2:]
    if normalized not in MODEL_MIT_LIMITS:
        raise ValueError(f"unsupported RobStride model '{model}'")
    return normalized


def limits_for_model(model: str) -> MitLimits:
    return MODEL_MIT_LIMITS[canonical_model(model)]


def compose_extended_id(
    communication_type: int, extra_data: int, device_id: int
) -> int:
    """Pack a RobStride communication type, extra field, and device ID."""

    if not 0 <= int(communication_type) <= 0x1F:
        raise ValueError("communication type must be in the range 0..31")
    if not 0 <= int(extra_data) <= 0xFFFF:
        raise ValueError("extra data must be in the range 0..65535")
    if not 1 <= int(device_id) <= 0xFF:
        raise ValueError("device ID must be in the range 1..255")
    return (int(communication_type) << 24) | (int(extra_data) << 8) | int(device_id)


def _encode_symmetric(value: float, limit: float) -> int:
    if not math.isfinite(value):
        value = 0.0
    value = max(-limit, min(limit, float(value)))
    # The RobStride sample uses 0x7fff for signed MIT fields and clips the
    # resulting integer to the 16-bit wire range.
    encoded = int(((value / limit) + 1.0) * 0x7FFF)
    return max(0, min(0xFFFF, encoded))


def _encode_positive(value: float, limit: float) -> int:
    if not math.isfinite(value):
        value = 0.0
    value = max(0.0, min(limit, float(value)))
    return max(0, min(0xFFFF, int((value / limit) * 0xFFFF)))


def decode_symmetric(encoded: int, limit: float) -> float:
    """Decode a signed MIT field using the sample's 0x7fff scale."""

    return (float(encoded) / 0x7FFF - 1.0) * limit


def make_operation_frame(
    device_id: int,
    model: str,
    position: float,
    kp: float,
    kd: float,
    velocity: float = 0.0,
    torque: float = 0.0,
) -> ProtocolFrame:
    """Build the reference sample's type-1 MIT operation frame."""

    limits = limits_for_model(model)
    position_u16 = _encode_symmetric(position, limits.position_rad)
    velocity_u16 = _encode_symmetric(velocity, limits.velocity_rad_s)
    kp_u16 = _encode_positive(kp, limits.kp)
    kd_u16 = _encode_positive(kd, limits.kd)
    torque_u16 = _encode_symmetric(torque, limits.torque_nm)
    data = struct.pack(">HHHH", position_u16, velocity_u16, kp_u16, kd_u16)
    return ProtocolFrame(
        compose_extended_id(
            CommunicationType.OPERATION_CONTROL, torque_u16, device_id
        ),
        data,
    )


def make_parameter_read_frame(
    host_id: int, device_id: int, parameter: ParameterSpec
) -> ProtocolFrame:
    data = struct.pack("<HHL", parameter.index, 0, 0)
    return ProtocolFrame(
        compose_extended_id(CommunicationType.READ_PARAMETER, host_id, device_id),
        data,
    )


def _pack_parameter_value(parameter: ParameterSpec, value: int | float) -> bytes:
    if parameter.fmt == "<b":
        return struct.pack("<bBH", int(value), 0, 0)
    if parameter.fmt == "<B":
        return struct.pack("<BBH", int(value), 0, 0)
    if parameter.fmt == "<h":
        return struct.pack("<hH", int(value), 0)
    if parameter.fmt == "<H":
        return struct.pack("<HH", int(value), 0)
    if parameter.fmt == "<I":
        return struct.pack("<I", int(value))
    if parameter.fmt == "<i":
        return struct.pack("<i", int(value))
    if parameter.fmt == "<f":
        return struct.pack("<f", float(value))
    raise ValueError(f"unsupported parameter type {parameter.name}")


def make_parameter_write_frame(
    host_id: int, device_id: int, parameter: ParameterSpec, value: int | float
) -> ProtocolFrame:
    data = struct.pack("<HH", parameter.index, 0) + _pack_parameter_value(
        parameter, value
    )
    return ProtocolFrame(
        compose_extended_id(CommunicationType.WRITE_PARAMETER, host_id, device_id),
        data,
    )


def make_enable_frame(host_id: int, device_id: int) -> ProtocolFrame:
    return ProtocolFrame(compose_extended_id(CommunicationType.ENABLE, host_id, device_id))


def make_disable_frame(host_id: int, device_id: int) -> ProtocolFrame:
    return ProtocolFrame(compose_extended_id(CommunicationType.DISABLE, host_id, device_id))


def make_ping_frame(host_id: int, device_id: int) -> ProtocolFrame:
    return ProtocolFrame(
        compose_extended_id(CommunicationType.GET_DEVICE_ID, host_id, device_id)
    )


def make_set_device_id_frame(
    host_id: int, old_device_id: int, new_device_id: int
) -> ProtocolFrame:
    """Build the reference sample's type-7 CAN-ID change frame."""

    if not 0 <= int(host_id) <= 0xFF:
        raise ValueError("host ID must be in the range 0..255")
    if not 1 <= int(old_device_id) <= 0xFF:
        raise ValueError("old device ID must be in the range 1..255")
    if not 1 <= int(new_device_id) <= 0xFF:
        raise ValueError("new device ID must be in the range 1..255")
    if int(new_device_id) == int(host_id):
        raise ValueError("new device ID must differ from host ID")
    extra_data = (int(new_device_id) << 8) | int(host_id)
    return ProtocolFrame(
        compose_extended_id(
            CommunicationType.SET_DEVICE_ID, extra_data, int(old_device_id)
        )
    )


def decode_parameter_response(parameter: ParameterSpec, data: bytes) -> int | float:
    """Decode the value from a type-17 response payload."""

    if len(data) < 8:
        raise ValueError("parameter response is shorter than eight bytes")
    value = struct.unpack(parameter.fmt, data[4: 4 + struct.calcsize(parameter.fmt)])[0]
    if isinstance(value, float) and not math.isfinite(value):
        raise ValueError(f"parameter {parameter.name} returned a non-finite value")
    return value


def decode_operation_status(
    model: str, data: bytes
) -> tuple[float, float, float, float]:
    """Decode position, velocity, torque, and temperature from type 2."""

    if len(data) < 8:
        raise ValueError("operation status frame is shorter than eight bytes")
    limits = limits_for_model(model)
    position_u16, velocity_u16, torque_u16, temperature_u16 = struct.unpack(
        ">HHHH", data[:8]
    )
    return (
        decode_symmetric(position_u16, limits.position_rad),
        decode_symmetric(velocity_u16, limits.velocity_rad_s),
        decode_symmetric(torque_u16, limits.torque_nm),
        float(temperature_u16) * 0.1,
    )

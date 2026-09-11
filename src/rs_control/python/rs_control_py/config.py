"""YAML configuration and joint/motor calibration for the Python driver."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from pathlib import Path
from typing import Any

from .protocol import canonical_model, limits_for_model


@dataclass
class MotorConfig:
    joint_name: str
    id: int
    model: str
    direction: int = 1
    position_offset: float = 0.0
    position_scale: float = 1.0
    motor_position_min: float | None = None
    motor_position_max: float | None = None
    kp: float = 500.0
    kd: float = 5.0

    @property
    def has_position_limits(self) -> bool:
        return self.motor_position_min is not None and self.motor_position_max is not None

    def to_joint_position(self, motor_position: float) -> float:
        return (
            (float(motor_position) - self.position_offset)
            * self.direction
            * self.position_scale
        )

    def to_joint_velocity(self, motor_velocity: float) -> float:
        return float(motor_velocity) * self.direction * self.position_scale

    def to_joint_effort(self, motor_torque: float) -> float:
        return float(motor_torque) * self.direction

    def to_motor_position(self, joint_position: float) -> float:
        motor_position = (
            float(joint_position) * self.direction / self.position_scale
            + self.position_offset
        )
        if self.has_position_limits:
            assert self.motor_position_min is not None
            assert self.motor_position_max is not None
            motor_position = max(
                self.motor_position_min, min(self.motor_position_max, motor_position)
            )
        return motor_position

    def to_motor_velocity(self, joint_velocity: float) -> float:
        return float(joint_velocity) * self.direction / self.position_scale


@dataclass
class RobotConfig:
    interface_name: str = "can0"
    bitrate: int = 1_000_000
    host_id: int = 0xFF
    response_timeout_ms: int = 20
    poll_rate_hz: float = 100.0
    read_only: bool = True
    motors: list[MotorConfig] = field(default_factory=list)


class ConfigurationError(ValueError):
    """Raised when a RobStride YAML file is invalid."""


def _required(mapping: dict[str, Any], key: str, context: str) -> Any:
    if key not in mapping:
        raise ConfigurationError(f"{context} is missing '{key}'")
    return mapping[key]


def _as_bool(value: Any, key: str) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"true", "1", "yes", "on"}:
            return True
        if normalized in {"false", "0", "no", "off"}:
            return False
    raise ConfigurationError(f"'{key}' must be boolean")


def _as_finite(value: Any, key: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ConfigurationError(f"'{key}' must be numeric") from exc
    if not math.isfinite(result):
        raise ConfigurationError(f"'{key}' must be finite")
    return result


def load_robot_config(path: str | Path) -> RobotConfig:
    """Load and validate the shared ``robstride.yaml`` format."""

    try:
        import yaml
    except ImportError as exc:
        raise ConfigurationError(
            "PyYAML is required; install the python3-yaml package"
        ) from exc

    config_path = Path(path).expanduser()
    if not config_path.is_file():
        raise ConfigurationError(f"configuration file does not exist: {config_path}")
    try:
        raw = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    except (OSError, yaml.YAMLError) as exc:
        raise ConfigurationError(f"unable to read {config_path}: {exc}") from exc
    if not isinstance(raw, dict):
        raise ConfigurationError("top-level configuration must be a mapping")

    result = RobotConfig(
        interface_name=str(raw.get("can_interface", "can0")),
        bitrate=int(raw.get("bitrate", 1_000_000)),
        host_id=int(raw.get("host_id", 0xFF)),
        response_timeout_ms=int(raw.get("response_timeout_ms", 20)),
        poll_rate_hz=_as_finite(raw.get("poll_rate_hz", 100.0), "poll_rate_hz"),
        read_only=_as_bool(raw.get("read_only", True), "read_only"),
    )

    raw_motors = raw.get("motors")
    if not isinstance(raw_motors, list) or not raw_motors:
        raise ConfigurationError("'motors' must contain at least one motor")

    for index, raw_motor in enumerate(raw_motors):
        context = f"motor entry {index}"
        if not isinstance(raw_motor, dict):
            raise ConfigurationError(f"{context} must be a mapping")
        joint_name = str(_required(raw_motor, "joint", context))
        motor_id = int(_required(raw_motor, "id", context))
        model = canonical_model(str(_required(raw_motor, "model", context)))
        direction = int(raw_motor.get("direction", 1))
        position_offset = _as_finite(
            raw_motor.get("position_offset", 0.0), f"{context}.position_offset"
        )
        position_scale = _as_finite(
            raw_motor.get("position_scale", 1.0), f"{context}.position_scale"
        )
        position_min = raw_motor.get("motor_position_min")
        position_max = raw_motor.get("motor_position_max")
        if (position_min is None) != (position_max is None):
            raise ConfigurationError(
                f"{context} must define both motor_position_min and motor_position_max"
            )
        if position_min is not None:
            position_min = _as_finite(position_min, f"{context}.motor_position_min")
            position_max = _as_finite(position_max, f"{context}.motor_position_max")
        kp = _as_finite(raw_motor.get("kp", 500.0), f"{context}.kp")
        kd = _as_finite(raw_motor.get("kd", 5.0), f"{context}.kd")
        result.motors.append(
            MotorConfig(
                joint_name=joint_name,
                id=motor_id,
                model=model,
                direction=direction,
                position_offset=position_offset,
                position_scale=position_scale,
                motor_position_min=position_min,
                motor_position_max=position_max,
                kp=kp,
                kd=kd,
            )
        )

    validate_robot_config(result)
    return result


def validate_robot_config(config: RobotConfig) -> None:
    if not config.interface_name:
        raise ConfigurationError("can_interface must not be empty")
    if config.bitrate <= 0:
        raise ConfigurationError("bitrate must be positive")
    if not 0 <= config.host_id <= 0xFF:
        raise ConfigurationError("host_id must be in the range 0..255")
    if config.response_timeout_ms <= 0:
        raise ConfigurationError("response_timeout_ms must be positive")
    if not math.isfinite(config.poll_rate_hz) or config.poll_rate_hz <= 0:
        raise ConfigurationError("poll_rate_hz must be positive")
    if not config.motors:
        raise ConfigurationError("motor config must contain at least one motor")

    joint_names: set[str] = set()
    motor_ids: set[int] = set()
    for motor in config.motors:
        if not motor.joint_name:
            raise ConfigurationError("motor joint names must not be empty")
        if motor.joint_name in joint_names:
            raise ConfigurationError(f"duplicate motor joint '{motor.joint_name}'")
        joint_names.add(motor.joint_name)
        if not 1 <= motor.id <= 0xFF:
            raise ConfigurationError(
                f"motor '{motor.joint_name}' CAN id must be in the range 1..255"
            )
        if motor.id in motor_ids:
            raise ConfigurationError(f"duplicate CAN id {motor.id}")
        motor_ids.add(motor.id)
        if motor.id == config.host_id:
            raise ConfigurationError(
                f"motor '{motor.joint_name}' CAN id must differ from host_id"
            )
        if motor.direction not in {-1, 1}:
            raise ConfigurationError(
                f"motor '{motor.joint_name}' direction must be 1 or -1"
            )
        if not math.isfinite(motor.position_scale) or motor.position_scale <= 0:
            raise ConfigurationError(
                f"motor '{motor.joint_name}' position_scale must be positive"
            )
        if motor.has_position_limits:
            assert motor.motor_position_min is not None
            assert motor.motor_position_max is not None
            if motor.motor_position_min >= motor.motor_position_max:
                raise ConfigurationError(
                    f"motor '{motor.joint_name}' motor position limits must be strictly increasing"
                )
        limits = limits_for_model(motor.model)
        if not 0 <= motor.kp <= limits.kp:
            raise ConfigurationError(
                f"motor '{motor.joint_name}' kp is outside the model range"
            )
        if not 0 <= motor.kd <= limits.kd:
            raise ConfigurationError(
                f"motor '{motor.joint_name}' kd is outside the model range"
            )

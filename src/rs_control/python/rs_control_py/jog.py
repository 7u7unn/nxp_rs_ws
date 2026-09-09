"""Pure target bookkeeping for the interactive RobStride jogger."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Mapping


ARM_JOINTS = tuple(f"joint-{index}" for index in range(1, 7))
GRIPPER_JOINTS = ("joint_right-finger",)
CONTROL_JOINTS = ARM_JOINTS + GRIPPER_JOINTS


@dataclass(frozen=True)
class JointLimit:
    """Allowed command interval in the ROS joint coordinate."""

    lower: float
    upper: float

    def __post_init__(self) -> None:
        if not math.isfinite(self.lower) or not math.isfinite(self.upper):
            raise ValueError("joint limits must be finite")
        if self.lower >= self.upper:
            raise ValueError("joint lower limit must be less than upper limit")


class JogTargetState:
    """Track a complete command pose while changing one joint at a time."""

    def __init__(
        self,
        limits: Mapping[str, JointLimit],
        required_joints: Iterable[str] = CONTROL_JOINTS,
    ) -> None:
        self._required_joints = tuple(required_joints)
        self._limits = dict(limits)
        missing_limits = [
            name for name in self._required_joints if name not in self._limits
        ]
        if missing_limits:
            raise ValueError(f"missing limits for: {', '.join(missing_limits)}")
        self._targets: dict[str, float] = {}

    @property
    def initialized(self) -> bool:
        return bool(self._targets)

    @property
    def joint_names(self) -> tuple[str, ...]:
        return self._required_joints

    @property
    def targets(self) -> dict[str, float]:
        """Return a copy so callers cannot alter the held pose accidentally."""
        return dict(self._targets)

    def initialize(self, positions: Mapping[str, float]) -> None:
        """Capture a complete feedback pose, including positions at a stop."""
        missing = [name for name in self._required_joints if name not in positions]
        if missing:
            raise ValueError(f"feedback is missing: {', '.join(missing)}")

        captured: dict[str, float] = {}
        for name in self._required_joints:
            value = float(positions[name])
            if not math.isfinite(value):
                raise ValueError(f"feedback for '{name}' is not finite")
            captured[name] = value
        self._targets = captured

    def target(self, joint_name: str) -> float:
        if not self.initialized:
            raise RuntimeError("jog target state is not initialized")
        try:
            return self._targets[joint_name]
        except KeyError as exc:
            raise ValueError(f"unknown joint '{joint_name}'") from exc

    def set_target(self, joint_name: str, value: float) -> float:
        """Set a target explicitly and clamp it to the configured joint limit."""
        if not self.initialized:
            raise RuntimeError("cannot set a target before the initial feedback pose")
        if joint_name not in self._limits:
            raise ValueError(f"unknown joint '{joint_name}'")
        if not math.isfinite(value):
            raise ValueError("target must be finite")
        limit = self._limits[joint_name]
        clamped = max(limit.lower, min(limit.upper, float(value)))
        self._targets[joint_name] = clamped
        return clamped

    def group_positions(self, joint_names: Iterable[str]) -> list[float]:
        return [self.target(name) for name in joint_names]

    def jog(self, joint_name: str, delta: float) -> tuple[float, float, bool]:
        """Apply one increment and return old value, new value, and clamp flag."""
        if not self.initialized:
            raise RuntimeError("cannot jog before the initial feedback pose arrives")
        if joint_name not in self._limits:
            raise ValueError(f"unknown joint '{joint_name}'")
        if not math.isfinite(delta):
            raise ValueError("jog delta must be finite")

        old_value = self.target(joint_name)
        requested = old_value + float(delta)
        limit = self._limits[joint_name]
        new_value = max(limit.lower, min(limit.upper, requested))
        self._targets[joint_name] = new_value
        return old_value, new_value, new_value != requested


class JogMotionProfile:
    """Jerk-limited one-dimensional profile used by the interactive jogger.

    The profile is deliberately independent of ROS.  A key press changes only
    ``target``; callers advance the profile from a timer and publish the
    resulting position and velocity.  This keeps repeated key presses bounded
    and gives trajectory replacement a continuous start state.
    """

    _POSITION_EPSILON = 1e-9
    _VELOCITY_EPSILON = 1e-7
    _ACCELERATION_EPSILON = 1e-7

    def __init__(
        self,
        limit: JointLimit,
        max_velocity: float,
        max_acceleration: float,
        max_jerk: float,
        max_target_lead: float,
    ) -> None:
        max_velocity = float(max_velocity)
        max_acceleration = float(max_acceleration)
        max_jerk = float(max_jerk)
        max_target_lead = float(max_target_lead)
        values = {
            "max_velocity": max_velocity,
            "max_acceleration": max_acceleration,
            "max_jerk": max_jerk,
            "max_target_lead": max_target_lead,
        }
        if any(
            not math.isfinite(float(value)) or float(value) <= 0.0
            for value in values.values()
        ):
            raise ValueError("motion limits must be finite and positive")
        if max_target_lead > limit.upper - limit.lower:
            raise ValueError("max_target_lead must fit inside the joint limits")

        self.limit = limit
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration
        self.max_jerk = max_jerk
        self.max_target_lead = max_target_lead
        braking_distance = self._stopping_distance(self.max_velocity, 0.0)
        if max_target_lead < braking_distance:
            raise ValueError(
                "max_target_lead must allow the profile to brake from max_velocity"
            )
        self._position = 0.0
        self._velocity = 0.0
        self._acceleration = 0.0
        self._target = 0.0

    @property
    def position(self) -> float:
        return self._position

    @property
    def velocity(self) -> float:
        return self._velocity

    @property
    def acceleration(self) -> float:
        return self._acceleration

    @property
    def target(self) -> float:
        return self._target

    @property
    def active(self) -> bool:
        return (
            abs(self._target - self._position) > self._POSITION_EPSILON
            or abs(self._velocity) > self._VELOCITY_EPSILON
            or abs(self._acceleration) > self._ACCELERATION_EPSILON
        )

    def initialize(
        self, position: float, velocity: float = 0.0, acceleration: float = 0.0
    ) -> None:
        """Initialize a profile from a feedback sample."""
        for name, value in (
            ("position", position),
            ("velocity", velocity),
            ("acceleration", acceleration),
        ):
            if not math.isfinite(float(value)):
                raise ValueError(f"{name} must be finite")
        self._position = max(self.limit.lower, min(self.limit.upper, float(position)))
        self._velocity = max(
            -self.max_velocity, min(self.max_velocity, float(velocity))
        )
        self._acceleration = max(
            -self.max_acceleration,
            min(self.max_acceleration, float(acceleration)),
        )
        self._target = self._position

    def set_target(self, target: float) -> tuple[float, bool]:
        """Set a bounded target and return ``(applied_target, was_clamped)``.

        The target lead bound limits how much motion can remain queued after a
        key is released.  As the profile advances, subsequent key presses can
        extend the bound without allowing one burst of keyboard repeat to
        create a long, unexpected move.
        """
        if not math.isfinite(float(target)):
            raise ValueError("target must be finite")
        requested = float(target)
        lower = max(self.limit.lower, self._position - self.max_target_lead)
        upper = min(self.limit.upper, self._position + self.max_target_lead)
        applied = max(lower, min(upper, requested))
        self._target = applied
        return applied, applied != requested

    def request_stop(self) -> tuple[float, bool]:
        """Request a stop at the reachable braking point for current velocity."""
        if abs(self._velocity) <= self._VELOCITY_EPSILON:
            return self.set_target(self._position)
        direction = math.copysign(1.0, self._velocity)
        stopping_distance = self._stopping_distance(
            abs(self._velocity), self._acceleration * direction
        )
        return self.set_target(self._position + direction * stopping_distance)

    def copy(self) -> "JogMotionProfile":
        """Return a profile copy for previewing a future state."""
        result = JogMotionProfile(
            self.limit,
            self.max_velocity,
            self.max_acceleration,
            self.max_jerk,
            self.max_target_lead,
        )
        result._position = self._position
        result._velocity = self._velocity
        result._acceleration = self._acceleration
        result._target = self._target
        return result

    def preview(self, duration: float) -> tuple[float, float]:
        """Return the position and velocity after ``duration`` seconds."""
        if not math.isfinite(float(duration)) or duration < 0.0:
            raise ValueError("preview duration must be finite and non-negative")
        result = self.copy()
        result.advance(float(duration))
        return result.position, result.velocity

    def advance(self, duration: float) -> tuple[float, float]:
        """Advance the profile and return its new position and velocity."""
        if not math.isfinite(float(duration)) or duration < 0.0:
            raise ValueError("advance duration must be finite and non-negative")
        remaining_time = float(duration)
        # Splitting long timer gaps keeps the jerk bound meaningful after a
        # scheduler stall or a suspended terminal.
        step = min(remaining_time, 0.02)
        while remaining_time > 0.0:
            current_step = min(step, remaining_time)
            self._advance_step(current_step)
            remaining_time -= current_step
        return self._position, self._velocity

    def _advance_step(self, duration: float) -> None:
        if duration <= 0.0:
            return
        remaining = self._target - self._position
        if (
            abs(remaining) <= self._POSITION_EPSILON
            and abs(self._velocity) <= self._VELOCITY_EPSILON
            and abs(self._acceleration) <= self._ACCELERATION_EPSILON
        ):
            self._position = self._target
            self._velocity = 0.0
            self._acceleration = 0.0
            return

        if abs(remaining) > self._POSITION_EPSILON:
            direction = math.copysign(1.0, remaining)
        elif abs(self._velocity) > self._VELOCITY_EPSILON:
            direction = math.copysign(1.0, self._velocity)
        else:
            direction = math.copysign(1.0, self._acceleration)

        speed_toward_target = self._velocity * direction
        acceleration_toward_target = self._acceleration * direction
        braking_distance = self._stopping_distance(
            speed_toward_target, acceleration_toward_target
        )
        should_brake = (
            speed_toward_target >= -self._VELOCITY_EPSILON
            and braking_distance > self._POSITION_EPSILON
            and abs(remaining) <= braking_distance
        )

        if (
            abs(remaining) <= self._POSITION_EPSILON
            and abs(self._velocity) <= self._VELOCITY_EPSILON
            and abs(self._acceleration) > self._ACCELERATION_EPSILON
        ):
            requested_acceleration = -direction * self.max_acceleration
        elif speed_toward_target > self.max_velocity or (
            should_brake
        ):
            requested_acceleration = -direction * self.max_acceleration
        elif speed_toward_target < self.max_velocity:
            requested_acceleration = direction * self.max_acceleration
        else:
            requested_acceleration = 0.0

        acceleration_delta = requested_acceleration - self._acceleration
        maximum_delta = self.max_jerk * duration
        acceleration_delta = max(
            -maximum_delta, min(maximum_delta, acceleration_delta)
        )
        new_acceleration = self._acceleration + acceleration_delta
        jerk = acceleration_delta / duration
        new_velocity = (
            self._velocity
            + self._acceleration * duration
            + 0.5 * jerk * duration * duration
        )
        new_position = (
            self._position
            + self._velocity * duration
            + 0.5 * self._acceleration * duration * duration
            + jerk * duration * duration * duration / 6.0
        )

        # Do not let numerical integration cross the requested target or the
        # configured hard-stop margin.  A final sample at the target has zero
        # velocity and acceleration, ready for a stable hold command.
        crossed_target = (self._target - self._position) * (
            self._target - new_position
        ) < 0.0
        if crossed_target or abs(self._target - new_position) <= self._POSITION_EPSILON:
            self._position = self._target
            self._velocity = 0.0
            self._acceleration = 0.0
            return

        self._position = max(
            self.limit.lower, min(self.limit.upper, new_position)
        )
        self._velocity = max(-self.max_velocity, min(self.max_velocity, new_velocity))
        self._acceleration = max(
            -self.max_acceleration,
            min(self.max_acceleration, new_acceleration),
        )

    def _stopping_distance(
        self, speed_toward_target: float, acceleration_toward_target: float
    ) -> float:
        """Estimate distance needed to stop with the configured jerk limit.

        The simple ``v²/(2a)`` estimate starts braking too late when the
        acceleration itself must change gradually.  Integrating the same
        jerk-limited deceleration law used by :meth:`_advance_step` keeps a
        short target reachable without an abrupt velocity reset at the end.
        """
        velocity = max(0.0, float(speed_toward_target))
        acceleration = max(
            -self.max_acceleration,
            min(self.max_acceleration, float(acceleration_toward_target)),
        )
        if velocity <= self._VELOCITY_EPSILON and acceleration <= 0.0:
            return 0.0

        distance = 0.0
        integration_step = 0.002
        # The configured limits are small, but keep a finite guard in case a
        # user supplies unusually low jerk or very high velocity parameters.
        for _ in range(10000):
            requested_acceleration = -self.max_acceleration
            acceleration_delta = requested_acceleration - acceleration
            maximum_delta = self.max_jerk * integration_step
            acceleration_delta = max(
                -maximum_delta, min(maximum_delta, acceleration_delta)
            )
            jerk = acceleration_delta / integration_step
            next_velocity = (
                velocity
                + acceleration * integration_step
                + 0.5 * jerk * integration_step * integration_step
            )
            displacement = (
                velocity * integration_step
                + 0.5 * acceleration * integration_step * integration_step
                + jerk * integration_step * integration_step * integration_step / 6.0
            )
            distance += max(0.0, displacement)
            if next_velocity <= self._VELOCITY_EPSILON:
                break
            velocity = next_velocity
            acceleration += acceleration_delta
        return distance

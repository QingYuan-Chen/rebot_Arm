from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Mapping


def _finite_tuple(values, *, length: int, label: str) -> tuple[float, ...]:
    if isinstance(values, (str, bytes)):
        raise TypeError(f"{label} must be a numeric sequence")
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError) as exc:
        raise TypeError(f"{label} must be a numeric sequence") from exc
    if len(result) != length:
        raise ValueError(f"{label} must contain exactly {length} values")
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{label} values must be finite")
    return result


def _finite_scalar(value: Any, label: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{label} must be finite")
    return result


@dataclass(frozen=True)
class TrajectorySample:
    schema_version: int
    episode_id: str
    step_index: int
    simulation_time: float
    joint_positions: tuple[float, ...]
    joint_velocities: tuple[float, ...]
    joint_targets: tuple[float, ...]
    actuator_torques: tuple[float, ...]
    gripper_width: float
    gripper_target_width: float
    end_effector_position: tuple[float, ...]
    end_effector_orientation_xyzw: tuple[float, ...]
    action: tuple[float, ...]
    max_contact_force: float
    contact_count: int
    source: str
    max_contact_penetration: float = 0.0

    def __post_init__(self) -> None:
        version = int(self.schema_version)
        if version <= 0:
            raise ValueError("schema_version must be positive")
        if not isinstance(self.episode_id, str) or not self.episode_id:
            raise ValueError("episode_id must be a non-empty string")
        step_index = int(self.step_index)
        if step_index < 0:
            raise ValueError("step_index must be non-negative")
        source = str(self.source)
        if source not in {"sim", "real"}:
            raise ValueError("source must be 'sim' or 'real'")
        object.__setattr__(self, "schema_version", version)
        object.__setattr__(self, "step_index", step_index)
        object.__setattr__(self, "simulation_time", _finite_scalar(self.simulation_time, "simulation_time"))
        for field in ("joint_positions", "joint_velocities", "joint_targets", "actuator_torques"):
            object.__setattr__(
                self,
                field,
                _finite_tuple(getattr(self, field), length=6, label=field),
            )
        object.__setattr__(self, "gripper_width", _finite_scalar(self.gripper_width, "gripper_width"))
        object.__setattr__(self, "gripper_target_width", _finite_scalar(self.gripper_target_width, "gripper_target_width"))
        if self.gripper_width < 0.0 or self.gripper_target_width < 0.0:
            raise ValueError("gripper widths must be non-negative")
        object.__setattr__(
            self,
            "end_effector_position",
            _finite_tuple(self.end_effector_position, length=3, label="end_effector_position"),
        )
        object.__setattr__(
            self,
            "end_effector_orientation_xyzw",
            _finite_tuple(
                self.end_effector_orientation_xyzw,
                length=4,
                label="end_effector_orientation_xyzw",
            ),
        )
        action = tuple(float(value) for value in self.action)
        if len(action) not in (6, 7):
            raise ValueError("action must contain exactly 6 or 7 values")
        if not all(math.isfinite(value) for value in action):
            raise ValueError("action values must be finite")
        object.__setattr__(self, "action", action)
        object.__setattr__(
            self,
            "max_contact_force",
            _finite_scalar(self.max_contact_force, "max_contact_force"),
        )
        if self.max_contact_force < 0.0:
            raise ValueError("max_contact_force must be non-negative")
        contact_count = int(self.contact_count)
        if contact_count < 0:
            raise ValueError("contact_count must be non-negative")
        object.__setattr__(self, "contact_count", contact_count)
        object.__setattr__(self, "source", source)
        penetration = _finite_scalar(
            self.max_contact_penetration, "max_contact_penetration"
        )
        if penetration < 0.0:
            raise ValueError("max_contact_penetration must be non-negative")
        object.__setattr__(self, "max_contact_penetration", penetration)

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "episode_id": self.episode_id,
            "step_index": self.step_index,
            "simulation_time": self.simulation_time,
            "joint_positions": list(self.joint_positions),
            "joint_velocities": list(self.joint_velocities),
            "joint_targets": list(self.joint_targets),
            "actuator_torques": list(self.actuator_torques),
            "gripper_width": self.gripper_width,
            "gripper_target_width": self.gripper_target_width,
            "end_effector_position": list(self.end_effector_position),
            "end_effector_orientation_xyzw": list(self.end_effector_orientation_xyzw),
            "action": list(self.action),
            "max_contact_force": self.max_contact_force,
            "contact_count": self.contact_count,
            "source": self.source,
            "max_contact_penetration": self.max_contact_penetration,
        }

    @classmethod
    def from_dict(cls, payload: Mapping[str, Any]) -> "TrajectorySample":
        return cls(**dict(payload))


@dataclass(frozen=True)
class TrajectoryMetrics:
    joint_position_rmse: float
    joint_position_max: float
    joint_velocity_rmse: float
    joint_velocity_max: float
    end_effector_position_rmse: float
    end_effector_position_max: float
    gripper_width_rmse: float
    gripper_width_max: float
    actuator_torque_rmse: float
    actuator_torque_max: float
    contact_force_rmse: float
    contact_force_max: float

    def __post_init__(self) -> None:
        for field_name in self.__dataclass_fields__:
            value = _finite_scalar(getattr(self, field_name), field_name)
            if value < 0.0:
                raise ValueError(f"{field_name} must be non-negative")
            object.__setattr__(self, field_name, value)

    def to_dict(self) -> dict[str, float]:
        return {
            field_name: float(getattr(self, field_name))
            for field_name in self.__dataclass_fields__
        }


@dataclass(frozen=True)
class ComparisonReport:
    ok: bool
    metrics: TrajectoryMetrics | None
    validation_errors: tuple[str, ...]
    sample_count: int

    def __post_init__(self) -> None:
        errors = tuple(str(error) for error in self.validation_errors)
        count = int(self.sample_count)
        if count < 0:
            raise ValueError("sample_count must be non-negative")
        if self.metrics is None and (self.ok or count != 0):
            raise ValueError("missing metrics require a failed comparison with no samples")
        object.__setattr__(self, "validation_errors", errors)
        object.__setattr__(self, "sample_count", count)

    def to_dict(self) -> dict[str, Any]:
        return {
            "ok": bool(self.ok),
            "metrics": None if self.metrics is None else self.metrics.to_dict(),
            "validation_errors": list(self.validation_errors),
            "sample_count": self.sample_count,
        }

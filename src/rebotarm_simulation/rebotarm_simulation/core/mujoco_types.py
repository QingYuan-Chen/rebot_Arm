from __future__ import annotations

from dataclasses import dataclass
import math
from types import MappingProxyType
from typing import Mapping


Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]
Pose = tuple[float, float, float, float, float, float, float]


def _float_tuple(values, *, length: int | None = None, label: str) -> tuple[float, ...]:
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must contain numeric values") from exc
    if length is not None and len(result) != length:
        raise ValueError(f"{label} must contain exactly {length} values")
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{label} values must be finite")
    return result


@dataclass(frozen=True)
class SimulationState:
    joint_names: tuple[str, ...]
    joint_positions: tuple[float, ...]
    joint_velocities: tuple[float, ...]
    actuator_forces: tuple[float, ...]
    end_effector_position: Vector3
    end_effector_orientation: Quaternion
    gripper_width: float
    object_poses: Mapping[str, Pose]
    simulation_time: float

    def __post_init__(self) -> None:
        names = tuple(str(name) for name in self.joint_names)
        if not names or any(not name for name in names):
            raise ValueError("joint names must be non-empty")
        object.__setattr__(self, "joint_names", names)
        for field_name in (
            "joint_positions",
            "joint_velocities",
            "actuator_forces",
        ):
            object.__setattr__(
                self, field_name, _float_tuple(getattr(self, field_name), label=field_name)
            )
        object.__setattr__(self, "end_effector_position", _float_tuple(
            self.end_effector_position, length=3, label="end_effector_position"
        ))
        object.__setattr__(self, "end_effector_orientation", _float_tuple(
            self.end_effector_orientation, length=4, label="end_effector_orientation"
        ))
        width = float(self.gripper_width)
        time = float(self.simulation_time)
        if not math.isfinite(width) or not math.isfinite(time):
            raise ValueError("gripper_width and simulation_time must be finite")
        object.__setattr__(self, "gripper_width", width)
        object.__setattr__(self, "simulation_time", time)
        immutable_poses = {
            str(name): _float_tuple(pose, length=7, label=f"object pose {name!r}")
            for name, pose in self.object_poses.items()
        }
        object.__setattr__(self, "object_poses", MappingProxyType(immutable_poses))


@dataclass(frozen=True)
class ControlStatus:
    """Read-only snapshot of the simulation control layer."""

    mode: str
    joint_targets: tuple[float, ...]
    joint_positions: tuple[float, ...]
    joint_velocities: tuple[float, ...]
    requested_torques: tuple[float, ...]
    applied_torques: tuple[float, ...]
    saturated: tuple[bool, ...]
    watchdog_remaining_s: float | None
    gripper_target_width_m: float
    gripper_width_m: float
    gripper_control_force_n: tuple[float, float]

    def __post_init__(self) -> None:
        if self.mode not in ("position", "hold", "gravity_comp", "raw_torque"):
            raise ValueError(f"unsupported control mode: {self.mode}")
        for field_name in (
            "joint_targets",
            "joint_positions",
            "joint_velocities",
            "requested_torques",
            "applied_torques",
        ):
            object.__setattr__(
                self,
                field_name,
                _float_tuple(getattr(self, field_name), length=6, label=field_name),
            )
        saturated = tuple(bool(value) for value in self.saturated)
        if len(saturated) != 6:
            raise ValueError("saturated must contain exactly 6 values")
        object.__setattr__(self, "saturated", saturated)
        remaining = self.watchdog_remaining_s
        if remaining is not None:
            remaining = float(remaining)
            if not math.isfinite(remaining) or remaining < 0.0:
                raise ValueError("watchdog_remaining_s must be finite and non-negative")
            object.__setattr__(self, "watchdog_remaining_s", remaining)
        for field_name in ("gripper_target_width_m", "gripper_width_m"):
            value = float(getattr(self, field_name))
            if not math.isfinite(value):
                raise ValueError(f"{field_name} must be finite")
            object.__setattr__(self, field_name, value)
        object.__setattr__(
            self,
            "gripper_control_force_n",
            _float_tuple(self.gripper_control_force_n, length=2, label="gripper_control_force_n"),
        )


@dataclass(frozen=True)
class ContactInfo:
    body1: str
    body2: str
    geom1: str
    geom2: str
    position: Vector3
    force: float
    penetration_depth: float = 0.0
    normal: Vector3 = (0.0, 0.0, 0.0)

    def __post_init__(self) -> None:
        names = (self.body1, self.body2, self.geom1, self.geom2)
        if any(not isinstance(name, str) or not name for name in names):
            raise ValueError("contact names must be non-empty strings")
        object.__setattr__(self, "position", _float_tuple(
            self.position, length=3, label="contact position"
        ))
        force = float(self.force)
        if not math.isfinite(force):
            raise ValueError("contact force must be finite")
        if force < 0.0:
            raise ValueError("contact force must be non-negative")
        object.__setattr__(self, "force", force)
        penetration_depth = float(self.penetration_depth)
        if not math.isfinite(penetration_depth) or penetration_depth < 0.0:
            raise ValueError("contact penetration_depth must be finite and non-negative")
        object.__setattr__(self, "penetration_depth", penetration_depth)
        object.__setattr__(
            self, "normal", _float_tuple(self.normal, length=3, label="contact normal")
        )


@dataclass(frozen=True)
class RandomizedScene:
    cube_pose: Pose
    reach_target_position: Vector3
    seed: int | None = None

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "cube_pose",
            _float_tuple(self.cube_pose, length=7, label="cube_pose"),
        )
        object.__setattr__(
            self,
            "reach_target_position",
            _float_tuple(
                self.reach_target_position,
                length=3,
                label="reach_target_position",
            ),
        )
        if self.seed is not None:
            object.__setattr__(self, "seed", int(self.seed))


@dataclass(frozen=True)
class SavedSimulationState:
    model_identity: int
    model_fingerprint: str
    model_dimensions: tuple[int, ...]
    state_spec: int
    state: tuple[float, ...]
    control_targets: tuple[float, ...] = ()
    position_integral: tuple[float, ...] = ()
    velocity_integral: tuple[float, ...] = ()
    applied_torque: tuple[float, ...] = ()
    control_phase: int = 0
    control_mode: str = "hold"
    raw_torque_command: tuple[float, ...] = ()
    raw_torque_requested: tuple[float, ...] = ()
    raw_torque_deadline: float | None = None
    gripper_max_force_n: float | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "model_dimensions", tuple(int(value) for value in self.model_dimensions))
        object.__setattr__(self, "state", _float_tuple(self.state, label="saved state"))
        object.__setattr__(self, "control_targets", _float_tuple(self.control_targets, label="control_targets"))
        object.__setattr__(self, "position_integral", _float_tuple(self.position_integral, label="position_integral"))
        object.__setattr__(self, "velocity_integral", _float_tuple(self.velocity_integral, label="velocity_integral"))
        object.__setattr__(self, "applied_torque", _float_tuple(self.applied_torque, label="applied_torque"))
        object.__setattr__(self, "control_phase", int(self.control_phase))
        object.__setattr__(self, "control_mode", str(self.control_mode))
        object.__setattr__(self, "raw_torque_command", _float_tuple(
            self.raw_torque_command, label="raw_torque_command"
        ))
        object.__setattr__(self, "raw_torque_requested", _float_tuple(
            self.raw_torque_requested, label="raw_torque_requested"
        ))
        for field_name in ("raw_torque_deadline", "gripper_max_force_n"):
            value = getattr(self, field_name)
            if value is not None:
                value = float(value)
                if not math.isfinite(value):
                    raise ValueError(f"{field_name} must be finite")
                object.__setattr__(self, field_name, value)

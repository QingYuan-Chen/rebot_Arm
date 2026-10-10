from __future__ import annotations

from dataclasses import dataclass
import math
import os
from pathlib import Path
import sys
from typing import Mapping, Sequence

import numpy as np
import yaml

from rebotarm_simulation.core.mujoco_sim import ARM_JOINT_NAMES


REAL2SIM_MODES = ("mirror", "physics")


def default_real2sim_mapping_path() -> Path:
    relative = Path("config/real2sim_mapping.yaml")
    package_root = Path(__file__).resolve().parents[1]
    candidates = [
        package_root / relative,
        Path(sys.prefix) / "share/rebotarm_simulation" / relative,
    ]
    for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        if prefix:
            candidates.append(Path(prefix) / "share/rebotarm_simulation" / relative)
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    raise FileNotFoundError("could not locate real2sim_mapping.yaml")


def _finite_tuple(values, *, length: int, label: str) -> tuple[float, ...]:
    if isinstance(values, (str, bytes)):
        raise TypeError(f"{label} must be a numeric sequence")
    result = tuple(float(value) for value in values)
    if len(result) != length:
        raise ValueError(f"{label} must contain exactly {length} values")
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{label} values must be finite")
    return result


@dataclass(frozen=True)
class RobotStateSample:
    timestamp: float
    joint_names: tuple[str, ...]
    positions: tuple[float, ...]
    velocities: tuple[float, ...] = ()
    gripper_width: float | None = None

    def __post_init__(self) -> None:
        timestamp = float(self.timestamp)
        names = tuple(str(name) for name in self.joint_names)
        positions = _finite_tuple(self.positions, length=len(names), label="positions")
        if not math.isfinite(timestamp) or timestamp < 0.0:
            raise ValueError("timestamp must be finite and non-negative")
        if not names or any(not name for name in names) or len(set(names)) != len(names):
            raise ValueError("joint_names must be non-empty and unique")
        velocities = tuple(self.velocities)
        if velocities:
            velocities = _finite_tuple(velocities, length=len(names), label="velocities")
        width = self.gripper_width
        if width is not None:
            width = float(width)
            if not math.isfinite(width):
                raise ValueError("gripper_width must be finite")
        object.__setattr__(self, "timestamp", timestamp)
        object.__setattr__(self, "joint_names", names)
        object.__setattr__(self, "positions", positions)
        object.__setattr__(self, "velocities", velocities)
        object.__setattr__(self, "gripper_width", width)


@dataclass(frozen=True)
class MappedRobotState:
    timestamp: float
    positions: tuple[float, ...]
    velocities: tuple[float, ...]
    gripper_width: float | None


@dataclass(frozen=True)
class JointMappingConfig:
    source_joint_names: tuple[str, ...]
    position_scale: tuple[float, ...]
    position_offset: tuple[float, ...]
    filter_alpha: float = 0.35
    max_position_jump_rad: float = 0.5
    gripper_scale: float = 1.0
    gripper_offset_m: float = 0.0

    def __post_init__(self) -> None:
        names = tuple(str(name) for name in self.source_joint_names)
        if len(names) != 6 or len(set(names)) != 6 or any(not name for name in names):
            raise ValueError("source_joint_names must contain six unique names")
        scale = _finite_tuple(self.position_scale, length=6, label="position_scale")
        offset = _finite_tuple(self.position_offset, length=6, label="position_offset")
        if any(abs(value) < 1e-12 for value in scale):
            raise ValueError("position_scale values must be non-zero")
        alpha = float(self.filter_alpha)
        jump = float(self.max_position_jump_rad)
        gripper_scale = float(self.gripper_scale)
        gripper_offset = float(self.gripper_offset_m)
        if not 0.0 < alpha <= 1.0:
            raise ValueError("filter_alpha must be in (0, 1]")
        if not math.isfinite(jump) or jump <= 0.0:
            raise ValueError("max_position_jump_rad must be positive")
        if not math.isfinite(gripper_scale) or abs(gripper_scale) < 1e-12:
            raise ValueError("gripper_scale must be finite and non-zero")
        if not math.isfinite(gripper_offset):
            raise ValueError("gripper_offset_m must be finite")
        object.__setattr__(self, "source_joint_names", names)
        object.__setattr__(self, "position_scale", scale)
        object.__setattr__(self, "position_offset", offset)
        object.__setattr__(self, "filter_alpha", alpha)
        object.__setattr__(self, "max_position_jump_rad", jump)
        object.__setattr__(self, "gripper_scale", gripper_scale)
        object.__setattr__(self, "gripper_offset_m", gripper_offset)

    @classmethod
    def from_yaml(cls, path: str | Path, *, profile: str = "rebotarm") -> "JointMappingConfig":
        payload = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        if not isinstance(payload, Mapping) or profile not in payload:
            raise ValueError(f"real2sim mapping profile {profile!r} is missing")
        values = payload[profile]
        if not isinstance(values, Mapping):
            raise ValueError(f"real2sim mapping profile {profile!r} must be a mapping")
        required = {
            "source_joint_names",
            "position_scale",
            "position_offset",
            "filter_alpha",
            "max_position_jump_rad",
            "gripper_scale",
            "gripper_offset_m",
        }
        unknown = set(values) - required
        missing = required - set(values)
        if unknown:
            raise ValueError(f"unknown real2sim mapping keys: {sorted(unknown)}")
        if missing:
            raise ValueError(f"missing real2sim mapping keys: {sorted(missing)}")
        return cls(**dict(values))


class Real2SimMapper:
    def __init__(self, config: JointMappingConfig) -> None:
        self.config = config
        self._last_raw_positions: np.ndarray | None = None
        self._last_filtered_positions: np.ndarray | None = None
        self._last_filtered_width: float | None = None
        self._last_timestamp: float | None = None

    def reset(self) -> None:
        self._last_raw_positions = None
        self._last_filtered_positions = None
        self._last_filtered_width = None
        self._last_timestamp = None

    def map(self, sample: RobotStateSample) -> MappedRobotState:
        indices = _indices_for_names(sample.joint_names, self.config.source_joint_names)
        raw = np.asarray([sample.positions[index] for index in indices], dtype=float)
        mapped = raw * np.asarray(self.config.position_scale) + np.asarray(
            self.config.position_offset
        )
        if self._last_timestamp is not None and sample.timestamp <= self._last_timestamp:
            raise ValueError("source timestamps must be strictly increasing")
        if self._last_raw_positions is not None:
            jump = float(np.max(np.abs(mapped - self._last_raw_positions)))
            if jump > self.config.max_position_jump_rad:
                raise ValueError(
                    f"source joint position jump {jump:.6f} rad exceeds limit"
                )
        alpha = self.config.filter_alpha
        filtered = (
            mapped.copy()
            if self._last_filtered_positions is None
            else alpha * mapped + (1.0 - alpha) * self._last_filtered_positions
        )
        if sample.velocities:
            raw_velocity = np.asarray([sample.velocities[index] for index in indices], dtype=float)
            velocities = raw_velocity * np.asarray(self.config.position_scale)
        elif self._last_filtered_positions is None:
            velocities = np.zeros(6, dtype=float)
        else:
            dt = sample.timestamp - float(self._last_timestamp)
            velocities = (filtered - self._last_filtered_positions) / dt
        width = None
        if sample.gripper_width is not None:
            mapped_width = (
                sample.gripper_width * self.config.gripper_scale
                + self.config.gripper_offset_m
            )
            width = (
                mapped_width
                if self._last_filtered_width is None
                else alpha * mapped_width + (1.0 - alpha) * self._last_filtered_width
            )
            width = min(0.09, max(0.0, float(width)))
        self._last_raw_positions = mapped.copy()
        self._last_filtered_positions = filtered.copy()
        self._last_filtered_width = width
        self._last_timestamp = sample.timestamp
        return MappedRobotState(
            timestamp=sample.timestamp,
            positions=tuple(float(value) for value in filtered),
            velocities=tuple(float(value) for value in velocities),
            gripper_width=width,
        )


@dataclass(frozen=True)
class Real2SimResult:
    mode: str
    source_timestamp: float
    target_positions: tuple[float, ...]
    simulated_positions: tuple[float, ...]
    max_tracking_error_rad: float
    gripper_width: float
    simulation_time: float


class Real2SimSynchronizer:
    def __init__(
        self,
        simulation,
        mapper: Real2SimMapper,
        *,
        mode: str = "mirror",
        physics_steps_per_update: int = 5,
    ) -> None:
        if mode not in REAL2SIM_MODES:
            raise ValueError(f"mode must be one of {REAL2SIM_MODES}")
        if isinstance(physics_steps_per_update, bool) or int(physics_steps_per_update) <= 0:
            raise ValueError("physics_steps_per_update must be positive")
        self.simulation = simulation
        self.mapper = mapper
        self.mode = mode
        self.physics_steps_per_update = int(physics_steps_per_update)

    def apply(self, sample: RobotStateSample) -> Real2SimResult:
        mapped = self.mapper.map(sample)
        if self.mode == "mirror":
            self.simulation.mirror_joint_state(
                mapped.positions,
                mapped.velocities,
                gripper_width=mapped.gripper_width,
            )
            self.simulation.step(self.physics_steps_per_update)
            state = self.simulation.mirror_joint_state(
                mapped.positions,
                mapped.velocities,
                gripper_width=mapped.gripper_width,
            )
        else:
            self.simulation.set_joint_position_targets(mapped.positions)
            if mapped.gripper_width is not None:
                self.simulation.set_gripper_width(mapped.gripper_width)
            state = self.simulation.step(self.physics_steps_per_update)
        simulated = tuple(float(value) for value in state.joint_positions[:6])
        error = max(
            abs(target - actual) for target, actual in zip(mapped.positions, simulated)
        )
        return Real2SimResult(
            mode=self.mode,
            source_timestamp=mapped.timestamp,
            target_positions=mapped.positions,
            simulated_positions=simulated,
            max_tracking_error_rad=error,
            gripper_width=float(state.gripper_width),
            simulation_time=float(state.simulation_time),
        )


def _indices_for_names(
    available_names: Sequence[str], required_names: Sequence[str]
) -> tuple[int, ...]:
    lookup = {name: index for index, name in enumerate(available_names)}
    missing = [name for name in required_names if name not in lookup]
    if missing:
        raise ValueError(f"source joint state is missing joints: {missing}")
    return tuple(lookup[name] for name in required_names)

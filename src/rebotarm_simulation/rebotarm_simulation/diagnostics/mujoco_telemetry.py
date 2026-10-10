"""Bounded, viewer-independent telemetry history for MuJoCo front ends."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from threading import Lock
from typing import Iterable

from rebotarm_simulation.core.mujoco_types import ContactInfo, ControlStatus


ARM_DOF = 6
VECTOR_FIELDS = frozenset(
    {
        "joint_positions",
        "joint_targets",
        "joint_errors",
        "joint_velocities",
        "requested_torques",
        "applied_torques",
    }
)
SCALAR_FIELDS = frozenset(
    {"gripper_width_m", "max_contact_force_n", "total_contact_force_n"}
)


def _finite_tuple(values: Iterable[float], *, field: str) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if len(result) != ARM_DOF:
        raise ValueError(f"{field} must contain exactly {ARM_DOF} values")
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{field} values must be finite")
    return result


@dataclass(frozen=True)
class TelemetrySample:
    simulation_time: float
    joint_positions: tuple[float, ...]
    joint_targets: tuple[float, ...]
    joint_errors: tuple[float, ...]
    joint_velocities: tuple[float, ...]
    requested_torques: tuple[float, ...]
    applied_torques: tuple[float, ...]
    gripper_width_m: float
    max_contact_force_n: float
    total_contact_force_n: float

    def __post_init__(self) -> None:
        simulation_time = float(self.simulation_time)
        if not math.isfinite(simulation_time) or simulation_time < 0.0:
            raise ValueError("simulation_time must be finite and non-negative")
        object.__setattr__(self, "simulation_time", simulation_time)
        for field in VECTOR_FIELDS:
            object.__setattr__(
                self,
                field,
                _finite_tuple(getattr(self, field), field=field),
            )
        for field in SCALAR_FIELDS:
            value = float(getattr(self, field))
            if not math.isfinite(value):
                raise ValueError(f"{field} must be finite")
            if field.endswith("contact_force_n") and value < 0.0:
                raise ValueError(f"{field} must be non-negative")
            object.__setattr__(self, field, value)


@dataclass(frozen=True)
class TelemetrySnapshot:
    samples: tuple[TelemetrySample, ...]
    reset_count: int

    @property
    def times(self) -> tuple[float, ...]:
        return tuple(sample.simulation_time for sample in self.samples)


@dataclass(frozen=True)
class PlotSeries:
    """Finite sequences and a stable Y range suitable for ``MjvFigure`` lines."""

    times: tuple[float, ...]
    values: tuple[float, ...]
    y_min: float
    y_max: float


class MujocoTelemetryHistory:
    """Thread-safe fixed-capacity telemetry buffer.

    A backwards simulation clock means MuJoCo was reset.  Existing samples are
    discarded automatically so plots never connect two simulation episodes.
    """

    def __init__(self, capacity: int = 600) -> None:
        if isinstance(capacity, bool) or not isinstance(capacity, int) or capacity <= 0:
            raise ValueError("capacity must be a positive integer")
        self._capacity = capacity
        self._samples: deque[TelemetrySample] = deque(maxlen=capacity)
        self._reset_count = 0
        self._lock = Lock()

    @property
    def capacity(self) -> int:
        return self._capacity

    def __len__(self) -> int:
        with self._lock:
            return len(self._samples)

    def reset(self) -> None:
        with self._lock:
            self._samples.clear()
            self._reset_count += 1

    def append(
        self,
        simulation_time: float,
        status: ControlStatus,
        contacts: Iterable[ContactInfo] = (),
    ) -> TelemetrySample:
        forces = tuple(float(contact.force) for contact in contacts)
        if not all(math.isfinite(force) and force >= 0.0 for force in forces):
            raise ValueError("contact forces must be finite and non-negative")
        targets = _finite_tuple(status.joint_targets, field="joint_targets")
        positions = _finite_tuple(status.joint_positions, field="joint_positions")
        sample = TelemetrySample(
            simulation_time=simulation_time,
            joint_positions=positions,
            joint_targets=targets,
            joint_errors=tuple(target - actual for target, actual in zip(targets, positions)),
            joint_velocities=status.joint_velocities,
            requested_torques=status.requested_torques,
            applied_torques=status.applied_torques,
            gripper_width_m=status.gripper_width_m,
            max_contact_force_n=max(forces, default=0.0),
            total_contact_force_n=sum(forces),
        )
        with self._lock:
            if self._samples and sample.simulation_time < self._samples[-1].simulation_time:
                self._samples.clear()
                self._reset_count += 1
            self._samples.append(sample)
        return sample

    def snapshot(self, limit: int | None = None) -> TelemetrySnapshot:
        if limit is not None and (
            isinstance(limit, bool) or not isinstance(limit, int) or limit <= 0
        ):
            raise ValueError("limit must be a positive integer or None")
        with self._lock:
            samples = tuple(self._samples)
            reset_count = self._reset_count
        if limit is not None:
            samples = samples[-limit:]
        return TelemetrySnapshot(samples=samples, reset_count=reset_count)

    def plot_series(
        self,
        field: str,
        *,
        joint_index: int | None = None,
        limit: int | None = None,
        padding_ratio: float = 0.05,
    ) -> PlotSeries:
        if field in VECTOR_FIELDS:
            if isinstance(joint_index, bool) or not isinstance(joint_index, int):
                raise ValueError(f"joint_index is required for {field}")
            if not 0 <= joint_index < ARM_DOF:
                raise ValueError(f"joint_index must be from 0 to {ARM_DOF - 1}")
        elif field in SCALAR_FIELDS:
            if joint_index is not None:
                raise ValueError(f"joint_index is not valid for {field}")
        else:
            raise ValueError(f"unsupported telemetry field: {field}")
        padding_ratio = float(padding_ratio)
        if not math.isfinite(padding_ratio) or padding_ratio < 0.0:
            raise ValueError("padding_ratio must be finite and non-negative")

        snapshot = self.snapshot(limit)
        times = snapshot.times
        if joint_index is None:
            values = tuple(float(getattr(sample, field)) for sample in snapshot.samples)
        else:
            values = tuple(float(getattr(sample, field)[joint_index]) for sample in snapshot.samples)
        if not values:
            return PlotSeries(times=(), values=(), y_min=-1.0, y_max=1.0)
        low, high = min(values), max(values)
        span = high - low
        padding = span * padding_ratio if span > 0.0 else max(abs(low) * padding_ratio, 1e-6)
        return PlotSeries(times=times, values=values, y_min=low - padding, y_max=high + padding)

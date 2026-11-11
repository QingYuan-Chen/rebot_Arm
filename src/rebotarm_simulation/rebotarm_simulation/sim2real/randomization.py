from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Mapping

import numpy as np
import yaml
from rebotarm_simulation.core.resource_paths import package_resource


def default_randomization_config_path() -> Path:
    return package_resource("rebotarm_simulation", "config/sim2real_randomization.yaml")


@dataclass(frozen=True)
class RandomizationRange:
    minimum: float
    maximum: float
    strictly_positive: bool = False
    integer: bool = False

    def __post_init__(self) -> None:
        minimum = float(self.minimum)
        maximum = float(self.maximum)
        if not math.isfinite(minimum) or not math.isfinite(maximum):
            raise ValueError("randomization range values must be finite")
        if minimum > maximum:
            raise ValueError("randomization range must be ordered")
        if self.strictly_positive and minimum <= 0.0:
            raise ValueError("randomization range must be positive")
        if self.integer and (minimum != int(minimum) or maximum != int(maximum)):
            raise ValueError("integer randomization range must use integer bounds")
        object.__setattr__(self, "minimum", int(minimum) if self.integer else minimum)
        object.__setattr__(self, "maximum", int(maximum) if self.integer else maximum)

    def sample(self, rng: np.random.Generator) -> float | int:
        if self.integer:
            return int(rng.integers(int(self.minimum), int(self.maximum) + 1))
        if self.minimum == self.maximum:
            return float(self.minimum)
        return float(rng.uniform(self.minimum, self.maximum))


@dataclass(frozen=True)
class RandomizationSample:
    seed: int | None
    mass_scale: float
    damping_scale: float
    friction_scale: float
    torque_scale: float
    control_latency_steps: int
    action_noise_std: float
    position_noise_std: float
    velocity_noise_std: float

    def __post_init__(self) -> None:
        if self.seed is not None:
            object.__setattr__(self, "seed", int(self.seed))
        for field_name in (
            "mass_scale",
            "damping_scale",
            "friction_scale",
            "torque_scale",
            "action_noise_std",
            "position_noise_std",
            "velocity_noise_std",
        ):
            value = float(getattr(self, field_name))
            if not math.isfinite(value):
                raise ValueError(f"{field_name} must be finite")
            if field_name.endswith("scale") and value <= 0.0:
                raise ValueError(f"{field_name} must be positive")
            if not field_name.endswith("scale") and value < 0.0:
                raise ValueError(f"{field_name} must be non-negative")
            object.__setattr__(self, field_name, value)
        latency = int(self.control_latency_steps)
        if latency < 0:
            raise ValueError("control_latency_steps must be non-negative")
        object.__setattr__(self, "control_latency_steps", latency)


@dataclass(frozen=True)
class RandomizationConfig:
    mass_scale: RandomizationRange
    damping_scale: RandomizationRange
    friction_scale: RandomizationRange
    torque_scale: RandomizationRange
    control_latency_steps: RandomizationRange
    action_noise_std: RandomizationRange
    position_noise_std: RandomizationRange
    velocity_noise_std: RandomizationRange

    @classmethod
    def from_yaml(cls, path: str | Path, *, profile: str = "default") -> "RandomizationConfig":
        payload = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        if not isinstance(payload, Mapping) or profile not in payload:
            raise ValueError(f"randomization profile {profile!r} is missing")
        values = payload[profile]
        if not isinstance(values, Mapping):
            raise ValueError(f"randomization profile {profile!r} must be a mapping")
        required = {
            "mass_scale": (True, False),
            "damping_scale": (True, False),
            "friction_scale": (True, False),
            "torque_scale": (True, False),
            "control_latency_steps": (False, True),
            "action_noise_std": (False, False),
            "position_noise_std": (False, False),
            "velocity_noise_std": (False, False),
        }
        unknown = set(values) - set(required)
        if unknown:
            raise ValueError(f"unknown randomization keys: {sorted(unknown)}")
        missing = set(required) - set(values)
        if missing:
            raise ValueError(f"missing randomization keys: {sorted(missing)}")
        ranges = {
            key: _parse_range(values[key], strictly_positive=positive, integer=integer)
            for key, (positive, integer) in required.items()
        }
        return cls(**ranges)

    def sample(self, seed: int | None = None) -> RandomizationSample:
        rng = np.random.default_rng(seed)
        return RandomizationSample(
            seed=seed,
            mass_scale=self.mass_scale.sample(rng),
            damping_scale=self.damping_scale.sample(rng),
            friction_scale=self.friction_scale.sample(rng),
            torque_scale=self.torque_scale.sample(rng),
            control_latency_steps=self.control_latency_steps.sample(rng),
            action_noise_std=self.action_noise_std.sample(rng),
            position_noise_std=self.position_noise_std.sample(rng),
            velocity_noise_std=self.velocity_noise_std.sample(rng),
        )


class RandomizationSession:
    """Apply one sample to a simulator and restore its baseline on exit."""

    def __init__(self, simulation, sample: RandomizationSample) -> None:
        if not isinstance(sample, RandomizationSample):
            raise TypeError("sample must be a RandomizationSample")
        self._simulation = simulation
        self.sample = sample
        self._entered = False

    def __enter__(self) -> "RandomizationSession":
        if self._entered:
            raise RuntimeError("randomization session cannot be entered twice")
        self._simulation.apply_randomization(self.sample)
        self._entered = True
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        if self._entered:
            self._simulation.restore_randomization()
            self._entered = False


def _parse_range(value: Any, *, strictly_positive: bool, integer: bool) -> RandomizationRange:
    if isinstance(value, (int, float)):
        values = (value, value)
    elif isinstance(value, (list, tuple)) and len(value) == 2:
        values = tuple(value)
    else:
        raise ValueError("randomization range must be a scalar or two-value list")
    return RandomizationRange(
        values[0],
        values[1],
        strictly_positive=strictly_positive,
        integer=integer,
    )

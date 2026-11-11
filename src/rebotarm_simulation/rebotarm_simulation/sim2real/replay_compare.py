from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Callable, Sequence

import numpy as np

from .schemas import ComparisonReport, TrajectoryMetrics, TrajectorySample
from .trajectory_log import TrajectoryRecorder


@dataclass(frozen=True)
class ComparisonThresholds:
    joint_position_max: float = math.inf
    joint_velocity_max: float = math.inf
    end_effector_position_max: float = math.inf
    gripper_width_max: float = math.inf
    actuator_torque_max: float = math.inf
    contact_force_max: float = math.inf

    def __post_init__(self) -> None:
        for field_name in self.__dataclass_fields__:
            value = float(getattr(self, field_name))
            if math.isnan(value) or value < 0.0:
                raise ValueError(f"{field_name} must be non-negative")
            object.__setattr__(self, field_name, value)


def _metric(values: np.ndarray) -> tuple[float, float]:
    absolute = np.abs(values)
    return float(np.sqrt(np.mean(np.square(values)))), float(np.max(absolute))


def compare_trajectories(
    reference: Sequence[TrajectorySample],
    candidate: Sequence[TrajectorySample],
    *,
    thresholds: ComparisonThresholds | None = None,
) -> ComparisonReport:
    thresholds = thresholds or ComparisonThresholds()
    reference = tuple(reference)
    candidate = tuple(candidate)
    errors: list[str] = []
    if len(reference) != len(candidate):
        errors.append("sample_count")
    count = min(len(reference), len(candidate))
    if count == 0:
        errors.append("no samples")
        return ComparisonReport(False, None, tuple(errors), 0)
    for index, (left, right) in enumerate(zip(reference[:count], candidate[:count])):
        if left.step_index != right.step_index:
            errors.append(f"step_index[{index}]")
        if not math.isclose(left.simulation_time, right.simulation_time, rel_tol=0.0, abs_tol=1e-9):
            errors.append(f"simulation_time[{index}]")
        if len(left.action) != len(right.action):
            errors.append(f"action_shape[{index}]")
    if errors:
        return ComparisonReport(False, None, tuple(errors), 0)

    position_rmse, position_max = _metric(
        _array(reference, "joint_positions") - _array(candidate, "joint_positions")
    )
    velocity_rmse, velocity_max = _metric(
        _array(reference, "joint_velocities") - _array(candidate, "joint_velocities")
    )
    ee_rmse, ee_max = _metric(
        _array(reference, "end_effector_position") - _array(candidate, "end_effector_position")
    )
    gripper_rmse, gripper_max = _metric(
        _array(reference, "gripper_width") - _array(candidate, "gripper_width")
    )
    torque_rmse, torque_max = _metric(
        _array(reference, "actuator_torques") - _array(candidate, "actuator_torques")
    )
    contact_rmse, contact_max = _metric(
        _array(reference, "max_contact_force") - _array(candidate, "max_contact_force")
    )
    metrics = TrajectoryMetrics(
        joint_position_rmse=position_rmse,
        joint_position_max=position_max,
        joint_velocity_rmse=velocity_rmse,
        joint_velocity_max=velocity_max,
        end_effector_position_rmse=ee_rmse,
        end_effector_position_max=ee_max,
        gripper_width_rmse=gripper_rmse,
        gripper_width_max=gripper_max,
        actuator_torque_rmse=torque_rmse,
        actuator_torque_max=torque_max,
        contact_force_rmse=contact_rmse,
        contact_force_max=contact_max,
    )
    for field_name, actual, limit in (
        ("joint_position_max", position_max, thresholds.joint_position_max),
        ("joint_velocity_max", velocity_max, thresholds.joint_velocity_max),
        ("end_effector_position_max", ee_max, thresholds.end_effector_position_max),
        ("gripper_width_max", gripper_max, thresholds.gripper_width_max),
        ("actuator_torque_max", torque_max, thresholds.actuator_torque_max),
        ("contact_force_max", contact_max, thresholds.contact_force_max),
    ):
        if actual > limit:
            errors.append(field_name)
    return ComparisonReport(not errors, metrics, tuple(errors), count)


def _array(samples: Sequence[TrajectorySample], field_name: str) -> np.ndarray:
    return np.asarray([getattr(sample, field_name) for sample in samples], dtype=float)


def replay_actions(
    env_factory: Callable,
    actions: Sequence[Sequence[float]],
    *,
    seed: int | None,
    episode_id: str,
    randomization=None,
) -> TrajectoryRecorder:
    env = env_factory()
    recorder = TrajectoryRecorder(episode_id=episode_id, source="sim")
    try:
        if randomization is None:
            env.reset(seed=seed)
        else:
            env.reset(seed=seed, randomization=randomization)
        for step_index, action in enumerate(actions):
            _obs, _reward, terminated, truncated, _info = env.step(action)
            recorder.append(
                env.sample_from_last_step(
                    action,
                    episode_id=episode_id,
                    step_index=step_index,
                )
            )
            if terminated or truncated:
                break
    finally:
        env.close()
    return recorder

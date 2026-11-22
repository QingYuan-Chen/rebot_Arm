"""Run validated joint paths against an isolated MuJoCo instance."""

from __future__ import annotations

from collections.abc import Callable, Sequence

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from rebotarm_simulation.core.mujoco_sim import RebotArmMujoco

from rebotarm_simulation.core.model_contract import ARM_JOINT_NAMES
from rebotarm_simulation.execution.trajectory_sampler import NamedTrajectoryPoint, TrajectorySampler


def _point_time(point) -> float:
    if isinstance(point, tuple) and len(point) == 2:
        return float(point[0])
    if not hasattr(point, "time_from_start"):
        raise TypeError("trajectory point must provide time_from_start or be a (time, positions) pair")
    value = point.time_from_start
    if isinstance(value, (int, float)):
        return float(value)
    return float(value.sec) + float(value.nanosec) * 1e-9


def _point_positions(point) -> tuple[float, ...]:
    if isinstance(point, tuple) and len(point) == 2:
        return tuple(float(value) for value in point[1])
    if not hasattr(point, "positions"):
        raise TypeError("trajectory point must provide positions or be a (time, positions) pair")
    return tuple(float(value) for value in point.positions)


def normalized_path(points: Sequence, joint_names: Sequence[str]) -> list[tuple[float, tuple[float, ...]]]:
    names = tuple(joint_names)
    if len(names) != 6 or set(names) != set(ARM_JOINT_NAMES):
        raise ValueError("path must contain each arm joint exactly once")
    sampler = TrajectorySampler(names, [NamedTrajectoryPoint(_point_time(p), _point_positions(p)) for p in points])
    if sampler.duration <= 0:
        raise ValueError("path must have positive duration")
    return [(p.time_from_start, p.positions) for p in sampler.points]


def play_path(
    sim: RebotArmMujoco,
    points: Sequence,
    joint_names: Sequence[str],
    *,
    observe: Callable[[object, tuple], None] | None = None,
    on_step: Callable[[RebotArmMujoco], None] | None = None,
) -> dict[str, float | int]:
    """Follow a path in simulated time and report tracking/contact maxima."""
    path = normalized_path(points, joint_names)
    start = tuple(sim.get_state().joint_positions[:6])
    elapsed = 0.0
    sampler = TrajectorySampler(ARM_JOINT_NAMES, [NamedTrajectoryPoint(t, q) for t, q in path], initial_positions=start)
    max_error = max_force = max_penetration = 0.0
    contact_steps = steps = 0
    while elapsed < path[-1][0] - 1e-12:
        elapsed = min(elapsed + sim.timestep, path[-1][0])
        target = sampler.sample(elapsed)
        sim.command_joint_positions(target)
        state = sim.step()
        contacts = sim.get_contacts()
        max_error = max(max_error, max(abs(a - b) for a, b in zip(target, state.joint_positions[:6])))
        max_force = max(max_force, max((contact.force for contact in contacts), default=0.0))
        max_penetration = max(max_penetration, max((contact.penetration_depth for contact in contacts), default=0.0))
        contact_steps += bool(contacts)
        steps += 1
        if observe is not None:
            observe(state, contacts)
        if on_step is not None:
            on_step(sim)
    return {
        "duration_sec": elapsed,
        "steps": steps,
        "max_tracking_error_rad": max_error,
        "max_contact_force_n": max_force,
        "max_contact_penetration_m": max_penetration,
        "contact_steps": contact_steps,
    }

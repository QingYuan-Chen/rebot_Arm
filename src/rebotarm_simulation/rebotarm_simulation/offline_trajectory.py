"""Run validated joint paths against an isolated MuJoCo instance."""

from __future__ import annotations

import math
from collections.abc import Callable, Sequence

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .mujoco_sim import RebotArmMujoco

ARM_JOINT_NAMES = tuple(f"joint{index}" for index in range(1, 7))


def _point_time(point) -> float:
    value = getattr(point, "time_from_start", None)
    if value is None:
        return float(point[0])
    if isinstance(value, (int, float)):
        return float(value)
    return float(value.sec) + float(value.nanosec) * 1e-9


def _point_positions(point) -> tuple[float, ...]:
    return tuple(float(value) for value in getattr(point, "positions", point[1] if isinstance(point, tuple) else ()))


def normalized_path(points: Sequence, joint_names: Sequence[str]) -> list[tuple[float, tuple[float, ...]]]:
    names = tuple(joint_names)
    if len(names) != 6 or set(names) != set(ARM_JOINT_NAMES):
        raise ValueError("path must contain each arm joint exactly once")
    if not points:
        raise ValueError("path has no points")
    order = tuple(names.index(name) for name in ARM_JOINT_NAMES)
    result = []
    previous = -1.0
    for point in points:
        when = _point_time(point)
        raw = _point_positions(point)
        if len(raw) != 6 or not math.isfinite(when) or when < 0 or when <= previous:
            raise ValueError("path times and positions must be finite, complete and increasing")
        positions = tuple(raw[index] for index in order)
        if any(not math.isfinite(value) for value in positions):
            raise ValueError("path positions must be finite")
        result.append((when, positions))
        previous = when
    if result[-1][0] <= 0:
        raise ValueError("path must have positive duration")
    return result


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
    cursor = 0
    max_error = max_force = max_penetration = 0.0
    contact_steps = steps = 0
    while elapsed < path[-1][0] - 1e-12:
        elapsed = min(elapsed + sim.timestep, path[-1][0])
        while cursor < len(path) - 1 and elapsed > path[cursor][0]:
            cursor += 1
        lower_time, lower_q = (0.0, start) if cursor == 0 else path[cursor - 1]
        upper_time, upper_q = path[cursor]
        ratio = min(1.0, max(0.0, (elapsed - lower_time) / max(upper_time - lower_time, 1e-12)))
        target = tuple(a + (b - a) * ratio for a, b in zip(lower_q, upper_q))
        sim.set_joint_position_targets(target)
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

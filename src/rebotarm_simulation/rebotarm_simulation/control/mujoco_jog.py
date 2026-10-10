"""Shared manual-jog speed configuration for MuJoCo front ends."""

from __future__ import annotations


JOG_SPEED_LEVELS = (
    ("PRECISION", 0.25),
    ("NORMAL", 1.0),
    ("FAST", 2.5),
    ("TURBO", 5.0),
    ("MAX", 7.5),
)


def jog_speed(index: int, base_rate: float) -> tuple[str, float]:
    """Return the display name and effective speed for a bounded gear index."""

    name, scale = JOG_SPEED_LEVELS[index]
    return name, float(base_rate) * scale

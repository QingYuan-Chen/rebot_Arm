from __future__ import annotations

import math

import pytest

from rebotarm_simulation.diagnostics.mujoco_telemetry import MujocoTelemetryHistory
from rebotarm_simulation.core.mujoco_types import ContactInfo, ControlStatus


def _status(offset: float = 0.0) -> ControlStatus:
    return ControlStatus(
        mode="position",
        joint_targets=tuple(offset + index + 0.5 for index in range(6)),
        joint_positions=tuple(offset + index for index in range(6)),
        joint_velocities=tuple(0.1 * index for index in range(6)),
        requested_torques=tuple(1.0 + index for index in range(6)),
        applied_torques=tuple(0.5 + index for index in range(6)),
        saturated=(False,) * 6,
        watchdog_remaining_s=None,
        gripper_target_width_m=0.04,
        gripper_width_m=0.039,
        gripper_control_force_n=(1.0, -1.0),
    )


def _contact(force: float) -> ContactInfo:
    return ContactInfo("finger", "cube", "finger_geom", "cube_geom", (0, 0, 0), force)


def test_history_has_fixed_capacity_and_preserves_chronological_samples() -> None:
    history = MujocoTelemetryHistory(capacity=3)
    for index in range(5):
        history.append(index * 0.01, _status(float(index)))

    snapshot = history.snapshot()
    assert history.capacity == 3
    assert len(history) == 3
    assert snapshot.times == pytest.approx((0.02, 0.03, 0.04))
    assert snapshot.samples[-1].joint_positions[0] == pytest.approx(4.0)


def test_sample_contains_six_axis_control_and_contact_fields() -> None:
    history = MujocoTelemetryHistory()
    sample = history.append(1.25, _status(), (_contact(2.0), _contact(3.5)))

    assert sample.joint_errors == pytest.approx((0.5,) * 6)
    for field in (
        "joint_positions", "joint_targets", "joint_errors", "joint_velocities",
        "requested_torques", "applied_torques",
    ):
        assert len(getattr(sample, field)) == 6
    assert sample.gripper_width_m == pytest.approx(0.039)
    assert sample.max_contact_force_n == pytest.approx(3.5)
    assert sample.total_contact_force_n == pytest.approx(5.5)


def test_time_rollback_starts_a_new_plot_episode() -> None:
    history = MujocoTelemetryHistory()
    history.append(1.0, _status())
    history.append(1.1, _status())
    history.append(0.0, _status())

    snapshot = history.snapshot()
    assert snapshot.times == (0.0,)
    assert snapshot.reset_count == 1

    history.reset()
    assert history.snapshot().samples == ()
    assert history.snapshot().reset_count == 2


def test_plot_series_is_limited_finite_and_has_nonzero_scale() -> None:
    history = MujocoTelemetryHistory()
    for index in range(5):
        history.append(float(index), _status(float(index)))

    series = history.plot_series("joint_errors", joint_index=2, limit=2)
    assert series.times == pytest.approx((3.0, 4.0))
    assert series.values == pytest.approx((0.5, 0.5))
    assert series.y_min < 0.5 < series.y_max
    assert all(math.isfinite(value) for value in (*series.times, *series.values, series.y_min, series.y_max))

    empty = MujocoTelemetryHistory().plot_series("max_contact_force_n")
    assert empty.times == empty.values == ()
    assert (empty.y_min, empty.y_max) == (-1.0, 1.0)


@pytest.mark.parametrize("capacity", [0, -1, True, 1.5])
def test_capacity_must_be_a_positive_integer(capacity) -> None:
    with pytest.raises(ValueError, match="capacity"):
        MujocoTelemetryHistory(capacity)


def test_plot_series_validates_field_joint_and_limit() -> None:
    history = MujocoTelemetryHistory()
    history.append(0.0, _status())

    with pytest.raises(ValueError, match="joint_index"):
        history.plot_series("joint_positions")
    with pytest.raises(ValueError, match="joint_index"):
        history.plot_series("gripper_width_m", joint_index=0)
    with pytest.raises(ValueError, match="unsupported"):
        history.plot_series("unknown")
    with pytest.raises(ValueError, match="limit"):
        history.snapshot(0)

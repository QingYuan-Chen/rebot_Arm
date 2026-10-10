from __future__ import annotations

import math

import pytest


ARM_JOINTS = tuple(f"joint{index}" for index in range(1, 7))
START = (0.0, 0.1, 0.2, 0.3, 0.4, 0.5)


def point(time_from_start: float, *positions: float):
    from rebotarm_simulation.execution.trajectory_sampler import NamedTrajectoryPoint

    return NamedTrajectoryPoint(time_from_start, positions)


def sampler(names=ARM_JOINTS, points=None, initial_positions=START):
    from rebotarm_simulation.execution.trajectory_sampler import TrajectorySampler

    return TrajectorySampler(
        names,
        points or [point(1.0, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5)],
        initial_positions=initial_positions,
    )


@pytest.mark.parametrize(
    ("names", "points", "message"),
    [
        ((), [point(1.0)], "joint names"),
        (ARM_JOINTS, [], "point"),
        (("joint1", "joint1"), [point(1.0, 0.0, 0.0)], "duplicate"),
        (("joint1", "finger"), [point(1.0, 0.0, 0.0)], "unknown"),
    ],
)
def test_rejects_invalid_joint_name_sets(names, points, message: str) -> None:
    from rebotarm_simulation.execution.trajectory_sampler import TrajectorySampler

    with pytest.raises(ValueError, match=message):
        TrajectorySampler(names, points, initial_positions=START)


def test_rejects_missing_canonical_joints_without_initial_positions() -> None:
    from rebotarm_simulation.execution.trajectory_sampler import TrajectorySampler

    with pytest.raises(ValueError, match="initial positions"):
        TrajectorySampler(("joint2",), [point(1.0, 0.8)])


def test_rejects_position_length_mismatch() -> None:
    with pytest.raises(ValueError, match="position"):
        sampler(points=[point(1.0, 0.0)])


@pytest.mark.parametrize(
    "points",
    [
        [point(-0.1, *START)],
        [point(math.nan, *START)],
        [point(1.0, *(START[:-1] + (math.inf,)))],
        [point(2.0, *START), point(1.0, *START)],
        [point(1.0, *START), point(1.0, *START)],
    ],
)
def test_rejects_invalid_numeric_values_or_timestamps(points) -> None:
    with pytest.raises(ValueError):
        sampler(points=points)


def test_maps_permuted_joint_names_to_canonical_order() -> None:
    result = sampler(
        names=("joint6", "joint2", "joint4", "joint1", "joint5", "joint3"),
        points=[point(1.0, 6.0, 2.0, 4.0, 1.0, 5.0, 3.0)],
    ).sample(1.0)

    assert result == pytest.approx((1.0, 2.0, 3.0, 4.0, 5.0, 6.0))


def test_partial_points_preserve_omitted_initial_joint_positions() -> None:
    result = sampler(
        names=("joint2", "joint5"),
        points=[point(1.0, 0.8, -0.4)],
    ).sample(1.0)

    assert result == pytest.approx((0.0, 0.8, 0.2, 0.3, -0.4, 0.5))


def test_sample_interpolates_from_initial_state_at_zero() -> None:
    result = sampler(points=[point(2.0, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5)]).sample(0.5)

    assert result == pytest.approx((0.25, 0.35, 0.45, 0.55, 0.65, 0.75))


def test_sample_interpolates_between_trajectory_points() -> None:
    result = sampler(
        points=[
            point(1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0),
            point(3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0),
        ]
    ).sample(2.0)

    assert result == pytest.approx((2.0,) * 6)


def test_exact_endpoints_are_returned_without_blending() -> None:
    trajectory = sampler(
        points=[
            point(1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0),
            point(2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0),
        ]
    )

    assert trajectory.sample(0.0) == START
    assert trajectory.sample(1.0) == (1.0,) * 6
    assert trajectory.sample(2.0) == (2.0,) * 6


def test_duration_comes_from_last_point() -> None:
    trajectory = sampler(points=[point(2.5, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0)])

    assert trajectory.duration == pytest.approx(2.5)


def test_completion_changes_at_duration() -> None:
    trajectory = sampler(points=[point(2.5, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0)])

    assert not trajectory.is_complete(2.499)
    assert trajectory.is_complete(2.5)


def test_sampling_after_duration_holds_final_point() -> None:
    trajectory = sampler(points=[point(2.5, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0)])

    assert trajectory.sample(99.0) == (1.0,) * 6


def test_cancel_blocks_sampling_until_cancel_is_cleared() -> None:
    trajectory = sampler()

    trajectory.cancel()
    assert trajectory.is_cancelled
    with pytest.raises(RuntimeError, match="cancelled"):
        trajectory.sample(0.5)
    trajectory.clear_cancel()

    assert not trajectory.is_cancelled
    assert trajectory.sample(0.0) == START


def test_reset_clears_cancelled_state() -> None:
    trajectory = sampler()
    trajectory.cancel()

    trajectory.reset()

    assert not trajectory.is_cancelled


def test_sampling_rejects_negative_or_nonfinite_time() -> None:
    trajectory = sampler()

    for invalid_time in (-0.1, math.nan, math.inf):
        with pytest.raises(ValueError, match="simulation time"):
            trajectory.sample(invalid_time)

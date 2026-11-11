from __future__ import annotations

import numpy as np
import pytest
from dataclasses import replace

from rebotarm_simulation.sim2real.replay_compare import (
    ComparisonThresholds,
    compare_trajectories,
    replay_actions,
)
from rebotarm_simulation.sim2real.schemas import TrajectorySample


def _sample(step_index=0, simulation_time=0.01, offset=0.0):
    return TrajectorySample(
        schema_version=1,
        episode_id="episode-1",
        step_index=step_index,
        simulation_time=simulation_time,
        joint_positions=(offset,) * 6,
        joint_velocities=(offset,) * 6,
        joint_targets=(0.2 + offset,) * 6,
        actuator_torques=(1.0 + offset,) * 6,
        gripper_width=0.05 + offset,
        gripper_target_width=0.04 + offset,
        end_effector_position=(0.2 + offset, 0.0, 0.2),
        end_effector_orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        action=(offset,) * 7,
        max_contact_force=2.0 + offset,
        contact_count=1,
        source="sim",
    )


def test_compare_trajectories_computes_metrics_and_threshold_result():
    reference = [_sample(0, 0.01), _sample(1, 0.02)]
    candidate = [_sample(0, 0.01, offset=0.1), _sample(1, 0.02, offset=0.1)]

    report = compare_trajectories(
        reference,
        candidate,
        thresholds=ComparisonThresholds(
            joint_position_max=0.2,
            joint_velocity_max=0.2,
            end_effector_position_max=0.2,
            gripper_width_max=0.2,
            actuator_torque_max=0.2,
            contact_force_max=0.2,
        ),
    )

    assert report.ok is True
    assert report.sample_count == 2
    assert report.metrics.joint_position_rmse == pytest.approx(0.1)
    assert report.metrics.joint_position_max == pytest.approx(0.1)

    failed = compare_trajectories(
        reference,
        candidate,
        thresholds=ComparisonThresholds(joint_position_max=0.05),
    )
    assert failed.ok is False
    assert "joint_position_max" in failed.validation_errors
    assert failed.to_dict()["metrics"]["joint_position_max"] == pytest.approx(0.1)


def test_compare_trajectories_reports_structural_mismatch():
    report = compare_trajectories(
        [_sample()],
        [_sample(1, 0.02)],
    )

    assert report.ok is False
    assert report.sample_count == 0
    assert report.metrics is None
    assert report.to_dict()["metrics"] is None
    assert any("step_index" in error or "simulation_time" in error for error in report.validation_errors)


@pytest.mark.parametrize("reference,candidate,error", [
    ([], [], "no samples"),
    ([_sample()], [], "sample_count"),
    ([_sample()], [replace(_sample(), action=(0.,) * 6)], "action_shape"),
])
def test_comparison_with_incompatible_samples_has_no_metrics(reference, candidate, error):
    report = compare_trajectories(reference, candidate)
    assert report.ok is False
    assert report.sample_count == 0
    assert report.to_dict()["metrics"] is None
    assert any(error in item for item in report.validation_errors)


class FakeReplayEnv:
    def __init__(self):
        self.step_index = 0
        self.value = 0.0

    def reset(self, *, seed):
        self.step_index = 0
        self.value = float(seed or 0)
        return {}, {}

    def step(self, action):
        self.value += float(np.asarray(action)[0])
        self.step_index += 1
        return {}, 0.0, False, self.step_index >= 3, {}

    def sample_from_last_step(self, action, *, episode_id, step_index):
        return _sample(
            step_index=step_index,
            simulation_time=step_index + 1.0,
            offset=self.value,
        )

    def close(self):
        pass


def test_replay_actions_is_deterministic_for_same_seed_and_actions():
    actions = [[0.1], [0.2], [0.3]]
    first = replay_actions(lambda: FakeReplayEnv(), actions, seed=7, episode_id="episode-1")
    second = replay_actions(lambda: FakeReplayEnv(), actions, seed=7, episode_id="episode-1")

    assert first.samples == second.samples
    assert len(first) == 3

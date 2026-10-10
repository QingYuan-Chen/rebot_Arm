from __future__ import annotations

import io
import json
from types import MappingProxyType, SimpleNamespace

import numpy as np
import pytest

from rebotarm_simulation.diagnostics import mujoco_batch
from rebotarm_simulation.core.mujoco_env import RebotArmReachEnv, ReachEnvConfig
from rebotarm_simulation.sim2real.randomization import RandomizationSample
from rebotarm_simulation.sim2real.trajectory_log import TrajectoryRecorder
from rebotarm_simulation.core.mujoco_types import RandomizedScene


class FakeSim:
    def __init__(self, model_path=None):
        self.model_path = model_path
        self.targets = np.zeros(8, dtype=float)
        self.positions = np.zeros(8, dtype=float)
        self.velocities = np.zeros(8, dtype=float)
        self.width = 0.06
        self.ee = np.array([0.2, 0.0, 0.2], dtype=float)
        self.cube_pose = (0.28, 0.0, 0.04, 0.0, 0.0, 0.0, 1.0)
        self.closed = False
        self.randomization_sample = None

    @property
    def control_targets(self):
        return tuple(self.targets[:6]) + (self.width / 2.0, -self.width / 2.0)

    def reset_home(self, seed=None):
        self.targets[:] = 0.0
        self.positions[:] = 0.0
        self.ee[:] = [0.2, 0.0, 0.2]
        return self.get_state()

    def reset(self, seed=None):
        return self.reset_home(seed=seed)

    def randomize_scene(self, seed=None):
        target = (0.21 + 0.01 * float(seed or 0), 0.0, 0.2)
        return RandomizedScene(
            cube_pose=self.cube_pose,
            reach_target_position=target,
            seed=seed,
        )

    def set_joint_position_targets(self, targets):
        self.targets[:6] = np.asarray(targets, dtype=float)
        return tuple(self.targets[:6])

    def set_gripper_width(self, width):
        self.width = min(0.09, max(0.0, float(width)))
        return self.width

    def step(self, n_steps=1):
        previous = self.positions.copy()
        self.positions[:6] = self.targets[:6]
        self.velocities[:] = self.positions - previous
        self.ee[0] += 0.001 * float(np.sum(self.targets[:6]))
        return self.get_state()

    def get_contacts(self):
        return (SimpleNamespace(force=0.25),)

    def get_state(self):
        self.positions[-2:] = (self.width / 2.0, -self.width / 2.0)
        return SimpleNamespace(
            joint_positions=tuple(float(v) for v in self.positions),
            joint_velocities=tuple(float(v) for v in self.velocities),
            actuator_forces=(0.0,) * 8,
            end_effector_position=tuple(float(v) for v in self.ee),
            end_effector_orientation=(0.0, 0.0, 0.0, 1.0),
            gripper_width=self.width,
            object_poses=MappingProxyType({"test_cube": self.cube_pose}),
            simulation_time=0.0,
        )

    def close(self):
        self.closed = True

    def apply_randomization(self, sample):
        self.randomization_sample = sample

    def restore_randomization(self):
        self.randomization_sample = None


def test_reach_env_reset_returns_gym_style_observation_and_info():
    with RebotArmReachEnv(config=ReachEnvConfig(max_steps=5), sim_factory=FakeSim) as env:
        obs, info = env.reset(seed=3)

        assert obs["joint_positions"].shape == (6,)
        assert obs["joint_velocities"].shape == (6,)
        assert obs["ee_position"].shape == (3,)
        assert obs["target_position"] == pytest.approx((0.24, 0.0, 0.2))
        assert obs["cube_pose"].shape == (7,)
        assert obs["max_contact_force"] == pytest.approx(0.25)
        assert info["step_count"] == 0
        assert info["distance_to_target_m"] >= 0.0


def test_reach_env_step_clips_action_and_reports_termination_shape():
    with RebotArmReachEnv(config=ReachEnvConfig(max_steps=1), sim_factory=FakeSim) as env:
        env.reset(seed=0)
        obs, reward, terminated, truncated, info = env.step([2, -2, 0, 0, 0, 0, 2])

        assert obs["joint_positions"][0] == pytest.approx(0.02)
        assert obs["joint_positions"][1] == pytest.approx(-0.02)
        assert obs["gripper_width"] == pytest.approx(0.062)
        assert isinstance(reward, float)
        assert isinstance(terminated, bool)
        assert truncated is True
        assert info["done"] is True
        assert info["terminated"] is terminated
        assert info["truncated"] is True
        assert info["step_count"] == 1


def test_reach_env_step_done_returns_legacy_gym_shape():
    with RebotArmReachEnv(config=ReachEnvConfig(max_steps=1), sim_factory=FakeSim) as env:
        env.reset(seed=0)
        obs, reward, done, info = env.step_done([0.0] * 6)

        assert obs["joint_positions"].shape == (6,)
        assert isinstance(reward, float)
        assert done is True
        assert info["done"] is True


@pytest.mark.parametrize("action", [[0.0] * 5, [0.0] * 8, [float("nan")] * 6])
def test_reach_env_rejects_invalid_actions(action):
    with RebotArmReachEnv(sim_factory=FakeSim) as env:
        env.reset(seed=0)
        with pytest.raises((TypeError, ValueError)):
            env.step(action)


def test_headless_batch_outputs_json_summary():
    output = io.StringIO()
    code = mujoco_batch.main(
        ["--episodes", "2", "--steps", "3", "--seed", "4"],
        env_factory=lambda model: RebotArmReachEnv(model, sim_factory=FakeSim),
        stdout=output,
    )

    assert code == 0
    payload = json.loads(output.getvalue())
    assert payload["ok"] is True
    assert payload["episodes"] == 2
    assert len(payload["results"]) == 2
    assert payload["requested_steps"] == 3


def test_reach_env_randomization_and_sample_recording_are_optional():
    sample = RandomizationSample(
        seed=7,
        mass_scale=1.0,
        damping_scale=1.0,
        friction_scale=1.0,
        torque_scale=1.0,
        control_latency_steps=0,
        action_noise_std=0.0,
        position_noise_std=0.0,
        velocity_noise_std=0.0,
    )
    with RebotArmReachEnv(sim_factory=FakeSim) as env:
        env.reset(seed=7, randomization=sample)
        assert env.randomization_sample == sample
        assert env.sim.randomization_sample == sample
        action = [0.0] * 7
        env.step(action)
        recorded = env.sample_from_last_step(action, episode_id="episode-1", step_index=0)
        assert recorded.episode_id == "episode-1"
        assert recorded.source == "sim"
        assert recorded.action == tuple(action)
        assert recorded.joint_targets == tuple(env.sim.control_targets[:6])

        recorder = TrajectoryRecorder(episode_id="episode-1", source="sim")
        recorder.append(recorded)
        assert recorder.summary()["sample_count"] == 1

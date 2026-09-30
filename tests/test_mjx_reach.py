"""GPU MJX Reach contract and short CPU MuJoCo parity check."""
import numpy as np
import pytest

pytest.importorskip("mujoco")
pytest.importorskip("jax")
pytest.importorskip("stable_baselines3")

from rebotarm_simulation.gym_reach import RebotArmReachEnv
from rebotarm_simulation.mjx_reach import MJXReachVecEnv


def test_mjx_fixed_reach_gpu_batch_and_cpu_short_trajectory():
    import jax
    if not any(device.platform == "gpu" for device in jax.devices()):
        pytest.skip("JAX CUDA GPU unavailable")
    with RebotArmReachEnv(curriculum_stage="fixed") as cpu:
        cpu_obs, _ = cpu.reset(seed=7)
        mjx = MJXReachVecEnv(2, curriculum_stage="fixed", seed=7)
        try:
            assert not np.any(mjx._mj_model.geom_contype)
            assert not np.any(mjx._mj_model.geom_conaffinity)
            gpu_obs = mjx.reset()
            np.testing.assert_allclose(gpu_obs[0], cpu_obs, atol=1e-4)
            for action in (np.zeros(6, dtype=np.float32), np.full(6, 0.1, dtype=np.float32)):
                cpu_obs, _, _, _, cpu_info = cpu.step(action)
                gpu_obs, rewards, dones, infos = mjx.step(np.tile(action, (2, 1)))
                assert gpu_obs.flags.writeable and rewards.flags.writeable
                assert np.all(np.isfinite(gpu_obs)) and np.all(np.isfinite(rewards))
                assert not np.any(dones)
                np.testing.assert_allclose(gpu_obs[0, :6], cpu_obs[:6], atol=2e-4)
                assert infos[0]["distance_m"] == pytest.approx(cpu_info["distance_m"], abs=2e-4)
        finally:
            mjx.close()


def test_mjx_local_targets_stay_in_cpu_reach_distance_band_and_vary():
    import jax
    if not any(device.platform == "gpu" for device in jax.devices()):
        pytest.skip("JAX CUDA GPU unavailable")
    mjx = MJXReachVecEnv(128, curriculum_stage="local", seed=17)
    try:
        observations = mjx.reset()
        distance = np.linalg.norm(observations[:, 12:15], axis=1)
        assert np.all(distance >= 0.02 - 1e-5)
        assert np.all(distance <= 0.10 + 1e-5)
        assert np.unique(np.round(observations[:, 12:18], 4), axis=0).shape[0] > 100
        next_observations = mjx.reset()
        assert not np.array_equal(observations, next_observations)
    finally:
        mjx.close()


def test_mjx_replays_cpu_easy_goals_and_pose_reward_is_finite():
    import jax
    if not any(device.platform == "gpu" for device in jax.devices()):
        pytest.skip("JAX CUDA GPU unavailable")
    with RebotArmReachEnv(curriculum_stage="local_easy") as cpu:
        goals = []
        for seed in range(7, 11):
            cpu.reset(seed=seed)
            goals.append(cpu._goal_q.copy())
        env = MJXReachVecEnv(4, curriculum_stage="local_easy", seed=7, reward_profile="pose_v2")
        try:
            observations = env.reset_to_goals(goals)
            distance = np.linalg.norm(observations[:, 12:15], axis=1)
            angle = np.linalg.norm(observations[:, 15:18], axis=1)
            assert np.all((distance >= 0.02 - 1e-5) & (distance <= 0.05 + 1e-5))
            assert np.all(angle <= np.deg2rad(8.0) + 1e-4)
            cpu_observation, _ = cpu.reset(options={"goal_q": goals[0]})
            np.testing.assert_allclose(observations[0], cpu_observation, atol=1e-4)
            _, reward, _, _ = env.step(np.zeros((4, 6), dtype=np.float32))
            assert np.all(np.isfinite(reward))
        finally:
            env.close()


def test_adaptive_position_uses_initial_distance_and_keeps_goals():
    import jax
    if not any(device.platform == "gpu" for device in jax.devices()):
        pytest.skip("JAX CUDA GPU unavailable")
    with RebotArmReachEnv(curriculum_stage="local") as cpu:
        goals = []
        for seed in (101, 102):
            cpu.reset(seed=seed)
            goals.append(cpu._goal_q.copy())
    env = MJXReachVecEnv(2, curriculum_stage="local", seed=7,
                         reward_profile="adaptive_position")
    try:
        observations = env.reset_to_goals(goals)
        distances = np.linalg.norm(observations[:, 12:15], axis=1)
        np.testing.assert_allclose(np.asarray(env._state.initial_distance), distances, atol=1e-5)
        _, rewards, _, _ = env.step(np.zeros((2, 6), dtype=np.float32))
        assert np.all(np.isfinite(rewards))
    finally:
        env.close()

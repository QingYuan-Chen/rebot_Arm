"""使用真实 MuJoCo 的 Reach 接口与回合语义回归。"""
import numpy as np
import pytest

pytest.importorskip("gymnasium")
pytest.importorskip("mujoco")
from rebotarm_simulation.gym_reach import RebotArmReachEnv


def test_seed_replays_targets_and_dynamics():
    with RebotArmReachEnv() as env:
        a, _ = env.reset(seed=7)
        first = env.step(np.ones(6, dtype=np.float32) * 0.1)
        b, _ = env.reset(seed=7)
        second = env.step(np.ones(6, dtype=np.float32) * 0.1)
        np.testing.assert_array_equal(a, b)
        np.testing.assert_array_equal(first[0], second[0])
        assert first[1:] == second[1:]
        assert env.observation_space.contains(second[0])
        assert second[4]["simulation_time"] == pytest.approx(0.02)


def test_time_limit_and_reset_required():
    with RebotArmReachEnv(max_episode_steps=1) as env:
        with pytest.raises(RuntimeError):
            env.step(np.zeros(6))
        env.reset(seed=8)
        _, _, terminated, truncated, _ = env.step(np.zeros(6))
        assert not terminated and truncated
        with pytest.raises(RuntimeError):
            env.step(np.zeros(6))


@pytest.mark.parametrize("action", [np.full(6, np.nan), np.zeros(5), np.full(6, 1.01)])
def test_invalid_action_does_not_advance_physics(action):
    with RebotArmReachEnv() as env:
        env.reset(seed=9)
        before = env.sim.get_state()
        with pytest.raises(ValueError):
            env.step(action)
        assert env.sim.get_state() == before


def test_reach_scene_is_contact_free_and_success_is_pose_based():
    import mujoco
    with RebotArmReachEnv() as env:
        model = env.sim._model
        assert "reach_scene.xml" in env.sim.model_path
        assert all("table" not in name and "bottle" not in name for name in (
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) or ""
            for i in range(model.nbody)))
        assert not np.any(model.geom_contype)
        assert not np.any(model.geom_conaffinity)
        env.reset(seed=10)
        current = env.sim.get_state()
        env._goal = np.array(current.end_effector_position)
        env._goal_orientation = np.array(current.end_effector_orientation)
        for _ in range(9):
            result = env.step(np.zeros(6))
            assert not result[2]
        result = env.step(np.zeros(6))
        assert result[2] and result[4]["is_success"]
        assert not result[4]["collision"]


def test_gymnasium_checker():
    from gymnasium.utils.env_checker import check_env
    with RebotArmReachEnv() as env:
        check_env(env, skip_render_check=True)


def test_replay_goal_and_easy_stage_limits():
    with RebotArmReachEnv(curriculum_stage="local_easy") as env:
        _, start = env.reset(seed=17)
        goal_q = env._goal_q.copy()
        assert 0.02 <= start["distance_m"] <= 0.05
        assert start["orientation_error_rad"] <= np.deg2rad(8.0)
        obs, replay = env.reset(options={"goal_q": goal_q})
        assert env.observation_space.contains(obs)
        assert replay["distance_m"] == pytest.approx(start["distance_m"])
        with pytest.raises(ValueError):
            env.reset(options={"goal_q": np.zeros(5)})

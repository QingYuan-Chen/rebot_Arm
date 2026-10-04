"""Opt-in real physics tests. No optimizer steps, real hardware or ROS."""
import os

import numpy as np
import pytest

pytestmark = pytest.mark.skipif(os.environ.get("REBOT_DRL_GPU_TESTS") != "1", reason="opt-in CUDA physics test")


@pytest.fixture(scope="module")
def env():
    from rebotarm_drl.reach_workflow import make_env
    instance = make_env(num_envs=2, auto_reset=False)
    yield instance
    instance.close()


def test_hold_interrupt_reset_and_terminal_semantics(env):
    import torch
    from rebotarm_drl.reach import state, timeout
    env.reset()
    s = state(env)
    s.target[:] = env.scene["robot"].data.site_pos_w[:, s.site]-env.scene.env_origins
    zeros = torch.zeros(2, 6, device=env.device)
    for _ in range(25):
        _, _, term, _, _ = env.step(zeros)
        assert not term.any()
    assert s.hold.tolist() == [25, 25]
    s.update()  # An extra observation/metric read cannot advance hold.
    assert s.hold.tolist() == [25, 25]
    s.target[0, 0] += .011
    env.step(zeros)
    assert s.hold.tolist() == [0, 26]
    env.reset(env_ids=torch.tensor([0], device=env.device))
    assert s.hold.tolist() == [0, 26]
    for _ in range(24):
        _, _, term, _, _ = env.step(zeros)
    assert term.tolist() == [False, True]
    assert s.hold.tolist() == [0, 50]
    env.episode_length_buf[:] = env.max_episode_length
    assert timeout(env).tolist() == [True, False]
    env.reset()


def test_physical_violation_and_effort_limits(env):
    import torch
    from rebotarm_drl.reach import state
    env.reset()
    s = state(env)
    s.command[:] += 1.
    s.apply_actions()
    env.scene.write_data_to_sim()
    effort = env.sim.data.ctrl
    assert torch.all(effort.abs() <= s.cap+1e-5)
    # Collision pose is diagnosed directly; never integrate an invalid reset.
    robot = env.scene["robot"]
    collision_q = torch.tensor([.6258543, -3.08195, -.304154, 1.469244, -.649658, 1.938426], device=env.device).repeat(2, 1)
    robot.write_joint_state_to_sim(collision_q, torch.zeros(2, 6, device=env.device))
    env.sim.forward()
    env.sim.sense()
    assert s.check_substep().bool().all()
    env.reset()
    velocity = torch.zeros(2, 6, device=env.device)
    velocity[:, 0] = 2.1
    robot.write_joint_state_to_sim(robot.data.joint_pos, velocity)
    assert s.check_substep().bool().all()
    env.reset()


def test_task_scene_export_and_recording_relocate(env, tmp_path):
    from rebotarm_drl.replay import replay
    env.reset()
    env.scene.write(tmp_path/"model")
    q = env.scene["robot"].data.joint_pos[0].cpu().numpy()
    recording = tmp_path/"record.npz"
    np.savez(recording, qpos=np.stack([q, q]), time_s=[0., .02], target_m=[.1, .1, .3])
    assert replay(tmp_path/"model/scene.xml", recording, headless=True)["frames"] == 2

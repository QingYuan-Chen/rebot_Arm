"""Cross-engine gravity checks; no ROS graph or hardware is constructed."""
from pathlib import Path

import numpy as np
import pytest

from rebotarmcontroller.gravity_control import (
    GravitySettings, bounded_mit_command, load_gravity_model,
)

URDF = Path(__file__).resolve().parents[1] / 'src/rebotarm_moveit_config/config/rebotarm.urdf'
EXTRA_BODIES = ('left_finger_link', 'right_finger_link',
                'gemini2_mount_part0', 'gemini2_camera')


@pytest.mark.parametrize('pose', [
    [0., -.5, -1., 0., .2, 0.],
    [.3, -1., -.6, .1, -.3, .2],
    [-.4, -.8, -1.3, -.2, .4, .2],
])
def test_shared_gravity_model_with_explicit_simulation_payload_holds_pose(pose):
    mujoco = pytest.importorskip('mujoco')
    pytest.importorskip('pinocchio')
    from rebotarm_simulation.core.mujoco_sim import RebotArmMujoco

    sim = RebotArmMujoco(collisionless=True)
    try:
        sim.reset_joint_positions(pose)
        mujoco.mj_forward(sim._model, sim._data)
        tool = mujoco.mj_name2id(sim._model, mujoco.mjtObj.mjOBJ_BODY, 'end_link')
        rotation = sim._data.xmat[tool].reshape(3, 3)
        mass, com = 0., np.zeros(3)
        # Explicit fixture-only payload: never copied to real-arm settings.
        # The URDF lacks these finger/camera inertials; compare equal payloads.
        for name in EXTRA_BODIES:
            body = mujoco.mj_name2id(sim._model, mujoco.mjtObj.mjOBJ_BODY, name)
            body_mass = float(sim._model.body_mass[body])
            mass += body_mass
            com += body_mass * (rotation.T @ (sim._data.xipos[body] - sim._data.xpos[tool]))
        settings = GravitySettings(payload_mass_kg=mass, payload_com_xyz_m=com / mass)
        gravity = load_gravity_model(URDF, settings)
        indices = list(sim._joint_ids[:6])
        q_indices, v_indices = sim._model.jnt_qposadr[indices], sim._model.jnt_dofadr[indices]
        target = np.array(pose)
        previous = gravity.gravity(target)
        np.testing.assert_allclose(previous, sim._data.qfrc_bias[v_indices], atol=1e-5)
        integral = np.zeros(6)
        peak = 0.
        for _ in range(200):
            q, qd = sim._data.qpos[q_indices].copy(), sim._data.qvel[v_indices].copy()
            integral = np.clip(integral + (target - q) * .01, -.5, .5)
            kp, kd, ff, total = bounded_mit_command(
                target, q, qd, np.full(6, 7.), np.full(6, .8), gravity.gravity(q) + integral,
                gravity.effort, settings.torque_rate_limits_nm_s, previous, .01)
            assert (np.abs(total - previous) <= np.array(settings.torque_rate_limits_nm_s) * .01 + 1e-9).all()
            previous = total
            sim.command_joint_torques(kp * (target - q) - kd * qd + ff, timeout_s=.03)
            sim.step(round(.01 / sim.timestep))
            peak = max(peak, float(np.max(np.abs(sim._data.qpos[q_indices] - target))))
        assert peak < .001
    finally:
        sim.close()

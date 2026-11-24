"""关节阻抗的真实 MuJoCo 物理,周期,互斥与快照回归."""
from dataclasses import replace

import numpy as np
import pytest

from rebotarm_force_control.joint_impedance import JointImpedanceCommand
from rebotarm_simulation.core.mujoco_sim import RebotArmMujoco
from rebotarm_simulation.diagnostics.impedance_gain_budget import (
    RECOMMENDED_DAMPING, RECOMMENDED_STIFFNESS,
)
from rebotarm_simulation.sim2real.randomization import RandomizationSample


# 增益取自按额定力矩推导的预算常量,详见 joint_impedance_gains.md.
STIFFNESS = RECOMMENDED_STIFFNESS
DAMPING = RECOMMENDED_DAMPING


@pytest.fixture
def sim():
    # 接触隔离只用于自由空间控制律测试;原模型和运动惯量保持.
    with RebotArmMujoco(collisionless=True) as instance:
        yield instance


def advance(sim, seconds):
    return sim.step(round(seconds / sim.timestep))


@pytest.mark.parametrize('pose', [
    (0., -.8, -1., .3, 0., 0.),
    (.2, -.6, -.8, .6, .2, -.2),
    (-.3, -1., -.5, .8, -.2, .3),
])
def test_impedance_gravity_holds_three_poses(sim, pose):
    sim.reset_joint_positions(pose)
    sim.command_joint_impedance(pose, STIFFNESS, DAMPING)
    state = advance(sim, .5)
    assert max(abs(q - reference) for q, reference in zip(state.joint_positions, pose)) < 1e-4
    assert sim.control_mode == 'joint_impedance'


def test_step_and_external_disturbance_return_to_equilibrium(sim):
    # 增益按额定力矩与协议上限取值后比旧配置软,观测窗口相应延长;
    # 判定阈值保持原值,只说明"能回稳",不因此放宽精度要求.
    target = np.asarray(sim.get_state().joint_positions[:6]) + (.05, .02, -.02, .03, 0., 0.)
    sim.command_joint_impedance(target, STIFFNESS, DAMPING)
    state = advance(sim, 6.)
    assert max(abs(np.asarray(state.joint_positions[:6]) - target)) < .003
    model, data = sim.borrow_viewer_handles()
    data.qfrc_applied[model.jnt_dofadr[0]] = 2.
    disturbed = advance(sim, .5)
    data.qfrc_applied[:] = 0.
    displaced = abs(disturbed.joint_positions[0] - target[0])
    restored = advance(sim, 4.)
    assert displaced > .005
    assert abs(restored.joint_positions[0] - target[0]) < .002
    assert max(abs(np.asarray(restored.joint_positions[:6]) - target)) < .003


def test_dual_rate_mit_inner_loop_updates_at_500hz_and_stabilizes_j6(sim):
    target = np.asarray(sim.get_state().joint_positions[:6]) + (.15, 0., 0., 0., 0., .2)
    sim.command_joint_impedance(target, STIFFNESS, DAMPING)
    # 跳过 t=0 首个静止外环拍,在 t=0.01 (phase=0) 产生非零运动与非零 Kp/Kd 锁存.
    sim.step(6)
    model, data = sim.borrow_viewer_handles()
    assert sim.save_state().control_phase == 1
    outer_time = sim.save_state().last_control_time
    latched = sim.save_state().impedance_mit_output
    assert latched is not None and latched.kp[0] > 0. and latched.kp[5] > 0.
    substep_torques = [np.asarray(sim.get_control_status().applied_torques)]
    for _ in range(4):
        sim.step()
        snap = sim.save_state()
        assert snap.last_control_time == outer_time
        substep_torques.append(np.asarray(sim.get_control_status().applied_torques))
    # 500 Hz 内环随实时 (q, qd) 更新 data.ctrl,不锁死在同一常值力矩.
    assert np.max(np.abs(substep_torques[-1] - substep_torques[0])) > 1e-4
    settled = advance(sim, 1.5)
    assert abs(settled.joint_positions[5] - target[5]) < .002
    assert abs(settled.joint_velocities[5]) < .02


def test_pure_gravity_uses_zero_velocity_copy(sim):
    model, data = sim.borrow_viewer_handles()
    data.qvel[:6] = (.3, -.2, .1, -.4, .2, .3)
    before = data.qvel.copy()
    actual = sim._joint_gravity_torque(list(range(6)), data)
    reference = sim._mj.MjData(model)
    reference.qpos[:] = data.qpos
    sim._mj.mj_forward(model, reference)
    assert actual == pytest.approx(reference.qfrc_bias[:6], abs=1e-12)
    assert np.array_equal(data.qvel, before)


def test_hot_update_keeps_cadence_and_rate_budget(sim):
    target = np.asarray(sim.get_state().joint_positions[:6]) + (.2, 0., 0., 0., 0., 0.)
    sim.command_joint_impedance(target, STIFFNESS, DAMPING)
    initial = sim.get_control_status().applied_torques
    sim.step()
    assert sim.get_control_status().applied_torques == initial
    checkpoint = sim.save_state()
    for _ in range(29):
        regular = sim.step()
    regular_control = sim.get_control_status()
    sim.restore_state(checkpoint)
    for _ in range(29):
        previous = sim.get_control_status().applied_torques
        phase = sim.save_state().control_phase
        sim.command_joint_impedance(target, STIFFNESS, DAMPING)
        assert sim.get_control_status().applied_torques == previous
        assert sim.save_state().control_phase == phase
        renewed = sim.step()
    assert renewed == regular
    assert sim.get_control_status().applied_torques == regular_control.applied_torques


def test_final_torque_and_rate_limits_survive_randomization_and_handoff(sim):
    sim.apply_randomization(RandomizationSample(1, 1., 1., 1., 2., 0, 0., 0., 0.))
    target = np.asarray(sim.get_state().joint_positions[:6]) + (.8, 0., 0., 0., 0., 0.)
    sim.command_joint_impedance(target, (500.,) * 6, DAMPING)
    effort = np.asarray(sim._control._motor_parameters.arm.effort_limit)
    rate = np.asarray(sim._control._motor_parameters.arm.torque_rate_limit_nm_s)
    previous = np.asarray(sim.get_control_status().applied_torques)
    previous_time = sim.save_state().last_control_time
    saturated = False
    for _ in range(60):
        sim.step()
        status, snapshot = sim.get_control_status(), sim.save_state()
        torque = np.asarray(status.applied_torques)
        assert np.all(abs(torque) <= effort + 1e-12)
        if snapshot.last_control_time > previous_time:
            dt = snapshot.last_control_time - previous_time
            assert np.all(abs(torque - previous) <= rate * dt + 1e-10)
            previous_time = snapshot.last_control_time
        previous = torque
        saturated |= any(status.saturated)
    sim.set_mode('hold')
    for _ in range(10):
        sim.step()
        status, snapshot = sim.get_control_status(), sim.save_state()
        torque = np.asarray(status.applied_torques)
        assert np.all(abs(torque) <= effort + 1e-12)
        if snapshot.last_control_time > previous_time:
            dt = snapshot.last_control_time - previous_time
            assert np.all(abs(torque - previous) <= rate * dt + 1e-10)
            previous_time = snapshot.last_control_time
        previous = torque
    assert saturated and sim.control_mode == 'hold'


def test_continuous_impedance_stays_active_until_explicit_hold_and_allows_position(sim):
    target = np.asarray(sim.get_state().joint_positions[:6]) + (.05, 0., 0., 0., 0., 0.)
    sim.command_joint_impedance(target, STIFFNESS, DAMPING)
    advance(sim, 1.0)
    assert sim.control_mode == 'joint_impedance'
    assert sim.get_control_status().watchdog_remaining_s is None
    previous = sim.get_control_status().applied_torques
    sim.set_mode('hold')
    assert sim.get_control_status().applied_torques == previous
    assert sim.control_mode == 'hold'
    saved = sim.save_state()
    sim.set_mode('hold')
    assert sim.save_state() == saved
    assert sim.get_control_status().impedance_command is None
    sim.command_joint_positions(sim.get_state().joint_positions[:6])
    assert sim.control_mode == 'position'


@pytest.mark.parametrize('operation', [
    lambda sim: sim.command_joint_positions((0., -.8, -1., .3, 0., 0.)),
    lambda sim: sim.command_joint_torques((0.,) * 6),
    lambda sim: sim.set_mode('position'),
    lambda sim: sim.set_mode('gravity_comp'),
    lambda sim: sim.mirror_joint_state((0., -.8, -1., .3, 0., 0.)),
])
def test_conflicting_commands_preserve_impedance(sim, operation):
    sim.command_joint_impedance(sim.get_state().joint_positions[:6], STIFFNESS, DAMPING)
    before = sim.save_state()
    with pytest.raises(RuntimeError, match='impedance active'):
        operation(sim)
    assert sim.save_state() == before


@pytest.mark.parametrize('change', [
    {'targets': (99.,) * 6}, {'targets': (float('nan'),) * 6},
    {'stiffness': (-1.,) * 6}, {'damping': (float('inf'),) * 6},
    {'stiffness': (501.,) * 6}, {'damping': (5.1,) * 6},
    {'stiffness': (1e308,) * 6},
])
def test_invalid_impedance_command_rejected_without_mutation(sim, change):
    args = dict(targets=sim.get_state().joint_positions[:6], stiffness=STIFFNESS,
                damping=DAMPING)
    args.update(change)
    before = sim.save_state()
    with pytest.raises(ValueError):
        sim.command_joint_impedance(**args)
    assert sim.save_state() == before


def test_joint_impedance_requires_explicit_command_and_hold_admission(sim):
    before = sim.save_state()
    with pytest.raises(ValueError, match='command_joint_impedance'):
        sim.set_mode('joint_impedance')
    assert sim.save_state() == before
    sim.set_mode('position')
    before = sim.save_state()
    with pytest.raises(RuntimeError, match='enter hold'):
        sim.command_joint_impedance(sim.get_state().joint_positions[:6], STIFFNESS, DAMPING)
    assert sim.save_state() == before


def test_impedance_snapshot_replays_mid_substep_and_rejects_forgery(sim):
    target = np.asarray(sim.get_state().joint_positions[:6]) + (.05, 0., 0., 0., 0., 0.)
    sim.command_joint_impedance(target, STIFFNESS, DAMPING)
    sim.step(8)
    assert sim.save_state().control_phase == 3
    # 在子步中间热更新目标后立刻快照,验证 command 与 latched_mit 差异时的快照兼容性.
    hot_target = target + (.02, 0., 0., 0., 0., 0.)
    sim.command_joint_impedance(hot_target, STIFFNESS, DAMPING)
    checkpoint = sim.save_state()
    assert checkpoint.impedance_mit_output is not None
    continued = sim.step(29)
    expected = sim.save_state()
    sim.restore_state(checkpoint)
    assert sim.step(29) == continued
    assert sim.save_state() == expected
    bad = replace(checkpoint, impedance_command=JointImpedanceCommand((99.,) * 6, STIFFNESS, DAMPING))
    with pytest.raises(ValueError, match='outside limits'):
        sim.restore_state(bad)
    assert sim.save_state() == expected
    bad = replace(checkpoint, impedance_command=JointImpedanceCommand(target, (600.,) * 6, DAMPING))
    with pytest.raises(ValueError, match='stiffness'):
        sim.restore_state(bad)
    assert sim.save_state() == expected
    bad = replace(checkpoint, last_control_time=100.)
    with pytest.raises(ValueError, match='control time'):
        sim.restore_state(bad)
    assert sim.save_state() == expected


def test_reset_clears_impedance_command_and_history(sim):
    sim.command_joint_impedance(sim.get_state().joint_positions[:6], STIFFNESS, DAMPING)
    sim.step(7)
    sim.reset()
    status = sim.get_control_status()
    assert status.mode == 'hold' and status.impedance_command is None
    assert status.watchdog_remaining_s is None
    assert sim.save_state().last_control_time == 0.
    assert sim.save_state().impedance_mit_output is None


def test_position_after_impedance_hold_waits_for_original_control_tick(sim):
    target = np.asarray(sim.get_state().joint_positions[:6]) + (.4, 0., 0., 0., 0., 0.)
    sim.command_joint_impedance(target, (500.,) * 6, (1.,) * 6)
    sim.step(11)
    assert sim.get_state().simulation_time == pytest.approx(.022)
    before = sim.save_state()
    previous = np.asarray(sim.get_control_status().applied_torques)
    sim.set_mode('hold')
    sim.command_joint_positions(sim.get_state().joint_positions[:6])
    assert sim.get_control_status().applied_torques == tuple(previous)
    pending = sim.save_state()
    assert pending.control_phase == before.control_phase
    assert pending.last_control_time == before.last_control_time
    assert pending.impedance_handoff and pending.control_mode == 'position'
    sim.step(4)
    assert sim.get_control_status().applied_torques == tuple(previous)
    sim.step()
    applied = np.asarray(sim.get_control_status().applied_torques)
    elapsed = sim.save_state().last_control_time - before.last_control_time
    rate = np.asarray(sim._control._motor_parameters.arm.torque_rate_limit_nm_s)
    assert np.all(abs(applied - previous) <= rate * elapsed + 1e-12)
    assert not sim.save_state().impedance_handoff
    completed = sim.save_state()
    sim.restore_state(pending)
    sim.step(4)
    sim.step()
    assert sim.save_state() == completed


@pytest.mark.parametrize('steps', [0, 2, 7])
@pytest.mark.parametrize('field', ['qpos', 'qvel', 'time'])
def test_invalid_feedback_fails_before_physics_or_output_mutation(sim, steps, field):
    sim.command_joint_impedance(sim.get_state().joint_positions[:6], STIFFNESS, DAMPING)
    if steps:
        sim.step(steps)
    model, data = sim.borrow_viewer_handles()
    control_before = sim._control.capture()
    output_before = data.ctrl.copy()
    time_before = data.time
    if field == 'time':
        data.time = float('nan')
    else:
        getattr(data, field)[0] = float('nan')
    position_before = data.qpos.copy()
    with pytest.raises(ValueError, match='finite'):
        sim.step()
    assert sim._control.capture() == control_before
    assert np.array_equal(data.ctrl, output_before)
    assert np.array_equal(data.qpos, position_before, equal_nan=True)
    assert np.isnan(data.time) if field == 'time' else data.time == time_before


@pytest.mark.parametrize('joint_index', [0, 5])
def test_low_passive_damping_exposes_active_kd_sensitivity(sim, joint_index):
    # 验证阶段 B 分级机械阻尼([0.5*3, 0.2, 0.12, 0.06])与柔顺阻抗刚度下,
    # 大关节 J1 与腕部小关节 J6 在 Kd=0 时均呈现清晰欠阻尼超调,投入推荐 Kd 后平顺收敛.
    home = np.asarray(sim.get_state().joint_positions[:6], dtype=float)
    step_rad = 0.15
    target = home.copy()
    target[joint_index] += step_rad

    sim.reset_joint_positions(home)
    sim.command_joint_impedance(target, STIFFNESS, (0.0,) * 6)
    q_zero_kd = np.asarray([sim.step().joint_positions[joint_index] for _ in range(1000)], dtype=float)
    overshoot_zero_kd = float(np.max(q_zero_kd - target[joint_index]) / step_rad)

    sim.reset_joint_positions(home)
    sim.command_joint_impedance(target, STIFFNESS, DAMPING)
    q_rec_kd = np.asarray([sim.step().joint_positions[joint_index] for _ in range(1000)], dtype=float)
    overshoot_rec_kd = float(np.max(q_rec_kd - target[joint_index]) / step_rad)

    assert overshoot_zero_kd > 0.25
    assert overshoot_rec_kd < 0.05
    assert abs(q_rec_kd[-1] - target[joint_index]) < 0.002

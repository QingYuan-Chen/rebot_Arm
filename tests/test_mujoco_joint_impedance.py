"""关节阻抗的真实 MuJoCo 物理,周期,互斥与快照回归."""
from dataclasses import replace

import numpy as np
import pytest

from rebotarm_force_control.joint_impedance import JointImpedanceCommand
from rebotarm_simulation.core.mujoco_sim import RebotArmMujoco
from rebotarm_simulation.sim2real.randomization import RandomizationSample


STIFFNESS = (80., 80., 80., 20., 10., 5.)
DAMPING = (8., 8., 8., 1., .5, .1)


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
    sim.command_joint_impedance(pose, STIFFNESS, DAMPING, timeout_s=2.)
    state = advance(sim, .5)
    assert max(abs(q - reference) for q, reference in zip(state.joint_positions, pose)) < 1e-4
    assert sim.control_mode == 'joint_impedance'


def test_step_and_external_disturbance_return_to_equilibrium(sim):
    target = np.asarray(sim.get_state().joint_positions[:6]) + (.05, .02, -.02, .03, 0., 0.)
    sim.command_joint_impedance(target, STIFFNESS, DAMPING, timeout_s=10.)
    state = advance(sim, 2.)
    assert max(abs(np.asarray(state.joint_positions[:6]) - target)) < .003
    model, data = sim.borrow_viewer_handles()
    data.qfrc_applied[model.jnt_dofadr[0]] = 2.
    disturbed = advance(sim, .3)
    data.qfrc_applied[:] = 0.
    displaced = abs(disturbed.joint_positions[0] - target[0])
    restored = advance(sim, 2.)
    assert displaced > .005
    assert abs(restored.joint_positions[0] - target[0]) < .002
    assert max(abs(np.asarray(restored.joint_positions[:6]) - target)) < .003


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


def test_renewal_keeps_cadence_and_rate_budget(sim):
    target = np.asarray(sim.get_state().joint_positions[:6]) + (.2, 0., 0., 0., 0., 0.)
    sim.command_joint_impedance(target, STIFFNESS, DAMPING, timeout_s=1.)
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
        sim.command_joint_impedance(target, STIFFNESS, DAMPING, timeout_s=1.)
        assert sim.get_control_status().applied_torques == previous
        assert sim.save_state().control_phase == phase
        renewed = sim.step()
    assert renewed == regular
    assert sim.get_control_status().applied_torques == regular_control.applied_torques


def test_final_torque_and_rate_limits_survive_randomization_and_handoff(sim):
    sim.apply_randomization(RandomizationSample(1, 1., 1., 1., 2., 0, 0., 0., 0.))
    target = np.asarray(sim.get_state().joint_positions[:6]) + (.8, 0., 0., 0., 0., 0.)
    sim.command_joint_impedance(target, (10000.,) * 6, DAMPING, timeout_s=.2)
    effort = np.asarray(sim._control._motor_parameters.arm.effort_limit)
    rate = np.asarray(sim._control._motor_parameters.arm.torque_rate_limit_nm_s)
    previous = np.asarray(sim.get_control_status().applied_torques)
    previous_time = sim.save_state().last_control_time
    saturated = False
    for _ in range(200):
        sim.step()
        status, snapshot = sim.get_control_status(), sim.save_state()
        torque = np.asarray(status.applied_torques)
        assert np.all(abs(torque) <= effort + 1e-12)
        assert np.all(abs(torque - previous) <= rate * (snapshot.last_control_time - previous_time) + 1e-10)
        previous, previous_time = torque, snapshot.last_control_time
        saturated |= any(status.saturated)
    assert saturated and sim.control_mode == 'hold'


@pytest.mark.parametrize('explicit', [False, True])
def test_expiry_or_explicit_hold_preserves_output_and_allows_position(sim, explicit):
    target = np.asarray(sim.get_state().joint_positions[:6]) + (.05, 0., 0., 0., 0., 0.)
    sim.command_joint_impedance(target, STIFFNESS, DAMPING, timeout_s=.025)
    sim.step(7)
    if explicit:
        previous = sim.get_control_status().applied_torques
        sim.set_mode('hold')
        assert sim.get_control_status().applied_torques == previous
    else:
        advance(sim, .014)
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
    {'timeout_s': 0.}, {'timeout_s': float('nan')},
])
def test_invalid_impedance_command_rejected_without_mutation(sim, change):
    args = dict(targets=sim.get_state().joint_positions[:6], stiffness=STIFFNESS,
                damping=DAMPING, timeout_s=.1)
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


def test_impedance_snapshot_replays_across_deadline_and_rejects_forgery(sim):
    target = np.asarray(sim.get_state().joint_positions[:6]) + (.05, 0., 0., 0., 0., 0.)
    sim.command_joint_impedance(target, STIFFNESS, DAMPING, timeout_s=.045)
    sim.step(7)
    checkpoint = sim.save_state()
    continued = sim.step(29)
    expected = sim.save_state()
    sim.restore_state(checkpoint)
    assert sim.step(29) == continued
    assert sim.save_state() == expected
    bad = replace(checkpoint, impedance_command=JointImpedanceCommand((99.,) * 6, STIFFNESS, DAMPING))
    with pytest.raises(ValueError, match='outside limits'):
        sim.restore_state(bad)
    assert sim.save_state() == expected
    bad = replace(checkpoint)
    object.__setattr__(bad, 'impedance_deadline', float('nan'))
    with pytest.raises(ValueError, match='finite'):
        sim.restore_state(bad)
    assert sim.save_state() == expected
    bad = replace(checkpoint, last_control_time=100.)
    with pytest.raises(ValueError, match='control time'):
        sim.restore_state(bad)
    assert sim.save_state() == expected


def test_reset_clears_impedance_command_and_history(sim):
    sim.command_joint_impedance(sim.get_state().joint_positions[:6], STIFFNESS, DAMPING)
    sim.step(7)
    sim.reset_home()
    status = sim.get_control_status()
    assert status.mode == 'hold' and status.impedance_command is None
    assert status.watchdog_remaining_s is None
    assert sim.save_state().last_control_time == 0.


def test_position_after_impedance_hold_waits_for_original_control_tick(sim):
    target = np.asarray(sim.get_state().joint_positions[:6]) + (.4, 0., 0., 0., 0., 0.)
    sim.command_joint_impedance(target, (1000.,) * 6, (1.,) * 6, timeout_s=1.)
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


@pytest.mark.parametrize('steps', [0, 7])
@pytest.mark.parametrize('field', ['qpos', 'qvel', 'time'])
def test_invalid_feedback_fails_before_physics_or_output_mutation(sim, steps, field):
    sim.command_joint_impedance(sim.get_state().joint_positions[:6], STIFFNESS, DAMPING, timeout_s=1.)
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


def test_overflowing_gains_rejected_on_entry_renewal_and_restore(sim):
    target = list(sim.get_state().joint_positions[:6])
    target[0] = 2.8
    before = sim.save_state()
    with pytest.raises(ValueError, match='overflowed'):
        sim.command_joint_impedance(target, (1e308,)*6, DAMPING, timeout_s=1.)
    assert sim.save_state() == before
    sim.command_joint_impedance(sim.get_state().joint_positions[:6], STIFFNESS, DAMPING, timeout_s=1.)
    checkpoint = sim.save_state()
    with pytest.raises(ValueError, match='overflowed'):
        sim.command_joint_impedance(target, (1e308,)*6, DAMPING, timeout_s=1.)
    assert sim.save_state() == checkpoint
    bad = replace(checkpoint, impedance_command=JointImpedanceCommand(target, (1e308,)*6, DAMPING))
    with pytest.raises(ValueError, match='overflowed'):
        sim.restore_state(bad)
    assert sim.save_state() == checkpoint

"""关节阻抗控制律的解析样本,参数边界与限矩限速检查."""

from dataclasses import FrozenInstanceError
from pathlib import Path

import numpy as np
import pytest

from rebotarm_force_control.joint_impedance import (
    ImpedanceMitOutput,
    JointImpedanceCommand,
    compute_joint_impedance,
)
from rebotarm_force_control.joint_impedance_budget import budget_violations, evaluate_gains
from rebotarm_force_control.joint_impedance_session import JointImpedanceSession
from rebotarm_force_control.joint_impedance_settings import JointImpedanceSettings
from rebotarm_force_control.mit_command import (
    PROTOCOL_KD_MAX,
    PROTOCOL_KP_MAX,
    evaluate_mit_torque,
)


def compute(command, **kwargs):
    inputs = dict(position=(0.,)*6, velocity=(0.,)*6, gravity=(0.,)*6,
                  effort_limits=(27.,)*6, rate_limits=(100.,)*6,
                  previous=(0.,)*6, dt=1.)
    inputs.update(kwargs)
    return compute_joint_impedance(command, **inputs)


def test_spring_damping_and_gravity_have_correct_sign_and_units():
    command = JointImpedanceCommand((.2, -.2, .1, -.1, 0., 0.), (10.,)*6, (2.,)*6)
    output = compute(command, position=(.1, -.1, 0., 0., 0., 0.),
                     velocity=(.3, -.3, 0., 0., .5, -.5), gravity=(1.,)*6)
    assert output.requested_torques == pytest.approx((1.4, .6, 2., 0., 0., 2.))
    assert output.applied_torques == output.requested_torques
    assert not any(output.saturated)


def test_zero_gains_leave_only_gravity():
    command = JointImpedanceCommand((1.,)*6, (0.,)*6, (0.,)*6)
    assert compute(command, gravity=(1., -2., 3., -4., 5., -6.)).applied_torques == (
        1., -2., 3., -4., 5., -6.)


def test_total_torque_and_slew_are_bounded_from_actual_previous_output():
    command = JointImpedanceCommand((1., -1., 1., -1., 1., -1.), (100.,)*6, (0.,)*6)
    result = compute(command, effort_limits=(5.,)*6, rate_limits=(2.,)*6,
                     previous=(4.9, -4.9, 0., 0., -4.9, 4.9), dt=.1)
    assert result.applied_torques == pytest.approx((5., -5., .2, -.2, -4.7, 4.7))
    assert all(result.saturated)
    assert max(abs(v) for v in result.applied_torques) <= 5.


def test_command_copies_mutable_input_and_is_immutable():
    target, stiffness = [0.]*6, np.ones(6)
    command = JointImpedanceCommand(target, stiffness, (2.,)*6)
    target[0] = 9.
    stiffness[0] = 9.
    assert command.target == (0.,)*6
    assert command.stiffness == (1.,)*6
    with pytest.raises(FrozenInstanceError):
        command.target = (1.,)*6


@pytest.mark.parametrize('field', ['target', 'stiffness', 'damping'])
@pytest.mark.parametrize('value', [(0.,)*5, [[0.]*6], (float('nan'),)*6, (float('inf'),)*6])
def test_command_rejects_invalid_vectors(field, value):
    fields = dict(target=(0.,)*6, stiffness=(1.,)*6, damping=(1.,)*6)
    fields[field] = value
    with pytest.raises(ValueError):
        JointImpedanceCommand(**fields)


@pytest.mark.parametrize('field', ['stiffness', 'damping'])
def test_command_rejects_negative_gains(field):
    fields = dict(target=(0.,)*6, stiffness=(1.,)*6, damping=(1.,)*6)
    fields[field] = (-.1,)*6
    with pytest.raises(ValueError):
        JointImpedanceCommand(**fields)


@pytest.mark.parametrize('field', ['position', 'velocity', 'gravity', 'effort_limits', 'rate_limits', 'previous'])
def test_calculation_rejects_nonfinite_inputs(field):
    command = JointImpedanceCommand((0.,)*6, (1.,)*6, (1.,)*6)
    with pytest.raises(ValueError):
        compute(command, **{field: (float('nan'),)*6})


@pytest.mark.parametrize('kwargs', [
    {'dt': 0.}, {'dt': -1.}, {'dt': float('nan')}, {'dt': float('inf')},
    {'effort_limits': (0.,)*6}, {'rate_limits': (-1.,)*6}, {'previous': (28.,)*6},
])
def test_calculation_rejects_invalid_constraints(kwargs):
    with pytest.raises(ValueError):
        compute(JointImpedanceCommand((0.,)*6, (1.,)*6, (1.,)*6), **kwargs)


def test_overflow_is_rejected_before_clamping():
    command = JointImpedanceCommand((1e200,)*6, (1e200,)*6, (0.,)*6)
    with pytest.raises(ValueError, match='overflowed'):
        compute(command)


def _session_inputs(now: float = 0.0) -> dict:
    return dict(
        position=(0.0,) * 6,
        velocity=(0.0,) * 6,
        gravity=(1.0,) * 6,
        effort_limits=(27.0, 27.0, 27.0, 7.0, 7.0, 7.0),
        rate_limits=(108.0, 108.0, 108.0, 28.0, 28.0, 28.0),
        previous_torque=(0.0,) * 6,
        now=now,
    )


def test_settings_loads_repo_yaml_and_rejects_invalid_limits(tmp_path):
    yaml_path = (
        Path(__file__).resolve().parents[1]
        / "src/rebotarm_force_control/config/joint_impedance.yaml"
    )
    settings = JointImpedanceSettings.from_yaml(yaml_path)
    assert settings.max_stiffness_nm_rad == (PROTOCOL_KP_MAX,) * 6
    assert settings.max_damping_nm_s_rad == (PROTOCOL_KD_MAX,) * 6
    assert not hasattr(settings, "default_stiffness")
    assert not hasattr(settings, "min_timeout_s")

    bad_yaml = tmp_path / "bad.yaml"
    bad_yaml.write_text("unknown_key: 1\n", encoding="utf-8")
    with pytest.raises(ValueError, match="unknown"):
        JointImpedanceSettings.from_yaml(bad_yaml)

    with pytest.raises(ValueError):
        JointImpedanceSettings(max_stiffness_nm_rad=(501.0,) * 6)
    with pytest.raises(ValueError):
        JointImpedanceSettings(max_damping_nm_s_rad=(5.1,) * 6)

    guarded = JointImpedanceSettings(
        max_stiffness_nm_rad=(30.0,) * 6,
        max_damping_nm_s_rad=(3.0,) * 6,
    )
    guarded.validate_command(JointImpedanceCommand((0.0,) * 6, (30.0,) * 6, (3.0,) * 6))
    with pytest.raises(ValueError, match="stiffness"):
        guarded.validate_command(JointImpedanceCommand((0.0,) * 6, (31.0,) * 6, (1.0,) * 6))
    with pytest.raises(ValueError, match="damping"):
        guarded.validate_command(JointImpedanceCommand((0.0,) * 6, (10.0,) * 6, (3.1,) * 6))


def test_session_accept_two_phase_torque_and_commit_isolation():
    session = JointImpedanceSession()
    limits = ((-2.8, 2.8),) * 6
    cmd = JointImpedanceCommand((0.1,) * 6, (20.0,) * 6, (2.0,) * 6)
    inputs = _session_inputs(now=0.1)
    accepted = session.accept(
        cmd,
        position_limits=limits,
        nominal_dt=0.01,
        **inputs,
    )
    assert accepted == cmd.target
    assert session.active and session.latched_mit is None
    assert session.last_control_time == pytest.approx(0.0)

    # 非法溢出命令在 accept 前被拒绝,保留原状态.
    bad_cmd = JointImpedanceCommand((2.8,) * 6, (1e308,) * 6, (0.0,) * 6)
    with pytest.raises(ValueError, match="overflowed"):
        session.accept(
            bad_cmd,
            position_limits=limits,
            nominal_dt=0.01,
            **_session_inputs(now=0.15),
        )
    assert session.command == cmd

    # 阶段 1 只计算不推进控制时刻或历史请求;未调 commit 前重复调用结果一致.
    step1 = session.compute_torque_step(**_session_inputs(now=0.01))
    assert session.last_control_time == pytest.approx(0.0)
    assert session.last_requested_torques == (0.0,) * 6
    step1_retry = session.compute_torque_step(**_session_inputs(now=0.01))
    assert step1_retry == step1

    session.commit(
        requested_torques=step1.requested_torques,
        applied_torques=step1.applied_torques,
        now=0.01,
    )
    assert session.last_control_time == pytest.approx(0.01)
    assert session.last_requested_torques == step1.requested_torques

    # 同拍重入(elapsed <= 0)与热更新不消耗变化率预算.
    renew_cmd = JointImpedanceCommand((0.2,) * 6, (20.0,) * 6, (2.0,) * 6)
    session.accept(
        renew_cmd,
        position_limits=limits,
        nominal_dt=0.01,
        **_session_inputs(now=0.01),
    )
    assert session.last_control_time == pytest.approx(0.01)
    same_tick = session.compute_torque_step(
        position=(0.0,) * 6,
        velocity=(0.0,) * 6,
        gravity=(1.0,) * 6,
        effort_limits=inputs["effort_limits"],
        rate_limits=inputs["rate_limits"],
        previous_torque=step1.applied_torques,
        now=0.01,
    )
    assert same_tick.applied_torques == step1.applied_torques


def test_session_mit_step_latching_scale_and_protocol_bounds():
    session = JointImpedanceSession()
    limits = ((-2.8, 2.8),) * 6
    cmd = JointImpedanceCommand(
        (0.2, -0.1, 0.05, 0.0, 0.0, 0.0),
        (45.0, 45.0, 45.0, 17.5, 17.5, 17.5),
        (3.32, 5.0, 5.0, 1.28, 0.78, 0.34),
    )
    session.accept(
        cmd,
        position_limits=limits,
        nominal_dt=0.01,
        settings=JointImpedanceSettings(),
        **_session_inputs(now=0.0),
    )
    # t=0 首次调用时 elapsed == 0,不消耗变化率预算并保持 previous_torque.
    zero_tick = session.compute_mit_step(
        position=(0.0,) * 6,
        velocity=(0.0,) * 6,
        gravity=(1.0,) * 6,
        effort_limits=(27.0, 27.0, 27.0, 7.0, 7.0, 7.0),
        rate_limits=(108.0, 108.0, 108.0, 28.0, 28.0, 28.0),
        previous_torque=(1.0,) * 6,
        now=0.0,
    )
    assert zero_tick.kp == (0.0,) * 6 and zero_tick.kd == (0.0,) * 6
    assert zero_tick.estimated_torques == pytest.approx((1.0,) * 6)

    mit = session.compute_mit_step(
        position=(0.0,) * 6,
        velocity=(0.2, -0.1, 0.0, 0.0, 0.0, 0.0),
        gravity=(1.0, -2.0, 3.0, 0.5, 0.0, 0.0),
        effort_limits=(27.0, 27.0, 27.0, 7.0, 7.0, 7.0),
        rate_limits=(108.0, 108.0, 108.0, 28.0, 28.0, 28.0),
        previous_torque=(0.0,) * 6,
        now=0.01,
    )
    assert isinstance(mit, ImpedanceMitOutput)
    torque_ref = session.compute_torque_step(
        position=(0.0,) * 6,
        velocity=(0.2, -0.1, 0.0, 0.0, 0.0, 0.0),
        gravity=(1.0, -2.0, 3.0, 0.5, 0.0, 0.0),
        effort_limits=(27.0, 27.0, 27.0, 7.0, 7.0, 7.0),
        rate_limits=(108.0, 108.0, 108.0, 28.0, 28.0, 28.0),
        previous_torque=(0.0,) * 6,
        now=0.01,
    )
    assert mit.estimated_torques == pytest.approx(torque_ref.applied_torques)
    assert mit.requested_torques == pytest.approx(torque_ref.requested_torques)
    # 当前反馈下 MIT 内环合成总力矩精确等于 bounded estimated_torques.
    raw_eval, bounded_eval = evaluate_mit_torque(
        mit.target,
        np.zeros(6),
        np.array([0.2, -0.1, 0.0, 0.0, 0.0, 0.0]),
        mit.kp,
        mit.kd,
        mit.feedforward_torques,
        (27.0, 27.0, 27.0, 7.0, 7.0, 7.0),
    )
    assert raw_eval == pytest.approx(mit.estimated_torques, abs=1e-12)
    assert bounded_eval == pytest.approx(mit.estimated_torques, abs=1e-12)
    assert session.last_control_time == pytest.approx(0.0)
    assert session.latched_mit is None

    session.commit(
        requested_torques=mit.requested_torques,
        applied_torques=mit.estimated_torques,
        now=0.01,
        mit_output=mit,
    )
    assert session.latched_mit == mit

    # 域随机化 torque_scale 与 compute_torque_step 对齐.
    scaled_mit = session.compute_mit_step(
        position=(0.0,) * 6,
        velocity=(0.0,) * 6,
        gravity=(1.0,) * 6,
        effort_limits=(27.0, 27.0, 27.0, 7.0, 7.0, 7.0),
        rate_limits=(108.0, 108.0, 108.0, 28.0, 28.0, 28.0),
        previous_torque=mit.estimated_torques,
        now=0.02,
        torque_scale=2.0,
    )
    scaled_ref = session.compute_torque_step(
        position=(0.0,) * 6,
        velocity=(0.0,) * 6,
        gravity=(1.0,) * 6,
        effort_limits=(27.0, 27.0, 27.0, 7.0, 7.0, 7.0),
        rate_limits=(108.0, 108.0, 108.0, 28.0, 28.0, 28.0),
        previous_torque=mit.estimated_torques,
        now=0.02,
        torque_scale=2.0,
    )
    assert scaled_mit.estimated_torques == pytest.approx(scaled_ref.applied_torques)

    over_session = JointImpedanceSession()
    over_session.accept(
        JointImpedanceCommand((0.0,) * 6, (600.0,) * 6, (1.0,) * 6),
        position_limits=limits,
        nominal_dt=0.01,
        **_session_inputs(now=0.0),
    )
    with pytest.raises(ValueError, match="protocol limit"):
        over_session.compute_mit_step(**_session_inputs(now=0.01))


def test_session_leave_handoff_clamp_and_snapshot_restore():
    session = JointImpedanceSession()
    limits = ((-2.8, 2.8),) * 6
    cmd = JointImpedanceCommand((0.2,) * 6, (40.0,) * 6, (2.0,) * 6)
    session.accept(cmd, position_limits=limits, nominal_dt=0.01, **_session_inputs(now=0.0))
    mit = session.compute_mit_step(**_session_inputs(now=0.01))
    session.commit(
        requested_torques=mit.requested_torques,
        applied_torques=mit.estimated_torques,
        now=0.01,
        mit_output=mit,
    )
    snap = session.snapshot()
    assert snap["impedance_command"] == cmd
    assert snap["impedance_mit_output"] == mit
    assert not snap["impedance_handoff"]

    session.leave()
    assert not session.active and session.latched_mit is None and session.handoff
    clamped = session.clamp_handoff_torque(
        (20.0,) * 6,
        (27.0, 27.0, 27.0, 7.0, 7.0, 7.0),
        (100.0,) * 6,
        mit.estimated_torques,
        now=0.02,
    )
    assert np.all(np.abs(clamped - np.asarray(mit.estimated_torques)) <= 1.0 + 1e-12)
    session.commit(requested_torques=clamped, applied_torques=clamped, now=0.02, clear_handoff=True)
    assert not session.handoff

    restored = JointImpedanceSession()
    restored.restore(
        command=snap["impedance_command"],
        mit_output=snap["impedance_mit_output"],
        requested_torques=snap["impedance_requested_torques"],
        handoff=snap["impedance_handoff"],
        last_control_time=snap["last_control_time"],
        candidate_time=0.01,
        candidate_torque=mit.estimated_torques,
        position_limits=limits,
        position=(0.0,) * 6,
        velocity=(0.0,) * 6,
        gravity=(1.0,) * 6,
        effort_limits=(27.0, 27.0, 27.0, 7.0, 7.0, 7.0),
        rate_limits=(108.0, 108.0, 108.0, 28.0, 28.0, 28.0),
        nominal_dt=0.01,
    )
    assert restored.snapshot() == snap

    with pytest.raises(ValueError, match="inactive"):
        restored.validate_snapshot_state(
            command=None,
            mit_output=mit,
            requested_torques=(),
            handoff=False,
            last_control_time=0.01,
            candidate_time=0.01,
            candidate_torque=(0.0,) * 6,
            position_limits=limits,
            position=(0.0,) * 6,
            velocity=(0.0,) * 6,
            gravity=(1.0,) * 6,
            effort_limits=(27.0, 27.0, 27.0, 7.0, 7.0, 7.0),
            rate_limits=(108.0, 108.0, 108.0, 28.0, 28.0, 28.0),
            nominal_dt=0.01,
        )


def test_evaluate_mit_torque_validates_inputs_and_overflow():
    with pytest.raises(ValueError, match="nonnegative"):
        evaluate_mit_torque((0.,)*6, (0.,)*6, (0.,)*6, (-1.,)*6, (1.,)*6, (0.,)*6, (10.,)*6)
    with pytest.raises(ValueError, match="overflowed"):
        evaluate_mit_torque((1e200,)*6, (0.,)*6, (0.,)*6, (1e200,)*6, (1.,)*6, (0.,)*6, (10.,)*6)


def test_pure_budget_functions_validate_protocol_and_torque_limits():
    report = evaluate_gains(
        stiffness=(45.0, 45.0, 45.0, 17.5, 17.5, 17.5),
        damping=(3.32, 5.0, 5.0, 1.28, 0.78, 0.34),
        rated=(9.0, 9.0, 9.0, 3.5, 3.5, 3.5),
        effort=(27.0, 27.0, 27.0, 7.0, 7.0, 7.0),
        gravity=(0.0, 3.6, 8.5, 2.6, 0.0, 0.0),
        inertia=(0.12, 0.37, 0.33, 0.05, 0.02, 0.004),
    )
    assert budget_violations(report, (9.0, 9.0, 9.0, 3.5, 3.5, 3.5), (27.0, 27.0, 27.0, 7.0, 7.0, 7.0)) == []

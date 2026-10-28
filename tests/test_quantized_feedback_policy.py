"""量化反馈的反例：真实漂移、振荡、持续速度、旧缓存和空档不能确认静止。"""
import math
from types import SimpleNamespace as NS

import numpy as np
import pytest

from rebotarmcontroller.stationary_evidence import StationaryEvidence, DisabledTransitionEvidence, POSITION_CODE
from rebotarmcontroller.arm_stop import ArmStop
from rebotarm_motion.stop_feedback import StopFeedback
from rebotarm_motion.trajectory_runtime_limits import JointRuntimeLimit, TrajectoryRuntimeLimitGuard


def message(t, q, v):
    sec, ns = divmod(round(t*1e9), 1_000_000_000)
    return NS(header=NS(stamp=NS(sec=sec, nanosec=ns)), name=[f'joint{i}' for i in range(1, 7)],
              position=[q]*6, velocity=[v]*6)


@pytest.mark.parametrize('owner', ['controller', 'motion'])
@pytest.mark.parametrize('case', ['quiet', 'pulse', 'continuous', 'slow_drift', 'oscillation',
                                  'sustained_speed', 'duplicate', 'gap', 'invalid'])
def test_stationary_counterexamples(owner, case):
    # 固定请求锚点；漂移速度低于阈值，单个滑窗也不足三个编码，仍须拒绝累积漂移。
    motion = StopFeedback()
    device = StationaryEvidence(window=.3)
    device.reset([0.]*6)
    result_at_pulse = None
    for k in range(101):
        t = 10.+k*.02
        q = (.02 if case == 'continuous' else .001 if case == 'slow_drift' else 0.)*(t-10.)
        if case == 'oscillation':
            q = .003*math.sin(k*.7)
        v = .11 if case == 'sustained_speed' or (case == 'pulse' and k == 20) else 0.
        if case == 'invalid' and k == 90:
            q = float('nan')
        if case == 'gap' and 75 <= k < 90:
            continue
        stamp = 10. if case == 'duplicate' else t
        motion.update(message(stamp, q, v), t, round(t*1e9))
        reason = device.observe([q]*6, [v]*6, stamp, t, sequences=[0 if case == 'duplicate' else k]*6)
        if reason in ('invalid_or_stale', 'source_gap', 'source_regressed'):
            device_broken = True
        elif k == 0:
            device_broken = False
        mc = motion.stationary(10., t, round(t*1e9))
        dc = not device_broken and device.confirmed()
        if k == 20:
            result_at_pulse = mc if owner == 'motion' else dc
    assert (mc if owner == 'motion' else dc) == (case in ('quiet', 'pulse'))
    if case == 'pulse':
        assert not result_at_pulse  # 不是当场放过单帧峰值，而是等待后续完整正常窗口。


def test_controller_stop_requires_all_motor_sequences_to_advance():
    clock = [10.]
    stop = ArmStop(clock=lambda: clock[0])
    stop.begin([0.]*6, 1)
    stop.sent(1)
    for k in range(1, 20):
        clock[0] = 10.+k*.02
        result = stop.update([0.]*6, [0.]*6, clock[0], generation=1, enabled=True,
                             sequences=[k]*5+[1])
        assert result.state == 'stopping'


def test_gap_recovery_cannot_confirm_old_motion_request_but_new_request_can():
    evidence = StopFeedback()
    for k in list(range(20))+list(range(40, 70)):
        t = 10.+k*.02
        evidence.update(message(t, 0., 0.), t, round(t*1e9))
        evidence.stationary(10., t, round(t*1e9))
    assert not evidence.stationary(10., t, round(t*1e9))
    assert evidence.stationary(10.82, t, round(t*1e9))


def test_derivative_diagnostic_policy_keeps_direct_limits_and_exact_differences():
    limit = {'joint1': JointRuntimeLimit(max_velocity=1., max_effort=2., max_acceleration=5., max_jerk=20.)}
    guard = TrajectoryRuntimeLimitGuard(limit, derivative_policy='diagnostic')
    for t, v in zip([0., .02, .04], [-.5, .5, -.5]):
        assert guard.observe(joint_names=['joint1'], velocities=[v*20/4095], efforts=[0.], elapsed=t) is None
    assert guard.jerks['joint1'] == pytest.approx(-24.42002442)
    assert [v.kind for v in guard.derivative_diagnostics] == ['jerk']
    assert guard.observe(joint_names=['joint1'], velocities=[1.1], efforts=[0.], elapsed=.06).kind == 'velocity'
    assert guard.observe(joint_names=['joint1'], velocities=[0.], efforts=[2.1], elapsed=.08).kind == 'effort'


def test_historical_jerk_requires_two_code_pulse_at_logged_intervals():
    guard = TrajectoryRuntimeLimitGuard({'joint1': JointRuntimeLimit(max_jerk=20.)}, derivative_policy='diagnostic')
    for stamp, v in [(1793198598261875309, .0024423599243164062),
                     (1793198598286196359, .012209892272949219),
                     (1793198598309828550, .0024423599243164062)]:
        assert guard.observe(joint_names=['joint1'], velocities=[v], efforts=[0.], elapsed=stamp/1e9) is None
    assert guard.jerks['joint1'] == pytest.approx(-34.4837183000466)


@pytest.mark.parametrize('fault', ['', 'no_receipt', 'unknown_status', 'position', 'gap', 'velocity'])
def test_disabled_transition_requires_receipt_all_statuses_and_full_position_evidence(fault):
    transition = DisabledTransitionEvidence([0.]*7, [0]*7, 10.)
    if fault != 'no_receipt':
        transition.service_result(True)
    for k in range(1, 100):
        t = 10.+k*.02
        if fault == 'gap' and 10 <= k < 25:
            continue
        q = 4*POSITION_CODE if fault == 'position' and k == 15 else 0.
        # 失能后的孤立速度峰必须记录和延迟确认，但不单独判过渡失败。
        v = .1685 if k == 3 or fault == 'velocity' else 0.
        codes = [0]*7
        if fault == 'unknown_status' and k == 20:
            codes[-1] = 255
        transition.observe([q]*7, [v]*7, codes, [k]*7, t, t)
        if k < 54:
            assert not transition.confirmed(t)
    assert bool(transition.confirmed(t)) == (fault == '')
    assert not transition.confirmed(t+1.)
    assert transition.velocity_events >= 1


def test_position_extrema_reject_same_endpoints():
    evidence = StationaryEvidence()
    for k, q in enumerate([0., 2*POSITION_CODE, -2*POSITION_CODE, 0., 0.]):
        evidence.observe([q]*6, [0.]*6, 10.+k*.05, 10.+k*.05)
    assert not evidence.confirmed()


def test_sequence_regression_and_missing_first_transition_frames_are_failures():
    evidence = StationaryEvidence()
    evidence.observe([0.]*6, [0.]*6, 10., 10., sequences=[10]*6)
    assert evidence.observe([0.]*6, [0.]*6, 10.02, 10.02, sequences=[0]*6) == 'source_regressed'
    transition = DisabledTransitionEvidence([0.]*7, [0]*7, 10.)
    transition.service_result(True)
    transition.observe([0.]*7, [0.]*7, [0]*7, [1]*7, 10.3, 10.3)
    assert transition.failure == 'source_gap' and not transition.confirmed(10.3)

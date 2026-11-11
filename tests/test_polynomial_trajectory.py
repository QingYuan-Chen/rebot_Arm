"""P005 曲线生成、规范化和安全检查的独立数值回归。

文件职责：用已知多项式、闭式峰值和段内反例验证分级插值、极值覆盖、碰撞预算及交接规则。
对外接口：pytest 用例；LIMITS/LOWER/UPPER、context、collision_pass 供相关测试复用。
安全约束：不使用真实设备或 ROS 图；collision_pass 只是覆盖计数替身，不证明真实无碰撞。
    捕获的 MoveIt 样本保留为 jerk 超限拒绝证据，不能修改样本或限值使测试通过。
文件结构：规范限值与夹具 → 分级/连续性/往返 → 解析峰值和段内极值
    → 起点与上下文 → 碰撞失败 → 真实样本和离线 CLI。
"""

from dataclasses import replace
import json
import math
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

from rebotarm_motion.polynomial_trajectory import (
    JOINT_NAMES, TrajectoryPoint as Point, build_curve, curve_from_payload,
)
from rebotarm_motion.trajectory_preparation import (
    PreparationContext, PreparedTrajectory, analyze_trajectory,
    check_curve, compare_reference, load_reference_limits, minimum_point_to_point_duration,
    normalize_boundaries, point_to_point,
)
from rebotarm_motion.trajectory_runtime_limits import JointRuntimeLimit


ROOT = Path(__file__).resolve().parents[1]
START = (0., -1., -1., 0., .3, 0.)
ZERO = (0.,) * 6
TARGET = (.02, -1., -1., 0., .3, 0.)
LIMITS = {name: JointRuntimeLimit(max_velocity=.5, max_acceleration=5., max_jerk=20.)
          for name in JOINT_NAMES}
LOWER, UPPER = (-3.,) * 6, (3.,) * 6


def context(**values):
    return replace(PreparationContext(1, 2, START, START, 10., True, True), **values)


def collision_pass(names, positions):
    assert names == JOINT_NAMES
    assert all(len(q) == 6 for q in positions)
    return dict(state='pass', checked_samples=len(positions), requested_samples=len(positions))


def analyze(points=None, **kwargs):
    options = dict(lower=LOWER, upper=UPPER, limits=LIMITS,
                   context=context(), now=10., collision_check=collision_pass)
    options.update(kwargs)
    return analyze_trajectory(JOINT_NAMES, points or point_to_point(START, TARGET), **options)


@pytest.mark.parametrize('degree', [1, 3, 5])
def test_analytic_polynomial_reconstructed_at_all_nodes_and_between(degree):
    # 用独立已知多项式约束构造, 而非用实现公式计算预期值.
    p = np.polynomial.Polynomial((.2, -.1, .08, -.02, .003, -.0002)[:degree + 1])
    points = [Point(t, (p(t),) * 6,
        (p.deriv(1)(t),) * 6 if degree >= 3 else (),
        (p.deriv(2)(t),) * 6 if degree == 5 else ()) for t in (0., .2, .9, 2.)]
    curve = build_curve(JOINT_NAMES, points)
    for t in (0., .1, .2, .5, .9, 1.2, 2.):
        for order, values in enumerate(curve.sample(t)):
            assert values == pytest.approx((p.deriv(order)(t),) * 6, abs=1e-8)
    assert curve.continuity()['guaranteed_order'] == {1: 0, 3: 1, 5: 2}[degree]


def test_shared_nonzero_interior_derivatives_preserved_and_c2_with_hold():
    points = (Point(0., START, ZERO, ZERO),
              Point(1., TARGET, (.03,) * 6, (-.02,) * 6),
              Point(2., START, ZERO, ZERO))
    curve = build_curve(JOINT_NAMES, points)
    assert curve.sample(1.)[1] == pytest.approx((.03,) * 6)
    assert curve.sample(1.)[2] == pytest.approx((-.02,) * 6)
    assert curve.continuity()['c2_with_hold']
    for t, expected in ((-1., START), (3., START)):
        assert curve.sample(t) == (expected, ZERO, ZERO, ZERO)


def test_cubic_and_linear_report_boundary_jumps_instead_of_claiming_c2():
    for degree, points in ((1, (Point(0, START), Point(1, TARGET), Point(2, START))),
            (3, (Point(0, START, ZERO), Point(1, TARGET, ZERO), Point(2, START, ZERO)))):
        curve = build_curve(JOINT_NAMES, points)
        continuity = curve.continuity()
        assert not continuity['c2_with_hold']
        assert continuity['max_boundary_gaps']['velocity' if degree == 1 else 'acceleration'] > 0


def test_joint_reorder_moves_positions_and_all_derivatives_together():
    points = (Point(0, START, (.01, .02, .03, .04, .05, .06), ZERO),
              Point(1, TARGET, ZERO, ZERO))
    reversed_points = [Point(p.time, p.positions[::-1], p.velocities[::-1],
                             p.accelerations[::-1]) for p in points]
    first = build_curve(JOINT_NAMES, points)
    second = build_curve(JOINT_NAMES[::-1], reversed_points)
    assert first.to_payload() == second.to_payload()


@pytest.mark.parametrize('points,match', [
    ([Point(0, START), Point(0, TARGET)], 'strictly'),
    ([Point(1, START), Point(.5, TARGET)], 'strictly'),
    ([Point(0, START), Point(1, TARGET, ZERO)], 'mixed missing velocities'),
    ([Point(0, START, ZERO), Point(1, TARGET, ZERO, ZERO)], 'mixed missing accelerations'),
    ([Point(0, START, (), ZERO)], 'require velocities'),
    ([Point(1, TARGET)], 'explicit initial'),
])
def test_bad_point_contract_rejected(points, match):
    with pytest.raises(ValueError, match=match):
        build_curve(JOINT_NAMES, points)


@pytest.mark.parametrize('kwargs', [dict(time=float('nan'), positions=START),
    dict(time=-1, positions=START), dict(time=0, positions=[0.] * 5),
    dict(time=0, positions=[float('inf')] * 6),
    dict(time=0, positions=START, velocities=[float('nan')] * 6)])
def test_nonfinite_or_bad_shape_rejected_at_value_boundary(kwargs):
    with pytest.raises(ValueError):
        Point(**kwargs)


def test_immutable_input_and_json_roundtrip_preserve_actual_coefficients():
    values = list(START)
    points = (Point(0, values, ZERO, ZERO), Point(.4, TARGET, ZERO, ZERO))
    values[0] = 99.
    curve = build_curve(JOINT_NAMES, points)
    payload = json.loads(json.dumps(curve.to_payload(), allow_nan=False))
    restored = curve_from_payload(payload)
    assert curve.points[0].positions == START
    for t in (0., .003, .12, .2, .399, .4):
        assert restored.sample(t) == curve.sample(t)
    payload['coefficients'][0][0][-1] += .001
    with pytest.raises(ValueError, match='coefficient'):
        curve_from_payload(payload)


def test_quintic_peaks_and_minimum_duration_match_closed_form():
    curve = build_curve(JOINT_NAMES, point_to_point(START, TARGET))
    report = check_curve(curve, LOWER, UPPER, LIMITS)
    peaks = report['peaks']['joint1']
    assert report['state'] == 'pass'
    assert peaks['velocity']['absolute'] == pytest.approx(1.875 * .02 / .4)
    assert peaks['acceleration']['absolute'] == pytest.approx(10 / math.sqrt(3) * .02 / .4**2)
    assert peaks['jerk']['absolute'] == pytest.approx(60 * .02 / .4**3)
    assert minimum_point_to_point_duration(START, TARGET, LIMITS) == pytest.approx((.06)**(1/3))
    rejected, report = analyze(point_to_point(START, TARGET, .2))
    assert rejected is None and report['state'] == 'rejected'
    assert any(v['kind'] == 'jerk' and v['value'] == pytest.approx(150.)
               for v in report['limits']['issues'])


@pytest.mark.parametrize('kind', ['position', 'velocity', 'acceleration', 'jerk'])
def test_checks_interior_extrema_not_only_waypoints(kind):
    # q=4u(1-u), 原节点位置为零、速度绝对值4, 加速度为-8.
    p = Point(0., ZERO, (4.,) * 6)
    q = Point(1., ZERO, (-4.,) * 6)
    curve = build_curve(JOINT_NAMES, (p, q))
    limits = {name: JointRuntimeLimit(max_velocity=10, max_acceleration=10,
                                     max_jerk=10) for name in JOINT_NAMES}
    upper = (.5,) * 6 if kind == 'position' else UPPER
    if kind != 'position':
        # 用完整五次曲线得到端点速度/加速度为零, 但内部存在峰值的反例.
        curve = build_curve(JOINT_NAMES, point_to_point(ZERO, (.1,) * 6, 1.))
        for name in limits:
            limits[name] = replace(limits[name], **{'max_' + kind: .01})
    report = check_curve(curve, LOWER, upper, limits)
    assert report['state'] == 'rejected'
    assert any(issue['kind'] == kind for issue in report['issues'])
    if kind == 'position':
        assert any(issue['time'] == pytest.approx(.5) and issue['value'] == pytest.approx(1.)
                   for issue in report['issues'])


def test_collision_subdivision_covers_returning_curve_and_extremum():
    curve = build_curve(JOINT_NAMES, (Point(0, ZERO, (4.,)*6), Point(1, ZERO, (-4.,)*6)))
    times = curve.collision_times()
    assert .5 in times and times[0] == 0 and times[-1] == 1
    assert len(times) >= 201  # 相同端点不能被误当成零位移.
    for a, b in zip(times, times[1:]):
        assert 4 * (b - a) <= .02000000001
    with pytest.raises(ValueError, match='sample budget'):
        curve.collision_times(max_samples=10)


def test_jerk_extremum_inside_segment_is_checked_when_endpoints_are_zero():
    # q'''(t)=t-t², 两端 jerk 为零, 内部 t=.5 达到 .25.
    p = np.polynomial.Polynomial((0., 0., 0., 0., 1/24, -1/60))
    points = tuple(Point(t, (p(t),)*6, (p.deriv(1)(t),)*6, (p.deriv(2)(t),)*6)
                   for t in (0., 1.))
    curve = build_curve(JOINT_NAMES, points)
    limits = {name: replace(limit, max_jerk=.2) for name, limit in LIMITS.items()}
    assert curve.sample(0.)[3] == pytest.approx(ZERO)
    assert curve.sample(1.)[3] == pytest.approx(ZERO)
    report = check_curve(curve, LOWER, UPPER, limits)
    assert report['state'] == 'rejected'
    assert all(issue['kind'] == 'jerk' and issue['time'] == pytest.approx(.5)
               and issue['value'] == pytest.approx(.25) for issue in report['issues'])


def test_normalize_once_preserves_middle_timestamps_and_target():
    source = (Point(0, tuple(v + .0001 for v in START), (.1,)*6, (.2,)*6),
              Point(.4, TARGET, (.02,)*6, (.01,)*6),
              Point(1., TARGET, (.1,)*6, (.2,)*6))
    points = normalize_boundaries(JOINT_NAMES, source, context(), now=10.)
    assert points[0] == Point(0., START, ZERO, ZERO)
    assert points[1] == source[1]
    assert points[-1] == Point(1., TARGET, ZERO, ZERO)
    assert source[0].velocities == (.1,) * 6
    shifted = normalize_boundaries(JOINT_NAMES,
        (Point(0., tuple(v + .003 for v in START)), Point(1., TARGET)), context(), now=10.)
    assert shifted[0].positions == START
    assert shifted[-1].positions == TARGET


def test_delayed_single_target_gets_explicit_hold_origin_and_zero_time_motion_rejected():
    points = normalize_boundaries(JOINT_NAMES, (Point(.4, TARGET),), context(), now=10.)
    assert points == (Point(0., START), Point(.4, TARGET))
    assert build_curve(JOINT_NAMES, points).sample(.2)[0][0] == pytest.approx(.01)
    with pytest.raises(ValueError, match='zero-duration'):
        normalize_boundaries(JOINT_NAMES, (Point(0., TARGET),), context(), now=10.)
    hold = build_curve(JOINT_NAMES, (Point(0., START, ZERO, ZERO),))
    assert hold.sample(1.) == (START, ZERO, ZERO, ZERO)


@pytest.mark.parametrize('changes', [dict(enabled=False), dict(busy=True),
    dict(observed_at=9.8), dict(observed_at=10.1)])
def test_bad_hold_evidence_prevents_preparation(changes):
    with pytest.raises(ValueError):
        analyze(context=context(**changes))


@pytest.mark.parametrize('residual', [.003, .01])
def test_stationary_residual_does_not_gate_checked_curve_or_reanchor_hold(residual):
    feedback = tuple(v+residual for v in START)
    snapshot = context(feedback=feedback)
    prepared, report = analyze(points=point_to_point(feedback,TARGET,1.),context=snapshot)
    assert report['state'] == 'pass'
    assert prepared.curve.points[0].positions == START
    assert prepared.context.hold == START
    assert prepared.context.feedback == feedback
    with pytest.raises(ValueError,match='first input position outside limits'):
        analyze(points=point_to_point((4.,)*6,TARGET,1.),context=snapshot)


@pytest.mark.parametrize('collision_check', [None,
    lambda *_: dict(state='unknown', message='timeout'),
    lambda *_: dict(state='disabled'),
    lambda *_: dict(state='pass', checked_samples=0, requested_samples=0),
    lambda *_: dict(state='collision'),
    lambda *_: (_ for _ in ()).throw(RuntimeError('service unavailable'))])
def test_collision_failures_never_produce_dispatch_candidate(collision_check):
    prepared, report = analyze(collision_check=collision_check)
    assert prepared is None and report['state'] != 'pass'


def test_no_hold_context_only_produces_analysis():
    candidate, report = analyze(context=None)
    assert candidate is None and report['state'] == 'unknown'
    assert report['limits']['state'] == 'pass'


@pytest.mark.parametrize('changes', [dict(session=2), dict(generation=3),
    dict(hold=tuple(v + .0001 for v in START)), dict(enabled=False),
    dict(busy=True)])
def test_stale_result_rejected_after_serialization(changes):
    candidate, report = analyze()
    assert report['state'] == 'pass'
    restored = PreparedTrajectory.from_payload(json.loads(json.dumps(candidate.to_payload())))
    with pytest.raises(ValueError):
        restored.require_current(context(**changes), now=10.)
    restored.require_current(context(observed_at=11.), now=11.)


def test_numerical_extrema_failure_is_unknown(monkeypatch):
    monkeypatch.setattr(np.polynomial.polynomial, 'polyroots',
                        lambda _: (_ for _ in ()).throw(np.linalg.LinAlgError('injected')))
    candidate, report = analyze()
    assert candidate is None and report['state'] == 'unknown'


def test_numerical_construction_failure_and_missing_limits_are_unknown():
    candidate, report = analyze(point_to_point(START, TARGET, 1e-200))
    assert candidate is None and report['state'] == 'unknown'
    missing = dict(LIMITS, joint1=replace(LIMITS['joint1'], max_jerk=None))
    candidate, report = analyze(limits=missing)
    assert candidate is None and report['state'] == 'unknown'
    assert 'complete velocity' in report['reason']
    with pytest.raises(ValueError, match='zero-duration hold'):
        build_curve(JOINT_NAMES, (Point(0., START, (.1,) * 6, ZERO),))


def test_reference_comparison_and_actual_model_limits():
    curve = build_curve(JOINT_NAMES, point_to_point(START, TARGET))
    report = compare_reference(curve, curve.points)
    assert report['source_max_gap'] == .4
    assert report['max_staircase_deviation'] > .019
    assert report['max_reference_increment'] < .0002
    assert report['max_linear_path_deviation'] > .002
    lower, upper, limits = load_reference_limits(ROOT/'src/rebotarm_moveit_config/config')
    assert upper[1:3] == pytest.approx((.02, .02))
    assert limits['joint6'].max_velocity == 1.8
    assert check_curve(curve, lower, upper, limits)['state'] == 'pass'


def test_captured_moveit_curve_preserves_derivatives_and_rejects_jerk():
    source = json.loads((ROOT/'tests/fixtures/p005_moveit_trajectory.json').read_text())
    points = tuple(Point(p['time_from_start'], p['positions'], p['velocities'],
                         p['accelerations']) for p in source['points'])
    lower, upper, limits = load_reference_limits(ROOT/'src/rebotarm_moveit_config/config')
    candidate, report = analyze_trajectory(source['joint_names'], points, lower=lower,
        upper=upper, limits=limits, context=context(observed_at=0.), now=0.,
        collision_check=lambda *_: pytest.fail('over-limit curve must not reach collision service'))
    assert candidate is None and report['state'] == 'rejected'
    assert report['source_points'] == 11 and report['degree'] == 5
    normalized = curve_from_payload(report['normalized_trajectory'])
    assert normalized.points[1:-1] == points[1:-1]
    assert report['limits']['continuity']['c2_with_hold']
    assert report['limits']['peaks']['joint1']['jerk']['absolute'] == pytest.approx(66.322633018, rel=1e-7)
    assert {issue['kind'] for issue in report['limits']['issues']} == {'jerk'}


def test_position_only_fjt_inputs_are_graded_without_inventing_derivatives():
    # 公共 FJT 的 q-only 输入契约保留，执行方式统一交由 continuous 准备。
    for source in ((Point(0., START), Point(.4, TARGET)), (Point(.2, TARGET),)):
        candidate, report = analyze(source)
        assert candidate is not None and report['degree'] == 1
        assert not report['limits']['continuity']['c2_with_hold']
        assert not report['limits']['derivative_limits_cover_node_jumps']
        assert all(not p.velocities and not p.accelerations for p in candidate.curve.points)
    assert analyze()[1]['limits']['continuity']['c2_with_hold']


def test_cli_is_analysis_only_and_does_not_overwrite_evidence(tmp_path):
    source, output = tmp_path/'input.json', tmp_path/'report.json'
    source.write_text(json.dumps(build_curve(JOINT_NAMES, point_to_point(START, TARGET)).to_payload()))
    command = [sys.executable, str(ROOT/'scripts/analyze_p005_trajectory.py'),
               str(source), '--output', str(output)]
    result = subprocess.run(command, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    report = json.loads(output.read_text())
    assert report['state'] == 'unknown' and report['collision']['state'] == 'unknown'
    before = output.read_bytes()
    assert subprocess.run(command, capture_output=True).returncode != 0
    assert output.read_bytes() == before

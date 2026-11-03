"""共享求值不依赖运动策略，仿真缺陷用物理状态回归。"""

import ast
import copy
import math
from pathlib import Path

import numpy as np
import pytest

from rebotarm_trajectory.polynomial import curve_from_payload
from rebotarm_trajectory.sampling import linear_sample, step_index, validate_samples
from rebotarm_motion.polynomial_trajectory import JOINT_NAMES, TrajectoryPoint, build_curve
from rebotarm_motion.trajectory_preparation import point_to_point
from rebotarm_simulation.core.mujoco_sim import RebotArmMujoco


def test_shared_package_has_only_standard_library_imports():
    root = Path(__file__).resolve().parents[1] / 'src/rebotarm_trajectory/rebotarm_trajectory'
    for path in root.glob('*.py'):
        for node in ast.walk(ast.parse(path.read_text())):
            if isinstance(node, ast.ImportFrom) and not node.level:
                assert node.module in ('bisect', 'dataclasses'), path
            elif isinstance(node, ast.Import):
                assert all(alias.name == 'math' for alias in node.names), path


@pytest.mark.parametrize('grade', [0, 1, 2])
def test_serialized_curve_samples_match_motion_and_preserve_coefficients(grade):
    zero = (0.,) * 6
    points = tuple(TrajectoryPoint(t, (q,) * 6,
        (v,) * 6 if grade >= 1 else (), zero if grade == 2 else ())
        for t, q, v in ((0., 0., 0.), (.4, .01, .03), (.9, .025, 0.)))
    motion = build_curve(JOINT_NAMES, points)
    decoded = curve_from_payload(motion.to_payload())
    for elapsed in (-.1, 0., .123, .4, .41, .9, 1.):
        np.testing.assert_allclose(decoded.sample(elapsed), motion.sample(elapsed), atol=1e-12)
    assert decoded.to_payload() == motion.to_payload()


def test_shared_quintic_matches_independent_analytic_midpoint_and_jerk():
    curve = curve_from_payload(build_curve(JOINT_NAMES,
        point_to_point((0.,) * 6, (.02,) * 6, .4)).to_payload())
    q, v, a, jerk = curve.sample(.2)
    assert q == pytest.approx((.01,) * 6)
    assert v == pytest.approx((.09375,) * 6)
    assert a == pytest.approx((0.,) * 6, abs=1e-12)
    assert jerk == pytest.approx((-9.375,) * 6)


@pytest.mark.parametrize('damage', ['nan', 'shape', 'endpoint', 'duration'])
def test_shared_decoder_rejects_damaged_payload(damage):
    payload = copy.deepcopy(build_curve(JOINT_NAMES,
        point_to_point((0.,) * 6, (.02,) * 6)).to_payload())
    payload['coefficients'] = [[list(row) for row in payload['coefficients'][0]]]
    if damage == 'nan':
        payload['coefficients'][0][0][2] = math.nan
    elif damage == 'shape':
        payload['coefficients'][0].pop()
    elif damage == 'endpoint':
        payload['coefficients'][0][0][0] += .1
    else:
        payload['points'][-1]['time_from_start'] = 0.
    with pytest.raises(ValueError):
        curve_from_payload(payload)


def test_legacy_lookup_matches_prior_due_point_and_linear_semantics():
    points, times = validate_samples([(1.,) * 6, (2.,) * 6, (3.,) * 6], [.2, .5, 1.])
    for elapsed in (0., .1, .2, .49, .5, .9, 1., 2.):
        assert step_index(times, elapsed) == sum(t <= elapsed for t in times)
    assert linear_sample(times, points, (0.,) * 6, .1) == (.5,) * 6
    assert linear_sample(times, points, (0.,) * 6, .5) == points[1]
    assert linear_sample(times, points, (0.,) * 6, 2.) == points[-1]


def test_simulation_rejects_out_of_limit_batch_without_partial_target_change():
    with RebotArmMujoco() as sim:
        before = sim.get_control_status()
        with pytest.raises(ValueError, match='outside'):
            sim.command_joint_positions({'joint1': .1, 'joint2': 99.})
        after = sim.get_control_status()
        assert after.joint_targets == before.joint_targets
        assert after.mode == before.mode
        assert after.applied_torques == before.applied_torques


def test_stationary_position_to_hold_preserves_support_torque_and_pose():
    start = (0., -1., -1., 0., .3, 0.)
    with RebotArmMujoco() as sim:
        sim.reset_joint_positions(start)
        sim.command_joint_positions(start)
        sim.step(1000)
        before = sim.get_control_status()
        assert max(abs(value) for value in before.applied_torques) > 1.
        sim.set_mode('hold')
        after = sim.get_control_status()
        np.testing.assert_allclose(after.applied_torques, before.applied_torques, atol=.01)
        for _ in range(500):
            state = sim.step()
            assert max(abs(a-b) for a,b in zip(state.joint_positions[:6], start)) <= 3*25/65535


def test_shared_direct_construction_copies_mutable_arrays():
    from rebotarm_trajectory.polynomial import PolynomialSegment, PolynomialTrajectory, TrajectoryPoint
    rows = [[0., .02] for _ in range(6)]
    segment = PolynomialSegment(0., .4, rows)
    points = [TrajectoryPoint(0., [0.]*6), TrajectoryPoint(.4, [.02]*6)]
    curve = PolynomialTrajectory(points, [segment], 1)
    rows[0][0] = 10.
    points.clear()
    assert curve.sample(.2)[0] == pytest.approx((.01,)*6)
    assert curve.times == (0., .4)
    assert isinstance(curve.points, tuple) and isinstance(curve.segments, tuple)


def test_motion_message_factory_builds_complete_quintic_without_dense_points():
    from types import SimpleNamespace
    from rebotarm_motion.trajectory_messages import point_to_point_message
    def trajectory(): return SimpleNamespace(joint_names=[], points=[])
    def point(): return SimpleNamespace(time_from_start=SimpleNamespace(sec=0, nanosec=0))
    message = point_to_point_message(JOINT_NAMES, (0.,)*6, (.02,)*6, .4,
        trajectory_factory=trajectory, point_factory=point)
    assert len(message.points) == 2
    assert message.points[0].positions == [0.]*6
    assert message.points[-1].positions == [.02]*6
    assert all(p.velocities == p.accelerations == [0.]*6 for p in message.points)
    assert message.points[-1].time_from_start.nanosec == 400_000_000


def test_operator_capability_rejects_unknown_or_mismatched_backend_without_fallback():
    from types import SimpleNamespace
    from rebotarm_teleop.trajectory_capability import TrajectoryCapability
    capability = TrajectoryCapability('prepared_v1')
    with pytest.raises(ValueError, match='mismatch'): capability.require()
    capability.update(SimpleNamespace(data='legacy'))
    with pytest.raises(ValueError, match='mismatch'): capability.require()
    capability.update(SimpleNamespace(data='prepared_v1'))
    capability.require()


def test_teach_does_not_drop_moveit_acceleration_to_match_lower_replay_grade():
    from types import SimpleNamespace
    from rebotarm_teach.teach_replay_trajectory_builder import TeachReplayTrajectoryBuilder
    from test_status_panel_layering import _Trajectory, _TrajectoryPoint, _PreparedReplay
    builder = TeachReplayTrajectoryBuilder(trajectory_factory=_Trajectory,
        trajectory_point_factory=_TrajectoryPoint)
    def align(trajectory, **kwargs):
        point = _TrajectoryPoint()
        point.positions = [.1,-.1]
        point.velocities = [0.,0.]
        point.accelerations = [.01,-.01]
        trajectory.points.append(point)
        return .25
    config = SimpleNamespace(use_moveit_start_align=True, yellow_max_speed=.6,
        initial_replay_delay_sec=.05)
    with pytest.raises(ValueError, match='different accelerations grades'):
        builder.build(prepared=_PreparedReplay(), current_positions={'joint1':0.,'joint2':0.},
            start_band='align',settings={'final_hold_sec':1.},config=config,
            moveit_start_alignment=align)


def test_reference_writes_do_not_advance_pi_without_a_physics_control_tick():
    with RebotArmMujoco() as sim:
        sim.command_joint_positions((0.,-1.,-1.,0.,.3,0.))
        before=sim._control.capture()
        for index in range(10):
            sim.command_joint_positions((.002*index,-1.,-1.,0.,.3,0.))
        after=sim._control.capture()
        for field in ('position_integral','velocity_integral','applied_torque','control_phase'):
            assert after[field]==before[field]
        sim.step(5)
        assert sim._control.capture()['position_integral']!=before['position_integral']

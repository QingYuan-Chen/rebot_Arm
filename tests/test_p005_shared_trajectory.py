"""共享轨迹迁移、入口兼容与 MuJoCo 控制边界回归。

文件职责：用独立解析值和旧采样语义核对共享内核，并复现越界目标、支撑力矩及重复 PI 推进问题。
对外接口：pytest 用例，直接构造数值曲线、消息替身或本地 MuJoCo 实例。
安全约束：不访问真实硬件；数值等价、物理回归与完整 ROS 操作链是不同验证层。
文件结构：纯依赖/系数往返 → 解析值和非法输入 → 旧采样 → 物理控制缺陷
    → 不可变对象、消息/能力/示教等级 → 高频目标写入不推进 PI。
"""

import ast
import copy
import math
import sys
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
                assert node.module.split('.')[0] in sys.stdlib_module_names, path
            elif isinstance(node, ast.Import):
                assert all(alias.name.split('.')[0] in sys.stdlib_module_names for alias in node.names), path


def test_named_coordinated_payload_mirror_validation_and_gripper_only_freeze():
    from rebotarm_trajectory.joint_contract import canonical_input
    from rebotarm_trajectory.polynomial import COORDINATED_JOINT_NAMES
    points = point_to_point((.01,-.01), (.02,-.02), 2.)
    names, result = canonical_input(('left_finger_joint','right_finger_joint'), points, (.1,)*6)
    assert names == COORDINATED_JOINT_NAMES
    assert result[-1].positions == (.1,)*6+(.02,)
    decoded = curve_from_payload(build_curve(names, result).to_payload())
    assert decoded.to_payload()['version'] == 2
    assert decoded.sample(1.)[0] == pytest.approx((.1,)*6+(.015,))
    with pytest.raises(ValueError, match='mirror'):
        canonical_input(('left_finger_joint','right_finger_joint'),
            point_to_point((.01,-.01), (.02,-.019), 2.), (.1,)*6)
    for field in ('velocities','accelerations'):
        from dataclasses import replace
        invalid = (replace(points[0],**{field:(.001,0.)}),points[1])
        with pytest.raises(ValueError,match='mirror'):
            canonical_input(('left_finger_joint','right_finger_joint'),invalid,(.1,)*6)


def test_merge_tracks_preserves_arm_derivatives_at_inserted_finger_nodes():
    from rebotarm_motion.coordinated_trajectory import merge_tracks
    arm = build_curve(JOINT_NAMES, point_to_point((0.,)*6,(.02,)*6,2.))
    finger = build_curve(('left_finger_joint',), (
        TrajectoryPoint(0.,(0.,),(0.,),(0.,)),
        TrajectoryPoint(.5,(0.,),(0.,),(0.,)),
        TrajectoryPoint(1.5,(.001,),(0.,),(0.,)),
        TrajectoryPoint(2.,(.001,),(0.,),(0.,))))
    merged = merge_tracks(arm, finger)
    assert merged.sample(.5)[1][0] > 0
    for stamp in np.linspace(0,2,31):
        for order in range(4):
            assert merged.sample(stamp)[order][:6] == pytest.approx(arm.sample(stamp)[order], abs=1e-10)
    assert merged.sample(.25)[0][-1] == 0.
    assert merged.sample(1.8)[1][-1] == pytest.approx(0., abs=1e-12)


def test_web_coordinated_plan_is_single_use():
    from rebotarm_teleop.web_coordinated import CoordinatedWebPlans
    from rebotarm_trajectory.capability import make_capability
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    cap = make_capability('prepared_v1',1,.5,1.)
    plans = CoordinatedWebPlans(capability=lambda:cap,retime=lambda message,*args:message,
        joint_names=JOINT_NAMES,joint_limits={n:(-3.,3.) for n in JOINT_NAMES},
        trajectory_factory=JointTrajectory,point_factory=JointTrajectoryPoint)
    current = dict.fromkeys(JOINT_NAMES,0.);current['gripper'] = .02
    payload = dict(joint_positions=dict.fromkeys(JOINT_NAMES,.02),duration=2.,gripper_position=.021,
        gripper_keyframes=[dict(time=.5,width=.02),dict(time=1.5,width=.021)])
    preview = plans.preview(payload,current,max_delta=.2,min_duration=.1,max_duration=10.,max_speed=1.)
    message, _ = plans.take(preview['plan_id'],current)
    assert len(message.joint_names) == 7
    assert [p.time for p in build_curve(message.joint_names,tuple(TrajectoryPoint(
        p.time_from_start.sec+p.time_from_start.nanosec*1e-9,p.positions,p.velocities,p.accelerations)
        for p in message.points)).points] == [0.,.5,1.5,2.]
    with pytest.raises(ValueError,match='missing'):
        plans.take(preview['plan_id'],current)
    preview = plans.preview(payload,current,max_delta=.2,min_duration=.1,max_duration=10.,max_speed=1.)
    message, _ = plans.take(preview['plan_id'],dict(current,joint1=.01))
    assert len(message.joint_names) == 7


@pytest.mark.parametrize('level_name,ready', [('OK',True),('WARN',False),('ERROR',False),('STALE',False)])
def test_web_admission_reads_ros_byte_level_without_constructing_node(level_name,ready):
    """使用真实 ROS 字节字段；诊断接收不构造节点、读设备或改变准入。"""
    from types import SimpleNamespace
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from rebotarm_dashboard.teleop_status_panel_node import TeleopStatusPanelNode
    received=[]
    owner=SimpleNamespace(_arm_namespace='rebotarm',_store=SimpleNamespace(
        update_teleop_status=lambda name,value:received.append((name,value))))
    status=DiagnosticStatus(name='rebotarm/trajectory_admission',
        level=getattr(DiagnosticStatus,level_name),message=level_name,
        values=[KeyValue(key='generation',value='4')])
    TeleopStatusPanelNode._on_admission_diagnostic(owner,DiagnosticArray(status=[status]))
    assert received[0][0]=='trajectory_admission'
    assert received[0][1]['ready'] is ready
    assert received[0][1]['values']=={'generation':'4'}


def test_coordinated_teach_uses_source_time_and_preserves_raw_samples():
    from dataclasses import replace
    from ament_index_python.packages import get_package_share_directory
    from rebotarm_trajectory.capability import make_capability, constraint_revision
    from rebotarm_teach.teach_models import TeachSample
    from rebotarm_teach.teach_preparation import prepare_teach_replay_samples
    config = Path(get_package_share_directory('rebotarm_moveit_config'))/'config'
    revision = constraint_revision(*(config.joinpath(n).read_bytes() for n in ('rebotarm.urdf','joint_limits.yaml')))
    cap = make_capability('prepared_v1',1,.5,1.,revision)
    samples = [TeachSample(i*.1,JOINT_NAMES,(i*.0002,0.,0.,0.,0.,0.),(),(),{},'GRAVITY_COMP',
        format_version=2,gripper_width_m=.02+i*.00001,gripper_source_stamp=i*.1,gripper_valid=True)
        for i in range(11)]
    original = copy.deepcopy(samples)
    result = prepare_teach_replay_samples(samples,coordinated_capability=cap,
        smoothing_enabled=False,filter_enabled=False,resample_enabled=False)
    assert samples == original
    assert all(len(p.positions) == 7 and len(p.velocities) == len(p.accelerations) == 7 for p in result.retimed_points)
    assert result.retimed_points[-1].positions[-1] == pytest.approx(samples[-1].gripper_width_m/2.)
    with pytest.raises(ValueError,match='valid source-timed'):
        prepare_teach_replay_samples([replace(s,gripper_valid=False) for s in samples],coordinated_capability=cap)


def test_coordinated_retiming_respects_backend_speed_and_quantized_extrema():
    from dataclasses import replace
    from rebotarm_motion.coordinated_trajectory import retime_curve
    from rebotarm_motion.trajectory_preparation import load_reference_limits, check_curve
    from rebotarm_trajectory.polynomial import COORDINATED_JOINT_NAMES
    config = Path(__file__).resolve().parents[1]/'src/rebotarm_moveit_config/config'
    low, high, limits = load_reference_limits(config, COORDINATED_JOINT_NAMES)
    limits['left_finger_joint'] = replace(limits['left_finger_joint'], max_velocity=.0045)
    points = point_to_point((0.,)*7,(.02,)*6+(.002,),.4)
    curve, scale = retime_curve(COORDINATED_JOINT_NAMES, points, low, high, limits)
    assert scale > 2
    assert curve.points[-1].positions == points[-1].positions
    assert check_curve(curve,low,high,limits)['state'] == 'pass'
    assert all(abs(p.time*1e9-round(p.time*1e9)) < 1e-5 for p in curve.points)
    with pytest.raises(ValueError, match='minimum duration'):
        retime_curve(COORDINATED_JOINT_NAMES,points,low,high,limits,fixed=True)


@pytest.mark.parametrize('names', [JOINT_NAMES, (*JOINT_NAMES, 'left_finger_joint')])
def test_fixed_stationary_plan_does_not_check_artificial_start_displacement(names):
    """恒定曲线不因额外首点扰动而被动态限值拒绝。"""
    from rebotarm_motion.coordinated_trajectory import retime_curve
    from rebotarm_motion.trajectory_preparation import load_reference_limits
    config = Path(__file__).resolve().parents[1]/'src/rebotarm_moveit_config/config'
    low, high, limits = load_reference_limits(config, names)
    points = point_to_point((0.,)*len(names), (0.,)*len(names), .05)
    curve, scale = retime_curve(names, points, low, high, limits, fixed=True)
    assert scale == 1. and curve.points == points


def test_capability_binding_changes_on_reconnect_and_rejects_tampering():
    from rebotarm_trajectory.capability import make_capability, decode_capability, binding
    first = make_capability('prepared_v1',1,.5,1.)
    assert decode_capability(first) == first
    assert binding(first) != binding(make_capability('prepared_v1',2,.5,1.))
    first['gripper_torque_cap_nm'] = 2.
    with pytest.raises(ValueError, match='revision'):
        decode_capability(first)


def test_five_captured_moveit_rejections_pass_only_after_planning_retime():
    import json
    from rebotarm_motion.coordinated_trajectory import retime_curve
    from rebotarm_motion.trajectory_preparation import check_curve, load_reference_limits
    root = Path(__file__).resolve().parents[1]
    fixtures = json.loads((root/'tests/fixtures/p005_moveit_trajectory.json').read_text())['captured_rejections']
    assert len(fixtures) == 5
    low, high, limits = load_reference_limits(root/'src/rebotarm_moveit_config/config')
    for fixture in fixtures:
        points = tuple(TrajectoryPoint(p['time_from_start'],p['positions'],p['velocities'],p['accelerations'])
            for p in fixture['points'])
        original = build_curve(fixture['joint_names'],points)
        report = check_curve(original,low,high,limits)
        assert report['state'] == 'rejected' and any(i['kind'] == 'jerk' for i in report['issues'])
        assert report['peaks']['joint3']['jerk']['absolute'] == pytest.approx(fixture['joint3_jerk'])
        final, scale = retime_curve(fixture['joint_names'],points,low,high,limits)
        assert scale > 1. and final.positions == original.positions
        assert check_curve(final,low,high,limits)['state'] == 'pass'


def test_planning_rejects_geometry_capability_and_nanosecond_damage():
    from rebotarm_motion.coordinated_trajectory import retime_curve, effective_limits
    from rebotarm_motion.trajectory_preparation import load_reference_limits
    from rebotarm_trajectory.capability import make_capability
    config = Path(__file__).resolve().parents[1]/'src/rebotarm_moveit_config/config'
    with pytest.raises(ValueError,match='revision mismatch'):
        effective_limits(config,JOINT_NAMES,make_capability('prepared_v1',1,.5,1.))
    low,high,limits = load_reference_limits(config)
    with pytest.raises(ValueError,match='position extremum'):
        retime_curve(JOINT_NAMES,point_to_point((0.,)*6,(99.,)*6),low,high,limits)
    # 两个合法浮点时刻在 ROS 纳秒编码后合为一个节点，必须拒绝而非悄悄丢点。
    with pytest.raises(ValueError,match='strictly increase'):
        retime_curve(JOINT_NAMES,tuple(TrajectoryPoint(t,(0.,)*6,(0.,)*6,(0.,)*6)
            for t in (0.,1.,1.+1e-12,2.)),low,high,limits)


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
            # 第一轴合法、第二轴越界：同时断言没有部分写入前一轴，而不只检查异常类型。
            sim.command_joint_positions({'joint1': .1, 'joint2': 99.})
        after = sim.get_control_status()
        assert after.joint_targets == before.joint_targets
        assert after.mode == before.mode
        assert after.applied_torques == before.applied_torques


def test_stationary_position_to_hold_preserves_support_torque():
    start = (0., -1., -1., 0., .3, 0.)
    with RebotArmMujoco() as sim:
        sim.reset_joint_positions(start)
        sim.command_joint_positions(start)
        sim.step(1000)
        before = sim.get_control_status()
        # 先证明当前确有支撑负载，零力矩姿态无法暴露切换时清零控制器状态的缺陷。
        assert max(abs(value) for value in before.applied_torques) > 1.
        sim.set_mode('hold')
        after = sim.get_control_status()
        np.testing.assert_allclose(after.applied_torques, before.applied_torques, atol=.01)
        for _ in range(500):
            state = sim.step()
            assert np.isfinite(state.joint_positions).all()


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

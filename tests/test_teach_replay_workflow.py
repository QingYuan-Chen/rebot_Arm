import ast
import json
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace

from rebotarm_teach.teach_replay_workflow import TeachReplayWorkflow


ROOT = Path(__file__).resolve().parents[1]


def test_partial_joint_record_remains_readable_but_retiming_requires_six_axes():
    import pytest
    from rebotarm_teach.teach_models import TeachSample
    from rebotarm_teach.teach_preparation import prepare_teach_replay_samples, retime_teach_samples
    samples = [TeachSample(i*.1, ('joint1',), (i*.001,), (), (), {}, 'record') for i in range(3)]
    analyzed = prepare_teach_replay_samples(samples, retime_enabled=False)
    assert analyzed.samples[0].joint_names == ('joint1',)
    assert analyzed.samples[-1].positions == samples[-1].positions
    with pytest.raises(ValueError, match='joint1..joint6'):
        prepare_teach_replay_samples(samples, retime_enabled=True)
    with pytest.raises(ValueError, match='joint1..joint6'):
        retime_teach_samples(samples, replay_speed=1., max_velocity_rad_s=1.)


def workflow_fixture(tmp_path):
    # Exercise the actual dashboard parameter contract without a ROS or HTTP server.
    source = ROOT / "src/rebotarm_dashboard/rebotarm_dashboard/teleop_status_panel_node.py"
    parameters = {}
    for call in ast.walk(ast.parse(source.read_text())):
        if isinstance(call, ast.Call) and getattr(call.func, "attr", "") == "declare_parameter":
            try:
                parameters[ast.literal_eval(call.args[0])] = ast.literal_eval(call.args[1])
            except ValueError:
                pass
    parameters["record_path"] = str(tmp_path / "missing.jsonl")
    events = {}
    calls = []

    def call_async(request):
        calls.append(request)
        future = Future()
        future.set_result(SimpleNamespace(success=True, message="stopped"))
        return future

    client = SimpleNamespace(
        wait_for_service=lambda **kwargs: True, service_is_ready=lambda: True,
        call_async=call_async,
    )
    node = SimpleNamespace(
        get_parameter=lambda name: SimpleNamespace(value=parameters[name]),
        create_client=lambda *args: client,
    )
    workflow = TeachReplayWorkflow(
        node=node, joint_names=tuple(parameters["joint_names"]),
        snapshot=lambda: SimpleNamespace(joints={}, teleop=events),
        publish_status=lambda channel, value: events.update({channel: value}),
        collision_defaults=lambda names: (), action_client=object(),
        trajectory_stop_client=client, request_stop=lambda **kwargs: True,
        trajectory_capability=lambda: {"mode": "continuous", "revision": "test"},
        retime_trajectory=lambda trajectory, *_args: trajectory,
    )
    return workflow, events, calls


def test_workflow_owns_dry_run_failure_without_action_or_dashboard(tmp_path):
    workflow, events, calls = workflow_fixture(tmp_path)
    result = workflow.dry_run({})
    assert not result["accepted"]
    assert workflow._last_teach_dry_run is None
    assert events["replay"] == result
    assert calls == []


def test_workflow_without_valid_dry_run_never_sends_goal(tmp_path):
    workflow, events, calls = workflow_fixture(tmp_path)
    result = workflow.execute({})
    assert not result["accepted"]
    assert events["replay"] == result
    assert calls == []


def test_missing_action_result_is_reported_as_failure(tmp_path):
    workflow, events, _ = workflow_fixture(tmp_path)
    workflow._teach_replay_goal_handle = object()
    future = Future()
    future.set_result(SimpleNamespace(status=4, result=None))
    workflow._on_teach_replay_result(future, {}, 0)
    assert events["replay"]["state"] == "failed"
    assert "error_code" in events["replay"]["message"]
    assert workflow._teach_replay_goal_handle is None


def test_invalid_dry_run_settings_invalidate_previous_token(tmp_path):
    import pytest
    workflow, _, _ = workflow_fixture(tmp_path)
    workflow._last_teach_dry_run = {"accepted": True}
    with pytest.raises(ValueError, match="settings"):
        workflow.dry_run({"settings": None})
    assert workflow._last_teach_dry_run is None


def test_workflow_stop_uses_controller_fallback_without_active_goal(tmp_path):
    workflow, events, calls = workflow_fixture(tmp_path)
    result = workflow.stop()
    assert result["accepted"]
    assert result["state"] == "stop_requested"
    assert len(calls) == 1
    assert events["replay"] == result


def test_missing_tracking_feedback_requests_stop_before_cancel_once(tmp_path, monkeypatch):
    import rebotarm_teach.teach_replay_workflow as module
    workflow, events, calls = workflow_fixture(tmp_path)
    order = []
    workflow._request_controller_trajectory_stop = lambda **kwargs: order.append('stop')
    workflow._teach_replay_goal_handle = SimpleNamespace(cancel_goal_async=lambda: order.append('cancel'))
    workflow._active_teach_replay_trajectory = dict(joint_names=['joint1'],
        points=[dict(time_from_start=0.,positions=[0.])])
    workflow._active_teach_replay_started_at = 0.
    workflow._snapshot = lambda: SimpleNamespace(joints={'joint1': {'position': 0.}},teleop=events)
    now = [10.]
    monkeypatch.setattr(module.time,'monotonic',lambda: now[0])
    workflow.check_tracking()
    assert order == []
    now[0] = 100.
    workflow.check_tracking()
    assert order == ['stop','cancel']
    assert events['replay']['state'] == 'safety_stop'
    assert events['replay']['runtime_monitor']['reason'] == 'missing_joint_state'
    workflow.check_tracking()
    assert order == ['stop','cancel']


def test_workflow_prepares_dry_run_then_executes_through_injected_action(tmp_path):
    workflow, events, calls = workflow_fixture(tmp_path)
    names = tuple(f"joint{i}" for i in range(1, 7))
    path = tmp_path / "record.jsonl"
    path.write_text("\n".join(json.dumps({
        "stamp": i / 50., "joint_names": names,
        "positions": [i * .0001] * 6, "velocities": [0.] * 6, "efforts": [0.] * 6,
        "motor_status": dict.fromkeys(names, 1), "arm_state": "GRAVITY_COMP",
    }) for i in range(101)) + "\n")
    workflow._snapshot = lambda: SimpleNamespace(
        joints={name: {"position": 0.} for name in names}, teleop=events,
    )
    checked = []
    workflow._collision_prechecker = SimpleNamespace(
        check_positions=lambda **kwargs: checked.append(kwargs) or {"state": "pass", "message": "valid"},
    )
    pending = Future()
    goals = []
    workflow._action_client = SimpleNamespace(
        wait_for_server=lambda **kwargs: True,
        send_goal_async=lambda goal: goals.append(goal) or pending,
    )
    payload = {"record_path": str(path)}
    dry_run = workflow.dry_run(payload)
    assert dry_run["accepted"], dry_run
    assert goals == []
    result = workflow.execute(payload)
    assert result["accepted"], result
    assert len(goals) == 1
    assert goals[0].trajectory.points
    assert len(checked) == 2
    assert calls == []


def test_dashboard_no_longer_owns_replay_algorithms_or_planner():
    source = (ROOT / "src/rebotarm_dashboard/rebotarm_dashboard/teleop_status_panel_node.py").read_text()
    for forbidden in ("MoveItMotionPlanner", "CollisionPrechecker", "ReplayRuntimeMonitor", "prepare_teach_replay_samples", "._client"):
        assert forbidden not in source
    assert "self._teach_workflow.execute(payload)" in source
    workflow_source = (ROOT / "src/rebotarm_teach/rebotarm_teach/teach_replay_workflow.py").read_text()
    assert "rebotarm_dashboard" not in workflow_source
    assert "_route_web_command" not in workflow_source


def _qv_record(workflow, tmp_path):
    """模拟现场 309 帧 q/v,无 a;保留原记录,使用有非零曲率的六轴路径."""
    import math
    from rebotarm_teach.teach_models import TeachSample
    from rebotarm_teach.teach_record_io import encode_teach_sample
    names = tuple(f'joint{i}' for i in range(1,7))
    base = (0.,-.8,-1.,0.,.3,0.)
    samples = [TeachSample(i*7.4/308,names,
        tuple(q+(.08*math.sin(math.pi*i/308)**2 if j == 1 else 0.) for j,q in enumerate(base)),
        (0.,)*6,(),dict.fromkeys(names,1),'GRAVITY_COMP') for i in range(309)]
    path = tmp_path/'qv_record.jsonl'
    path.write_text('\n'.join(encode_teach_sample(s) for s in samples)+'\n')
    workflow._snapshot = lambda: SimpleNamespace(joints={n:{'position':q} for n,q in zip(names,base)},teleop={})
    return samples,path


def test_qv_teach_builds_c1_moveit_splice_checks_actual_curve_and_preserves_raw(tmp_path):
    import pytest
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from rebotarm_motion.teach_replay_start_alignment import MoveItStartAligner
    from rebotarm_motion.polynomial_trajectory import build_curve, TrajectoryPoint
    from rebotarm_motion.trajectory_preparation import check_curve, load_reference_limits
    from rebotarm_teach.teach_record_io import write_prepared_teach_record, load_teach_samples
    workflow,_,_=workflow_fixture(tmp_path)
    samples,path = _qv_record(workflow,tmp_path)
    original=path.read_bytes()
    def plan(**kwargs):
        first=list(kwargs['target_positions']);current=first.copy();current[0]-=.01
        points=[]
        for i,q in enumerate((current,[(a+b)/2 for a,b in zip(current,first)],first)):
            p=JointTrajectoryPoint(positions=q,velocities=[.01 if i==1 and j==0 else 0. for j in range(6)],
                accelerations=[.003 if i==1 and j==0 else 0. for j in range(6)])
            p.time_from_start.sec=i
            points.append(p)
        return SimpleNamespace(success=True,trajectory=JointTrajectory(joint_names=list(samples[0].joint_names),points=points))
    base=samples[0].positions
    workflow._snapshot=lambda:SimpleNamespace(joints={n:{'position':q-(.01 if i==0 else 0.)}
        for i,(n,q) in enumerate(zip(samples[0].joint_names,base))},teleop={})
    workflow._moveit_start_aligner=MoveItStartAligner(planner=SimpleNamespace(plan_joint_positions=plan),
        trajectory_point_factory=JointTrajectoryPoint)
    settings=workflow._teach_replay_settings_from_payload({})
    trajectory=workflow._build_teach_replay_trajectory(samples,'align',settings)
    curve=build_curve(trajectory.joint_names,tuple(TrajectoryPoint(
        p.time_from_start.sec+p.time_from_start.nanosec*1e-9,p.positions,p.velocities,p.accelerations)
        for p in trajectory.points))
    assert curve.degree == 3
    assert curve.continuity()['guaranteed_order'] == 1
    assert curve.continuity()['max_boundary_gaps']['velocity'] < 1e-9
    assert all(not p.accelerations for p in curve.points)
    assert any(any(abs(v)>0 for v in p.velocities) for p in curve.points[1:-1])
    report = workflow._last_teach_prepared_payload
    assert report['input_grade'] == report['reference_grade'] == 'q/v'
    assert report['velocity_source'] == 'derived_filtered_q_t'
    assert report['alignment_conversion']['input_grade'] == 'q/v/a'
    assert report['alignment_conversion']['reference_grade'] == 'q/v'
    assert report['alignment_conversion']['original_points'][1]['accelerations'][0] == .003
    alignment_scale = curve.points[2].time-curve.points[1].time
    assert curve.points[2].velocities[0] == pytest.approx(.01/alignment_scale)
    low,high,limits=load_reference_limits(ROOT/'src/rebotarm_moveit_config/config')
    assert check_curve(curve,low,high,limits)['state']=='pass'
    checked=[]
    workflow._collision_prechecker=SimpleNamespace(check_positions=lambda **kwargs:
        checked.append(kwargs) or {'state':'pass','checked_samples':len(kwargs['positions_list'])})
    assert workflow._collision_precheck_trajectory(trajectory)['state']=='pass'
    expected=[curve.sample(t)[0] for t in curve.collision_times(.02)]
    assert checked[0]['positions_list']==expected
    prepared=workflow._prepare_teach_replay_samples(samples,settings)
    exported=load_teach_samples(write_prepared_teach_record(path,prepared))
    assert all(s.format_version==5 and not s.accelerations for s in exported)
    assert exported[100].velocities==pytest.approx(prepared.retimed_points[100].velocities)
    assert exported[100].input_grade == 'q/v'
    assert exported[100].velocity_source == 'derived_filtered_q_t'
    first_export = write_prepared_teach_record(path,prepared)
    saved = first_export.read_bytes()
    assert write_prepared_teach_record(path,prepared) != first_export
    assert first_export.read_bytes() == saved
    assert path.read_bytes()==original


def test_splice_failure_is_not_reported_as_a_collision_failure(tmp_path):
    workflow,_,_=workflow_fixture(tmp_path)
    _,path=_qv_record(workflow,tmp_path)
    workflow._moveit_align_summary=lambda *args,**kwargs:{'state':'planned'}
    def fail(*args):
        raise ValueError('invalid splice')
    workflow._build_teach_replay_trajectory=fail
    workflow._collision_precheck_trajectory=lambda trajectory: (_ for _ in ()).throw(AssertionError('must not check collision'))
    result=workflow.dry_run({'record_path':str(path)})
    assert not result['accepted']
    assert result['trajectory_preparation']['state']=='failed'
    assert 'invalid splice' in result['trajectory_preparation']['message']
    assert result['collision_precheck']['state']=='not_checked'
    assert workflow._last_teach_dry_run is None


def test_teach_reference_preserves_input_grade_and_never_completes_missing_acceleration():
    import pytest
    from rebotarm_motion.trajectory_time_parameterization import prepare_teach_reference
    from rebotarm_motion.polynomial_trajectory import TrajectoryPoint, build_curve
    names=tuple(f'joint{i}' for i in range(1,7))
    points=tuple(TrajectoryPoint(t,(t*t*.01,)*6,(t*.02,)*6) for t in (0.,.4,1.,2.))
    result=prepare_teach_reference(points)
    assert result[1].velocities==points[1].velocities
    assert all(not p.accelerations for p in result)
    assert build_curve(names,result).degree == 3
    derived = prepare_teach_reference(points,derive_velocity=True)
    assert derived[1].velocities == pytest.approx((.008,)*6)
    assert all(not p.accelerations for p in derived)
    constrained=tuple(TrajectoryPoint(p.time,p.positions,p.velocities,(.003,)*6) for p in points)
    assert prepare_teach_reference(constrained)[1].accelerations==constrained[1].accelerations
    assert build_curve(names,prepare_teach_reference(constrained)).degree == 5
    q_only = tuple(TrajectoryPoint(p.time,p.positions) for p in points)
    assert build_curve(names,prepare_teach_reference(q_only,derive_velocity=True)).degree == 1
    with pytest.raises(ValueError,match='mixed'):
        prepare_teach_reference((points[0],TrajectoryPoint(.4,points[1].positions)))
    with pytest.raises(ValueError,match='strictly increase'):
        prepare_teach_reference((points[0],points[0]))


def test_final_teach_curve_rejects_interior_position_violation():
    import pytest
    from rebotarm_motion.trajectory_time_parameterization import finalize_teach_curve
    from rebotarm_motion.polynomial_trajectory import TrajectoryPoint
    names=tuple(f'joint{i}' for i in range(1,7))
    # 两端位置合法,中间速度使曲线越界;拉长时间不能修复几何.
    points=(TrajectoryPoint(0.,(0.,)*6,(0.,)*6,(0.,)*6),
        TrajectoryPoint(1.,(0.,)*6,(100.,)*6,(0.,)*6),
        TrajectoryPoint(2.,(0.,)*6,(0.,)*6,(0.,)*6))
    with pytest.raises(ValueError,match='position extremum'):
        finalize_teach_curve(names,points,ROOT/'src/rebotarm_moveit_config/config',(1.,)*6,.5,20.)


def test_collision_coverage_budget_failure_is_not_checked_and_never_queried(tmp_path):
    workflow,_,calls=workflow_fixture(tmp_path)
    samples,_=_qv_record(workflow,tmp_path)
    parameters=workflow.get_parameter
    workflow.get_parameter=lambda n:SimpleNamespace(value=1) if n=='collision_check_max_samples' else parameters(n)
    result=workflow._collision_precheck(samples)
    assert result['state']=='not_checked' and result['checked_samples']==0
    assert calls==[]


def test_auto_alignment_derived_duration_is_not_a_user_token_setting():
    from rebotarm_teach.teach_replay_coordinator import TeachReplayCoordinator,TeachReplayLimits
    coordinator=TeachReplayCoordinator()
    settings=dict(replay_speed=1.,align_duration=5.,align_steps=30,final_hold_sec=1.)
    token=dict(accepted=True,record_path='record',prepared_risk_level='green',settings=settings)
    def decision(current,auto):
        return coordinator.evaluate_execute_request(info_payload=dict(path='record',start_band='direct',
            quality=dict(risk_level='green')),settings=current,
            prepared_quality=dict(risk_level='green',max_jump_rad=0.,max_acceleration_rad_s2=0.,max_jerk_rad_s3=0.),
            dry_run_token=token,limits=TeachReplayLimits(.02,5.,20.),yellow_max_speed=.5,
            align_duration_auto=auto)
    assert decision(dict(settings,align_duration=4.),True).accepted
    assert not decision(dict(settings,align_duration=4.),False).accepted
    assert not decision(dict(settings,replay_speed=.5),True).accepted


def test_preprocessing_preserves_grades_and_source_derivatives_when_position_processing_off():
    import pytest
    from dataclasses import replace
    from rebotarm_teach.teach_models import TeachSample
    from rebotarm_teach.teach_preparation import prepare_teach_replay_samples
    from rebotarm_motion.polynomial_trajectory import build_curve, TrajectoryPoint
    names=tuple(f'joint{i}' for i in range(1,7))
    base=[TeachSample(t,names,(t*.01,)*6,(.02,)*6,(),{},'record') for t in (0.,.2,1.)]
    for grade,degree in (('q',1),('q/v',3),('q/v/a',5)):
        samples=[replace(s,velocities=() if grade=='q' else s.velocities,
            accelerations=(.03,)*6 if grade=='q/v/a' else ()) for s in base]
        result=prepare_teach_replay_samples(samples,smoothing_enabled=False,filter_enabled=False,
            resample_enabled=True,retime_enabled=True,replay_speed=.5)
        points=tuple(TrajectoryPoint(p.time_from_start,p.positions,p.velocities,p.accelerations)
                     for p in result.retimed_points)
        assert build_curve(names,points).degree==degree
        assert result.input_grade==result.reference_grade==grade
        assert [p.positions for p in points]==[s.positions for s in samples]
        scale=points[-1].time/(base[-1].stamp-base[0].stamp)
        assert points[1].time==pytest.approx(base[1].stamp*scale)
        if grade!='q':
            assert result.velocity_source=='input'
            assert points[1].velocities==pytest.approx(tuple(v/scale for v in base[1].velocities))
        assert all(bool(p.accelerations)==(grade=='q/v/a') for p in points)


def test_invalid_input_is_rejected_before_filtering_or_resampling():
    import pytest
    from dataclasses import replace
    from rebotarm_teach.teach_models import TeachSample
    from rebotarm_teach.teach_preparation import prepare_teach_replay_samples
    names=tuple(f'joint{i}' for i in range(1,7))
    base=[TeachSample(float(i),names,(.01*i,)*6,(0.,)*6,(),{},'record') for i in range(3)]
    for bad in (replace(base[1],velocities=()),replace(base[1],positions=(0.,)),
                replace(base[1],velocities=(float('nan'),)*6),replace(base[1],stamp=0.),
                replace(base[1],accelerations=(0.,)*6),replace(base[1],stamp=float('inf'))):
        with pytest.raises(ValueError):
            prepare_teach_replay_samples([base[0],bad,base[2]],retime_enabled=True)


def test_c1_quality_uses_segment_jerk_and_reports_node_acceleration_jumps():
    from dataclasses import replace
    from rebotarm_teach.teach_models import TeachSample
    from rebotarm_teach.teach_preparation import prepare_teach_replay_samples
    from rebotarm_teach.teach_quality import analyze_teach_trajectory
    from rebotarm_motion.polynomial_trajectory import build_curve,TrajectoryPoint
    names=tuple(f'joint{i}' for i in range(1,7))
    samples=[TeachSample(float(i),names,(.01*i,)*6,((.1 if i==1 else 0.),)*6,(),{},'record') for i in range(3)]
    result=prepare_teach_replay_samples(samples,smoothing_enabled=False,filter_enabled=False,
        retime_enabled=True,resample_rate_hz=150.,max_jerk_rad_s3=20.)
    curve=build_curve(names,tuple(TrajectoryPoint(p.time_from_start,p.positions,p.velocities)
                                for p in result.retimed_points))
    sampled=[replace(samples[0],stamp=curve.duration*i/600,positions=curve.sample(curve.duration*i/600)[0])
             for i in range(601)]
    assert analyze_teach_trajectory(sampled,max_jerk_rad_s3=20.).max_jerk_rad_s3>20.
    assert result.after_quality.max_jerk_rad_s3<20.
    assert result.after_quality.risk_level=='green'
    assert not result.curve_analysis['derivative_limits_cover_node_jumps']
    assert any(abs(x)>.1 for node in result.curve_analysis['acceleration_jumps'] for x in node['delta'])


def test_actual_collision_is_distinct_from_preparation_failure(tmp_path):
    workflow,_,_=workflow_fixture(tmp_path)
    _,path=_qv_record(workflow,tmp_path)
    workflow._collision_precheck_trajectory=lambda trajectory:dict(state='collision',message='joint path collision')
    result=workflow.dry_run({'record_path':str(path)})
    assert not result['accepted']
    assert result['trajectory_preparation']['state']=='pass'
    assert result['collision_precheck']['state']=='collision'


def test_legacy_v4_acceleration_is_read_as_derived_and_is_not_rewritten(tmp_path):
    from rebotarm_teach.teach_record_io import load_teach_samples
    record=tmp_path/'old.prepared.v4.jsonl'
    data=dict(stamp=0.,joint_names=[f'joint{i}' for i in range(1,7)],positions=[0.]*6,
              velocities=[0.]*6,accelerations=[0.]*6,format_version=4)
    record.write_text(json.dumps(data)+'\n')
    original=record.read_bytes()
    assert load_teach_samples(record)[0].acceleration_source=='derived_legacy_v4'
    assert record.read_bytes()==original


def test_alignment_validates_original_derivative_grade_before_c1_conversion():
    import pytest
    from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
    from rebotarm_motion.teach_replay_start_alignment import MoveItStartAligner,MoveItStartAlignmentConfig
    names=[f'joint{i}' for i in range(1,7)]
    original=JointTrajectory(joint_names=names,points=[
        JointTrajectoryPoint(positions=[.01*i]*6,velocities=[0.]*6,
                             accelerations=[0.]*6 if i!=1 else []) for i in range(3)])
    for i,p in enumerate(original.points):
        p.time_from_start.sec=i
    aligner=MoveItStartAligner(planner=SimpleNamespace(plan_joint_positions=lambda **kwargs:
        SimpleNamespace(success=True,trajectory=original)),trajectory_point_factory=JointTrajectoryPoint)
    with pytest.raises(ValueError,match='mixed missing accelerations'):
        aligner.append(JointTrajectory(joint_names=names),current_positions=(0.,)*6,
            first_positions=(.02,)*6,config=MoveItStartAlignmentConfig(.8,.3,.005,.005,.1,.1))
    assert list(original.points[0].accelerations)==[0.]*6 and not original.points[1].accelerations


def test_prepared_preview_samples_actual_cubic_curve_instead_of_node_polyline():
    import pytest
    from rebotarm_teach.teach_models import TeachSample
    from rebotarm_teach.teach_preparation import teach_trajectory_preview_to_dict
    names=tuple(f'joint{i}' for i in range(1,7))
    samples=[TeachSample(0.,names,(0.,)*6,(0.,)*6,(),{},'PREPARED_REPLAY'),
             TeachSample(1.,names,(.01,)*6,(0.,)*6,(),{},'PREPARED_REPLAY')]
    preview=teach_trajectory_preview_to_dict(samples,max_points=5)
    assert preview['raw_samples']==2
    assert preview['points'][1]['t']==pytest.approx(.25)
    assert preview['points'][1]['positions']['joint1']==pytest.approx(.0015625)


def test_single_stationary_qv_record_remains_c1_without_duplicate_quality_times():
    from rebotarm_teach.teach_models import TeachSample
    from rebotarm_teach.teach_preparation import prepare_teach_replay_samples
    names=tuple(f'joint{i}' for i in range(1,7))
    prepared=prepare_teach_replay_samples([TeachSample(0.,names,(0.,)*6,(0.,)*6,(),{},'record')],
        retime_enabled=True)
    assert prepared.after_quality.risk_level=='green'
    assert len(prepared.retimed_points)==1 and not prepared.retimed_points[0].accelerations

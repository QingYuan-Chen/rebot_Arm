"""continuous 正式控制器所有者与设备周期的假设备回归。

文件职责：验证资源预约、原子接纳、周期采样、成功保持、失败不确认以及取消/重连失效。
对外接口：pytest 用例；enabled_manager/reserve/shutdown 复用正式装配和假驱动夹具。
安全约束：connect/enable 只作用于内存假设备，不打开真实串口；不把任务成功当作实机物理证据。
    用例中的少量 Action 替身只隔离协议竞态，周期与发送用例仍运行正式所有者。
文件结构：就绪与交接夹具 → 正式周期/上下文/夹爪测试 → 发送失败与延迟
    → Goal 身份及迟到响应 → legacy 接替、场景变化和取消回执顺序。
历史参数 dimensions=7 现在表示六轴加独立夹爪组合，不表示七轴 FJT。
"""
from dataclasses import replace
import time
import threading
from types import SimpleNamespace

import numpy as np
import pytest

from test_controller_local_runtime import make_local_manager, wait_until
from rebotarm_motion.polynomial_trajectory import JOINT_NAMES, build_curve
from rebotarm_motion.trajectory_preparation import point_to_point


def enabled_manager(monkeypatch):
    """等待六轴和夹爪各自的连续静止证据，避免用单帧或强改 ready 标志绕过准入。"""
    manager, buses = make_local_manager(monkeypatch)
    manager.lifecycle.connect()
    manager.lifecycle.enable()
    manager.arm.update_trajectory_scene('scene-a')
    wait_until(lambda: manager.arm._hold_evidence.confirmed(), timeout=2.)
    wait_until(lambda: manager.gripper._stationary.confirmed(), timeout=2.)
    return manager, buses[0]


def reserve(manager, duration=.4):
    """按真实预约身份构造同值准备载荷；这里只替代远端准备服务，不跳过后端接纳检查。"""
    token = manager.arm.reserve_arm_preparation(time.monotonic()+2.)
    target = list(token.hold)
    target[0] += .02
    curve = build_curve(JOINT_NAMES, point_to_point(token.hold, target, duration))
    payload = dict(curve=curve.to_payload(), context=dict(session=token.session,
        generation=token.generation, hold=token.hold), gripper_positions=token.gripper_positions,
        gripper_generation=token.gripper_generation, scene_revision=token.scene_revision,
        capability_revision=token.capability)
    return token, curve, manager.arm.decode_prepared_arm_trajectory(payload)


def shutdown(manager):
    if manager.lifecycle.enabled:
        manager.lifecycle.disable()
    manager.lifecycle.shutdown()


@pytest.mark.parametrize('change',['none','stop','motion','disable','reconnect'])
def test_readonly_planning_snapshot_and_stale_plan_admission(monkeypatch,change):
    from rebotarm_trajectory.planning import plan_binding,decode_plan
    manager,bus=enabled_manager(monkeypatch)
    try:
        state=manager.arm.planning_snapshot('continuous')
        assert manager.runtime.execution.reservation is None
        assert state['hold']==list(manager.runtime.execution.last_sent_target)
        points=point_to_point(state['hold'],(state['hold'][0]+.01,*state['hold'][1:]),1.)
        plan=decode_plan(plan_binding(state['capability_revision'],JOINT_NAMES,points,state=state),JOINT_NAMES,points)
        if change=='stop':manager.arm.stop_active_motion()
        elif change=='motion':
            token,curve,handoff=reserve(manager)
            task=manager.arm.admit_prepared_arm_trajectory(token,handoff)
            assert task.event.wait(2.)
        elif change in ('disable','reconnect'):
            manager.lifecycle.disable()
            preview=manager.arm.planning_snapshot('continuous')
            assert not preview['executable'] and preview['reason']=='disabled'
            if change=='reconnect':
                manager.lifecycle.shutdown();manager.lifecycle.connect()
            manager.lifecycle.enable()
        wait_until(lambda:manager.arm.admission_snapshot()['ready'],timeout=2.)
        if change=='none':
            token=manager.arm.reserve_arm_preparation(time.monotonic()+2.,planning=plan)
            manager.arm.cancel_arm_preparation(token)
        else:
            with pytest.raises(ValueError,match='re-plan'):
                manager.arm.reserve_arm_preparation(time.monotonic()+2.,planning=plan)
            assert manager.runtime.execution.reservation is None
    finally:shutdown(manager)


def test_prepared_curve_uses_actual_writer_cycle_and_final_command_hold(monkeypatch):
    manager, bus = enabled_manager(monkeypatch)
    try:
        token, curve, handoff = reserve(manager)
        before = len(bus.events)
        task = manager.arm.admit_prepared_arm_trajectory(token, handoff)
        assert task.event.wait(2.)
        assert task.result.state == 'succeeded'
        sent = [args[0] for joint, method, args in bus.events[before:] if joint == 1 and method == 'position']
        assert len(sent) >= 20 and len(set(sent)) > 10
        assert min(sent) >= token.hold[0]-1e-10
        assert max(sent) <= curve.points[-1].positions[0]+1e-10
        assert manager.runtime.execution.last_sent_target == curve.points[-1].positions
        assert manager.runtime.execution.target() == pytest.approx(curve.points[-1].positions)
        assert not manager.runtime.arm_resource_reserved
        assert manager.lifecycle.enabled
    finally:
        shutdown(manager)


@pytest.mark.parametrize('residual', [.003, .01])
def test_stationary_hold_residual_allows_two_checked_trajectories_without_reanchoring(monkeypatch,residual):
    """假电机保留固定跟踪残差，正式静止、准备及接纳不能借旧目标距离间接拒绝。"""
    from rebotarm_motion.trajectory_preparation import PreparationContext, analyze_trajectory, load_reference_limits
    from pathlib import Path
    manager,opened=make_local_manager(monkeypatch)
    manager.lifecycle.connect()
    motor=opened[0].motors[0]
    send=motor.send_pos_vel
    def offset_position(position,velocity):
        send(position,velocity)
        motor.state.pos=position+residual
    monkeypatch.setattr(motor,'send_pos_vel',offset_position)
    manager.lifecycle.enable()
    manager.arm.update_trajectory_scene('scene-a')
    low,high,limits=load_reference_limits(Path(__file__).resolve().parents[1]/'src/rebotarm_moveit_config/config')
    try:
        for _ in range(2):
            wait_until(lambda:manager.arm.admission_snapshot()['ready'],timeout=2.)
            hold=tuple(manager.runtime.execution.last_sent_target)
            token=manager.arm.reserve_arm_preparation(time.monotonic()+2.)
            assert token.feedback[0]-hold[0] == pytest.approx(residual)
            target=list(token.feedback);target[0]+=.02
            prepared,report=analyze_trajectory(JOINT_NAMES,point_to_point(token.feedback,target,1.),
                lower=low,upper=high,limits=limits,
                context=PreparationContext(token.session,token.generation,token.hold,token.feedback,
                    token.observed_at,True,True),now=time.monotonic(),
                collision_check=lambda _,positions:dict(state='pass',checked_samples=len(positions),requested_samples=len(positions)))
            assert report['state']=='pass'
            assert prepared.curve.points[0].positions==hold
            assert manager.runtime.execution.last_sent_target==hold
            payload=prepared.to_payload()
            payload.update(gripper_positions=token.gripper_positions,gripper_generation=token.gripper_generation,
                scene_revision=token.scene_revision,capability_revision=token.capability)
            task=manager.arm.admit_prepared_arm_trajectory(token,manager.arm.decode_prepared_arm_trajectory(payload))
            assert task.event.wait(3.)
            assert task.result.state=='succeeded'
            assert manager.runtime.execution.last_sent_target==tuple(target)
    finally:
        shutdown(manager)


@pytest.mark.parametrize('change', ['scene', 'gripper', 'hold', 'capability', 'missing_capability', 'deadline', 'disable', 'reconnect'])
def test_stale_handoff_never_starts_after_context_change(monkeypatch, change):
    manager, bus = enabled_manager(monkeypatch)
    try:
        token, curve, handoff = reserve(manager)
        if change == 'scene':
            manager.arm.update_trajectory_scene('scene-b')
        elif change == 'gripper':
            handoff = replace(handoff, gripper_generation=token.gripper_generation+1)
        elif change == 'hold':
            handoff = replace(handoff, hold=(.1,)*6)
        elif change in ('capability','missing_capability'):
            handoff = replace(handoff,capability='wrong' if change == 'capability' else '')
        elif change == 'deadline':
            token = replace(token, deadline=time.monotonic()-.1)
        elif change == 'disable':
            manager.lifecycle.disable()
        else:
            manager.lifecycle.disable()
            manager.lifecycle.shutdown()
            manager.lifecycle.connect()
        before = len(bus.events)
        with pytest.raises(RuntimeError):
            manager.arm.admit_prepared_arm_trajectory(token, handoff)
        assert not manager.runtime.execution.moving
        assert not any(method == 'position' and abs(args[0]) > .001
            for joint, method, args in bus.events[before:] if joint == 1)
        manager.arm.cancel_arm_preparation(token)
    finally:
        shutdown(manager)


def test_preparation_reserves_arm_gripper_and_cancel_waits_for_stop(monkeypatch):
    manager, bus = enabled_manager(monkeypatch)
    try:
        token, curve, handoff = reserve(manager)
        with pytest.raises(RuntimeError, match='busy'):
            manager.arm.reserve_arm_preparation(time.monotonic()+1.)
        with pytest.raises(RuntimeError, match='reserved'):
            manager.gripper.set_gripper_target(.01)
        with pytest.raises(RuntimeError, match='reserved'):
            manager.arm.send_joint_motor_cmd('joint1', SimpleNamespace(), arbitration='preempt')
        manager.arm.cancel_arm_preparation(token)
        assert not manager.runtime.arm_resource_reserved
        assert manager.lifecycle.enabled
        with pytest.raises(RuntimeError):
            manager.arm.admit_prepared_arm_trajectory(token, handoff)
    finally:
        shutdown(manager)


@pytest.mark.parametrize('dimensions', [6, 7])
def test_finger_position_change_without_new_owner_does_not_reject_checked_trajectory(monkeypatch, dimensions):
    """夹爪位置变化不再按量化偏差拒绝，原预约和发送确认仍须完整收尾。"""
    manager, bus = enabled_manager(monkeypatch)
    try:
        follow_gripper(bus, monkeypatch)
        token, _, handoff = reserve(manager) if dimensions == 6 else coordinated_reserve(manager)
        original = manager.arm.gripper
        def changed_position():
            state = original.snapshot()
            return dict(state, raw_position=state['raw_position']-.01,
                positions=(state['positions'][0]+.00009, state['positions'][1]-.00009))
        manager.arm.gripper = replace(original, snapshot=changed_position)
        task = manager.arm.admit_prepared_arm_trajectory(token, handoff)
        assert task.event.wait(3.) and task.result.state == 'succeeded'
        wait_until(lambda: not manager.runtime.arm_resource_reserved)
        assert manager.lifecycle.enabled
    finally:
        shutdown(manager)


def test_changed_gripper_generation_invalidates_active_curve(monkeypatch):
    manager, bus = enabled_manager(monkeypatch)
    try:
        token, curve, handoff = reserve(manager, 1.)
        task = manager.arm.admit_prepared_arm_trajectory(token, handoff)
        time.sleep(.04)
        original = manager.arm.gripper
        manager.arm.gripper = replace(original, snapshot=lambda: dict(
            original.snapshot(), generation=token.gripper_generation + 1))
        wait_until(lambda: task.event.is_set(), 2.)
        assert task.result.state != 'succeeded'
        assert manager.lifecycle.enabled
        wait_until(lambda: not manager.runtime.arm_resource_reserved)
    finally:
        shutdown(manager)


def test_prepared_partial_write_failure_does_not_acknowledge_candidate(monkeypatch):
    manager, bus = enabled_manager(monkeypatch)
    try:
        token, curve, handoff = reserve(manager)
        previous = manager.runtime.execution.last_sent_target
        # 第三轴失败意味着前两轴可能已发送；整批不能被确认，更不能更新保持证据。
        bus.failure = (3, 'position')
        manager.arm.admit_prepared_arm_trajectory(token, handoff)
        wait_until(lambda: not manager.runtime.control_loop_active)
        assert manager.runtime.execution.last_sent_target == previous
        assert not manager.runtime.execution.moving
        assert bus.disables == 1
    finally:
        bus.failure = None
        shutdown(manager)


def test_delayed_cycle_samples_current_time_without_replaying_backlog():
    from rebotarmcontroller.arm.arm_execution import ArmExecution
    execution = ArmExecution()
    execution.running = True
    curve = build_curve(JOINT_NAMES, point_to_point((0.,)*6, (.02,)*6))
    assert execution.submit_prepared(curve, execution.generation, now=10.)
    first = execution.sample(10.05)
    assert execution.progress == 0.
    later = execution.sample(10.29)
    # 采样本身不推进确认；必须显式 acknowledge 才能改写 last_sent_target。
    assert later.positions == pytest.approx(curve.sample(.29)[0])
    assert later.positions[0] > first.positions[0]
    assert execution.last_sent_target is None
    execution.acknowledge(later)
    assert execution.last_sent_target == later.positions
    final = execution.sample(12.)
    assert execution.moving
    execution.acknowledge(final)
    assert not execution.moving


def test_action_preparations_remain_bound_to_their_goal_after_invalidation():
    from rebotarmcontroller.ros.ros_actions import ArmActions, GoalResponse
    from control_msgs.action import FollowJointTrajectory
    actions = ArmActions.__new__(ArmActions)
    tokens = iter((object(), object()))
    canceled, seen = [], []
    actions._trajectory_mode = 'continuous'
    actions._pending_preparations = {}
    actions._lifecycle = SimpleNamespace(ready_for_motion=True)
    actions._runtime = SimpleNamespace(joint_names=JOINT_NAMES)
    actions._prepare_client = SimpleNamespace(service_is_ready=lambda: True)
    actions._arm = SimpleNamespace(reserve_arm_preparation=lambda deadline,**kwargs: next(tokens),
        cancel_arm_preparation=canceled.append,trajectory_capability=lambda mode:dict(revision='0'*64))
    actions._node = SimpleNamespace(publish_arm_status=lambda: None)
    def prepare(handle, token):
        seen.append(token)
        raise ValueError('deliberate preparation rejection')
    actions._prepare_follow_goal = prepare
    from trajectory_msgs.msg import JointTrajectoryPoint
    requests = [FollowJointTrajectory.Goal(), FollowJointTrajectory.Goal()]
    for request in requests:
        point = JointTrajectoryPoint(positions=[0.]*6);point.time_from_start.sec = 1
        request.trajectory.points = [point]
        request.trajectory.joint_names = list(JOINT_NAMES)
        assert actions.trajectory_goal_callback(request) == GoalResponse.ACCEPT
    expected = [actions._pending_preparations[id(request)][1] for request in requests]
    for request in reversed(requests):
        # 倒序返回模拟旧 Goal 的迟到回调，必须分别使用原预约，不能串用最后一个令牌。
        handle = SimpleNamespace(request=request, is_cancel_requested=False, abort=lambda: None)
        actions.execute_follow_joint_trajectory(handle)
    assert seen == expected[::-1]
    assert canceled == expected[::-1]
    assert not actions._pending_preparations


@pytest.mark.parametrize('reason', ['cancel', 'deadline'])
def test_action_wait_cancels_late_service_without_decoding_or_admitting(reason):
    from rebotarmcontroller.ros.ros_actions import ArmActions
    from rebotarm_msgs.srv import PrepareArmTrajectory
    from trajectory_msgs.msg import JointTrajectory
    actions = ArmActions.__new__(ArmActions)
    canceled = []
    actions._prepare_type = PrepareArmTrajectory
    actions._prepare_client = SimpleNamespace(call_async=lambda request: SimpleNamespace(
        done=lambda: False, cancel=lambda: canceled.append(True)))
    actions._trajectory_mode = 'continuous'
    actions._arm = SimpleNamespace(trajectory_capability=lambda mode: {})
    token = SimpleNamespace(session=1, generation=2, hold=(0.,)*6, feedback=(0.,)*6,
        observed_at=time.monotonic(), gripper_positions=(0.,0.), gripper_generation=0, collision_policy=0,
        scene_revision='a', deadline=time.monotonic()+(-1. if reason=='deadline' else 1.))
    goal = SimpleNamespace(request=SimpleNamespace(trajectory=JointTrajectory()),
        is_cancel_requested=reason=='cancel')
    with pytest.raises(RuntimeError, match='canceled or timed out'):
        actions._prepare_follow_goal(goal, token)
    assert canceled == [True]


def test_legacy_special_path_replaces_completed_prepared_curve():
    from rebotarmcontroller.arm.arm_execution import ArmExecution
    execution = ArmExecution();execution.running = True
    curve = build_curve(JOINT_NAMES, point_to_point((0.,)*6,(.02,)*6))
    assert execution.submit_prepared(curve, execution.generation, now=10.)
    execution.acknowledge(execution.sample(11.))
    assert not execution.moving
    first, last = (.03,0.,0.,0.,0.,0.), (.04,0.,0.,0.,0.,0.)
    assert execution.submit((first,last),(.2,.4),execution.generation,now=20.)
    assert execution.sample(20.25).positions == first


def test_observed_scene_change_stops_active_curve_and_keeps_enabled(monkeypatch):
    manager, bus = enabled_manager(monkeypatch)
    try:
        token, curve, handoff = reserve(manager, 1.)
        task = manager.arm.admit_prepared_arm_trajectory(token, handoff)
        time.sleep(.05)
        manager.arm.update_trajectory_scene('scene-b')
        assert task.event.wait(2.)
        assert task.result.state != 'succeeded'
        assert not manager.runtime.arm_resource_reserved
        assert manager.lifecycle.enabled
        assert not manager.runtime.execution.moving
    finally:
        shutdown(manager)


def test_action_cancel_invalidates_owner_before_returning_acceptance():
    from rebotarmcontroller.ros.ros_actions import ArmActions, CancelResponse
    actions = ArmActions.__new__(ArmActions)
    request,token = object(),object()
    canceled=[]
    actions._trajectory_mode='continuous'
    actions._pending_preparations={id(request):(request,token)}
    actions._arm=SimpleNamespace(cancel_arm_preparation=lambda token, *, wait_for_stop:
        canceled.append((token, wait_for_stop)))
    assert actions.cancel_follow_joint_trajectory(SimpleNamespace(request=request)) == CancelResponse.ACCEPT
    assert canceled == [(token, False)]


def coordinated_reserve(manager, duration=.4, finger=.005):
    """网页组合：六轴已检查曲线与独立夹爪目标，不存在第七轴参考。"""
    token = manager.arm.reserve_arm_preparation(time.monotonic()+2.,web=True,
        gripper_command=(finger*2.,1.))
    start = token.hold
    target = (start[0]+.02,*start[1:])
    curve = build_curve(JOINT_NAMES,point_to_point(start,target,duration))
    payload = dict(curve=curve.to_payload(),context=dict(session=token.session,
        generation=token.generation,hold=token.hold),gripper_positions=token.gripper_positions,
        gripper_generation=token.gripper_generation,scene_revision=token.scene_revision,
        capability_revision=token.capability,gripper_collision_policy=1)
    return token,curve,manager.arm.decode_prepared_arm_trajectory(payload)


def follow_gripper(bus, monkeypatch):
    motor = bus.motors[-1]
    original = motor.send_mit
    def send(*args):
        original(*args)
        if args[2] > 0:
            motor.state.pos = args[0]
    monkeypatch.setattr(motor, 'send_mit', send)


def test_combination_observes_independent_children_without_reference_resampling():
    from rebotarmcontroller.runtime.coordinated_trajectory import CoordinatedTrajectory
    from rebotarmcontroller.arm.arm_trajectory import ArmTrajectory
    from rebotarmcontroller.arm.arm_execution import ArmExecution
    execution = ArmExecution();execution.running = True
    arm = ArmTrajectory(execution,clock=lambda:10.)
    owner = CoordinatedTrajectory(execution,arm)
    curve = build_curve(JOINT_NAMES,point_to_point((0.,)*6,(.02,)*6,.4))
    registered = []
    gripper = SimpleNamespace(start_position=lambda *args:registered.append(args) or 3,
        position_state=lambda generation:('active',''),cancel_position=lambda *a,**k:None)
    token = SimpleNamespace(generation=execution.generation,gripper_command=(.02,1.))
    task = owner.submit(curve,token,gripper)
    owner.tick(gripper,SimpleNamespace(positions=(0.,)*6,observed_at=10.1))
    assert registered == [(token,.02,1.)]
    assert not isinstance(owner, ArmTrajectory)
    assert task.gripper_generation == 3 and not task.event.is_set()
    owner.stopping(task)
    owner.finish(task,'canceled','stopped')
    assert arm.task.event.is_set() and arm.task.result.state == 'canceled'


def test_combination_admission_rolls_back_gripper_before_any_device_cycle(monkeypatch):
    from rebotarmcontroller.runtime.coordinated_trajectory import CoordinatedTrajectory
    from rebotarmcontroller.arm.arm_trajectory import ArmTrajectory
    from rebotarmcontroller.arm.arm_execution import ArmExecution
    execution = ArmExecution(); execution.running = True
    arm = ArmTrajectory(execution)
    owner = CoordinatedTrajectory(execution,arm)
    curve = build_curve(JOINT_NAMES,point_to_point((0.,)*6,(.02,)*6,.4))
    calls = []
    def reject(*args):
        raise RuntimeError('admission invalidated')
    monkeypatch.setattr(arm,'submit_prepared',reject)
    gripper = SimpleNamespace(start_position=lambda *args: 9,
        cancel_position=lambda reason,**kwargs:calls.append(kwargs))
    with pytest.raises(RuntimeError,match='admission invalidated'):
        owner.submit(curve,SimpleNamespace(generation=execution.generation,
            gripper_command=(.02,1.)),gripper)
    assert calls == [dict(expected_generation=9)]
    assert owner.task is None and not execution.moving and execution.last_sent_target is None


def test_coordinated_writer_sends_both_tracks_and_releases_only_after_neutral(monkeypatch):
    manager, bus = enabled_manager(monkeypatch)
    try:
        follow_gripper(bus, monkeypatch)
        token, curve, handoff = coordinated_reserve(manager)
        before = len(bus.events)
        task = manager.arm.admit_prepared_arm_trajectory(token, handoff)
        assert task.event.wait(2.)
        assert task.result.state == 'succeeded', task.result
        events = bus.events[before:]
        finger = [args for joint,method,args in events if joint == 7 and method == 'mit']
        assert len(finger) >= 20
        assert any(args[0] < -.01 and args[2] > 0 for args in finger)
        assert finger[-1][2:5] == (0., 0., 0.)
        assert manager.runtime.execution.last_sent_target == curve.points[-1].positions[:6]
        wait_until(lambda: not manager.runtime.arm_resource_reserved)
        assert manager.lifecycle.enabled
    finally:
        shutdown(manager)


@pytest.mark.parametrize('change', ['cancel', 'scene', 'gripper_cancel', 'arm_failure', 'finger_failure'])
def test_coordinated_stop_or_partial_failure_never_reports_success(monkeypatch, change):
    manager, bus = enabled_manager(monkeypatch)
    try:
        follow_gripper(bus, monkeypatch)
        token, curve, handoff = coordinated_reserve(manager, 1.)
        task = manager.arm.admit_prepared_arm_trajectory(token, handoff)
        time.sleep(.03)
        if change == 'cancel':
            manager.arm.stop_active_motion(task)
        elif change == 'scene':
            manager.arm.update_trajectory_scene('scene-b')
        elif change == 'gripper_cancel':
            assert manager.gripper.cancel_gripper_position_command('injected cancellation')
        elif change == 'arm_failure':
            bus.failure = (3, 'position')
        else:
            bus.failure = (7, 'mit')
        assert task.event.wait(3.)
        assert task.result.state != 'succeeded'
        if change in ('finger_failure','arm_failure'):
            wait_until(lambda: not manager.runtime.control_loop_active)
            assert manager.lifecycle.state == 'DISABLING'
            assert bus.disables == 1
        else:
            wait_until(lambda: not manager.runtime.arm_resource_reserved)
            assert manager.lifecycle.enabled
    finally:
        bus.failure = None
        shutdown(manager)


def test_diagnostic_hold_residual_no_longer_rejects_or_changes_hold(monkeypatch):
    manager, bus = enabled_manager(monkeypatch)
    try:
        sample = manager.feedback.joint_snapshot()
        sample = replace(sample, positions=(.003, *sample.positions[1:]))
        reasons = manager.arm._admission_reasons(sample, (0.,)*6, time.monotonic())
        assert reasons == []
        assert manager.runtime.execution.last_sent_target == (0.,)*6
    finally:
        shutdown(manager)


@pytest.mark.parametrize('dimensions', [6, 7])
@pytest.mark.parametrize('residual', [.003, .01])
def test_cancel_with_stationary_residual_releases_resources_and_allows_next_goal(monkeypatch, dimensions, residual):
    """取消后的静止不依赖目标残差；反复空闲停止保留原保持命令，不逐次追锚。"""
    manager, bus = enabled_manager(monkeypatch)
    try:
        follow_gripper(bus, monkeypatch)
        motor = bus.motors[0]
        send = motor.send_pos_vel
        def offset(position, velocity):
            send(position, velocity)
            motor.state.pos = position+residual
        monkeypatch.setattr(motor, 'send_pos_vel', offset)
        wait_until(lambda: manager.arm.admission_snapshot()['ready']
            and abs(manager.feedback.joint_snapshot().positions[0]-manager.runtime.target()[0]-residual) < 1e-9)
        token, _, handoff = (reserve(manager, 1.) if dimensions == 6 else coordinated_reserve(manager, 1.))
        task = manager.arm.admit_prepared_arm_trajectory(token, handoff)
        time.sleep(.04)
        manager.arm.stop_active_motion(task)
        assert task.event.is_set() and task.result.state == 'canceled'
        if dimensions == 7:  # 历史参数名：现在表示六轴加独立夹爪组合，不是七轴 FJT。
            arm_state, gripper_state = manager.arm.teleop_execution_status(task)
            assert arm_state == 'canceled' and gripper_state == 'failed'
        assert not manager.runtime.arm_resource_reserved
        assert manager.arm._stop_wait is None
        hold = tuple(manager.runtime.execution.last_sent_target)
        manager.arm.stop_active_motion()
        assert manager.runtime.execution.last_sent_target == hold
        wait_until(lambda: manager.arm.admission_snapshot()['ready'])
        token, _, handoff = reserve(manager, 1.)
        next_task = manager.arm.admit_prepared_arm_trajectory(token, handoff)
        assert next_task.event.wait(3.) and next_task.result.state == 'succeeded'
        wait_until(lambda: not manager.runtime.arm_resource_reserved)
        assert manager.lifecycle.enabled
    finally:
        shutdown(manager)


@pytest.mark.parametrize('previous_result', ['succeeded', 'canceled', 'aborted'])
@pytest.mark.parametrize('dimensions', [6, 7])
def test_old_coordinated_terminal_cannot_finalize_next_task(monkeypatch, previous_result, dimensions):
    """旧终态不能提前释放新预约，也不能使新成功任务永久占用资源。"""
    manager, bus = enabled_manager(monkeypatch)
    try:
        follow_gripper(bus, monkeypatch)
        token, _, handoff = coordinated_reserve(manager, .4 if previous_result == 'succeeded' else 1.)
        old = manager.arm.admit_prepared_arm_trajectory(token, handoff)
        if previous_result == 'canceled':
            manager.arm.stop_active_motion(old)
        elif previous_result == 'aborted':
            original = manager.arm.gripper
            def fail_coordinated_state(generation):
                raise RuntimeError('injected coordinated state failure')
            manager.arm.gripper = replace(original, position_state=fail_coordinated_state)
        assert old.event.wait(3.) and old.result.state == previous_result
        if previous_result == 'aborted':
            manager.arm.gripper = original
        wait_until(lambda: manager.arm.admission_snapshot()['ready'])
        token, _, handoff = (reserve(manager) if dimensions == 6 else coordinated_reserve(manager))
        current = manager.arm.admit_prepared_arm_trajectory(token, handoff)
        time.sleep(.04)
        assert not current.event.is_set()
        assert manager.lifecycle.execution == 'TRAJ_RUNNING'
        assert manager.runtime.execution.reservation is token
        with pytest.raises(RuntimeError, match='reserved'):
            manager.gripper.set_gripper_target(.01)
        assert current.event.wait(2.) and current.result.state == 'succeeded'
        wait_until(lambda: not manager.runtime.arm_resource_reserved)
        assert manager.lifecycle.execution == 'IDLE'
    finally:
        shutdown(manager)


@pytest.mark.parametrize('fault', [None, 'gripper_timeout', 'arm_velocity'])
def test_coordinated_stop_waits_for_both_sides_without_restart_or_late_cleanup(monkeypatch, fault):
    """机械臂先停稳时仍加入原事务；超时保留预约，旧续体不能清理后继准备。"""
    manager, bus = enabled_manager(monkeypatch)
    allow_gripper = threading.Event()
    workers, errors = [], []
    try:
        follow_gripper(bus, monkeypatch)
        token, _, handoff = coordinated_reserve(manager, 1.)
        task = manager.arm.admit_prepared_arm_trajectory(token, handoff)
        original = manager.arm.gripper
        def stationary_gripper():
            if not allow_gripper.is_set():
                raise RuntimeError('injected gripper still moving')
            return original.snapshot()
        manager.arm.gripper = replace(original, snapshot=stationary_gripper)
        def stop():
            try:
                manager.arm.stop_active_motion(task)
            except RuntimeError as exc:
                errors.append(str(exc))
        worker = threading.Thread(target=stop)
        workers.append(worker); worker.start()
        wait_until(lambda: manager.arm._stop_wait is not None)
        old_poll, _ = manager.arm._stop_wait
        generation = manager.runtime.target_generation
        wait_until(lambda: manager.arm._stop.result.state == 'confirmed')
        with pytest.raises(RuntimeError, match='pending'):
            manager.arm.reserve_arm_preparation(time.monotonic()+2.)
        worker = threading.Thread(target=stop)
        workers.append(worker); worker.start()
        time.sleep(.05)
        assert manager.runtime.target_generation == generation
        assert not task.event.is_set() and manager.runtime.arm_resource_reserved
        if fault == 'arm_velocity':
            for motor in bus.motors[:6]:
                motor.state.vel = .2
            wait_until(lambda: manager.arm._stop.evidence.reason == 'velocity')
        if fault != 'gripper_timeout':
            allow_gripper.set()
        for worker in workers:
            worker.join(3.)
            assert not worker.is_alive()
        assert bool(errors) == (fault is not None)
        if fault is not None:
            assert task.result.state == 'aborted' and manager.runtime.arm_resource_reserved
            time.sleep(.1)
            assert manager.runtime.target_generation == generation
            allow_gripper.set()
            for motor in bus.motors[:6]:
                motor.state.vel = 0.
            manager.arm.stop_active_motion()
        else:
            assert task.result.state == 'canceled'
        assert manager.arm._stop_wait is None
        assert not manager.runtime.arm_resource_reserved
        wait_until(lambda: manager.arm.admission_snapshot()['ready'])
        newer = manager.arm.reserve_arm_preparation(time.monotonic()+2.)
        if fault is not None:
            with pytest.raises(RuntimeError, match='superseded'):
                manager.runtime.call(old_poll)
        else:
            assert manager.runtime.call(old_poll) == (True, None)
        assert manager.runtime.execution.reservation is newer
        assert manager.lifecycle.execution == 'TRAJ_PREPARING'
        manager.arm.cancel_arm_preparation(newer)
    finally:
        allow_gripper.set()
        for worker in workers:
            worker.join(3.)
        shutdown(manager)

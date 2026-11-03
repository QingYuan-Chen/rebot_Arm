"""正式所有者、设备周期与假驱动的 prepared_v1 回归，不打开串口。"""
from dataclasses import replace
import time
from types import SimpleNamespace

import numpy as np
import pytest

from test_controller_local_runtime import make_local_manager, wait_until
from rebotarm_motion.polynomial_trajectory import JOINT_NAMES, build_curve
from rebotarm_motion.trajectory_preparation import point_to_point


def enabled_manager(monkeypatch):
    manager, buses = make_local_manager(monkeypatch)
    manager.lifecycle.connect()
    manager.lifecycle.enable()
    manager.arm.update_trajectory_scene('scene-a')
    wait_until(lambda: manager.arm._hold_evidence.confirmed(), timeout=2.)
    wait_until(lambda: manager.gripper._stationary.confirmed(), timeout=2.)
    return manager, buses[0]


def reserve(manager, duration=.4):
    token = manager.arm.reserve_arm_preparation(time.monotonic()+2.)
    target = list(token.hold)
    target[0] += .02
    curve = build_curve(JOINT_NAMES, point_to_point(token.hold, target, duration))
    payload = dict(curve=curve.to_payload(), context=dict(session=token.session,
        generation=token.generation, hold=token.hold), gripper_positions=token.gripper_positions,
        gripper_generation=token.gripper_generation, scene_revision=token.scene_revision)
    return token, curve, manager.arm.decode_prepared_arm_trajectory(payload)


def shutdown(manager):
    if manager.lifecycle.enabled:
        manager.lifecycle.disable()
    manager.lifecycle.shutdown()


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


@pytest.mark.parametrize('change', ['scene', 'gripper', 'hold', 'deadline', 'disable', 'reconnect'])
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


def test_protective_gripper_release_invalidates_active_curve(monkeypatch):
    manager, bus = enabled_manager(monkeypatch)
    try:
        token, curve, handoff = reserve(manager, 1.)
        task = manager.arm.admit_prepared_arm_trajectory(token, handoff)
        time.sleep(.04)
        manager.gripper.release_grasp_hold('protective release')
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
    from rebotarmcontroller.arm_execution import ArmExecution
    execution = ArmExecution()
    execution.running = True
    curve = build_curve(JOINT_NAMES, point_to_point((0.,)*6, (.02,)*6))
    assert execution.submit_prepared(curve, execution.generation, now=10.)
    first = execution.sample(10.05)
    assert execution.progress == 0.
    later = execution.sample(10.29)
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
    from rebotarmcontroller.ros_actions import ArmActions, GoalResponse
    from control_msgs.action import FollowJointTrajectory
    actions = ArmActions.__new__(ArmActions)
    tokens = iter((object(), object()))
    canceled, seen = [], []
    actions._trajectory_mode = 'prepared_v1'
    actions._pending_preparations = {}
    actions._lifecycle = SimpleNamespace(ready_for_motion=True)
    actions._prepare_client = SimpleNamespace(service_is_ready=lambda: True)
    actions._arm = SimpleNamespace(reserve_arm_preparation=lambda deadline: next(tokens),
        cancel_arm_preparation=canceled.append)
    actions._node = SimpleNamespace(publish_arm_status=lambda: None)
    def prepare(handle, token):
        seen.append(token)
        raise ValueError('deliberate preparation rejection')
    actions._prepare_follow_goal = prepare
    from trajectory_msgs.msg import JointTrajectoryPoint
    requests = [FollowJointTrajectory.Goal(), FollowJointTrajectory.Goal()]
    for request in requests:
        point = JointTrajectoryPoint();point.time_from_start.sec = 1
        request.trajectory.points = [point]
        assert actions.trajectory_goal_callback(request) == GoalResponse.ACCEPT
    expected = [actions._pending_preparations[id(request)][1] for request in requests]
    for request in reversed(requests):
        handle = SimpleNamespace(request=request, is_cancel_requested=False, abort=lambda: None)
        actions.execute_follow_joint_trajectory(handle)
    assert seen == expected[::-1]
    assert canceled == expected[::-1]
    assert not actions._pending_preparations


@pytest.mark.parametrize('reason', ['cancel', 'deadline'])
def test_action_wait_cancels_late_service_without_decoding_or_admitting(reason):
    from rebotarmcontroller.ros_actions import ArmActions
    from rebotarm_msgs.srv import PrepareArmTrajectory
    from trajectory_msgs.msg import JointTrajectory
    actions = ArmActions.__new__(ArmActions)
    canceled = []
    actions._prepare_type = PrepareArmTrajectory
    actions._prepare_client = SimpleNamespace(call_async=lambda request: SimpleNamespace(
        done=lambda: False, cancel=lambda: canceled.append(True)))
    actions._arm = SimpleNamespace()
    token = SimpleNamespace(session=1, generation=2, hold=(0.,)*6, feedback=(0.,)*6,
        observed_at=time.monotonic(), gripper_positions=(0.,0.), gripper_generation=0,
        scene_revision='a', deadline=time.monotonic()+(-1. if reason=='deadline' else 1.))
    goal = SimpleNamespace(request=SimpleNamespace(trajectory=JointTrajectory()),
        is_cancel_requested=reason=='cancel')
    with pytest.raises(RuntimeError, match='canceled or timed out'):
        actions._prepare_follow_goal(goal, token)
    assert canceled == [True]


def test_legacy_special_path_replaces_completed_prepared_curve():
    from rebotarmcontroller.arm_execution import ArmExecution
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
    from rebotarmcontroller.ros_actions import ArmActions, CancelResponse
    actions = ArmActions.__new__(ArmActions)
    request,token = object(),object()
    canceled=[]
    actions._trajectory_mode='prepared_v1'
    actions._pending_preparations={id(request):(request,token)}
    actions._arm=SimpleNamespace(cancel_arm_preparation=canceled.append)
    assert actions.cancel_follow_joint_trajectory(SimpleNamespace(request=request)) == CancelResponse.ACCEPT
    assert canceled == [token]

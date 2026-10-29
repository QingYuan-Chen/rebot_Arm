"""批次 6：资源代次、单写入线程、取消确认和迟到请求的离线故障注入。"""
import threading
import time
from types import SimpleNamespace
import pytest
from rebotarmcontroller.device_requests import DeviceRequests, RequestRejected
from test_controller_local_runtime import make_local_manager, wait_until


def test_reserved_stop_preempts_full_queue_and_invalidates_only_arm():
    clock = [0.]
    queue = DeviceRequests(capacity=2, reserved=1, clock=lambda: clock[0])
    old = queue.submit(lambda: None, resource='arm')
    gripper = queue.submit(lambda: None, resource='gripper')
    with pytest.raises(RequestRejected, match='full'):
        queue.submit(lambda: None)
    stop = queue.submit(lambda: None, resource='arm', priority=0, invalidate=('arm',))
    assert queue.take() is stop
    assert queue.take() is gripper
    assert old.event.is_set() and isinstance(old.error, RequestRejected)


def test_expired_and_waiter_canceled_requests_never_start():
    clock = [0.]
    queue = DeviceRequests(clock=lambda: clock[0])
    expired = queue.submit(lambda: pytest.fail('late execution'), ttl=.1)
    canceled = queue.submit(lambda: pytest.fail('canceled execution'))
    assert queue.cancel_pending(canceled)
    clock[0] = 1.
    assert queue.take() is None
    assert expired.event.is_set() and not expired.started
    assert canceled.event.is_set() and not canceled.started


def test_started_operation_keeps_its_actual_result_after_caller_deadline():
    queue = DeviceRequests()
    request = queue.submit(lambda: 7)
    assert queue.take() is request
    assert not queue.cancel_pending(request)
    queue.finish(request, result=7)
    assert request.result == 7 and request.error is None


def test_mode_generation_rejects_arm_and_gripper_requests():
    queue = DeviceRequests()
    arm = queue.submit(lambda: None, resource='arm')
    gripper = queue.submit(lambda: None, resource='gripper')
    mode = queue.submit(lambda: None, priority=5, invalidate=('device',))
    assert queue.take() is mode
    assert queue.take() is None
    assert arm.error and gripper.error


def test_every_device_call_and_disabled_feedback_stays_on_one_writer(monkeypatch):
    manager, opened = make_local_manager(monkeypatch)
    with pytest.raises(RuntimeError, match='outside runtime'):
        manager.runtime.driver.connect()
    assert not opened
    manager.lifecycle.connect()
    writer = manager.runtime.control_thread
    bus = opened[0]
    observed = []
    for motor in bus.motors:
        original = motor._record
        def record(*args, original=original):
            observed.append(threading.current_thread())
            original(*args)
        monkeypatch.setattr(motor, '_record', record)
    before = manager.feedback.get_cached_joint_sample()[4]
    wait_until(lambda: manager.feedback.get_cached_joint_sample()[4] != before)
    manager.lifecycle.enable()
    manager.arm.stop_active_motion()
    manager.lifecycle.disable()
    manager.lifecycle.set_mode('mit')
    manager.lifecycle.set_mode('pos_vel')
    with pytest.raises(RuntimeError, match='outside runtime'):
        manager.runtime.joint_device('joint1').send_vel(0.)
    with pytest.raises(RuntimeError, match='outside runtime'):
        manager.runtime.controllers()[0].close()
    manager.lifecycle.shutdown()
    assert observed and set(observed) == {writer}
    assert not writer.is_alive()


def test_trajectory_is_local_and_gripper_does_not_preempt_arm(monkeypatch):
    manager, _ = make_local_manager(monkeypatch)
    manager.lifecycle.connect(); manager.lifecycle.enable()
    try:
        task = manager.arm.submit_arm_trajectory(manager.runtime.joint_names,
            [(0.,)*6, (.1,-.1,-.1,0.,0.,0.)], [0., .1])
        generation = manager.gripper.set_gripper_target(.02)
        with pytest.raises(RuntimeError, match='busy'):
            manager.arm.submit_arm_trajectory(manager.runtime.joint_names, [(0.,)*6], [0.])
        assert task.event.wait(1.)
        assert manager.arm.arm_execution_status(task).state == 'succeeded'
        assert generation == manager.gripper._gripper_execution._generation
    finally:
        manager.lifecycle.disable(); manager.lifecycle.shutdown()


def test_cancel_waits_for_hold_and_old_cancel_cannot_stop_successor(monkeypatch):
    manager, _ = make_local_manager(monkeypatch)
    manager.lifecycle.connect(); manager.lifecycle.enable()
    try:
        task = manager.arm.submit_arm_trajectory(manager.runtime.joint_names,
            [(0.,)*6, (.1,-.1,-.1,0.,0.,0.)], [0., 1.])
        manager.arm.stop_active_motion(task)
        assert task.event.is_set()
        assert manager.arm.arm_execution_status(task).state == 'canceled'
        successor = manager.arm.submit_arm_trajectory(manager.runtime.joint_names,
            [(0.,)*6, (.05,-.05,-.05,0.,0.,0.)], [0., .1])
        manager.arm.stop_active_motion(task)
        assert successor.event.wait(1.)
        assert manager.arm.arm_execution_status(successor).state == 'succeeded'
        assert manager.lifecycle.enabled
    finally:
        manager.lifecycle.disable(); manager.lifecycle.shutdown()


def test_goal_tolerance_timeout_holds_without_disable(monkeypatch):
    manager, opened = make_local_manager(monkeypatch)
    manager.lifecycle.connect(); manager.lifecycle.enable()
    try:
        motor = opened[0].motors[0]
        monkeypatch.setattr(motor, 'send_pos_vel', lambda *_: None)
        task = manager.arm.submit_arm_trajectory(manager.runtime.joint_names,
            [(0.,)*6, (.2,0.,0.,0.,0.,0.)], [0., .02], timeout=.03)
        assert task.event.wait(1.)
        wait_until(lambda: manager.lifecycle.execution == 'IDLE')
        assert manager.arm.arm_execution_status(task).state == 'tolerance'
        assert manager.lifecycle.enabled and manager.arm.joint_target()[0] == 0.
        assert opened[0].disables == 0
    finally:
        manager.lifecycle.disable(); manager.lifecycle.shutdown()


def motor_command():
    return SimpleNamespace(mode=1, pos=.01, vel=0., kp=0., kd=0., tau=0., vlim=.3,
        use_pos=True, use_vel=False, use_kp=False, use_kd=False, use_tau=False, use_vlim=True)


def test_passthrough_reject_preempt_and_lease_expiry(monkeypatch):
    manager, _ = make_local_manager(monkeypatch)
    manager.lifecycle.connect(); manager.lifecycle.enable()
    try:
        task = manager.arm.submit_arm_trajectory(manager.runtime.joint_names,
            [(0.,)*6, (.1,0.,0.,0.,0.,0.)], [0., 1.])
        with pytest.raises(RuntimeError, match='busy'):
            manager.arm.send_joint_motor_cmd('joint1', motor_command())
        manager.arm.send_joint_motor_cmd('joint1', motor_command(), arbitration='preempt')
        assert manager.arm.arm_execution_status(task).state == 'canceled'
        assert manager.lifecycle.execution == 'LOWLEVEL_STREAMING'
        wait_until(lambda: manager.lifecycle.execution == 'IDLE')
        assert manager.arm.joint_target()[0] == pytest.approx(.01)
    finally:
        manager.lifecycle.disable(); manager.lifecycle.shutdown()


def test_disable_ends_waiting_trajectory_instead_of_hanging(monkeypatch):
    manager, _ = make_local_manager(monkeypatch)
    manager.lifecycle.connect(); manager.lifecycle.enable()
    task = manager.arm.submit_arm_trajectory(manager.runtime.joint_names,
        [(0.,)*6, (.1,0.,0.,0.,0.,0.)], [0., 10.])
    manager.lifecycle.disable()
    assert task.event.wait(.1)
    assert manager.arm.arm_execution_status(task).state == 'aborted'
    manager.lifecycle.shutdown()


@pytest.mark.parametrize('point', [(0.,)*5, (0.,)*7])
def test_wrong_point_dimension_rejected_without_replacing_hold(monkeypatch, point):
    manager, _ = make_local_manager(monkeypatch)
    manager.lifecycle.connect(); manager.lifecycle.enable()
    try:
        generation = manager.runtime.target_generation
        with pytest.raises(ValueError, match='six positions'):
            manager.arm.submit_arm_trajectory(manager.runtime.joint_names, [point], [0.])
        assert manager.runtime.target_generation == generation
        assert manager.lifecycle.enabled
    finally:
        manager.lifecycle.disable(); manager.lifecycle.shutdown()


def test_gravity_resource_rejects_passthrough_and_preempt_confirms_hold(monkeypatch):
    manager, _ = make_local_manager(monkeypatch)
    manager.lifecycle.connect(); manager.lifecycle.enable()
    try:
        manager.gravity.start_gravity_compensation()
        with pytest.raises(RuntimeError, match='busy'):
            manager.arm.send_joint_motor_cmd('joint1', motor_command())
        assert manager.gravity.gravity_compensation_active()
        manager.arm.send_joint_motor_cmd('joint1', motor_command(), arbitration='preempt')
        assert not manager.gravity.gravity_compensation_active()
        assert manager.runtime.mode == 'pos_vel'
        assert manager.lifecycle.enabled
    finally:
        manager.lifecycle.disable(); manager.lifecycle.shutdown()


def test_disable_clears_stream_lease_before_new_session(monkeypatch):
    manager, _ = make_local_manager(monkeypatch)
    manager.lifecycle.connect(); manager.lifecycle.enable()
    manager.arm.send_joint_motor_cmd('joint1', motor_command())
    manager.lifecycle.disable()
    manager.lifecycle.enable()
    try:
        assert manager.arm._arm_passthrough_until is None
        task = manager.arm.submit_arm_trajectory(manager.runtime.joint_names,
            [(0.01,0.,0.,0.,0.,0.)], [0.])
        assert task.event.wait(1.)
        assert manager.arm.arm_execution_status(task).state == 'succeeded'
    finally:
        manager.lifecycle.disable(); manager.lifecycle.shutdown()


def test_reconnect_waits_until_shutdown_worker_join_finishes(monkeypatch):
    manager, opened = make_local_manager(monkeypatch)
    manager.lifecycle.connect()
    runtime = manager.runtime
    old_writer = runtime.control_thread
    closing, proceed = threading.Event(), threading.Event()
    close = runtime.close_worker
    errors = []
    def delayed_close():
        closing.set()
        assert proceed.wait(2.)
        close()
    monkeypatch.setattr(runtime, 'close_worker', delayed_close)
    def shutdown():
        try:
            manager.lifecycle.shutdown()
        except Exception as exc:
            errors.append(exc)
    worker = threading.Thread(target=shutdown)
    worker.start()
    try:
        assert closing.wait(1.)
        old_writer.join(timeout=1.)
        assert not old_writer.is_alive()
        with pytest.raises(RuntimeError, match='closing'):
            manager.lifecycle.connect()
        assert len(opened) == 1
    finally:
        proceed.set()
        worker.join(timeout=2.)
    assert not errors and not worker.is_alive()
    monkeypatch.setattr(runtime, 'close_worker', close)
    manager.lifecycle.connect()
    assert len(opened) == 2 and runtime.control_thread is not old_writer
    manager.lifecycle.shutdown()


@pytest.mark.parametrize('entry,args,priority,ttl', [
    ('connect', (), 5, 10.), ('enable', (), 5, 5.), ('disable', (), 1, 10.),
    ('set_mode', ('mit',), 5, 5.), ('set_zero', ('joint1',), 5, 5.),
    ('shutdown', (), 5, 10.),
])
def test_lifecycle_component_queues_once_with_original_policy(
    monkeypatch, entry, args, priority, ttl,
):
    """从真实队列观测策略，生命周期命令不能漏排队或重复排队。"""
    manager, _ = make_local_manager(monkeypatch)
    runtime = manager.runtime
    if entry != 'connect':
        manager.lifecycle.connect()
    if entry == 'disable':
        manager.lifecycle.enable()
    observed = []
    submit = runtime.requests.submit

    def record(operation, **options):
        # 请求线程不能拿着生命周期锁等待设备线程。
        assert not manager.lifecycle.lock._is_owned()
        observed.append(options)
        return submit(operation, **options)

    monkeypatch.setattr(runtime.requests, 'submit', record)
    try:
        getattr(manager.lifecycle, entry)(*args)
        assert observed == [dict(resource='device', priority=priority, ttl=ttl, invalidate=('device',))]
        if entry == 'shutdown':
            assert not manager.lifecycle.connected and not runtime.writer_active
    finally:
        if manager.lifecycle.enabled:
            manager.lifecycle.disable()
        manager.lifecycle.shutdown()


def test_component_copies_mutable_input_before_runtime_handoff(monkeypatch):
    manager, _ = make_local_manager(monkeypatch)
    manager.lifecycle.connect()
    manager.lifecycle.enable()
    positions = [.01, -.01, -.01, 0., 0., 0.]
    expected = tuple(positions)
    call = manager.runtime.call

    def handoff(operation, **options):
        if not manager.runtime.in_writer:
            positions[0] = .2
        return call(operation, **options)

    monkeypatch.setattr(manager.runtime, 'call', handoff)
    try:
        manager.arm.set_joint_target(positions)
        assert positions[0] == .2
        assert tuple(manager.arm.joint_target()) == expected
    finally:
        manager.lifecycle.disable()
        manager.lifecycle.shutdown()


def test_lifecycle_enqueues_before_acquiring_transaction_lock(monkeypatch):
    manager, _ = make_local_manager(monkeypatch)
    manager.lifecycle.connect()
    queued = threading.Event()
    submit = manager.runtime.requests.submit
    errors = []

    def record(operation, **options):
        request = submit(operation, **options)
        queued.set()
        return request

    def change_mode():
        try:
            manager.lifecycle.set_mode('mit')
        except Exception as exc:
            errors.append(exc)

    monkeypatch.setattr(manager.runtime.requests, 'submit', record)
    worker = threading.Thread(target=change_mode)
    try:
        with manager.lifecycle.lock:
            worker.start()
            assert queued.wait(1.), 'caller waited for the lifecycle lock before enqueueing'
        worker.join(2.)
        assert not worker.is_alive() and not errors
    finally:
        worker.join(2.)
        manager.lifecycle.shutdown()


def test_direct_lifecycle_disable_uses_reserved_queue_and_invalidates_pending_commands(monkeypatch):
    manager, _ = make_local_manager(monkeypatch)
    manager.lifecycle.connect()
    manager.lifecycle.enable()
    runtime = manager.runtime
    started, release = threading.Event(), threading.Event()
    errors = []

    def block():
        started.set()
        assert release.wait(2.)

    def run(operation):
        try:
            operation()
        except Exception as exc:
            errors.append(exc)

    blocker = threading.Thread(target=lambda: run(lambda: runtime.call(block)))
    disable = threading.Thread(target=lambda: run(manager.lifecycle.disable))
    blocker.start()
    try:
        assert started.wait(1.)
        pending = [runtime.requests.submit(lambda: pytest.fail('obsolete motion ran'), resource='arm')
                   for _ in range(runtime.requests.capacity)]
        with pytest.raises(RequestRejected, match='full'):
            manager.lifecycle.set_mode('mit')
        disable.start()
        wait_until(lambda: any(request.priority == 1 for request in runtime.requests.pending))
        release.set()
        disable.join(2.)
        blocker.join(2.)
        assert not errors and not disable.is_alive() and not blocker.is_alive()
        assert not manager.lifecycle.enabled and manager.lifecycle.state == 'CONNECTED_DISABLED'
        assert all(request.event.wait(.5) and isinstance(request.error, RequestRejected)
                   and not request.started for request in pending)
    finally:
        release.set()
        blocker.join(2.)
        if disable.ident is not None:
            disable.join(2.)
        if manager.lifecycle.enabled:
            manager.lifecycle.disable()
        manager.lifecycle.shutdown()

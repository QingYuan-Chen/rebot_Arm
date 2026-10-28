"""使用假句柄和 POSIX 虚拟串口验证本地驱动与运行时。"""
from __future__ import annotations
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
import importlib.util
import sys
import threading
import time

import numpy as np
import pytest

from rebotarmcontroller.configuration import load_configuration
from rebotarmcontroller.controller_contracts import PositionCommand
from rebotarmcontroller.sdk_driver import SdkMotorDriver, CommandTransportError
from rebotarmcontroller.arm_execution import ArmExecution
from rebotarmcontroller.arm_position_control import ArmPositionControl
from rebotarmcontroller.controller_runtime import ControllerRuntime
from rebotarmcontroller.hardware_manager import HardwareManager


class HandleMotor:
    def __init__(self,controller,number):
        self.controller,self.number=controller,number
        self.state=SimpleNamespace(pos=0.,vel=0.,torq=0.,status_code=0)
        self.sequence=0
        self.requested=False
        self.closed=False

    def _record(self,method,*args):
        assert not self.closed
        self.controller.events.append((self.number,method,args))
        if (self.number,method)==self.controller.failure:
            raise RuntimeError('injected transport failure')

    def get_state_with_sequence(self): return self.state,self.sequence
    def request_feedback(self): self._record('request');self.requested=True
    def send_pos_vel(self,pos,vlim): self._record('position',pos,vlim);self.state.pos=pos
    def send_mit(self,*args): self._record('mit',*args)
    def send_vel(self,vel): self._record('velocity',vel)
    def write_register_f32(self,*args): self._record('register',*args)
    def ensure_mode(self,*args): self._record('mode',*args)
    def enable(self): self._record('enable');self.state.status_code=1
    def disable(self): self._record('disable');self.state.status_code=0
    def set_zero_position(self): self._record('zero');self.state.pos=0.
    def close(self): self._record('free');self.closed=True


class HandleController:
    def __init__(self):
        self.motors=[]
        self.events=[]
        self.failure=None
        self.fail_disable=False
        self.fail_add=None
        self.feedback=True
        self.closed=False
        self.disables=0

    def add_damiao_motor(self,number,feedback,model):
        if number==self.fail_add: raise RuntimeError('injected add failure')
        motor=HandleMotor(self,number);self.motors.append(motor)
        self.events.append((number,'add',(feedback,model)))
        return motor
    def poll_feedback_once(self):
        for motor in self.motors:
            if self.feedback and motor.requested:
                motor.sequence+=1;motor.requested=False
    def enable_all(self):
        for motor in self.motors: motor.enable()
    def disable_all(self):
        self.disables+=1
        if self.fail_disable: raise RuntimeError('injected disable failure')
        for motor in self.motors: motor.disable()
    def shutdown(self): self.events.append((0,'shutdown',()))
    def close_bus(self): self.events.append((0,'close_bus',()))
    def close(self): self.events.append((0,'free_controller',()));self.closed=True


def make_driver(*,controller=None,config=None):
    config=config or load_configuration()
    controller=controller or HandleController()
    opened=[]
    def factory(channel,baud): opened.append((channel,baud));return controller
    driver=SdkMotorDriver(config,controller_factory=factory,sleep=lambda _:None)
    return driver,controller,opened


def wait_until(predicate,timeout=1.):
    end=time.monotonic()+timeout
    while not predicate() and time.monotonic()<end: time.sleep(.001)
    assert predicate()


def make_local_manager(monkeypatch):
    opened=[]
    def factory(channel,baud):
        bus=HandleController();opened.append(bus);return bus
    monkeypatch.setattr(SdkMotorDriver,'_open_controller',staticmethod(factory))
    manager=HardwareManager()
    manager._backend.driver._sleep=lambda _:None
    return manager,opened


def test_driver_constructs_without_open_and_owns_seven_handles():
    driver,bus,opened=make_driver()
    assert not opened and not driver.connected
    driver.connect();driver.connect()
    assert opened==[('/dev/ttyACM0',921600)]
    assert len(bus.motors)==7 and len(driver.controllers())==1
    assert driver.motor('gripper')._lock is driver.motor('joint1')._lock
    port=driver.motor('joint1')
    driver.close();driver.close()
    assert all(m.closed for m in bus.motors) and bus.closed
    methods=[e[1] for e in bus.events]
    assert methods.index('free_controller')>max(i for i,m in enumerate(methods) if m=='free')
    with pytest.raises(RuntimeError,match='closed'): port.request_feedback()


def test_partial_construction_releases_all_acquired_handles():
    driver,bus,_=make_driver();bus.fail_add=4
    with pytest.raises(RuntimeError,match='add failure'): driver.connect()
    assert not driver.connected and len(bus.motors)==3
    assert bus.closed and all(m.closed for m in bus.motors)


def test_failed_position_batch_has_receipt_and_stops_remaining_writes():
    driver,bus,_=make_driver();driver.connect();bus.failure=(3,'position')
    command=PositionCommand((0.,)*6,(.3,)*6,0)
    with pytest.raises(CommandTransportError) as error: driver.send_positions(command)
    assert error.value.receipt.sent==('joint1','joint2')
    assert error.value.receipt.failed=='joint3'
    assert error.value.receipt.not_attempted==('joint4','joint5','joint6')
    assert [n for n,m,_ in bus.events if m=='position']==[1,2,3]
    driver.close()


@pytest.mark.parametrize('positions,velocities', [
    ((0.,)*5,(.3,)*6),((0.,0.,.03,0.,0.,0.),(.3,)*6),
    ((float('nan'),)*6,(.3,)*6),((0.,)*6,(6.,)*6),((0.,)*6,(0.,)*6),
])
def test_invalid_batch_is_rejected_before_any_write(positions,velocities):
    driver,bus,_=make_driver();driver.connect()
    with pytest.raises(ValueError): driver.send_positions(PositionCommand(positions,velocities,0))
    assert not any(m=='position' for _,m,_ in bus.events)
    driver.close()


def test_register_failure_aborts_mode_and_enable_path(monkeypatch):
    manager,opened=make_local_manager(monkeypatch);manager.connect()
    opened[0].failure=(2,'register')
    with pytest.raises(RuntimeError,match='enable failed'): manager.enable()
    bus=opened[0]
    assert not manager._lifecycle.enabled and not manager.control_loop_active
    assert manager.mode=='unknown'
    assert not any(n>2 and m in ('mode','register') for n,m,_ in bus.events)
    assert not any(m=='enable' for _,m,_ in bus.events)
    manager.shutdown()


@pytest.mark.parametrize("mode", ["mit", "vel", "pos_vel"])
def test_disabled_mode_settle_allows_new_verified_feedback(monkeypatch, mode):
    """缓慢的模式操作不能阻塞共享反馈缓存。"""
    manager, _ = make_local_manager(monkeypatch)
    manager.connect()
    driver = manager._backend.driver
    workers = []
    failures = []
    previous = dict(manager._feedback_component._verified_feedback_by_label)

    def settle(_duration):
        manager.refresh_feedback_if_due(force=True)
        refreshed = threading.Event()

        def acquire_feedback():
            try:
                # ROS 发布器在释放发布锁前读取该快照；此处阻塞也会阻碍后续刷新。
                assert manager.get_cached_joint_sample()[3] == [0] * 6
                assert manager.get_cached_gripper_sample()[3] == 0
                assert manager.get_feedback_status() == ([0] * 6, [])
            except Exception as exc:
                failures.append(exc)
            finally:
                refreshed.set()

        worker = threading.Thread(target=acquire_feedback)
        workers.append(worker)
        worker.start()
        assert refreshed.wait(.5), "mode settle blocked feedback on the bus lock"
        assert not failures
        for label, sample in manager._feedback_component._verified_feedback_by_label.items():
            assert sample.sequence > previous[label].sequence
            assert sample.observed_at > previous[label].observed_at
        previous.update(manager._feedback_component._verified_feedback_by_label)
        assert manager.get_feedback_status() == ([0] * 6, [])
        assert manager.get_cached_gripper_sample()[3] == 0
        assert not manager._lifecycle.enabled and not manager.control_loop_active

    driver._sleep = settle
    try:
        assert manager.set_mode(mode)
        assert manager.mode == mode
        assert workers, "mode transition did not exercise a settling delay"
    finally:
        driver._sleep = lambda _: None
        for worker in workers:
            worker.join(timeout=1.)
        manager.shutdown()


def test_mode_settle_allows_feedback_but_serializes_command_batches():
    driver, bus, _ = make_driver()
    driver.connect()
    settling, release, attempted, sent, refreshed = (
        threading.Event() for _ in range(5)
    )
    failures = []

    def settle(_duration):
        settling.set()
        if not release.wait(2.):
            raise RuntimeError("test did not release mode settle")

    def change_mode():
        try:
            driver.set_mode("mit")
        except Exception as exc:
            failures.append(exc)

    def send():
        attempted.set()
        driver.send_positions(PositionCommand((0.,) * 6, (.3,) * 6, 0))
        sent.set()

    def feedback():
        driver.motor("joint1").request_feedback()
        driver.controllers()[0].poll_feedback_once()
        assert driver.motor("joint1").get_state_with_sequence()[1] > 0
        refreshed.set()

    driver._sleep = settle
    mode_worker = threading.Thread(target=change_mode)
    sender = threading.Thread(target=send)
    reader = threading.Thread(target=feedback)
    try:
        mode_worker.start()
        assert settling.wait(1.)
        sender.start()
        assert attempted.wait(1.)
        reader.start()
        assert refreshed.wait(.5), "feedback blocked for entire mode transition"
        assert not sent.is_set()
        assert not any(method == "position" for _, method, _ in bus.events)
    finally:
        release.set()
        for worker in (mode_worker, sender, reader):
            if worker.ident is not None:
                worker.join(timeout=2.)
        driver._sleep = lambda _: None
        driver.close()
    assert not failures and sent.is_set()


@pytest.mark.parametrize('enabled', [True, False])
def test_transition_settle_allows_verified_feedback_refresh(monkeypatch, enabled):
    manager, _ = make_local_manager(monkeypatch)
    manager.connect()
    driver = manager._backend.driver
    if not enabled:
        manager._backend.call(driver.enable)
    previous = dict(manager._feedback_component._verified_feedback_by_label)
    failures = []
    workers = []

    def settle(_duration):
        manager.refresh_feedback_if_due(force=True)
        done = threading.Event()

        def refresh():
            try:
                manager.get_cached_joint_sample()
            except Exception as exc:
                failures.append(exc)
            finally:
                done.set()

        worker = threading.Thread(target=refresh)
        workers.append(worker)
        worker.start()
        assert done.wait(.5), 'transition blocked verified feedback refresh'
        assert not failures
        for label, sample in manager._feedback_component._verified_feedback_by_label.items():
            assert sample.sequence > previous[label].sequence
            assert sample.observed_at > previous[label].observed_at
        previous.update(manager._feedback_component._verified_feedback_by_label)

    driver._sleep = settle
    try:
        manager._backend.call(driver.enable if enabled else driver.disable)
        assert workers
    finally:
        driver._sleep = lambda _: None
        for worker in workers:
            worker.join(timeout=1.)
        if enabled:
            manager._backend.call(driver.disable)
        manager.shutdown()


@pytest.mark.parametrize('enabled', [True, False])
@pytest.mark.parametrize('delay', ['settle', 'retry'])
def test_enable_transition_wait_allows_feedback_and_excludes_commands(enabled, delay):
    driver, bus, _ = make_driver()
    driver.connect()
    if not enabled:
        driver.enable()
    bus.feedback = delay != 'retry'
    waiting, release, refreshed, sent, attempted = (threading.Event() for _ in range(5))
    errors = []

    def sleep(duration):
        if (delay == 'settle' and duration == driver.config.timing.command_motor_delay
                or delay == 'retry' and duration == driver.config.timing.status_interval):
            waiting.set()
            if not release.wait(2.):
                raise RuntimeError('wait was not released')
            bus.feedback = True

    def transition():
        try:
            (driver.enable if enabled else driver.disable)()
        except Exception as exc:
            errors.append(exc)

    def feedback():
        with driver.bus_lock:
            driver.motor('joint1').request_feedback()
            driver.controllers()[0].poll_feedback_once()
        refreshed.set()

    def command():
        attempted.set()
        driver.send_positions(PositionCommand((0.,) * 6, (.3,) * 6, 0))
        sent.set()

    driver._sleep = sleep
    workers = [threading.Thread(target=transition), threading.Thread(target=feedback),
               threading.Thread(target=command)]
    try:
        workers[0].start()
        assert waiting.wait(1.)
        workers[1].start()
        workers[2].start()
        assert attempted.wait(1.)
        assert refreshed.wait(.5), 'transition wait held the feedback bus lock'
        assert not sent.is_set()
    finally:
        release.set()
        for worker in workers:
            if worker.ident is not None:
                worker.join(timeout=2.)
        driver._sleep = lambda _: None
        driver.close()
    assert not errors and sent.is_set()


def test_driver_disable_does_not_stop_runtime_and_demands_new_status():
    driver,bus,_=make_driver();driver.connect();driver.enable()
    before=bus.motors[0].sequence
    driver.disable()
    assert bus.motors[0].sequence>before and bus.motors[0].state.status_code==0
    bus.feedback=False
    with pytest.raises(RuntimeError,match='not confirmed'): driver.disable()
    driver.close()


def test_zero_rejects_stale_disabled_cache_before_zero_packet():
    driver,bus,_=make_driver();driver.connect();bus.feedback=False
    with pytest.raises(RuntimeError,match='not confirmed'): driver.set_zero(('joint2',))
    assert not any(m=='zero' for _,m,_ in bus.events)
    driver.close()


def test_execution_cancel_rejects_inflight_plan_and_old_samples():
    execution=ArmExecution();execution.running=True
    generation=execution.generation
    assert execution.submit([(0.,)*6,(.1,-.1,-.1,.1,.1,.1)],[0.,1.],generation,now=10.)
    target=execution.sample(10.)
    execution.acknowledge(target)
    assert execution.progress==.5
    execution.cancel()
    assert not execution.submit([(0.,)*6],[0.],generation,now=10.)
    execution.acknowledge(target)
    assert not execution.moving
    assert execution.target()==pytest.approx((0.,)*6)
    execution.set_speed_override((.3,)*6)
    output=ArmPositionControl((5.,5.,5.,3.,3.,3.)).command(execution.sample(12.))
    assert output.velocity_limits==(.3,)*6


def test_target_is_copied_and_trajectory_advances_only_after_send_ack():
    execution=ArmExecution();execution.running=True
    source=np.zeros(6);execution.set_target(source);source[0]=99
    assert execution.target()[0]==0
    generation=execution.generation
    assert execution.submit([(0.,)*6,(.1,-.1,-.1,.1,.1,.1)],[0.,.1],generation,now=10.)
    target=execution.sample(10.2)
    assert execution.moving and execution.progress==0.
    execution.acknowledge(target)
    assert not execution.moving and execution.progress==1.


def test_runtime_self_stop_and_bounded_queue_never_start_second_writer():
    config = load_configuration()
    driver, _, _ = make_driver(config=config)
    runtime = ControllerRuntime(config, None, driver=driver)
    finished = threading.Event()
    def stop_from_callback(*_):
        runtime.stop_control_loop()
        finished.set()
    runtime.start_control_loop(stop_from_callback)
    assert finished.wait(1.)
    blocked, release = threading.Event(), threading.Event()
    def callback(*_):
        blocked.set()
        release.wait(1.)
    runtime.start_control_loop(callback)
    assert blocked.wait(1.)
    old = runtime.control_thread
    try:
        with pytest.raises(RuntimeError, match='timed out'):
            runtime.call(lambda: None, ttl=.03)
        assert runtime.control_thread is old
    finally:
        release.set()
        runtime.stop_control_loop()
        runtime.close_worker()


def test_real_manager_local_path_default_disabled_enable_recover_reconnect(monkeypatch):
    manager,opened=make_local_manager(monkeypatch)
    assert not opened and not manager._lifecycle.enabled and not manager._lifecycle.connected
    assert manager.get_joint_status_codes()==[255]*6
    with pytest.raises(RuntimeError,match='disabled|not connected'): manager.set_gripper_target(.01)
    manager.connect()
    bus=opened[0]
    assert not manager._lifecycle.enabled and manager.get_joint_status_codes()==[0]*6
    assert len(opened)==1 and len(bus.motors)==7
    manager.enable()
    wait_until(lambda:any(m=='position' for _,m,_ in bus.events))
    assert manager._lifecycle.enabled and manager.control_loop_active
    with pytest.raises(RuntimeError,match='owns bus'): manager.refresh_feedback_if_due(force=True)
    manager.set_joint_target((.1,-.1,-.1,0.,0.,0.))
    wait_until(lambda:bus.motors[0].state.pos==.1)
    manager.stop_active_motion()
    assert manager._lifecycle.enabled and manager._lifecycle.state=='ENABLED_HOLD'
    manager.disable()
    assert not manager._lifecycle.enabled and not manager.control_loop_active
    manager.shutdown()
    assert bus.closed and not manager._lifecycle.connected
    manager.connect()
    assert len(opened)==2 and not manager._lifecycle.enabled
    assert manager.get_joint_status_codes()==[0]*6
    manager.shutdown()


@pytest.mark.parametrize('disable_fails',[False,True])
def test_local_writer_partial_send_cancels_and_protects_shared_bus(monkeypatch,disable_fails):
    manager,opened=make_local_manager(monkeypatch);manager.connect()
    bus=opened[0];bus.failure=(3,'position');bus.fail_disable=disable_fails
    manager.enable()
    wait_until(lambda:not manager.control_loop_active)
    assert not manager._backend.trajectory_moving and manager._lifecycle.state=='DISABLING'
    assert manager._lifecycle.enabled  # output state is unknown until disabled feedback is verified
    assert bus.disables==1
    assert [n for n,m,_ in bus.events if m=='position']==[1,2,3]
    assert 'COMMAND_PROTECTIVE_DISABLE' in manager.error_codes
    if disable_fails: assert any('PROTECTIVE_DISABLE_FAILED' in code for code in manager.error_codes)
    bus.failure=None;bus.fail_disable=False
    manager.disable();manager.shutdown()


@pytest.mark.parametrize('disable_fails', [False, True])
def test_persistent_stale_feedback_requests_protection_once_until_verified_disable(monkeypatch, caplog, disable_fails):
    manager, opened = make_local_manager(monkeypatch)
    manager.connect(); manager.enable()
    bus = opened[0]
    bus.fail_disable = disable_fails
    bus.feedback = False
    try:
        wait_until(lambda: bus.disables == 1)
        # 在真实唯一设备线程推进多个空闲周期，不让一次暂时恢复掩盖重复失能。
        manager._backend.call(lambda: [manager._runtime_feedback_tick() for _ in range(30)])
        assert bus.disables == 1
        assert manager._lifecycle.enabled and manager._lifecycle.state == 'DISABLING'
        assert not manager.control_loop_active
        assert sum('protective disable requested' in record.message for record in caplog.records) == 1
        if disable_fails:
            assert any('PROTECTIVE_DISABLE_FAILED' in code for code in manager.error_codes)
        bus.feedback = True
        bus.fail_disable = False
        manager._backend.call(lambda: manager.refresh_feedback_if_due(force=True))
        assert manager._command_guard.protection is not None
        with pytest.raises(RuntimeError, match='verify disable'):
            manager.enable()
        manager.disable()
        assert manager._command_guard.protection is None
        manager.enable()
        before = bus.disables
        bus.feedback = False
        wait_until(lambda: bus.disables == before + 1)
        assert manager._lifecycle.state == 'DISABLING'
    finally:
        bus.feedback = True; bus.fail_disable = False
        manager.disable(); manager.shutdown()


def test_protection_cleanup_failure_cannot_skip_disable(monkeypatch):
    manager, opened = make_local_manager(monkeypatch)
    manager.connect(); manager.enable()
    bus = opened[0]
    cancel = manager._backend.cancel_trajectory
    def fail():
        raise RuntimeError('injected cancellation failure')
    monkeypatch.setattr(manager._backend, 'cancel_trajectory', fail)
    try:
        manager._backend.call(lambda: manager._protective_disable_from_hardware_loop('fault'))
        assert bus.disables == 1 and manager._lifecycle.state == 'DISABLING'
        assert not manager.control_loop_active
        assert any('PROTECTIVE_PREPARE_FAILED' in code for code in manager.error_codes)
    finally:
        monkeypatch.setattr(manager._backend, 'cancel_trajectory', cancel)
        manager.disable(); manager.shutdown()


@pytest.mark.parametrize('resource', ['arm', 'gripper'])
@pytest.mark.parametrize('field,value', [('pos', float('nan')), ('pos', 99.), ('vel', 99.), ('kp', 501.), ('tau', 99.)])
def test_invalid_passthrough_cannot_preempt_active_task(monkeypatch, resource, field, value):
    manager, opened = make_local_manager(monkeypatch)
    manager.connect(); manager.enable()
    bus = opened[0]
    command = SimpleNamespace(mode=1 if resource == 'arm' else 0,
        **{f'use_{key}': True for key in ('pos', 'vel', 'kp', 'kd', 'tau', 'vlim')},
        pos=0., vel=0., kp=1., kd=.1, tau=0., vlim=.3)
    # MIT 增益与力矩只在 MIT 命令中约束；无效请求仍不得取消正在运行的轨迹。
    if resource == 'arm' and field in ('kp', 'tau'):
        command.mode = 0
    setattr(command, field, value)
    try:
        if resource == 'arm':
            task = manager.submit_arm_trajectory(manager.joint_names,
                [(0.,)*6, (.1,-.1,-.1,0.,0.,0.)], [0., 10.])
            generation = manager._backend.target_generation
            count = len(bus.events)
            with pytest.raises(ValueError):
                manager.send_joint_motor_cmd('joint1', command, arbitration='preempt')
            assert manager._backend.target_generation == generation
            assert not task.event.is_set()
        else:
            manager.set_gripper_target(.04)
            generation = manager._gripper_execution._generation
            count = len(bus.events)
            with pytest.raises(ValueError):
                manager.send_gripper_motor_cmd(command, arbitration='preempt')
            assert manager._gripper_execution._generation == generation
        assert not any(method in ('mode', 'disable') for _, method, _ in bus.events[count:])
    finally:
        manager.disable(); manager.shutdown()


def test_valid_passthrough_preempt_uses_feedback_after_stop_confirmation(monkeypatch):
    manager, opened = make_local_manager(monkeypatch)
    manager.connect(); manager.enable()
    bus = opened[0]
    task = manager.submit_arm_trajectory(manager.joint_names,
        [(0.,)*6, (.1,-.1,-.1,0.,0.,0.)], [0., 10.])
    command = SimpleNamespace(mode=1,
        **{f'use_{key}': False for key in ('pos', 'vel', 'kp', 'kd', 'tau', 'vlim')})
    original = manager._confirm_arm_hold
    def stopped(current):
        original(current)
        # 模拟停止确认后才收到的位置变化；未显式指定的位置不得沿用抢占前快照。
        bus.motors[0].state.pos = .012
        manager.refresh_feedback_if_due(force=True)
    monkeypatch.setattr(manager, '_confirm_arm_hold', stopped)
    try:
        manager.send_joint_motor_cmd('joint1', command, arbitration='preempt')
        assert task.event.is_set() and manager.arm_execution_status(task).state == 'canceled'
        sent = [args for number, method, args in bus.events if number == 1 and method == 'position']
        assert sent[-1][0] == pytest.approx(.012)
        assert manager._lifecycle.execution == 'LOWLEVEL_STREAMING'
        assert manager._lifecycle.enabled
    finally:
        manager.disable(); manager.shutdown()


def test_fk_canonical_matches_legacy_at_identical_inputs():
    root=Path(__file__).resolve().parents[1]
    spec=importlib.util.spec_from_file_location('controller_fk_probe',root/'scripts/check_controller_fk.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    result=module.compare()
    assert result['max_position_error_m']<1e-12
    assert result['max_rotation_error_rad']<1e-12
    assert result['canonical_joints'][2]=='joint3' and result['legacy_joints'][2]=='join3'


def test_installed_motorbridge_with_seven_virtual_handles():
    import os
    import pty
    motorbridge=pytest.importorskip('motorbridge')
    from test_motorbridge_transport_safety import packet
    master,slave=pty.openpty()
    config=replace(load_configuration(),channel=os.ttyname(slave))
    driver=SdkMotorDriver(config,sleep=lambda _:None)
    try:
        driver.connect()
        for i in range(1,8): os.write(master,packet(can_id=0x10+i,motor_id=i))
        def all_arrived():
            driver.controllers()[0].poll_feedback_once()
            return all(driver.motor(name).get_state_with_sequence()[1]==1 for name in (*[j.name for j in config.joints],'gripper'))
        wait_until(all_arrived)
        assert len(driver.controllers())==1
        driver.send_positions(PositionCommand((0.,)*6,(.3,)*6,0))
        assert driver.last_receipt.success
    finally:
        driver.close();os.close(slave);os.close(master)


def test_constructor_feedback_and_fk_need_no_legacy_sdk(monkeypatch):
    import builtins
    original=builtins.__import__
    def reject_sdk(name,*args,**kwargs):
        if name.startswith('reBotArm_control_py'): raise AssertionError('legacy device dependency')
        return original(name,*args,**kwargs)
    monkeypatch.setattr(builtins,'__import__',reject_sdk)
    manager,opened=make_local_manager(monkeypatch)
    assert not opened
    manager.connect()
    pose=manager.current_pose()
    assert np.isfinite([pose.position.x,pose.position.y,pose.position.z]).all()
    manager.shutdown()


def test_local_feedback_error_then_new_frame_recovers_without_query_effects(monkeypatch):
    manager,opened=make_local_manager(monkeypatch);manager.connect()
    bus=opened[0]
    bus.motors[2].state.pos=.03
    with pytest.raises(RuntimeError,match='joint3'): manager._backend.call(lambda: manager.refresh_feedback_if_due(force=True))
    assert manager.get_joint_status_codes()==[255]*6
    assert any('INVALID_COORDINATE' in code for code in manager.error_codes)
    before=(dict(manager._feedback_component._verified_feedback_by_label),dict(manager._feedback_component._feedback_error_by_label),len(bus.events))
    for _ in range(10): manager.get_feedback_status()
    assert before==(manager._feedback_component._verified_feedback_by_label,manager._feedback_component._feedback_error_by_label,len(bus.events))
    bus.motors[2].state.pos=.019
    manager._backend.call(lambda: manager.refresh_feedback_if_due(force=True))
    assert manager.get_joint_status_codes()==[0]*6
    assert not any('INVALID_COORDINATE' in code for code in manager.error_codes)
    bus.motors[1].state.status_code=11
    manager._backend.call(lambda: manager.refresh_feedback_if_due(force=True))
    assert manager.get_joint_status_codes()==[0,11,0,0,0,0]
    bus.motors[1].state.status_code=0
    manager.shutdown()


def test_shutdown_release_failure_does_not_claim_disabled(monkeypatch):
    manager,opened=make_local_manager(monkeypatch);manager.connect()
    opened[0].feedback=False
    with pytest.raises(RuntimeError,match='shutdown'): manager.shutdown()
    assert manager._lifecycle.connected and manager._lifecycle.enabled
    assert manager._lifecycle.state=='DISABLING' and not opened[0].closed
    assert any('SHUTDOWN_STATE_UNCONFIRMED' in code for code in manager.error_codes)
    opened[0].feedback=True
    manager.disable(); manager.shutdown()


def test_shutdown_rejects_enabled_arm_without_sending_disable(monkeypatch):
    manager, opened = make_local_manager(monkeypatch)
    manager.connect(); manager.enable()
    bus = opened[0]
    before = bus.disables
    with pytest.raises(RuntimeError, match='explicit disable'):
        manager.shutdown()
    assert manager._lifecycle.connected and manager._lifecycle.enabled and manager.control_loop_active
    assert bus.disables == before and not bus.closed
    manager.disable(); manager.shutdown()


def test_mode_failure_is_protective_and_motion_gate_stays_closed(monkeypatch):
    manager,opened=make_local_manager(monkeypatch);manager.connect();manager.enable()
    opened[0].failure=(2,'mode')
    with pytest.raises(CommandTransportError): manager.set_mode('mit')
    assert manager._lifecycle.state=='DISABLING'
    manager.set_state_machine('IDLE')
    assert manager._lifecycle.state=='DISABLING'
    for call in (lambda:manager.set_joint_target((0.,)*6),lambda:manager.set_gripper_target(.01),manager.enable):
        with pytest.raises(RuntimeError): call()
    opened[0].failure=None
    manager.disable();manager.shutdown()


def test_pure_math_uses_canonical_model_and_cannot_write_hardware(monkeypatch):
    from rebotarm_motion.pose_kinematics import PoseKinematics
    from rebotarm_motion.cartesian_trajectory import plan_cartesian
    manager, opened = make_local_manager(monkeypatch)
    kinematics = PoseKinematics()
    q = np.array((.2,-.8,-1.1,.3,-.2,.4))
    target = kinematics.pose(q)
    assert kinematics.solve(target, q).q == pytest.approx(q)
    path, duration = plan_cartesian(kinematics, q, target, .1)
    assert path[-1] == pytest.approx(q) and duration == .1
    assert not opened
    assert not hasattr(manager, 'move_to_pose_path')


def test_initial_gravity_send_failure_protects_instead_of_resuming_normal_output(monkeypatch):
    manager,opened=make_local_manager(monkeypatch);manager.connect();manager.enable()
    bus=opened[0];manager._stop_control_loop()
    bus.events.clear();bus.failure=(3,'mit')
    with pytest.raises(RuntimeError,match='joint3'): manager.start_gravity_compensation()
    assert manager._lifecycle.state=='DISABLING' and not manager.control_loop_active
    assert [n for n,m,_ in bus.events if m=='mit']==[1,2,3]
    assert not any(m=='position' for _,m,_ in bus.events)
    assert manager._backend.driver.last_receipt.sent==('joint1','joint2')
    bus.failure=None;manager.disable();manager.shutdown()


def test_disable_writer_stop_failure_keeps_enabled_state(monkeypatch):
    manager, opened = make_local_manager(monkeypatch)
    manager.connect(); manager.enable()
    original = manager._stop_control_loop
    monkeypatch.setattr(manager, '_stop_control_loop', lambda: (_ for _ in ()).throw(RuntimeError('join failed')))
    with pytest.raises(RuntimeError, match='join failed'):
        manager.disable()
    assert manager._lifecycle.enabled and manager._lifecycle.state == 'DISABLING'
    assert 'DISABLE_VERIFICATION_FAILED' in manager.error_codes
    monkeypatch.setattr(manager, '_stop_control_loop', original)
    manager.disable(); manager.shutdown()


def test_safe_home_completion_switches_to_hold_and_clears_task_speed(monkeypatch):
    manager, _ = make_local_manager(monkeypatch)
    manager.connect(); manager.enable()
    goal = (0.1, -0.1, -0.1, 0., 0., 0.)
    assert manager.safe_home(goal, timeout_sec=1.) == pytest.approx(goal, abs=.02)
    assert manager._lifecycle.enabled and manager._lifecycle.state == 'ENABLED_HOLD'
    assert manager._backend.execution.sample(time.monotonic()).velocity_override is None
    manager.disable(); manager.shutdown()


def test_safe_home_supersession_does_not_rewrite_new_target(monkeypatch):
    manager, opened = make_local_manager(monkeypatch)
    manager.connect(); manager.enable()
    successor = (0.2, -0.2, -0.2, 0., 0., 0.)
    # 假电机不跟随，确保原任务仍在等待；替换目标后只允许新目标保留。
    monkeypatch.setattr(opened[0].motors[0], 'send_pos_vel', lambda *_: None)
    task = manager.submit_arm_trajectory(manager.joint_names,
        [(0.,)*6, (.1,-.1,-.1,0.,0.,0.)], [0., .02])
    manager.set_joint_target(successor)
    assert task.event.wait(1.)
    assert manager.arm_execution_status(task).state == 'aborted'
    assert manager.joint_target() == pytest.approx(successor)
    assert manager._backend.execution.sample(time.monotonic()).velocity_override is None
    manager.disable(); manager.shutdown()

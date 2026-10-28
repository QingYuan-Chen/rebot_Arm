"""故障样本以设备操作及安全结果对照，不以 Python 私有方法名作跨语言要求。"""
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from rebotarmcontroller.sdk_driver import CommandTransportError
from rebotarmcontroller.controller_contracts import CommandReceipt
from test_lifecycle_transactions import lifecycle
from test_controller_local_runtime import make_local_manager
from test_hardware_manager_p0_safety import make_manager, configure_gravity_compensation_test

DATA = json.loads((Path(__file__).parent / 'fixtures/controller_fault_reference_v1.json').read_text())
OPERATIONS = {
    'connect': 'open_device', 'target': 'set_hold_target', 'mode_pos_vel': 'set_mode_pos_vel',
    'enable': 'enable_arm', 'disable': 'disable_arm', 'release': 'release_device',
}


@pytest.mark.parametrize('case', DATA['lifecycle_faults'], ids=lambda case: case['name'])
def test_lifecycle_faults_have_portable_device_results(case):
    owner, device = lifecycle()
    owner.connect()
    if case['initial'] == 'enabled':
        owner.enable()
    device.calls.clear()
    original = device.record
    faults = {
        'enable_feedback': ('feedback_1',),
        'enable_and_disable_write': ('enable', 'disable'),
        'disabled_feedback': ('feedback_0',),
        'disable_write': ('disable',),
    }[case['fault']]
    def record(name):
        original(name)
        if name in faults:
            raise RuntimeError('injected unavailable device result')
    device.record = record
    with pytest.raises(RuntimeError):
        getattr(owner, case['request'])()
    assert owner.connected
    assert owner.state == case['state'] and owner.enabled == case['enabled']
    assert [OPERATIONS[name] for name in device.calls if name in OPERATIONS] == case['operations']
    if case['error_code']:
        assert any(code.startswith(case['error_code']) for code in owner.errors)
    if case['state'] == 'DISABLING':
        before = list(device.calls)
        with pytest.raises(RuntimeError):
            owner.enable()
        assert before == device.calls  # 未确认失能不能重新使能。


def test_lifecycle_success_uses_semantic_operations():
    owner, device = lifecycle()
    for step in DATA['lifecycle_success']:
        device.calls.clear()
        getattr(owner, step['request'])()
        assert owner.state == step['state'] and owner.enabled == step['enabled']
        assert [OPERATIONS[name] for name in device.calls if name in OPERATIONS] == step['operations']


@pytest.mark.parametrize('case', DATA['protection'], ids=lambda case: case['name'])
def test_protection_latch_and_explicit_recovery(case, monkeypatch):
    manager, opened = make_local_manager(monkeypatch)
    manager.connect()
    manager.enable()
    bus = opened[0]
    baseline = bus.disables
    bus.fail_disable = case['disable_write_fails']
    try:
        for step in DATA['protection_steps']:
            request = step['request']
            if request == 'protect':
                manager._backend.call(lambda: manager.protection.protect('injected fault'))
            elif request == 'feedback_recovers':
                bus.fail_disable = False
                manager._backend.call(lambda: manager.feedback.refresh_feedback_if_due(force=True))
            elif request == 'enable_rejected':
                with pytest.raises(RuntimeError):
                    manager.enable()
            else:
                getattr(manager, request)()
            assert manager.lifecycle_state == step['state'] and manager.enabled == step['enabled']
            assert bus.disables - baseline == step['disable_writes']
            assert not manager.control_loop_active if step['state'] == 'DISABLING' else True
            if request == 'protect':
                assert manager.protection.decision.action.value == case['decision']
            if request == 'disable':
                assert manager.protection.decision is None
    finally:
        bus.fail_disable = False
        manager.disable()
        manager.shutdown()


@pytest.mark.parametrize('case', DATA['passthrough'], ids=lambda case: case['name'])
def test_passthrough_arbitration_result(case, monkeypatch):
    manager, opened = make_local_manager(monkeypatch)
    manager.connect()
    manager.enable()
    bus = opened[0]
    task = manager.submit_arm_trajectory(manager.joint_names,
        [(0.,) * 6, (.01, -.01, -.01, 0., 0., 0.)], [0., 10.])
    generation = manager._backend.target_generation
    command = SimpleNamespace(mode=1, pos=case['position'], vel=0., kp=1., kd=.1, tau=0., vlim=.3,
        **{f'use_{key}': True for key in ('pos', 'vel', 'kp', 'kd', 'tau', 'vlim')})
    before = bus.disables
    try:
        if case['accepted']:
            manager.send_joint_motor_cmd('joint1', command, arbitration=case['arbitration'])
            assert manager.state_machine == 'LOWLEVEL_STREAMING'
            assert manager._backend.target_generation != generation
            writes = [args for number, name, args in bus.events if number == 1 and name == 'position']
            assert writes[-1][0] == pytest.approx(case['position'], abs=DATA['absolute_tolerance'])
        else:
            with pytest.raises((RuntimeError, ValueError)):
                manager.send_joint_motor_cmd('joint1', command, arbitration=case['arbitration'])
            assert manager._backend.target_generation == generation
        assert manager.arm_execution_status(task).state == case['task_state']
        assert manager.enabled and bus.disables == before
    finally:
        manager.disable()
        manager.shutdown()


@pytest.mark.parametrize('case', DATA['gravity'], ids=lambda case: case['name'])
def test_gravity_handoff_and_fault_decisions(case, monkeypatch):
    values = DATA['gravity_input']
    manager = make_manager()
    manager._lifecycle.set_state('ENABLED_HOLD')
    q = np.array(values['positions'])
    commands = configure_gravity_compensation_test(
        manager, positions=q, velocities=np.array(values['velocities']))
    # 固定模型输入使样本不受 Pinocchio 版本或真实负载影响；生产控制律照常执行。
    manager.gravity._gc_compute_generalized_gravity = lambda q: np.array(values['gravity_torque'])
    disables = []
    manager.test_arm._ctrl_map['fake'].disable_all = lambda: disables.append(True)
    if case['fault'] == 'mode_rejected':
        manager.test_arm.mode_mit = lambda **kw: False
        with pytest.raises(RuntimeError):
            manager.start_gravity_compensation()
    else:
        manager.start_gravity_compensation()
        if case['fault'] == 'none':
            manager.stop_gravity_compensation()
        else:
            monkeypatch.setattr(manager.feedback, 'refresh_feedback_if_due', lambda **kw: False)
            def fail(**kw):
                if case['fault'] == 'transport':
                    raise CommandTransportError(CommandReceipt((), 'joint1', tuple(manager.joint_names[1:])))
                raise RuntimeError('injected model failure')
            if case['fault'] == 'transport':
                # 回执由设备批次返回，协调组件不得将其当作健康计算失败恢复输出。
                # 重力组件通过自己的运行时后端发送 MIT 批次；FakeArm 只是反馈
                # 夹具，不应伪装成第二个发送接口。
                monkeypatch.setattr(manager.gravity._backend, 'send_mit_batch', fail)
                del manager.gravity._send_gravity_mit
            else:
                monkeypatch.setattr(manager.gravity, '_gc_compute_generalized_gravity', fail)
            manager.gravity._gravity_hardware_tick(manager.test_arm, values['period'])
    assert manager.enabled == case['enabled'] and manager.lifecycle_state == case['state']
    assert manager.gravity_compensation_active() == case['active']
    assert len(disables) == case['disable_writes']
    if case['normal_control']:
        assert manager.control_loop_active
        np.testing.assert_allclose(manager.joint_target(), q, atol=DATA['absolute_tolerance'], rtol=0)
    decision = manager.protection.decision or manager.gravity.gravity_compensation_diagnostics()['safety_decision']
    assert (None if decision is None else decision.action.value) == case['decision']
    if commands:
        np.testing.assert_allclose(commands[-1]['pos'], q, atol=DATA['absolute_tolerance'], rtol=0)

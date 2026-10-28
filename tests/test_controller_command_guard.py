"""保护锁存和透传准入的纯值对照；不访问设备。"""
from dataclasses import replace

import pytest

from rebotarmcontroller.command_guard import (
    CommandGuard, SafetyAction, passthrough_decision,
)


@pytest.mark.parametrize('override', [
    {'pos': float('nan')}, {'pos': 3.}, {'vel': 2.1}, {'vlim': 0.},
    {'kp': 501.}, {'kd': 5.1}, {'tau': 28.}, {'mode': 3}, {'device_mode': 'pos_vel'},
])
def test_invalid_passthrough_has_explicit_reject_decision(override):
    values = dict(pos=0., vel=0., kp=1., kd=1., tau=0., vlim=1.,
                  bounds=(-2.8, 2.8), max_velocity=2., max_effort=27., mode=0,
                  device_mode='mit')
    values.update(override)
    decision = passthrough_decision(**values)
    assert decision.action == SafetyAction.REJECT
    with pytest.raises(ValueError, match='passthrough|JointMotorCmd'):
        decision.require_allowed()


def test_valid_passthrough_and_protection_episode_have_value_contracts():
    decision = passthrough_decision(0., 0., 1., 1., 0., 1., (-2.8, 2.8), 2., 27., 0,
                                    device_mode='mit')
    decision.require_allowed()
    guard = CommandGuard()
    assert guard.begin_protection('source stale', 'FEEDBACK')
    first = guard.protection
    assert first.action == SafetyAction.PROTECTIVE_DISABLE
    assert first.category == 'FEEDBACK'
    assert not guard.begin_protection('another stale reading', 'CONTROLLER')
    assert guard.protection == replace(first)
    guard.confirm_disabled()
    assert guard.begin_protection('new session failure', 'COMMAND')
    assert guard.protection.category == 'COMMAND'

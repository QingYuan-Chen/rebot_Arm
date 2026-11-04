"""停止状态机的源时间、发送确认和代次约束；不连接真实设备。"""
from rebotarmcontroller.arm.arm_stop import ArmStop
import pytest


def test_stop_requires_successful_send_and_new_stationary_window():
    clock = [10.]
    stop = ArmStop(clock=lambda: clock[0])
    stop.begin([0.] * 6, 3)
    def update(stamp, speed=0., generation=3, enabled=True):
        clock[0] = max(clock[0], stamp)
        return stop.update([0.] * 6, [speed] * 6, stamp,
                           generation=generation, enabled=enabled)
    assert update(10.01).state == 'stopping'
    stop.sent(2)
    assert stop.result.sent_at is None
    clock[0] = 10.
    stop.sent(3)
    assert update(10.).quiet_since is None
    assert update(10.01).quiet_since == 10.01
    clock[0] = 10.01
    assert update(10.01).state == 'stopping'  # 重读缓存不累计静止窗口。
    assert update(10.06, speed=.06).quiet_since is None
    assert update(10.07).state == 'stopping'
    assert update(10.17).state == 'stopping'
    assert update(10.27).state == 'confirmed'


def test_stop_timeout_and_generation_change_never_confirm():
    clock = [10.]
    stop = ArmStop(clock=lambda: clock[0])
    stop.begin([0.] * 6, 1)
    clock[0] = 12.1
    assert stop.update([0.]*6, [0.]*6, 10., generation=1, enabled=True).state == 'timeout'
    stop.begin([0.] * 6, 2)
    stop.sent(2)
    assert stop.update([0.]*6, [0.]*6, 12.2, generation=3, enabled=True).state == 'aborted'
    stop.begin([0.] * 6, 4)
    assert stop.update([0.]*6, [0.]*6, 12.2, generation=4, enabled=False).state == 'aborted'


@pytest.mark.parametrize('residual', [.003, .01])
def test_stop_uses_actual_stationary_window_instead_of_command_error(residual):
    """保持发送和新帧仍必须满足；静止残差不再被解释为停止失败。"""
    clock = [10.]
    stop = ArmStop(clock=lambda: clock[0])
    stop.begin([0.]*6, 1)
    stop.sent(1)
    for k in range(1, 13):
        clock[0] = 10.+k*.02
        result = stop.update([residual]*6, [0.]*6, clock[0],
            generation=1, enabled=True, sequences=[k]*6)
        if k < 11:
            assert result.state == 'stopping'
    assert result.state == 'confirmed'
    assert result.target == (0.,)*6


def test_stop_can_settle_away_from_first_feedback_without_reanchoring_command():
    clock = [10.]
    stop = ArmStop(clock=lambda: clock[0])
    stop.begin([0.]*6, 1)
    stop.sent(1)
    for k in range(1, 22):
        clock[0] = 10.+k*.02
        q = .01*min(k/6., 1.)
        result = stop.update([q]*6, [0.]*6, clock[0],
            generation=1, enabled=True, sequences=[k]*6)
        if k < 11:
            assert result.state == 'stopping'
    assert result.state == 'confirmed'
    assert result.target == (0.,)*6


def test_stop_without_position_gate_still_rejects_velocity_excess():
    clock = [10.]
    stop = ArmStop(clock=lambda: clock[0])
    stop.begin([0.]*6, 1)
    stop.sent(1)
    for k in range(1, 90):
        clock[0] = 10.+k*.02
        result = stop.update([.01]*6, [.06]*6, clock[0],
            generation=1, enabled=True, sequences=[k]*6)
        assert result.state == 'stopping'


def test_confirmed_arm_must_remain_quiet_while_joint_stop_waits_for_gripper():
    clock = [10.]
    stop = ArmStop(clock=lambda: clock[0])
    stop.begin([0.]*6, 1)
    stop.sent(1)
    for k in range(1, 13):
        clock[0] = 10.+k*.02
        stop.update([.003]*6, [0.]*6, clock[0], generation=1, enabled=True, sequences=[k]*6)
    assert stop.result.state == 'confirmed'
    clock[0] += .02
    assert stop.update([.003]*6, [.2]*6, clock[0], generation=1, enabled=True,
        sequences=[13]*6).state == 'stopping'
    clock[0] += .02
    assert stop.update([.003]*6, [0.]*6, clock[0], generation=1, enabled=False,
        sequences=[14]*6).state == 'aborted'

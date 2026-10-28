"""停止状态机的源时间、发送确认和代次约束；不连接真实设备。"""
from rebotarmcontroller.arm_stop import ArmStop


def test_stop_requires_successful_send_and_new_stationary_window():
    clock = [10.]
    stop = ArmStop(clock=lambda: clock[0])
    stop.begin([0.] * 6, 3)
    def update(stamp, speed=0., generation=3, enabled=True):
        return stop.update([0.] * 6, [speed] * 6, stamp,
                           generation=generation, enabled=enabled)
    assert update(10.01).state == 'stopping'
    stop.sent(2)
    assert stop.result.sent_at is None
    stop.sent(3)
    assert update(10.).quiet_since is None
    assert update(10.01).quiet_since == 10.01
    clock[0] = 10.5
    assert update(10.01).state == 'stopping'  # 重读缓存不累计静止窗口。
    assert update(10.06, speed=.06).quiet_since is None
    assert update(10.07).state == 'stopping'
    assert update(10.18).state == 'confirmed'


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

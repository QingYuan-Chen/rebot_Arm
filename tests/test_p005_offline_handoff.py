"""候选系数通过生产周期/驱动发送到假设备; 适配器仅属于测试, 不接入生产.

覆盖真实发送回执和设备线程; 不把此适配器当作未来 FJT 接纳事务已经实现的证明.
"""

from dataclasses import replace
import json
import threading

import pytest

from rebotarmcontroller.arm_execution import ArmExecution, ArmTarget
from rebotarmcontroller.controller_runtime import ControllerRuntime
from rebotarmcontroller.sdk_driver import CommandTransportError
from rebotarm_motion.polynomial_trajectory import JOINT_NAMES, TrajectoryPoint
from rebotarm_motion.trajectory_preparation import (
    PreparedTrajectory, PreparationContext, analyze_trajectory, point_to_point,
)
from test_controller_local_runtime import make_driver
from test_polynomial_trajectory import LIMITS, LOWER, UPPER, collision_pass


class OfflineCurveExecution(ArmExecution):
    """只在测试中把同一份系数接给生产 position_tick, 不实现另一套采样公式."""

    def __init__(self, prepared):
        super().__init__()
        self.prepared = prepared
        self.elapsed = 0.
        self.confirmed = None
        self._generation = prepared.context.generation
        self.stopped = False

    def sample(self, _now):
        q = self._target if self.stopped else self.prepared.curve.sample(self.elapsed)[0]
        return ArmTarget(q, None, self._generation, 0)

    def acknowledge(self, target):
        if target.generation == self._generation:
            self.confirmed = target


def setup_offline():
    driver, bus, _ = make_driver()
    runtime = ControllerRuntime(driver.config, None, driver=driver)
    # 确保所有测试设备调用确实来自生产唯一设备线程.
    original = bus.add_damiao_motor
    def add(*args):
        motor = original(*args)
        record = motor._record
        def checked_record(*values):
            assert threading.current_thread().name == 'rebotarm-hardware'
            return record(*values)
        motor._record = checked_record
        return motor
    bus.add_damiao_motor = add
    runtime.call(driver.connect)
    return runtime, driver, bus


def prepare(runtime, points=None, **kwargs):
    start = (0.,) * 6
    hold = PreparationContext(runtime.requests.generations['device'], runtime.execution.generation,
                              start, start, 10., True, True)
    candidate, report = analyze_trajectory(JOINT_NAMES,
        points or point_to_point(start, (.02, 0., 0., 0., 0., 0.)), lower=LOWER,
        upper=UPPER, limits=LIMITS, context=hold, now=10.,
        collision_check=kwargs.get('checker', collision_pass))
    return candidate, report, hold


def close(runtime, driver):
    runtime.call(driver.close)
    runtime.close_worker()


def test_jittered_time_and_skipped_waypoints_send_same_serialized_curve():
    runtime, driver, bus = setup_offline()
    try:
        # 三个节点, 中间导数非零; 跳周期时不能把所有旧节点补发一遍.
        zero = (0.,) * 6
        points = (TrajectoryPoint(0., zero, zero, zero),
                  TrajectoryPoint(.5, (.01, 0., 0., 0., 0., 0.), (.03, 0., 0., 0., 0., 0.), zero),
                  TrajectoryPoint(1., (.02, 0., 0., 0., 0., 0.), zero, zero))
        prepared, report, hold = prepare(runtime, points)
        assert report['state'] == 'pass'
        received = PreparedTrajectory.from_payload(json.loads(json.dumps(prepared.to_payload())))
        received.require_current(hold, now=10.)
        execution = OfflineCurveExecution(received)
        runtime.execution = execution
        stamps = (0., .003, .017, .49, .91, 1.01)
        for stamp in stamps:
            execution.elapsed = stamp
            runtime.call(lambda: runtime.position_tick(runtime, .002))
            sent = [args[0] for _, method, args in bus.events if method == 'position'][-6:]
            assert sent == pytest.approx(prepared.curve.sample(stamp)[0], abs=1e-12)
        assert len([m for _, m, _ in bus.events if m == 'position']) == 6 * len(stamps)
        assert execution.confirmed.positions == pytest.approx(points[-1].positions)
    finally:
        close(runtime, driver)


def test_partial_write_does_not_confirm_and_cancel_revokes_old_ack():
    runtime, driver, bus = setup_offline()
    try:
        prepared, _, _ = prepare(runtime)
        execution = OfflineCurveExecution(prepared)
        runtime.execution = execution
        execution.elapsed = .2
        bus.failure = (3, 'position')
        with pytest.raises(CommandTransportError) as error:
            runtime.call(lambda: runtime.position_tick(runtime, .002))
        assert error.value.receipt.sent == ('joint1', 'joint2')
        assert error.value.receipt.not_attempted == ('joint4', 'joint5', 'joint6')
        assert execution.confirmed is None
        old = execution.sample(0.)
        execution.cancel()
        execution.stopped = True
        execution._target = (0.,) * 6
        execution.acknowledge(old)
        assert execution.confirmed is None
        bus.failure = None
        runtime.call(lambda: runtime.position_tick(runtime, .002))
        assert execution.confirmed.positions == (0.,) * 6
    finally:
        close(runtime, driver)


@pytest.mark.parametrize('change', ['stop', 'disable', 'reconnect', 'replacement', 'stale', 'busy'])
def test_late_validation_never_sends_after_state_changed(change):
    runtime, driver, bus = setup_offline()
    try:
        prepared, _, hold = prepare(runtime)
        received = PreparedTrajectory.from_payload(json.loads(json.dumps(prepared.to_payload())))
        if change in ('stop', 'replacement'):
            runtime.call(runtime.execution.cancel)
        if change == 'reconnect':
            runtime.requests.invalidate('device')
        current = replace(hold, generation=runtime.execution.generation,
            session=runtime.requests.generations['device'], enabled=change != 'disable',
            busy=change == 'busy', observed_at=9. if change == 'stale' else 10.)
        with pytest.raises(ValueError):
            def dispatch():
                received.require_current(current, now=10.)
                runtime.execution = OfflineCurveExecution(received)
                runtime.position_tick(runtime, .002)
            runtime.call(dispatch)
        assert not any(m == 'position' for _, m, _ in bus.events)
    finally:
        close(runtime, driver)


def test_collision_uncertainty_has_no_device_side_effect_and_old_production_stays_stepwise():
    runtime, driver, bus = setup_offline()
    try:
        candidate, report, _ = prepare(runtime, checker=lambda *_: dict(state='unknown'))
        assert candidate is None and report['state'] == 'unknown'
        assert not any(m == 'position' for _, m, _ in bus.events)
        # 必须保留本批不切换生产策略的实际行为证据.
        old = runtime.execution
        old.running = True
        assert old.submit([(0.,) * 6, (.02, 0., 0., 0., 0., 0.)], [0., 1.],
                          old.generation, now=0.)
        assert old.sample(.5).positions == (0.,) * 6
        assert old.sample(1.).positions[0] == .02
    finally:
        close(runtime, driver)

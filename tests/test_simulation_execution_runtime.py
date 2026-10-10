import threading
import time
from types import SimpleNamespace

import pytest

from rebotarm_simulation.execution.trajectory_state import ActiveTrajectory
from rebotarm_simulation.execution.trajectory_state import ExecutionLifecycle
from rebotarm_simulation.execution.feedback_timing import FeedbackRateLimiter
from rebotarm_simulation.execution.trajectory_state import GateOutcome
from rebotarm_simulation.execution.goal_policy import GoalSettlingPolicy
from rebotarm_simulation.ros.message_codec import MonotonicStamp
from rebotarm_simulation.execution.simulation_access import SerializedSimulationAccess
from rebotarm_simulation.execution.trajectory_state import TrajectoryCommandGate
from rebotarm_simulation.execution.trajectory_state import terminal_disposition


def test_goal_settling_policy_is_pure_and_has_explicit_timeout():
    p = GoalSettlingPolicy(.01, .05, .2)
    assert p.evaluate((0,)*6, (0,)*6, (0,)*6, 0) == 'succeeded'
    assert p.evaluate((0,)*6, (.1,)*6, (0,)*6, .1) == 'settling'
    assert p.evaluate((0,)*6, (.1,)*6, (0,)*6, .2) == 'timed_out'
    with pytest.raises(ValueError): p.evaluate((0,)*5, (0,)*6, (0,)*6, 0)


def test_gate_serializes_cancel_and_completion():
    active = ActiveTrajectory()
    gate = TrajectoryCommandGate(active)
    token = object(); assert active.try_start(token)
    events=[]
    assert gate.apply_with_reason(token, lambda: False, lambda: events.append('apply'), lambda: events.append('hold')) is GateOutcome.APPLIED
    assert gate.stop_and_hold(lambda: events.append('hold'))
    assert gate.apply_with_reason(token, lambda: False, lambda: events.append('apply'), lambda: events.append('hold')) is GateOutcome.SERVICE_STOP
    assert gate.complete_with_reason(token, lambda: False, lambda: events.append('hold'), lambda: events.append('success')) is GateOutcome.SERVICE_STOP
    assert events == ['apply','hold','hold','hold']
    assert terminal_disposition(GateOutcome.ACTION_CANCEL, True) == 'canceled'
    assert terminal_disposition(GateOutcome.SERVICE_STOP, True) == 'aborted'


def test_lifecycle_releases_token_even_if_hold_or_abort_fails():
    active = ActiveTrajectory(); gate = TrajectoryCommandGate(active); token=object(); assert active.try_start(token)
    calls=[]
    def hold(): calls.append('hold'); raise RuntimeError
    def abort(): calls.append('abort'); raise RuntimeError
    with pytest.raises(RuntimeError): ExecutionLifecycle(active, gate).fail(token, hold, abort)
    assert not active.busy and calls == ['hold','abort']


def test_stamp_and_feedback_are_monotonic_and_final_forced():
    stamp=MonotonicStamp(); assert stamp.update(1.2) == (1,200000000); assert stamp.update(.1)==(1,200000000)
    limiter=FeedbackRateLimiter(10); assert limiter.should_publish(0); assert not limiter.should_publish(.01); assert limiter.should_publish(.01, final=True)
    with pytest.raises(ValueError): FeedbackRateLimiter(201)


def test_serialized_access_prevents_interleaving():
    events=[]; lock=threading.RLock(); access=SerializedSimulationAccess(SimpleNamespace(), lock)
    def op(_): events.append('start'); time.sleep(.01); events.append('end')
    a=threading.Thread(target=lambda:access.run(op)); b=threading.Thread(target=lambda:access.run(op)); a.start();b.start();a.join();b.join()
    assert events in (['start','end','start','end'], ['start','end','start','end'])

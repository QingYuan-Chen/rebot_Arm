"""Execution ownership and fake service timing regression tests; no hardware."""
from concurrent.futures import ThreadPoolExecutor
from types import SimpleNamespace

import pytest

from rebotarm_vision.utils.parameter_validation import (
    validate_candidate_parameters, validate_visual_parameters,
)
from rebotarm_vision.visual_grasp_state import VisualGraspState
from rebotarm_vision.visual_grasp_io import VisualGraspIoGateway
from rebotarm_vision.nodes.visual_grasp_executor_node import VisualGraspExecutorNode


@pytest.mark.parametrize('values', [
    {'service_timeout_sec': -1},
    {'motion_result_timeout_sec': float('nan')},
    {'acceleration_scaling': 1.1},
    {'min_open_position_m': .09, 'max_open_position_m': .04},
])
def test_visual_invalid_configuration_fails_closed(values):
    with pytest.raises(ValueError):
        validate_visual_parameters(values)


@pytest.mark.parametrize('values', [
    {'candidate_workspace_min_xyz': [1, 0, 0], 'candidate_workspace_max_xyz': [0, 1, 1]},
    {'max_candidates_per_frame': 0},
    {'candidate_min_jaw_width_m': .1, 'candidate_max_jaw_width_m': .08},
])
def test_candidate_invalid_configuration_fails_closed(values):
    with pytest.raises(ValueError):
        validate_candidate_parameters(values)


def test_only_one_execution_can_own_state_until_cleanup():
    state = VisualGraspState()
    with ThreadPoolExecutor(max_workers=8) as pool:
        assert sum(pool.map(lambda _: state.begin_run(), range(32))) == 1
    state.request_stop()
    assert state.phase == 'STOP_REQUESTED'
    assert not state.is_running()
    assert not state.begin_run()
    state.finish_run(preserve_abort=True)
    assert state.phase == 'STOP_REQUESTED'
    # A stop keeps the plan gate closed even after the worker callback exits;
    # confirmed downstream stop is the only boundary that permits a fresh execution.
    assert not state.begin_run()
    assert state.confirm_stop()
    assert state.begin_run()
    state.mark_aborting('execute_pose timed out')
    assert state.phase == 'ABORTING'
    assert not state.begin_run()
    state.finish_run()
    assert state.phase == 'ABORTING'
    assert state.abort_reason == 'execute_pose timed out'


def test_stop_latch_requires_confirmation_after_run_has_finished():
    state = VisualGraspState()
    assert state.begin_run()
    state.request_stop("operator stop")
    state.finish_run(preserve_abort=True)
    assert state.phase == "STOP_REQUESTED"
    assert not state.begin_run()
    assert state.confirm_stop()
    assert state.phase == "IDLE"
    assert state.begin_run()


def test_confirmation_cannot_clear_a_running_or_idle_state():
    state = VisualGraspState()
    assert not state.confirm_stop()
    assert state.begin_run()
    assert not state.confirm_stop()


class Future:
    def __init__(self, clock, ready_at):
        self.clock = clock
        self.ready_at = ready_at
        self.cancelled = False
        self.result_reads = 0

    def done(self):
        return self.clock[0] >= self.ready_at

    def result(self):
        self.result_reads += 1
        return SimpleNamespace(success=True)

    def cancel(self):
        self.cancelled = True


def gateway(state, clock, future, sleep):
    return VisualGraspIoGateway(
        logger=None,
        clients={'execute_pose': SimpleNamespace(
            wait_for_service=lambda **_: True, call_async=lambda _: future,
        )},
        cancelled=lambda: not state.is_running(),
        on_timeout=lambda name: state.mark_aborting(f'{name} timed out'),
        monotonic=lambda: clock[0], sleep=sleep, context_ok=lambda: True,
    )


def test_timeout_retains_remote_future_for_stop_confirmation():
    state = VisualGraspState()
    state.begin_run()
    clock = [0.0]
    future = Future(clock, 2.0)
    io = gateway(state, clock, future, lambda dt: clock.__setitem__(0, clock[0] + dt))
    result, error = io.call('execute_pose', object(), .1)
    assert result is None and 'timed out' in error
    assert state.phase == 'ABORTING'
    assert not future.cancelled
    clock[0] = 3.0
    assert future.result_reads == 0


def test_stop_wins_even_when_response_arrives_in_same_poll():
    state = VisualGraspState()
    state.begin_run()
    clock = [0.0]
    future = Future(clock, .02)
    def sleep(dt):
        clock[0] += dt
        state.request_stop('operator stop')
    result, error = gateway(state, clock, future, sleep).call('execute_pose', object(), 1)
    assert result is None and 'stopped' in error
    assert state.phase == 'STOP_REQUESTED'
    assert future.result_reads == 0


def test_timeout_handler_requests_independent_motion_stop():
    state = VisualGraspState()
    state.begin_run()
    calls = []
    node = SimpleNamespace(
        _state=state,
        _request_stop=lambda: calls.append("stop"),
    )
    VisualGraspExecutorNode._handle_io_timeout(node, "execute_pose")
    assert calls == ["stop"]
    assert state.phase == "ABORTING"
    assert state.abort_reason == "execute_pose timed out"


def test_invalid_plan_and_empty_candidates_revoke_pending_cache():
    from rebotarm_msgs.msg import GraspCandidate, GraspCandidateArray, GraspPlan

    state = VisualGraspState()
    state.latest_plan = GraspPlan()
    state.latest_plan.valid = True
    state.latest_candidates = GraspCandidateArray()
    state.latest_candidates.candidates.append(GraspCandidate())
    node = SimpleNamespace(_state=state)

    invalid = GraspPlan()
    invalid.valid = False
    invalid.reason = "target lost"
    VisualGraspExecutorNode._on_plan(node, invalid)
    assert state.latest_plan is None
    assert state.latest_candidates is None

    state.latest_plan = GraspPlan()
    state.latest_plan.valid = True
    candidates = GraspCandidateArray()
    VisualGraspExecutorNode._on_candidates(node, candidates)
    assert state.latest_plan is None
    assert state.latest_candidates is None


def test_timeout_path_requests_stop_and_keeps_abort_latch():
    state = VisualGraspState()
    state.begin_run()
    calls = []
    workflow = SimpleNamespace(
        _state=state,
        _plan_is_fresh=lambda _plan: True,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1_000_000_000)),
        _max_plan_age_sec=1.0,
        _candidate_plans_for_attempts=lambda: [(0, state.latest_plan)],
        _build_sequence_from_plan=lambda _plan: [],
        _append_place_stages=lambda stages: stages,
        _execute_stages=lambda _stages: (False, "timeout", "approach_grasp"),
        _request_stop=lambda: calls.append("stop"),
        _request_retry_retreat=lambda: (True, "ok"),
        _execution_enabled=lambda: False,
        _log_diagnostic=lambda *_args: None,
        _log_plan_snapshot=lambda *_args: None,
        _log_failure_snapshot=lambda *_args: None,
        _diagnostic_prefix=lambda _stage: "test",
        get_logger=lambda: SimpleNamespace(info=lambda *_args: None, warn=lambda *_args: None),
        _config=SimpleNamespace(input_topic="/test", auto_retry_enabled=False, safe_retreat_before_retry=True),
    )
    state.latest_plan = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0)))
    from rebotarm_vision.visual_grasp_runtime import VisualGraspRuntime

    response = VisualGraspRuntime(workflow).execute(None, SimpleNamespace(success=None, message=""), admitted=True)
    assert not response.success
    assert calls == ["stop"]  # recovery always stops the failed motion stage

    state.mark_aborting("execute_pose timed out")
    response = VisualGraspRuntime(workflow).execute(None, SimpleNamespace(success=None, message=""), admitted=True)
    assert not response.success
    assert calls == ["stop", "stop"]
    assert state.phase == "ABORTING"
    state.finish_run(preserve_abort=True)
    assert not state.begin_run()


def test_invalid_plan_and_empty_candidates_revoke_pending_cache():
    from rebotarm_vision.nodes.visual_grasp_executor_node import VisualGraspExecutorNode

    state = VisualGraspState(
        latest_plan=SimpleNamespace(valid=True),
        latest_candidates=SimpleNamespace(candidates=[object()]),
    )
    node = SimpleNamespace(_state=state)
    invalid = SimpleNamespace(valid=False, reason="target lost")
    VisualGraspExecutorNode._on_plan(node, invalid)
    assert state.latest_plan is None
    assert state.latest_candidates is None


def test_candidate_tf_query_uses_frame_stamp_and_reuses_same_frame_transform():
    from rebotarm_vision.candidate_ik_gateway import CandidateIkGateway

    calls = []
    from builtin_interfaces.msg import Time as RosTime
    stamp = RosTime(sec=12, nanosec=345)
    buffer = SimpleNamespace(
        lookup_transform=lambda target, source, query_time, timeout: calls.append(
            (target, source, query_time.nanoseconds)
        ) or object()
    )
    gateway = CandidateIkGateway(
        config=SimpleNamespace(service_timeout_sec=0.1), ik_client=None,
        validity_client=None, tf_buffer=buffer, joint_state=None,
        logger=SimpleNamespace(warn=lambda *_args: None),
        publish_ranked=lambda *_args: None, publish_empty=lambda *_args: None,
    )
    gateway.begin_frame()
    first = gateway.lookup_transform("base_link", "camera", stamp)
    second = gateway.lookup_transform("base_link", "camera", stamp)
    assert first is second
    assert calls == [("base_link", "camera", 12_000_000_345)]


@pytest.mark.parametrize("stamp", [None, SimpleNamespace(sec=0, nanosec=0)])
def test_candidate_tf_rejects_missing_sensor_stamp_before_lookup(stamp):
    from rebotarm_vision.candidate_ik_gateway import CandidateIkGateway

    calls = []
    gateway = CandidateIkGateway(
        config=SimpleNamespace(service_timeout_sec=0.1), ik_client=None,
        validity_client=None,
        tf_buffer=SimpleNamespace(lookup_transform=lambda *args, **kwargs: calls.append(args)),
        joint_state=None, logger=SimpleNamespace(warn=lambda *_args: None),
        publish_ranked=lambda *_args: None, publish_empty=lambda *_args: None,
    )
    with pytest.raises(RuntimeError, match="sensor timestamp"):
        gateway.lookup_transform("base_link", "camera", stamp)
    assert calls == []


def test_gateway_with_real_ros_client_and_fake_service():
    """ROS transport integration on a unique diagnostic-only service name."""
    import threading
    import uuid
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from std_srvs.srv import Trigger

    context = Context()
    rclpy.init(context=context)
    node = Node('vision_refactor_transport_test', context=context)
    name = '/vision_refactor_test_' + uuid.uuid4().hex
    def respond(_request, response):
        response.success = True
        response.message = 'fake service response'
        return response
    node.create_service(Trigger, name, respond)
    client = node.create_client(Trigger, name)
    executor = MultiThreadedExecutor(num_threads=2, context=context)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        state = VisualGraspState()
        state.begin_run()
        io = VisualGraspIoGateway(
            logger=node.get_logger(), clients={'test': client},
            cancelled=lambda: not state.is_running(),
            context_ok=lambda: context.ok(),
        )
        response, error = io.call('test', Trigger.Request(), 2.0)
        assert error == ''
        assert response.success
        assert response.message == 'fake service response'
    finally:
        executor.shutdown(timeout_sec=2.0)
        thread.join(timeout=2.0)
        node.destroy_node()
        context.shutdown()


def test_confirm_stop_requires_worker_exit_and_discards_old_plans():
    state = VisualGraspState(latest_plan=object(), latest_candidates=object())
    state.begin_run()
    state.request_stop()
    assert not state.confirm_stop()
    state.finish_run(preserve_abort=True)
    assert state.confirm_stop()
    assert state.phase == 'IDLE'
    assert state.latest_plan is None and state.latest_candidates is None
    assert not state.confirm_stop()


def test_live_expired_plan_revokes_old_cache_and_wait_accepts_new_input(monkeypatch):
    from rebotarm_msgs.msg import GraspPlan
    from rebotarm_vision.nodes import visual_grasp_executor_node as module
    state = VisualGraspState()
    state.begin_run()
    now = 100_000_000_000
    old = GraspPlan()
    old.valid = True
    old.header.stamp.sec = 90
    state.latest_plan = old
    state.latest_candidates = object()
    node = SimpleNamespace(
        _state=state, _service_timeout_sec=.2, _max_plan_age_sec=4.,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=now)),
    )
    node._plan_is_fresh = lambda p: VisualGraspExecutorNode._plan_is_fresh(node, p)
    VisualGraspExecutorNode._on_plan(node, old)
    assert state.latest_plan is None and state.latest_candidates is None
    assert 'age_sec=10.0' in state.last_plan_rejection
    assert 'max_plan_age_sec=4.0' in state.last_plan_rejection
    fresh = GraspPlan()
    fresh.valid = True
    fresh.header.stamp.sec = 100
    monkeypatch.setattr(module.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(module.time, 'sleep', lambda _: VisualGraspExecutorNode._on_plan(node, fresh))
    assert VisualGraspExecutorNode._wait_for_fresh_plan(node)[0]
    assert state.latest_plan.header.stamp.sec == 100


def test_fresh_plan_wait_is_stoppable_and_preserves_stop_latch(monkeypatch):
    from rebotarm_vision.nodes import visual_grasp_executor_node as module
    state = VisualGraspState()
    state.begin_run()
    node = SimpleNamespace(_state=state, _service_timeout_sec=.2)
    monkeypatch.setattr(module.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(module.time, 'sleep', lambda _: state.request_stop('operator stop'))
    ok, reason = VisualGraspExecutorNode._wait_for_fresh_plan(node)
    assert not ok and reason == 'operator stop'
    assert not state.begin_run()
    assert not state.confirm_stop()
    state.finish_run(preserve_abort=True)
    assert state.confirm_stop()


def test_plan_rejection_during_wait_is_reported_at_timeout(monkeypatch):
    from rebotarm_vision.nodes import visual_grasp_executor_node as module
    state = VisualGraspState(last_plan_rejection='invalid grasp plan: target out of workspace')
    state.begin_run()
    node = SimpleNamespace(_state=state, _service_timeout_sec=0.)
    monkeypatch.setattr(module.rclpy, 'ok', lambda: True)
    ok, reason = VisualGraspExecutorNode._wait_for_fresh_plan(node)
    assert not ok and 'target out of workspace' in reason
    assert 'timed out' in reason


def test_repeated_execute_waits_for_new_plan_without_ready_motion(monkeypatch):
    from rebotarm_msgs.msg import GraspPlan
    from rebotarm_vision.nodes import visual_grasp_executor_node as module
    from rebotarm_vision.visual_grasp_runtime import VisualGraspRuntime
    state = VisualGraspState()
    now = [100_000_000_000]
    calls = []
    fresh = GraspPlan()
    fresh.valid = True
    fresh.header.stamp.sec = 100
    node = SimpleNamespace(
        _state=state, _service_timeout_sec=.2, _max_plan_age_sec=4.,
        _refresh_config=lambda: None, _config=None, _io_gateway=SimpleNamespace(stop_pending=False), _tf_buffer=None,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=now[0])),
        get_logger=lambda: None,
    )
    node._plan_is_fresh = lambda p: VisualGraspExecutorNode._plan_is_fresh(node, p)
    node._wait_for_fresh_plan = lambda: VisualGraspExecutorNode._wait_for_fresh_plan(node)
    monkeypatch.setattr(module.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(module, 'VisualGraspWorkflow', lambda **kwargs: kwargs)
    def execute(runtime, request, response, **kwargs):
        calls.append(state.latest_plan.header.stamp.sec)
        response.success = True
        return response
    monkeypatch.setattr(VisualGraspRuntime, 'execute', execute)
    VisualGraspExecutorNode._on_plan(node, fresh)
    assert VisualGraspExecutorNode._execute_visual_grasp(node, None, SimpleNamespace()).success
    assert state.phase == 'IDLE'
    # Simulate time spent returning home. The previous plan is now expired,
    # but a newly captured plan arrives while the second execute waits.
    now[0] = 120_000_000_000
    fresh.header.stamp.sec = 120
    monkeypatch.setattr(module.time, 'sleep', lambda _: VisualGraspExecutorNode._on_plan(node, fresh))
    assert VisualGraspExecutorNode._execute_visual_grasp(node, None, SimpleNamespace()).success
    assert calls == [100, 120]
    assert state.phase == 'IDLE'


def test_stop_confirmation_waits_for_remote_call_then_new_motion_ack():
    from concurrent.futures import Future as RealFuture
    clock = [0.]
    pending, early_stop, confirmed_stop = RealFuture(), RealFuture(), RealFuture()
    requests = []
    io = VisualGraspIoGateway(logger=None, clients={'motion_stop': SimpleNamespace(
        wait_for_service=lambda **_: True,
        call_async=lambda _: requests.append('stop') or (early_stop if len(requests)==1 else confirmed_stop),
    )}, cancelled=lambda: False, monotonic=lambda: clock[0])
    io._pending_calls.append(pending)
    io.stop('motion_stop')
    early_stop.set_result(SimpleNamespace(success=True, message='early'))
    assert io.poll_stop_confirmation()[0] is None
    assert requests == ['stop']
    pending.set_result(SimpleNamespace(success=False))
    assert io.poll_stop_confirmation()[0] is None
    assert requests == ['stop', 'stop']
    confirmed_stop.set_result(SimpleNamespace(success=True, message='confirmed'))
    assert io.poll_stop_confirmation() == (True, 'confirmed')
    assert not io.stop_pending


def test_stop_failure_does_not_automatically_reopen_execution():
    from concurrent.futures import Future as RealFuture
    future = RealFuture()
    future.set_result(SimpleNamespace(success=False, message='feedback stale'))
    io = VisualGraspIoGateway(logger=None, clients={'motion_stop': SimpleNamespace(
        wait_for_service=lambda **_: True, call_async=lambda _: future,
    )}, cancelled=lambda: False)
    io.stop('motion_stop')
    assert io.poll_stop_confirmation() == (False, 'feedback stale')
    assert io.stop_pending
    assert io.poll_stop_confirmation() == (False, 'feedback stale')


@pytest.mark.parametrize('unknown', ['cancelled', 'exception', 'empty'])
def test_service_pruning_retains_unknown_remote_outcome(unknown):
    from concurrent.futures import Future as RealFuture
    old, current = RealFuture(), RealFuture()
    if unknown == 'cancelled':
        old.cancel()
    elif unknown == 'exception':
        old.set_exception(RuntimeError('transport failure'))
    else:
        old.set_result(None)
    current.set_result(SimpleNamespace(success=True))
    io = VisualGraspIoGateway(logger=None, clients={'execute_pose': SimpleNamespace(
        wait_for_service=lambda **_: True, call_async=lambda _: current,
    )}, cancelled=lambda: False, legacy_wait=lambda *_: True)
    io._pending_calls.append(old)
    assert io.call('execute_pose', None, 1.)[0].success
    assert old in io._pending_calls
    io._stop_pending = True
    io._stop_started = io.monotonic()
    assert io.poll_stop_confirmation()[0] is False
    assert io.stop_pending

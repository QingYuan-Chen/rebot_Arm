from __future__ import annotations

import math
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

from rebotarm_simulation.ros.mujoco_ros_node import (
    ActiveTrajectory,
    SerializedSimulationAccess,
    ExecutionLifecycle,
    FeedbackRateLimiter,
    GateOutcome,
    GoalSettlingPolicy,
    MonotonicStamp,
    TrajectoryCommandGate,
    SimulationControlApi,
    normalize_ros_control_mode,
    seconds_to_stamp_parts,
    trajectory_to_sampler,
    terminal_disposition,
    validate_gripper_width,
    validate_gripper_force,
)


class _LegacySimulation:
    def __init__(self):
        self.control_mode = "pos_vel"
        self.control_targets = (0.0,) * 8
        self.positions = (0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.03, -0.03)
        self.events = []

    def reset_home(self):
        self.events.append(("reset_home",))

    def set_control_mode(self, mode):
        self.control_mode = mode
        self.events.append(("mode", mode))
        return mode

    def set_joint_position_targets(self, values):
        reached = tuple(values)
        self.control_targets = reached + (0.03, -0.03)
        self.control_mode = "pos_vel"
        self.events.append(("positions", reached))
        return reached

    def set_gripper_width(self, width):
        self.events.append(("gripper", width))
        return width

    def get_state(self):
        return SimpleNamespace(joint_positions=self.positions)


def test_ros_mode_surface_excludes_raw_torque():
    assert normalize_ros_control_mode(" position ") == "position"
    assert normalize_ros_control_mode("hold") == "hold"
    assert normalize_ros_control_mode("gravity_comp") == "gravity_comp"
    with pytest.raises(ValueError, match="raw_torque"):
        normalize_ros_control_mode("raw_torque")


def test_ros_control_api_starts_home_hold_and_tracks_positions_with_legacy_core():
    simulation = _LegacySimulation()
    control = SimulationControlApi(simulation)
    control.reset_home_and_hold()
    assert simulation.events[:2] == [("reset_home",), ("mode", "hold")]
    assert control.command_joint_positions((1, 2, 3, 4, 5, 6)) == (1, 2, 3, 4, 5, 6)
    assert simulation.events[-1] == ("mode", "pos_vel")
    assert control.get_control_status()["mode"] == "position"


def test_ros_control_api_hold_captures_current_position_and_gripper_is_width_based():
    simulation = _LegacySimulation()
    control = SimulationControlApi(simulation)
    control.hold_current_position()
    assert simulation.events[-2:] == [
        ("positions", simulation.positions[:6]),
        ("mode", "hold"),
    ]
    assert control.command_gripper_width(0.04) == pytest.approx(0.04)


def test_gripper_effort_zero_means_default_and_positive_value_is_forwarded():
    assert validate_gripper_force(0.0) is None
    assert validate_gripper_force(12.5) == pytest.approx(12.5)
    for invalid in (-1.0, float("nan"), float("inf")):
        with pytest.raises(ValueError, match="max effort"):
            validate_gripper_force(invalid)


def _duration(seconds: float):
    whole = math.floor(seconds)
    return SimpleNamespace(sec=whole, nanosec=round((seconds - whole) * 1e9))


def _trajectory(names=("joint1", "joint2"), points=((1.0, (0.1, 0.2)),)):
    return SimpleNamespace(
        joint_names=list(names),
        points=[SimpleNamespace(time_from_start=_duration(t), positions=list(p)) for t, p in points],
    )


def test_trajectory_conversion_uses_canonical_order_and_simulation_time():
    sampler = trajectory_to_sampler(
        _trajectory(("joint2", "joint1"), ((1.0, (0.2, 0.1)),)),
        initial_positions=(0.0,) * 6,
    )
    assert sampler.sample(0.5) == pytest.approx((0.05, 0.1, 0, 0, 0, 0))


def test_simulation_stamp_normalizes_nanosecond_rounding():
    assert seconds_to_stamp_parts(1.9999999996) == (2, 0)
    with pytest.raises(ValueError, match="finite"):
        seconds_to_stamp_parts(math.nan)


def test_monotonic_stamp_never_moves_backwards():
    stamp = MonotonicStamp()
    assert stamp.update(2.5) == (2, 500_000_000)
    assert stamp.update(2.4) == (2, 500_000_000)
    assert stamp.update(3.0) == (3, 0)


def test_feedback_rate_limiter_bounds_flood_and_forces_settling_terminal_feedback():
    limiter = FeedbackRateLimiter(20.0)
    published = [now for now in (index * 0.001 for index in range(2001)) if limiter.should_publish(now)]
    assert len(published) <= 42  # 20 Hz for 2 s, plus numerical/initial allowance
    assert limiter.should_publish(2.001, final=True)


@pytest.mark.parametrize("rate", [0.0, -1.0, math.nan, 201.0])
def test_feedback_rate_limiter_rejects_unsafe_rate(rate):
    with pytest.raises(ValueError, match="feedback rate"):
        FeedbackRateLimiter(rate)


def test_goal_settling_policy_requires_position_and_velocity_and_times_out():
    policy = GoalSettlingPolicy(0.02, 0.05, 5.0)
    assert policy.evaluate((1.0,) * 6, (0.99,) * 6, (0.04,) * 6, 0.0) == "succeeded"
    assert policy.evaluate((1.0,) * 6, (0.97,) * 6, (0.0,) * 6, 4.9) == "settling"
    assert policy.evaluate((1.0,) * 6, (1.0,) * 6, (0.06,) * 6, 5.0) == "timed_out"


@pytest.mark.parametrize("values", [(0, .05, 5), (.02, math.nan, 5), (.02, .05, -1)])
def test_goal_settling_policy_rejects_unsafe_parameters(values):
    with pytest.raises(ValueError, match="positive finite"):
        GoalSettlingPolicy(*values)


@pytest.mark.parametrize(
    "trajectory, message",
    [
        (_trajectory((), ()), "joint names"),
        (_trajectory(("joint1", "joint1")), "duplicate"),
        (_trajectory(("joint1", "evil_joint")), "unknown"),
        (_trajectory(points=((1.0, (0.1,)),)), "position count"),
        (_trajectory(points=((1.0, (math.nan, 0.2)),)), "finite"),
        (_trajectory(points=((1.0, (0.1, 0.2)), (1.0, (0.2, 0.3)))), "strictly"),
    ],
)
def test_trajectory_validation_rejects_untrusted_content(trajectory, message):
    with pytest.raises(ValueError, match=message):
        trajectory_to_sampler(trajectory, initial_positions=(0.0,) * 6)


def test_trajectory_validation_caps_points_and_duration():
    too_many = _trajectory(points=((1.0, (0.1, 0.2)),) * 3)
    with pytest.raises(ValueError, match="too many"):
        trajectory_to_sampler(too_many, initial_positions=(0.0,) * 6, max_points=2)
    with pytest.raises(ValueError, match="duration"):
        trajectory_to_sampler(
            _trajectory(points=((301.0, (0.1, 0.2)),)),
            initial_positions=(0.0,) * 6,
            max_duration_sec=300.0,
        )


@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf, "bad"])
def test_gripper_rejects_nonfinite_width(value):
    with pytest.raises(ValueError, match="finite"):
        validate_gripper_width(value)


def test_only_one_trajectory_can_be_active_and_stop_cancels_it():
    active = ActiveTrajectory()
    first = object()
    assert active.try_start(first)
    assert not active.try_start(object())
    assert active.stop()
    assert active.cancel_requested
    assert active.token is first
    active.finish(first)
    assert not active.busy


def test_cancel_between_iterations_cannot_be_overwritten_and_reholds():
    active = ActiveTrajectory()
    gate = TrajectoryCommandGate(active)
    token = object()
    applied = []
    held = []
    assert active.try_start(token)
    assert gate.apply_if_active(token, lambda: False, lambda: applied.append(1), lambda: held.append(1))
    gate.stop_and_hold(lambda: held.append(1))
    assert not gate.apply_if_active(token, lambda: False, lambda: applied.append(2), lambda: held.append(1))
    assert applied == [1]
    assert held == [1, 1]


def test_gate_distinguishes_action_cancel_service_stop_and_inactive():
    token = object()
    events = []

    action_active = ActiveTrajectory()
    assert action_active.try_start(token)
    action_gate = TrajectoryCommandGate(action_active)
    assert action_gate.apply_with_reason(
        token, lambda: True, lambda: events.append("apply"), lambda: events.append("action hold")
    ) is GateOutcome.ACTION_CANCEL

    service_active = ActiveTrajectory()
    assert service_active.try_start(token)
    service_gate = TrajectoryCommandGate(service_active)
    assert service_active.stop()
    assert service_gate.apply_with_reason(
        token, lambda: False, lambda: events.append("apply"), lambda: events.append("service hold")
    ) is GateOutcome.SERVICE_STOP
    assert service_gate.complete_with_reason(
        token, lambda: False, lambda: events.append("terminal service hold"), lambda: events.append("success")
    ) is GateOutcome.SERVICE_STOP

    inactive_gate = TrajectoryCommandGate(ActiveTrajectory())
    assert inactive_gate.apply_with_reason(
        token, lambda: False, lambda: events.append("apply"), lambda: events.append("inactive hold")
    ) is GateOutcome.INACTIVE
    assert events == ["action hold", "service hold", "terminal service hold"]


def test_goal_terminal_disposition_never_cancels_non_canceling_goal():
    assert terminal_disposition(GateOutcome.ACTION_CANCEL, True) == "canceled"
    assert terminal_disposition(GateOutcome.ACTION_CANCEL, False) == "aborted"
    assert terminal_disposition(GateOutcome.SERVICE_STOP, False) == "aborted"
    assert terminal_disposition(GateOutcome.INACTIVE, False) == "aborted"


def test_terminal_completion_rechecks_cancel_and_never_succeeds_after_stop():
    active = ActiveTrajectory()
    gate = TrajectoryCommandGate(active)
    token = object()
    events = []
    assert active.try_start(token)
    def cancel_hook():
        # Deterministic interleaving hook: stop arrives after settling was
        # declared ready but during the final terminal recheck.
        active.stop()
        return False
    assert not gate.complete_if_active(
        token, cancel_hook, lambda: events.append("hold"), lambda: events.append("success")
    )
    assert events == ["hold"]


def test_terminal_success_linearizes_before_later_stop():
    active = ActiveTrajectory()
    gate = TrajectoryCommandGate(active)
    token = object()
    events = []
    assert active.try_start(token)
    assert gate.complete_if_active(
        token, lambda: False, lambda: events.append("hold"), lambda: events.append("success")
    )
    assert events == ["success"]
    assert not active.busy
    assert not gate.stop_and_hold(lambda: events.append("late hold"))
    assert events == ["success"]


def test_execution_failure_holds_aborts_and_always_clears_active_token():
    active = ActiveTrajectory()
    gate = TrajectoryCommandGate(active)
    lifecycle = ExecutionLifecycle(active, gate)
    token = object()
    events = []
    assert active.try_start(token)
    lifecycle.fail(token, lambda: events.append("hold"), lambda: events.append("abort"))
    assert events == ["hold", "abort"]
    assert not active.busy


def test_execute_callback_is_synchronous_and_does_not_use_asyncio():
    source = Path("src/rebotarm_simulation/rebotarm_simulation/ros/mujoco_ros_node.py").read_text()
    assert "asyncio" not in source
    assert "async def _execute_goal" not in source
    assert "def _execute_goal" in source
    assert "time.sleep(self._execute_wait_sec)" in source


def test_gate_timer_and_stop_sim_calls_are_serialized_on_one_sim_lock():
    class InstrumentedSim:
        def __init__(self):
            self.active = 0
            self.max_active = 0

        def _record(self):
            self.active += 1
            self.max_active = max(self.max_active, self.active)
            time.sleep(0.002)
            self.active -= 1

        def get_state(self):
            self._record()

        def set_joint_position_targets(self):
            self._record()

        def step(self):
            self._record()

    sim = InstrumentedSim()
    access = SerializedSimulationAccess(sim)
    apply_call = lambda: access.run(
        lambda guarded: (guarded.get_state(), guarded.set_joint_position_targets())
    )
    hold_call = lambda: access.run(
        lambda guarded: (guarded.get_state(), guarded.set_joint_position_targets())
    )
    step_call = lambda: access.run(lambda guarded: guarded.step())
    active = ActiveTrajectory()
    gate = TrajectoryCommandGate(active)
    token = object()
    assert active.try_start(token)
    start = threading.Event()
    def together(operation):
        start.wait()
        operation()
    threads = [
        threading.Thread(target=lambda: together(lambda: gate.apply_if_active(token, lambda: False, apply_call, hold_call))),
        threading.Thread(target=lambda: together(lambda: gate.stop_and_hold(hold_call))),
        threading.Thread(target=lambda: together(step_call)),
    ]
    for thread in threads:
        thread.start()
    start.set()
    for thread in threads:
        thread.join()
    assert sim.max_active == 1


def test_ros_adapter_source_has_required_safe_interfaces_and_no_hardware_imports():
    source = Path("src/rebotarm_simulation/rebotarm_simulation/ros/mujoco_ros_node.py").read_text()
    assert "FollowJointTrajectory" in source
    assert 'f"/{self._arm_namespace}/follow_joint_trajectory"' in source
    assert 'f"/{self._arm_namespace}/joint_states"' in source
    assert 'f"/{self._arm_namespace}/trajectory_stop"' in source
    assert 'f"/{self._arm_namespace}/gripper/set"' in source
    assert 'f"/{self._arm_namespace}/sim/set_mode"' in source
    assert '"/clock"' in source
    assert '"/diagnostics"' in source
    assert "DiagnosticArray" in source
    assert "SetMode" in source
    assert "normalize_ros_control_mode" in source
    assert "RebotArmMujoco" in source
    assert "ReentrantCallbackGroup" in source
    assert "MutuallyExclusiveCallbackGroup" in source
    assert "self._timer_callback_group" in source
    assert "callback_group=self._callback_group" in source
    assert "def _hold_current_position" in source
    assert "def _hold_current_position_unlocked" in source
    assert source.count("self._hold_current_position") >= 3
    assert "ActiveTrajectory(self._lock)" not in source
    assert "GOAL_TOLERANCE_VIOLATED" in source
    assert "complete_with_reason" in source
    assert "apply_with_reason" in source
    execution_source = Path(
        "src/rebotarm_simulation/rebotarm_simulation/execution/trajectory_state.py"
    ).read_text()
    assert "terminal_disposition" in source
    assert "GateOutcome.ACTION_CANCEL" in execution_source
    assert "simulation trajectory stopped by service" in source
    assert "type(exc).__name__" in source
    assert "FeedbackRateLimiter" in source
    assert "time.sleep(self._execute_wait_sec)" in source
    assert "time.sleep(0.001)" not in source
    assert "rebotarmcontroller" not in source.lower()


def test_ros_adapter_configures_home_hold_and_bounded_diagnostics():
    config = Path("src/rebotarm_simulation/config/mujoco_sim.yaml").read_text()
    assert "initial_joint_positions: [0.0, -0.8, -1.0, 0.3, 0.0, 0.0]" in config
    assert "diagnostic_rate_hz: 1.0" in config
    assert "max_contact_force_n:" in config
    assert "max_contact_penetration_m:" in config

    manifest = Path("src/rebotarm_simulation/package.xml").read_text()
    assert "<exec_depend>diagnostic_msgs</exec_depend>" in manifest


def test_ros_adapter_trajectory_lifecycle_explicitly_enters_position_and_hold():
    source = Path("src/rebotarm_simulation/rebotarm_simulation/ros/mujoco_ros_node.py").read_text()
    execute_body = source.split("def _execute_goal(self, goal_handle):", 1)[1].split(
        "\n        def _timer_callback", 1
    )[0]
    assert "self._control.command_joint_positions(desired)" in execute_body
    assert "lambda: (self._hold_current_position(), goal_handle.succeed())" in execute_body
    assert "GOAL_TOLERANCE_VIOLATED" in execute_body
    assert "self._command_gate.stop_and_hold(self._hold_current_position)" in execute_body

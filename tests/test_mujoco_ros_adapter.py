from __future__ import annotations

import math
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

from rebotarm_simulation.execution.trajectory_state import (
    ActiveTrajectory, GateOutcome, TrajectoryCommandGate, terminal_disposition,
)
from rebotarm_simulation.execution.trajectory_execution import TrajectoryExecution
from rebotarm_simulation.execution.gripper_execution import GripperExecution, GripperTask
from rebotarm_simulation.execution.simulation_access import SerializedSimulationAccess
from rebotarm_simulation.execution.feedback_timing import FeedbackRateLimiter
from rebotarm_simulation.execution.goal_policy import GoalSettlingPolicy
from rebotarm_simulation.execution.control_api import SimulationControlApi, normalize_ros_control_mode
from rebotarm_simulation.ros.message_codec import (
    MonotonicStamp,
    seconds_to_stamp_parts,
    trajectory_points,
    validate_gripper_width,
    validate_gripper_force,
)


class _Simulation:
    def __init__(self):
        self.control_mode = "position"
        self.control_targets = (0.0,) * 8
        self.positions = (0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.03, -0.03)
        self.events = []

    def reset_home(self):
        self.events.append(("reset_home",))

    def set_mode(self, mode):
        self.control_mode = mode
        self.events.append(("mode", mode))
        return mode

    def command_joint_positions(self, values):
        reached = tuple(values)
        self.control_targets = reached + (0.03, -0.03)
        self.control_mode = "position"
        self.events.append(("positions", reached))
        return reached

    def command_gripper_width(self, width, *, max_force_n=None):
        self.events.append(("gripper", width, max_force_n))
        return width

    def get_control_status(self):
        return SimpleNamespace(mode=self.control_mode)

    def get_state(self):
        return SimpleNamespace(joint_positions=self.positions)


@pytest.mark.parametrize('residual',[.003,.01])
def test_sim_stationary_snapshot_accepts_hold_residual_but_still_requires_quiet_fresh_window(residual):
    """不构造 ROS 节点或物理模型,只调用正式方法核对准入条件."""
    snapshot=TrajectoryExecution._stationary_snapshot
    hold=(0.,)*6
    positions=(residual,0.,0.,0.,0.,0.,.01,-.01)
    samples=[SimpleNamespace(simulation_time=t,joint_positions=positions,joint_velocities=(0.,)*8,
        joint_names=tuple(f'joint{i}' for i in range(1,7))+('left_finger_joint','right_finger_joint'))
        for t in (.8,.9,1.)]
    sim=SimpleNamespace(get_state=lambda:samples[-1],
        get_control_status=lambda:SimpleNamespace(joint_targets=hold))
    owner=SimpleNamespace(_sample_history=samples,_last_physics_wall=time.monotonic(),clock=time.monotonic)
    state,returned_hold=snapshot(owner,sim)
    assert state is samples[-1] and returned_hold==hold
    # 六轴与双指位置跨越旧量化门限,只要速度窗口合格仍可继续.
    samples[1].joint_positions=(.02,)*6+(.012,-.012)
    assert snapshot(owner,sim)[1]==hold
    samples[-1].joint_velocities=(.1,)+(0.,)*7
    with pytest.raises(RuntimeError,match='stationary evidence'):
        snapshot(owner,sim)
    samples[-1].joint_velocities=(0.,)*8
    owner._last_physics_wall=time.monotonic()-1.
    with pytest.raises(RuntimeError,match='fresh complete stationary window'):
        snapshot(owner,sim)


def test_ros_mode_surface_excludes_raw_torque():
    assert normalize_ros_control_mode(" position ") == "position"
    assert normalize_ros_control_mode("hold") == "hold"
    assert normalize_ros_control_mode("gravity_comp") == "gravity_comp"
    with pytest.raises(ValueError, match="raw_torque"):
        normalize_ros_control_mode("raw_torque")


def test_independent_gripper_requires_a_new_physics_frame_after_final_target():
    """即使小开口已在到位容差内,也不能用最终目标发送前的反馈报告成功."""
    state = SimpleNamespace(simulation_time=1.,gripper_width=0.)
    events = []
    sim = SimpleNamespace(get_state=lambda:state,
        command_gripper_position=lambda *args:events.append(('target',*args)),
        release_gripper_position=lambda:events.append(('neutral',)))
    task = GripperTask(object(),0.,0.,.0001,.01,2.,1.)
    owner = GripperExecution(sim,ActiveTrajectory(),{})
    owner.task = task
    owner.advance()
    owner.advance()  # 重读同一物理帧不算新反馈.
    assert task.state == 'active' and all(e[0]=='target' for e in events)
    state.simulation_time=1.002
    state.gripper_width=.0001
    owner.advance()
    assert task.state=='succeeded' and events[-1]==('neutral',)


def test_ros_control_api_starts_home_hold_and_tracks_positions():
    simulation = _Simulation()
    control = SimulationControlApi(simulation)
    control.reset_home_and_hold()
    assert simulation.events[:2] == [("reset_home",), ("mode", "hold")]
    assert control.command_joint_positions((1, 2, 3, 4, 5, 6)) == (1, 2, 3, 4, 5, 6)
    assert simulation.events[-1] == ("mode", "position")
    assert control.get_control_status().mode == "position"


def test_ros_control_api_hold_captures_current_position_and_gripper_is_width_based():
    simulation = _Simulation()
    control = SimulationControlApi(simulation)
    control.hold_current_position()
    assert simulation.events[-1] == ("mode", "hold")
    assert control.command_gripper_width(0.04) == pytest.approx(0.04)
    assert simulation.events[-1] == ("gripper", 0.04, None)


def test_control_diagnostic_requires_complete_status():
    from rebotarm_simulation.ros.ros_diagnostics import build_control_diagnostic
    params = dict(arm_namespace="audit", configured_rate_hz=100., measured_rate_hz=100.,
                  state=SimpleNamespace(joint_positions=(0.,) * 8), contacts=(),
                  max_contact_force_n=20., max_contact_penetration_m=.01)
    with pytest.raises(AttributeError):
        build_control_diagnostic(status=SimpleNamespace(mode="hold"), **params)
    with pytest.raises(ValueError, match="six arm positions"):
        build_control_diagnostic(status=SimpleNamespace(mode="hold", joint_targets=(0.,)), **params)


def test_gripper_effort_zero_means_default_and_positive_value_is_forwarded():
    assert validate_gripper_force(0.0) is None
    assert validate_gripper_force(12.5) == pytest.approx(12.5)
    for invalid in (-1.0, float("nan"), float("inf")):
        with pytest.raises(ValueError, match="max effort"):
            validate_gripper_force(invalid)


def _duration(seconds: float):
    whole = math.floor(seconds)
    return SimpleNamespace(sec=whole, nanosec=round((seconds - whole) * 1e9))


def _trajectory(names=tuple(f"joint{i}" for i in range(1, 7)), points=((1.0, (0.1,) * 6),)):
    return SimpleNamespace(
        joint_names=list(names),
        points=[SimpleNamespace(time_from_start=_duration(t), positions=list(p), velocities=[], accelerations=[]) for t, p in points],
    )


def test_trajectory_conversion_preserves_derivatives_and_canonical_order():
    value = _trajectory(tuple(f'joint{i}' for i in range(6, 0, -1)),
        ((0., (.6,.5,.4,.3,.2,.1)), (1., (.7,.6,.5,.4,.3,.2))))
    for point in value.points:
        point.velocities = [6.,5.,4.,3.,2.,1.]
        point.accelerations = [12.,10.,8.,6.,4.,2.]
    names, points = trajectory_points(value)
    assert names == tuple(f'joint{i}' for i in range(1,7))
    assert points[0].positions == pytest.approx((.1,.2,.3,.4,.5,.6))
    assert points[1].velocities == (1.,2.,3.,4.,5.,6.)
    assert points[1].accelerations == (2.,4.,6.,8.,10.,12.)
    assert points[1].time == 1.


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


@pytest.mark.parametrize('trajectory, message', [
    (_trajectory((), ()), 'exactly'),
    (_trajectory(('joint1',)*6), 'exactly'),
    (_trajectory(('joint1','joint2','joint3','joint4','joint5','evil_joint')), 'exactly'),
    (_trajectory(('joint1',)), 'exactly'),
    (_trajectory(points=((1., (.1,)),)), 'dimension'),
    (_trajectory(points=((1., (float('nan'),)*6),)), 'finite'),
    (_trajectory(points=((1., (.1,)*6),(1., (.2,)*6))), 'increas'),
])
def test_trajectory_validation_rejects_untrusted_content(trajectory, message):
    with pytest.raises(ValueError, match=message):
        trajectory_points(trajectory)


def test_trajectory_validation_caps_points_and_duration():
    with pytest.raises(ValueError, match='too many'):
        trajectory_points(_trajectory(points=((1.,(.1,)*6),)*3), max_points=2)
    with pytest.raises(ValueError, match='duration'):
        trajectory_points(_trajectory(points=((301.,(.1,)*6),)), max_duration_sec=300.)


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
    assert "ActiveTrajectory(self._lock)" not in source
    assert "GOAL_TOLERANCE_VIOLATED" in source
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


def test_execution_components_own_state_and_have_no_ros_dependencies():
    import ast
    root = Path("src/rebotarm_simulation/rebotarm_simulation")
    forbidden = {'rclpy', 'control_msgs', 'trajectory_msgs', 'rebotarm_msgs',
                 'rebotarm_motion', 'rebotarmcontroller', 'serial'}
    for name in ('trajectory_execution', 'gripper_execution'):
        tree = ast.parse((root/'execution'/f'{name}.py').read_text())
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                assert not forbidden.intersection(a.name.split('.')[0] for a in node.names)
            elif isinstance(node, ast.ImportFrom) and not node.level:
                assert (node.module or '').split('.')[0] not in forbidden
    tree = ast.parse((root/'ros/mujoco_ros_node.py').read_text())
    retired = {'_active', '_command_gate', '_pending_sampler', '_planning_generation',
               '_preparation', '_gripper_task', '_gripper_reservation',
               '_sample_history', '_last_physics_wall', '_stop_unconfirmed', '_retained_stop_token'}
    assert not retired.intersection(n.attr for n in ast.walk(tree) if isinstance(n,ast.Attribute))


def _execution_fixture():
    """确定性仿真替身:复用正式执行组件,记录命令和物理时间."""
    from rebotarm_trajectory.capability import make_capability
    from rebotarm_motion.polynomial_trajectory import JOINT_NAMES, build_curve
    from rebotarm_motion.trajectory_preparation import point_to_point

    class Sim:
        control_mode = 'hold'
        timestep = .002
        time = 0.
        width = 0.
        q = (0.,)*6
        target = (0.,)*6
        events = None

        def __init__(self):
            self.events = []

        def get_state(self):
            return SimpleNamespace(simulation_time=self.time, gripper_width=self.width,
                joint_positions=self.q+(self.width/2.,-self.width/2.), joint_velocities=(0.,)*8,
                joint_names=JOINT_NAMES+('left_finger_joint','right_finger_joint'))

        def get_control_status(self):
            return SimpleNamespace(joint_targets=self.target)

        def command(self, q):
            self.events.append(('arm',self.time,tuple(q)))
            self.target = self.q = tuple(q)

        def command_gripper_position(self, width, effort):
            self.events.append(('finger',self.time,width,effort))
            self.width = width

        def release_gripper_position(self):
            self.events.append(('neutral',self.time))

        def step(self):
            self.time += self.timestep
            return self.get_state()

    sim = Sim()
    control = SimpleNamespace(command_joint_positions=sim.command,
        hold_current_position=lambda:sim.events.append(('hold',sim.time)),
        set_mode=lambda mode:setattr(sim,'control_mode',mode) or mode)
    execution = TrajectoryExecution(SerializedSimulationAccess(sim),control,
        make_capability(11,.5,1.,'1'*64),GoalSettlingPolicy(),clock=lambda:10.)
    execution.scene_changed('scene-a')
    execution.step(120,lambda *args:None)
    curve = build_curve(JOINT_NAMES,point_to_point((0.,)*6,(.02,)*6,.4))
    return execution, sim, curve


def _reserve_simulation(execution, curve, *, width=None):
    token = object()
    pending = execution.reserve(token,curve.joint_names,curve.points,'',web=width is not None,
        gripper_command=None if width is None else (width,1.),received=10.)
    payload = dict(curve=curve.to_payload(),context=dict(session=pending.session,
        generation=pending.generation,hold=pending.hold_positions),
        scene_revision=pending.scene_revision,gripper_generation=pending.gripper_generation,
        gripper_positions=pending.gripper_positions,
        gripper_collision_policy=pending.gripper_collision_policy,
        capability_revision=execution.capability['revision'])
    return token,payload


@pytest.mark.parametrize('width',[None,.0009,.009])
def test_component_samples_exact_curve_and_waits_for_both_children(width):
    execution,sim,curve = _execution_fixture()
    token,payload = _reserve_simulation(execution,curve,width=width)
    before = len(sim.events)
    assert execution.active.busy
    assert execution.status_snapshot().phase == 'TRAJ_PREPARING'
    sampler = execution.admit(token,payload,'scene-a',lambda:False)
    start,grip = execution.accepted_context(token)
    assert len(sim.events) == before  # 两侧只登记,尚未发送.
    trace = []
    execution.step(1,lambda *args:trace.append(args))
    sends = sim.events[before:]
    assert sends[0][:2] == ('arm',start)
    if grip:
        assert sends[1][:2] == ('finger',start)
    # 周期延迟直接采样当前时刻,没有补发积压点.
    sim.time += .1
    before = len(sim.events)
    execution.step(1,lambda *args:trace.append(args))
    assert sim.events[before][2] == pytest.approx(curve.sample(.102)[0])
    for _ in range(700):
        execution.step(1,lambda *args:None)
        outcome,observation = execution.observe(token,sampler,start,lambda:False)
        assert outcome is GateOutcome.APPLIED
        if observation.settling == 'succeeded':
            break
    assert observation.settling == 'succeeded'
    assert sim.time-start >= max(curve.duration,0. if width is None else width/.009)
    notifications = []
    def notify():
        assert execution.active.token is token  # 先通知,再释放预约.
        notifications.append(True)
    assert execution.complete(token,sampler,lambda:False,notify) is GateOutcome.SUCCEEDED
    assert notifications == [True] and not execution.active.busy
    assert sim.target == pytest.approx(curve.points[-1].positions)
    if grip:
        assert grip.state == 'succeeded' and any(e[0]=='neutral' for e in sim.events)
    assert trace[0][2]['curve'].to_payload() == payload['curve']


@pytest.mark.parametrize('change',['scene','cancel','context','failure'])
def test_preparation_failure_and_late_reply_never_start_children(change):
    execution,sim,curve = _execution_fixture()
    token,payload = _reserve_simulation(execution,curve,width=.001)
    before = len(sim.events)
    if change == 'scene':
        execution.scene_changed('scene-b')
    elif change == 'cancel':
        execution.stop(token)
    elif change == 'context':
        payload['context']['generation'] += 1
    else:
        # 准备失败直接进入停止清理,未调用接纳.
        execution.stop(token)
    with pytest.raises(RuntimeError):
        execution.admit(token,payload,'scene-a',lambda:False)
    assert not any(e[0] in ('arm','finger') for e in sim.events[before:])
    assert execution.gripper.task is None
    execution.cleanup(token)
    other,_ = _reserve_simulation(execution,curve)
    before = len(sim.events)
    with pytest.raises(RuntimeError,match='ownership'):
        execution.admit(token,payload,'scene-a',lambda:False)
    execution.cleanup(token)
    assert not execution.stop(token)
    assert execution.active.token is other and len(sim.events) == before


@pytest.mark.parametrize('field', ['gripper_collision_policy', 'capability_revision'])
def test_preparation_requires_binding_fields(field):
    execution, sim, curve = _execution_fixture()
    token, payload = _reserve_simulation(execution, curve, width=.001)
    del payload[field]
    before = len(sim.events)
    with pytest.raises(KeyError, match=field):
        execution.admit(token, payload, 'scene-a', lambda: False)
    assert not any(event[0] in ('arm', 'finger') for event in sim.events[before:])
    execution.stop(token)
    execution.cleanup(token)


def test_unconfirmed_stop_retains_resources_and_old_wait_cannot_release_new_goal():
    execution,sim,curve = _execution_fixture()
    token,payload = _reserve_simulation(execution,curve,width=.001)
    execution.admit(token,payload,'scene-a',lambda:False)
    execution.step(5,lambda *args:None)
    assert execution.stop(token)
    wait = execution.begin_stop_wait(token)
    assert not execution.poll_stop(wait)
    execution.clock = lambda:16.
    with pytest.raises(RuntimeError,match='unconfirmed'):
        execution.poll_stop(wait)
    execution.cleanup(token)
    assert execution.active.token is token and execution.status_snapshot().stop_unconfirmed
    with pytest.raises(RuntimeError,match='trajectory active'):
        execution.change_mode('gravity_comp')
    # 新物理帧确认之后,由显式 Stop 收尾保留的原身份.
    execution.step(120,lambda *args:None)
    assert execution.poll_stop(wait)
    assert not execution.active.busy and execution.gripper.task.state == 'failed'
    other,_ = _reserve_simulation(execution,curve)
    assert execution.poll_stop(wait)
    execution.cleanup(token)
    assert execution.active.token is other


def test_hold_failure_remains_visible_and_keeps_resources_until_confirmed_stop():
    execution, sim, curve = _execution_fixture()
    token, payload = _reserve_simulation(execution, curve)
    execution.admit(token, payload, 'scene-a', lambda:False)
    hold = execution.control.hold_current_position
    def fail_hold():
        raise RuntimeError('hold transport failed')
    execution.control.hold_current_position = fail_hold
    with pytest.raises(RuntimeError, match='hold transport failed'):
        execution.stop(token)
    execution.cleanup(token)
    assert execution.active.token is token and execution.stop_unconfirmed
    with pytest.raises(RuntimeError, match='reserved'):
        _reserve_simulation(execution, curve)
    execution.control.hold_current_position = hold
    assert execution.stop(token)
    wait = execution.begin_stop_wait(token)
    assert not execution.poll_stop(wait)
    execution.step(120, lambda *args:None)
    assert execution.poll_stop(wait)
    assert not execution.active.busy and not execution.stop_unconfirmed


def test_idle_hold_failure_blocks_admission_until_explicit_stop_confirmation():
    execution, sim, curve = _execution_fixture()
    hold = execution.control.hold_current_position
    def fail_hold():
        raise RuntimeError('idle hold failed')
    execution.control.hold_current_position = fail_hold
    with pytest.raises(RuntimeError, match='idle hold failed'):
        execution.stop()
    with pytest.raises(RuntimeError, match='stop unconfirmed'):
        _reserve_simulation(execution, curve)
    execution.control.hold_current_position = hold
    assert not execution.stop()
    wait = execution.begin_stop_wait(None)
    execution.step(120, lambda *args:None)
    assert execution.poll_stop(wait) and not execution.stop_unconfirmed
    token, _ = _reserve_simulation(execution, curve)
    assert execution.active.token is token


@pytest.mark.parametrize('entry', ['goal', 'execute', 'service'])
def test_unconfirmed_idle_stop_rejects_every_gripper_entry(entry):
    from rclpy.action import GoalResponse
    from rebotarm_simulation.ros.mujoco_ros_node import create_node_class
    execution, sim, _ = _execution_fixture()
    execution.stop_unconfirmed = True
    errors = []
    owner = SimpleNamespace(_execution=execution, _sim_access=execution.access,
        get_logger=lambda:SimpleNamespace(error=errors.append), _closing=threading.Event())
    node_class = create_node_class()
    before = list(sim.events)
    if entry == 'goal':
        assert node_class._gripper_goal_callback(owner, None) is GoalResponse.REJECT
    elif entry == 'execute':
        terminal = []
        goal = SimpleNamespace(request=SimpleNamespace(command=SimpleNamespace(
            position=.01, max_effort=1.)), abort=lambda:terminal.append('aborted'))
        node_class._execute_gripper(owner, goal)
        assert terminal == ['aborted'] and 'unconfirmed' in errors[0]
    else:
        response = node_class._gripper_service(owner,
            SimpleNamespace(position=.01,max_effort=1.), SimpleNamespace())
        assert not response.success
    assert execution.gripper.task is None and execution.stop_unconfirmed
    assert sim.events == before


def test_component_observation_is_read_only_and_mode_change_requires_idle():
    execution, sim, curve = _execution_fixture()
    token, payload = _reserve_simulation(execution, curve)
    sampler = execution.admit(token, payload, 'scene-a', lambda:False)
    start, _ = execution.accepted_context(token)
    with pytest.raises(RuntimeError, match='trajectory active'):
        execution.change_mode('gravity_comp')
    execution.step(10, lambda *args:None)
    before = list(sim.events)
    outcome, observation = execution.observe(token, sampler, start, lambda:False)
    assert outcome is GateOutcome.APPLIED and observation.settling == 'tracking'
    assert sim.events == before
    execution.stop(token)
    wait = execution.begin_stop_wait(token)
    execution.step(120, lambda *args:None)
    assert execution.poll_stop(wait)
    execution.cleanup(token)
    assert execution.change_mode('gravity_comp') == 'gravity_comp'


def test_ros_preparation_failure_reports_hold_failure_and_retains_reservation():
    from rebotarm_simulation.ros.mujoco_ros_node import create_node_class
    node_class = create_node_class()
    execution, sim, curve = _execution_fixture()
    token, _ = _reserve_simulation(execution, curve)
    def fail_prepare(*args):
        raise RuntimeError('preparation failed')
    def fail_hold():
        raise RuntimeError('hold transport failed')
    execution.control.hold_current_position = fail_hold
    key = bytes(range(16))
    terminal = []
    owner = SimpleNamespace(_execution=execution, _goal_tokens={key:token},
        _prepare_sampler=fail_prepare, get_logger=lambda:SimpleNamespace(error=lambda text:None))
    goal = SimpleNamespace(goal_id=SimpleNamespace(uuid=key), is_cancel_requested=False, is_active=True,
        abort=lambda:terminal.append('aborted'))
    result = node_class._execute_goal(owner, goal)
    assert terminal == ['aborted'] and result.error_code != 0
    assert 'preparation failed' in result.error_string and 'hold transport failed' in result.error_string
    assert execution.active.token is token and execution.stop_unconfirmed
    assert key not in owner._goal_tokens


def test_ros_goal_transport_binding_prevents_late_cancel_from_stopping_next_goal():
    from rebotarm_simulation.ros.mujoco_ros_node import create_node_class
    from rclpy.action import CancelResponse
    node_class = create_node_class()
    execution,sim,curve = _execution_fixture()
    old,_ = _reserve_simulation(execution,curve)
    owner = SimpleNamespace(_execution=execution,_goal_tokens={})
    key = bytes(range(16))
    goal = SimpleNamespace(goal_id=SimpleNamespace(uuid=key),is_active=True,execute=lambda:None)
    node_class._handle_accepted(owner,goal)
    assert owner._goal_tokens[key] is old
    execution.cleanup(old)
    current,_ = _reserve_simulation(execution,curve)
    before = len(sim.events)
    assert node_class._cancel_callback(owner,goal) is CancelResponse.REJECT
    assert execution.active.token is current and not execution.active.cancel_requested
    assert len(sim.events) == before


def _continuous_failure_callback_fixture(monkeypatch, *, hold_fails, abort_failure, web=False):
    """使用真实执行组件和 ROS 消息,仅替换物理故障及 Action 传输句柄."""
    import rclpy
    from action_msgs.msg import GoalStatus
    from rebotarm_simulation.ros.mujoco_ros_node import create_node_class

    execution, sim, curve = _execution_fixture()
    token, payload = _reserve_simulation(execution, curve)
    sampler = execution.admit(token, payload, 'scene-a', lambda:False)
    calls = []
    logs = []

    def hold():
        calls.append('hold')
        if hold_fails:
            raise ValueError('injected hold failure')

    def observe(*args):
        raise RuntimeError('injected execution failure')

    class Goal:
        goal_id = SimpleNamespace(uuid=bytes(range(16)))
        request = SimpleNamespace(move_gripper=False)
        is_cancel_requested = False
        status = GoalStatus.STATUS_EXECUTING
        abort_calls = 0

        @property
        def is_active(self):
            return self.status == GoalStatus.STATUS_EXECUTING

        def abort(self):
            self.abort_calls += 1
            calls.append('abort')
            if abort_failure == 'before_always' or (
                    abort_failure == 'before_once' and self.abort_calls == 1):
                raise OSError('injected abort before state transition')
            self.status = GoalStatus.STATUS_ABORTED
            if abort_failure == 'after':
                raise OSError('injected abort after state transition')

        def destroy(self):
            self.status = GoalStatus.STATUS_UNKNOWN

        def publish_feedback(self, feedback):
            pass

    logger = SimpleNamespace(**{level: (lambda message, level=level: logs.append((level, message)))
                               for level in ('error', 'warning', 'debug')})
    goal = Goal()
    owner = SimpleNamespace(_execution=execution,
        _closing=threading.Event(),
        _goal_tokens={bytes(goal.goal_id.uuid): token}, _sim_access=execution.access,
        _feedback_rate_hz=20., get_logger=lambda: logger,
        _prepare_sampler=lambda handle, token:sampler)
    def wait_stop(token):
        wait = execution.begin_stop_wait(token)
        execution.step(120, lambda *args:None)
        assert execution.poll_stop(wait)
    owner._wait_prepared_stop = wait_stop
    monkeypatch.setattr(execution.control, 'hold_current_position', hold)
    monkeypatch.setattr(execution, 'observe', observe)
    monkeypatch.setattr(rclpy, 'ok', lambda: True)
    callback = lambda handle: create_node_class()._execute_goal(owner, handle, web=web)
    return callback, owner, goal, calls, logs, logger


@pytest.mark.parametrize('hold_fails,abort_failure', [
    (False, None), (True, None), (False, 'before_always'), (True, 'before_always'),
    (False, 'after'),
])
@pytest.mark.parametrize('web', [False, True])
def test_continuous_ros_callback_reports_execution_and_cleanup_failure(
        monkeypatch, hold_fails, abort_failure, web):
    from control_msgs.action import FollowJointTrajectory

    callback, owner, goal, calls, logs, _ = _continuous_failure_callback_fixture(
        monkeypatch, hold_fails=hold_fails, abort_failure=abort_failure, web=web)
    result = callback(goal)
    assert result.error_code == FollowJointTrajectory.Result.INVALID_GOAL
    assert 'injected execution failure' in result.error_string
    expected = []
    if hold_fails:
        expected.append('stop: ValueError: injected hold failure')
    if abort_failure:
        expected.append('abort: OSError: injected abort')
    for reason in expected:
        assert reason in result.error_string
        assert any(reason in message for _, message in logs)
    assert calls == ['hold', 'abort']
    assert owner._execution.active.busy == hold_fails and owner._goal_tokens == {}
    assert goal.is_active == (abort_failure == 'before_always')


@pytest.mark.parametrize('web', [False, True])
def test_continuous_stop_timeout_retains_reservation_and_original_error(monkeypatch, web):
    callback, owner, goal, calls, logs, _ = _continuous_failure_callback_fixture(
        monkeypatch, hold_fails=False, abort_failure=None, web=web)
    def timeout(token):
        owner._execution.stop_unconfirmed = True
        raise RuntimeError('simulation stop/hold unconfirmed')
    owner._wait_prepared_stop = timeout
    result = callback(goal)
    assert 'injected execution failure' in result.error_string
    assert 'stop/hold unconfirmed' in result.error_string
    assert owner._execution.active.busy and owner._execution.stop_unconfirmed
    assert calls == ['hold', 'abort'] and owner._goal_tokens == {}
    assert any('stop/hold unconfirmed' in message for _, message in logs)


def test_continuous_ros_callback_keeps_existing_action_terminal_state(monkeypatch):
    from action_msgs.msg import GoalStatus

    callback, owner, goal, calls, logs, _ = _continuous_failure_callback_fixture(
        monkeypatch, hold_fails=True, abort_failure=None)
    goal.status = GoalStatus.STATUS_ABORTED
    result = callback(goal)
    assert calls == ['hold'] and goal.abort_calls == 0
    assert goal.status == GoalStatus.STATUS_ABORTED
    assert 'injected hold failure' in result.error_string
    assert owner._execution.active.busy and owner._execution.stop_unconfirmed


def test_continuous_ros_failure_finally_keeps_new_goal_ownership(monkeypatch):
    callback, owner, goal, calls, _, _ = _continuous_failure_callback_fixture(
        monkeypatch, hold_fails=False, abort_failure=None)
    execution = owner._execution
    old = execution.active.token
    sampler = execution.active.prepared.curve
    successor = [None]

    def late_failure(*args):
        execution.cleanup(old)
        current, payload = _reserve_simulation(execution, sampler)
        successor[0] = current
        execution.admit(current, payload, 'scene-a', lambda:False)
        raise RuntimeError('old callback failed after successor admission')

    monkeypatch.setattr(execution, 'observe', late_failure)
    result = callback(goal)
    assert 'old callback failed after successor admission' in result.error_string
    assert calls == ['abort'] and owner._goal_tokens == {}
    assert execution.active.token is successor[0] and not execution.active.cancel_requested
    assert execution.active.prepared.curve == sampler


@pytest.mark.parametrize('abort_failure', [None, 'before_once', 'before_always', 'after'])
def test_trajectory_action_server_delivers_failure_even_if_abort_keeps_failing(monkeypatch, abort_failure):
    """运行安装版 ActionServer 外层;结果 Future 代表传输层待发送结果,无 DDS 节点."""
    import asyncio
    from action_msgs.msg import GoalStatus
    from control_msgs.action import FollowJointTrajectory
    from rebotarm_simulation.ros.trajectory_action_server import TrajectoryActionServer
    from rclpy.task import Future

    callback, owner, goal, calls, logs, logger = _continuous_failure_callback_fixture(
        monkeypatch, hold_fails=True, abort_failure=abort_failure)
    future = Future()
    key = bytes(goal.goal_id.uuid)
    server = object.__new__(TrajectoryActionServer)
    server._logger, server._action_type = logger, FollowJointTrajectory
    server._result_futures, server._goal_handles = {key: future}, {key: goal}
    server._terminal_fault = None

    async def run():
        await asyncio.wait_for(server._execute_goal(callback, goal), timeout=.5)
        assert future.done()
        response = future.result()
        expected = GoalStatus.STATUS_UNKNOWN if abort_failure == 'before_always' else GoalStatus.STATUS_ABORTED
        assert response.status == expected
        assert goal.is_active == (abort_failure == 'before_always')
        assert response.result.error_code == FollowJointTrajectory.Result.INVALID_GOAL
        assert 'injected execution failure' in response.result.error_string
        assert 'injected hold failure' in response.result.error_string
        if abort_failure:
            assert 'injected abort' in response.result.error_string
        if abort_failure == 'before_always':
            assert 'previous status=2' in response.result.error_string
            assert server._goal_handles[key] is goal and server._result_futures[key].done()
            assert server._terminal_fault == response.result.error_string

    asyncio.run(run())
    assert goal.abort_calls == (2 if abort_failure in ('before_once', 'before_always') else 1)
    assert calls[0] == 'hold'
    assert owner._execution.active.busy and owner._execution.stop_unconfirmed
    assert owner._goal_tokens == {}
    diagnostic = '\n'.join(message for level, message in logs if level == 'error')
    assert 'injected execution failure' in diagnostic and 'injected hold failure' in diagnostic
    if abort_failure:
        assert 'injected abort' in diagnostic

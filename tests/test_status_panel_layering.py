from __future__ import annotations


def test_stop_request_reports_service_exception():
    from types import SimpleNamespace as NS
    from rebotarm_dashboard.teleop_status_panel_node import TeleopStatusPanelNode
    errors = []
    def fail(**_kwargs):
        raise RuntimeError('transport failed')
    owner = NS(_trajectory_stop_client=NS(wait_for_service=fail),
               get_logger=lambda: NS(error=errors.append))
    ok, message = TeleopStatusPanelNode._request_controller_trajectory_stop(owner, timeout_sec=.2)
    assert not ok
    assert 'transport failed' in message
    assert 'transport failed' in errors[-1]


def test_keyboard_disable_reports_stop_failure():
    import threading
    from types import SimpleNamespace as NS
    from rebotarm_dashboard.teleop_status_panel_node import TeleopStatusPanelNode
    updates = []
    owner = NS(
        _route_web_command=lambda *_args: None,
        _web_keyboard_lock=threading.Lock(),
        _web_keyboard_enabled=True,
        _request_controller_trajectory_stop=lambda **_kwargs: (False, "transport failed"),
        _store=NS(update_teleop_status=lambda *args: updates.append(args)),
    )
    result = TeleopStatusPanelNode._handle_keyboard_disable(owner)
    assert not result["trajectory_stop_requested"]
    assert "transport failed" in result["message"]
    assert updates[-1][1] == result


def test_status_callback_accepts_plain_text():
    from types import SimpleNamespace as NS
    from rebotarm_dashboard.teleop_status_panel_node import TeleopStatusPanelNode
    updates = []
    owner = NS(_store=NS(update_teleop_status=lambda key, value: updates.append((key, value))))
    TeleopStatusPanelNode._on_status(owner, 'status', NS(data='ready'))
    assert updates == [('status', 'ready')]


def test_incomplete_action_result_is_reported_as_failed():
    import threading
    from types import SimpleNamespace as NS
    from rebotarm_dashboard.teleop_status_panel_node import TeleopStatusPanelNode
    updates = []
    owner = NS(_store=NS(update_teleop_status=lambda key, value: updates.append((key, value))),
               _execute_lock=threading.Lock(), _execute_goal_handle=object())
    future = NS(result=lambda: NS(status=4, result=NS()))
    TeleopStatusPanelNode._on_execute_result(owner, future, NS())
    assert updates[-1][1]['state'] == 'failed'
    assert 'error_code' in updates[-1][1]['message']
    assert owner._execute_goal_handle is None


def test_incomplete_gripper_result_is_reported_as_failed():
    from types import SimpleNamespace as NS
    from rebotarm_dashboard.teleop_status_panel_node import TeleopStatusPanelNode
    updates = []
    owner = NS(_store=NS(update_teleop_status=lambda key, value: updates.append((key, value))))
    TeleopStatusPanelNode._on_gripper_result(owner, NS(result=lambda: NS(result=NS())), NS())
    assert updates[-1][1]['state'] == 'failed'
    assert 'reached_goal' in updates[-1][1]['message']


def test_web_cancel_reply_is_not_a_terminal_result_and_cannot_overwrite_next_goal():
    import threading
    from types import SimpleNamespace as NS
    from rebotarm_dashboard.teleop_status_panel_node import TeleopStatusPanelNode
    updates=[]
    goal=object()
    owner=NS(_execute_lock=threading.Lock(),_execute_goal_handle=goal,
        _store=NS(update_teleop_status=lambda key,value:updates.append(value)))
    future=NS(result=lambda:NS(goals_canceling=[]))
    TeleopStatusPanelNode._on_execute_cancel_response(owner,future,goal)
    assert updates[-1]['state']=='cancel_requested'
    for successor in (None,object()):
        owner._execute_goal_handle=successor
        TeleopStatusPanelNode._on_execute_cancel_response(owner,future,goal)
    assert len(updates)==1

from rebotarm_dashboard.arm_control_client import ArmControlClient
from rebotarm_motion.collision_precheck import (
    CollisionPrecheckConfig,
    CollisionPrechecker,
    select_collision_samples,
)
from rebotarm_motion.replay_runtime_monitor import (
    ReplayRuntimeMonitor,
    ReplayRuntimeMonitorConfig,
)
from rebotarm_teach.teach_record_client import TeachRecordClient
from rebotarm_teach.teach_replay_client import TeachReplayClient
from rebotarm_teach.teach_replay_coordinator import (
    TeachReplayCoordinator,
    TeachReplayLimits,
)
from rebotarm_teach.teach_replay_settings import TeachReplaySettingsProvider
from rebotarm_motion.teach_replay_start_align_precheck import (
    MoveItStartAlignPrecheckConfig,
    MoveItStartAlignPrechecker,
)
from rebotarm_motion.teach_replay_start_alignment import (
    MoveItStartAlignmentConfig,
    MoveItStartAligner,
)
from rebotarm_teach.teach_replay_trajectory_builder import (
    TeachReplayTrajectoryBuilder,
    TeachReplayTrajectoryConfig,
)
from rebotarm_teleop.web_teleop_client import WebTeleopClient


class _Future:
    def __init__(self, *, success: bool = True, message: str = "ok") -> None:
        self._response = type("Response", (), {"success": success, "message": message})()

    def done(self) -> bool:
        return True

    def result(self):
        return self._response


class _AsyncFuture:
    def __init__(self, response) -> None:
        self._response = response

    def done(self) -> bool:
        return True

    def result(self):
        return self._response


class _TriggerClient:
    def __init__(self, *, available: bool = True, success: bool = True, message: str = "ok") -> None:
        self.available = available
        self.success = success
        self.message = message
        self.calls = 0

    def wait_for_service(self, timeout_sec: float) -> bool:
        return self.available

    def call_async(self, request):
        self.calls += 1
        return _Future(success=self.success, message=self.message)


class _JointState:
    def __init__(self) -> None:
        self.name = []
        self.position = []


class _RobotState:
    def __init__(self) -> None:
        self.joint_state = _JointState()


class _StateValidityRequest:
    def __init__(self) -> None:
        self.group_name = ""
        self.robot_state = _RobotState()


class _Contact:
    def __init__(self, body_1: str, body_2: str) -> None:
        self.contact_body_1 = body_1
        self.contact_body_2 = body_2


class _StateValidityResponse:
    def __init__(self, *, valid: bool, contacts=None) -> None:
        self.valid = valid
        self.contacts = contacts or []


class _StateValidityClient:
    def __init__(self, responses, *, available: bool = True) -> None:
        self.responses = list(responses)
        self.available = available
        self.requests = []

    def service_is_ready(self) -> bool:
        return self.available

    def wait_for_service(self, timeout_sec: float) -> bool:
        return self.available

    def call_async(self, request):
        self.requests.append(request)
        return _AsyncFuture(self.responses.pop(0))


class _SetPathFuture:
    def __init__(self, *, success: bool = True, message: str = "path ok", normalized_path: str = "teleop_records/a.jsonl") -> None:
        self._response = type(
            "Response",
            (),
            {"success": success, "message": message, "normalized_path": normalized_path},
        )()

    def done(self) -> bool:
        return True

    def result(self):
        return self._response


class _SetPathClient:
    def __init__(self, *, available: bool = True, success: bool = True) -> None:
        self.available = available
        self.success = success
        self.requests = []

    def wait_for_service(self, timeout_sec: float) -> bool:
        return self.available

    def call_async(self, request):
        self.requests.append(request)
        return _SetPathFuture(success=self.success, normalized_path=request.record_path)


class _RecordPathRequest:
    def __init__(self) -> None:
        self.record_path = ""


class _Duration:
    def __init__(self) -> None:
        self.sec = 0
        self.nanosec = 0


class _TrajectoryPoint:
    def __init__(self) -> None:
        self.positions = []
        self.time_from_start = _Duration()


class _Trajectory:
    def __init__(self) -> None:
        self.joint_names = []
        self.points = []


class _FollowGoal:
    def __init__(self) -> None:
        self.trajectory = None


class _GripperCommand:
    def __init__(self) -> None:
        self.position = 0.0
        self.max_effort = 0.0


class _GripperGoal:
    def __init__(self) -> None:
        self.command = _GripperCommand()


class _ActionClient:
    def __init__(self, *, available: bool = True) -> None:
        self.available = available
        self.goals = []

    def wait_for_server(self, timeout_sec: float) -> bool:
        return self.available

    def send_goal_async(self, goal):
        self.goals.append(goal)
        return _Future()


class _GoalHandle:
    def __init__(self, *, raises: bool = False) -> None:
        self.raises = raises
        self.cancel_calls = 0

    def cancel_goal_async(self):
        self.cancel_calls += 1
        if self.raises:
            raise RuntimeError("cancel failed")
        return _Future()


class _TeachSample:
    def __init__(self, *, stamp: float, positions: tuple[float, ...]) -> None:
        self.stamp = stamp
        self.joint_names = ("joint1", "joint2")
        self.positions = positions
        self.velocities = ()


class _ReplayQuality:
    risk_level = "green"


class _RetimedPoint:
    def __init__(self, *, time_from_start: float, positions: tuple[float, ...], velocities: tuple[float, ...]) -> None:
        self.time_from_start = time_from_start
        self.positions = positions
        self.velocities = velocities


class _PreparedReplay:
    def __init__(self) -> None:
        self.samples = [_TeachSample(stamp=0.0, positions=(0.1, -0.1))]
        self.effective_replay_speed = 1.0
        self.after_quality = _ReplayQuality()
        self.retimed_points = [
            _RetimedPoint(time_from_start=0.0, positions=(0.1, -0.1), velocities=(0.0, 0.0)),
            _RetimedPoint(time_from_start=0.5, positions=(0.2, -0.2), velocities=(0.0, 0.0)),
        ]


class _PlanResult:
    def __init__(self, *, success: bool, message: str, trajectory) -> None:
        self.success = success
        self.message = message
        self.trajectory = trajectory


class _Planner:
    def __init__(self, trajectory) -> None:
        self.trajectory = trajectory
        self.calls = []

    def plan_joint_positions(self, **kwargs):
        self.calls.append(kwargs)
        return _PlanResult(success=True, message="planned", trajectory=self.trajectory)


class _MoveItServiceClient:
    def __init__(self, *, available: bool = True) -> None:
        self.available = available
        self.wait_calls = []

    def service_is_ready(self) -> bool:
        return self.available

    def wait_for_service(self, timeout_sec: float) -> bool:
        self.wait_calls.append(timeout_sec)
        return self.available


def test_arm_control_client_requests_stop_before_safe_home() -> None:
    safe_home = _TriggerClient(message="home")
    stop = _TriggerClient(message="stopped")
    client = ArmControlClient(
        enable_client=_TriggerClient(),
        disable_client=_TriggerClient(),
        safe_home_client=safe_home,
        trajectory_stop_client=stop,
    )

    result = client.execute("safe_home")

    assert result["accepted"] is True
    assert result["state"] == "done"
    assert result["command"] == "safe_home"
    assert result["trajectory_stop_requested"] is True
    assert stop.calls == 1
    assert safe_home.calls == 1


def test_arm_control_client_blocks_unknown_command_without_service_calls() -> None:
    enable = _TriggerClient()
    client = ArmControlClient(
        enable_client=enable,
        disable_client=_TriggerClient(),
        safe_home_client=_TriggerClient(),
        trajectory_stop_client=_TriggerClient(),
    )

    result = client.execute("bad")

    assert result["accepted"] is False
    assert result["state"] == "rejected"
    assert enable.calls == 0


def test_select_collision_samples_evenly_keeps_first_and_last() -> None:
    selected = select_collision_samples(list(range(10)), max_samples=4)

    assert selected == [(0, 0), (3, 3), (6, 6), (9, 9)]


def test_collision_transport_errors_remain_unknown_with_original_cause():
    def fail():
        raise RuntimeError('transport disconnected')
    for boundary in ('discovery','send','result'):
        client = _StateValidityClient([_StateValidityResponse(valid=True)])
        if boundary == 'discovery':
            client.service_is_ready = fail
        elif boundary == 'send':
            client.call_async = lambda request: fail()
        else:
            client.call_async = lambda request: type('Future',(),{
                'done': staticmethod(lambda: True), 'result': staticmethod(fail)})()
        prechecker = CollisionPrechecker(client=client,request_factory=_StateValidityRequest)
        result = prechecker.check_positions(joint_names=('joint1',),positions_list=[(0.,)],
            config=CollisionPrecheckConfig(True,'/check_state_validity','arm',10,1.))
        assert result['state'] == 'unknown'
        assert 'transport disconnected' in result['message']
        assert result['checked_samples'] == 0


def test_collision_prechecker_returns_pass_after_sampling_state_validity() -> None:
    client = _StateValidityClient(
        [
            _StateValidityResponse(valid=True),
            _StateValidityResponse(valid=True),
            _StateValidityResponse(valid=True),
        ]
    )
    prechecker = CollisionPrechecker(
        client=client,
        request_factory=_StateValidityRequest,
    )

    result = prechecker.check_positions(
        joint_names=("joint1", "joint2"),
        positions_list=[(0.0, 0.0), (0.1, 0.1), (0.2, 0.2), (0.3, 0.3), (0.4, 0.4)],
        config=CollisionPrecheckConfig(
            enabled=True,
            service="/check_state_validity",
            group_name="",
            max_samples=3,
            timeout_sec=1.0,
        ),
    )

    assert result["state"] == "pass"
    assert result["checked_samples"] == 3
    assert result["requested_samples"] == 3
    assert [request.robot_state.joint_state.position for request in client.requests] == [
        [0.0, 0.0],
        [0.2, 0.2],
        [0.4, 0.4],
    ]
    assert client.requests[0].group_name == ""


def test_collision_prechecker_appends_actual_gripper_joints_for_full_robot() -> None:
    client = _StateValidityClient([_StateValidityResponse(valid=True)])
    prechecker = CollisionPrechecker(
        client=client,
        request_factory=_StateValidityRequest,
    )

    result = prechecker.check_positions(
        joint_names=("joint1", "joint2"),
        positions_list=[(0.1, 0.2)],
        config=CollisionPrecheckConfig(
            enabled=True,
            service="/check_state_validity",
            group_name="",
            max_samples=1,
            timeout_sec=1.0,
            default_joint_positions=(
                ("left_finger_joint", 0.03),
                ("right_finger_joint", -0.03),
            ),
        ),
    )

    request = client.requests[0]
    assert result["state"] == "pass"
    assert result["added_default_joints"] == ["left_finger_joint", "right_finger_joint"]
    assert request.robot_state.joint_state.name == [
        "joint1",
        "joint2",
        "left_finger_joint",
        "right_finger_joint",
    ]
    assert request.robot_state.joint_state.position == [0.1, 0.2, 0.03, -0.03]


def test_collision_prechecker_returns_collision_with_compact_contacts() -> None:
    client = _StateValidityClient(
        [
            _StateValidityResponse(
                valid=False,
                contacts=[
                    _Contact("left_finger_link", "table"),
                    _Contact("right_finger_link", "object"),
                ],
            )
        ]
    )
    prechecker = CollisionPrechecker(
        client=client,
        request_factory=_StateValidityRequest,
    )

    result = prechecker.check_positions(
        joint_names=("joint1",),
        positions_list=[(0.0,)],
        config=CollisionPrecheckConfig(
            enabled=True,
            service="/check_state_validity",
            group_name="",
            max_samples=10,
            timeout_sec=1.0,
        ),
    )

    assert result["state"] == "collision"
    assert result["collisions"] == [
        {
            "sample": 0,
            "contacts": [
                {"body_1": "left_finger_link", "body_2": "table"},
                {"body_1": "right_finger_link", "body_2": "object"},
            ],
        }
    ]


def test_replay_runtime_monitor_waits_for_violation_grace_before_stopping() -> None:
    monitor = ReplayRuntimeMonitor()
    trajectory = {
        "joint_names": ["joint1"],
        "points": [
            {"time_from_start": 0.0, "positions": [0.0]},
            {"time_from_start": 2.0, "positions": [0.0]},
        ],
    }
    config = ReplayRuntimeMonitorConfig(
        enabled=True,
        start_grace_sec=0.5,
        violation_grace_sec=0.3,
        max_tracking_error_rad=0.1,
        max_live_velocity_rad_s=5.0,
    )

    first = monitor.check(
        trajectory=trajectory,
        started_at=10.0,
        joints={"joint1": {"position": 1.0, "velocity": 0.0}},
        now=11.0,
        config=config,
    )
    second = monitor.check(
        trajectory=trajectory,
        started_at=10.0,
        joints={"joint1": {"position": 1.0, "velocity": 0.0}},
        now=11.2,
        config=config,
    )
    third = monitor.check(
        trajectory=trajectory,
        started_at=10.0,
        joints={"joint1": {"position": 1.0, "velocity": 0.0}},
        now=11.31,
        config=config,
    )

    assert first.should_stop is False
    assert second.should_stop is False
    assert third.should_stop is True
    assert third.status["state"] == "safety_stop"
    assert third.status["runtime_monitor"]["tracking_error"] is True
    assert monitor.stop_requested is True


def test_replay_runtime_monitor_clears_pending_violation_after_recovery() -> None:
    monitor = ReplayRuntimeMonitor()
    trajectory = {
        "joint_names": ["joint1"],
        "points": [{"time_from_start": 0.0, "positions": [0.0]}],
    }
    config = ReplayRuntimeMonitorConfig(
        enabled=True,
        start_grace_sec=0.0,
        violation_grace_sec=0.2,
        max_tracking_error_rad=0.1,
        max_live_velocity_rad_s=5.0,
    )

    monitor.check(
        trajectory=trajectory,
        started_at=0.0,
        joints={"joint1": {"position": 1.0, "velocity": 0.0}},
        now=1.0,
        config=config,
    )
    recovered = monitor.check(
        trajectory=trajectory,
        started_at=0.0,
        joints={"joint1": {"position": 0.0, "velocity": 0.0}},
        now=1.1,
        config=config,
    )

    assert recovered.should_stop is False
    assert monitor.violation_since is None
    assert monitor.stop_requested is False


def test_missing_or_nonfinite_feedback_stops_after_grace_and_latches():
    trajectory = dict(joint_names=['joint1'],points=[dict(time_from_start=0.,positions=[0.])])
    config = ReplayRuntimeMonitorConfig(True,0.,.2,.1,5.)
    for joints, reason in [({'joint1': {'position': 0.}}, 'missing_joint_state'),
            ({'joint1': {'velocity': 0.}}, 'missing_joint_state'),
            ({}, 'missing_joint_state'),
            ({'joint1': {'position': float('nan'), 'velocity': 0.}}, 'invalid_joint_state'),
            ({'joint1': {'position': 0., 'velocity': float('inf')}}, 'invalid_joint_state')]:
        monitor = ReplayRuntimeMonitor()
        assert not monitor.check(trajectory=trajectory,started_at=0.,joints=joints,now=1.,config=config).should_stop
        decision = monitor.check(trajectory=trajectory,started_at=0.,joints=joints,now=1.21,config=config)
        assert decision.should_stop and decision.status['runtime_monitor']['reason'] == reason
        assert not monitor.check(trajectory=trajectory,started_at=0.,joints=joints,now=1.5,config=config).should_stop
        monitor.reset()
        assert not monitor.check(trajectory=trajectory,started_at=0.,joints=joints,now=2.,config=config).should_stop
        good = {'joint1': {'position': 0., 'velocity': 0.}}
        assert not monitor.check(trajectory=trajectory,started_at=0.,joints=good,now=2.1,config=config).should_stop
        assert monitor.violation_since is None


def test_tracking_rejects_partial_velocity_and_nonfinite_feedback_but_accepts_optional_empty_velocity():
    from rebotarm_motion.trajectory_safety_monitor import evaluate_replay_tracking
    trajectory = dict(joint_names=['joint1','joint2'],points=[dict(time_from_start=0.,positions=[0.,0.])])
    kwargs = dict(joint_names=['joint1','joint2'], positions=[0.,0.],elapsed_sec=0.,
        max_tracking_error_rad=.1,max_live_velocity_rad_s=5.)
    assert evaluate_replay_tracking(trajectory,**kwargs).ok
    assert evaluate_replay_tracking(trajectory,**kwargs,velocities=[0.]).reason == 'missing_joint_state'
    for values in ([0.,float('nan')], [0.,float('inf')], [0.,'bad']):
        assert evaluate_replay_tracking(trajectory,**kwargs,velocities=values).reason == 'invalid_joint_state'


def test_teach_record_client_sets_path_then_starts_gravity_and_recording() -> None:
    set_path = _SetPathClient()
    gravity = _TriggerClient(message="gravity")
    record = _TriggerClient(message="record")
    client = TeachRecordClient(
        set_path_client=set_path,
        start_client=record,
        stop_client=_TriggerClient(),
        gravity_start_client=gravity,
        gravity_stop_client=_TriggerClient(),
        record_path_request_factory=_RecordPathRequest,
    )

    result = client.start({"record_path": "teleop_records/teach1.jsonl"})

    assert result["accepted"] is True
    assert result["state"] == "starting"
    assert result["record_path"] == "teleop_records/teach1.jsonl"
    assert set_path.requests[0].record_path == "teleop_records/teach1.jsonl"
    assert gravity.calls == 1
    assert record.calls == 1


def test_teach_record_client_stop_stops_recording_and_gravity() -> None:
    record_stop = _TriggerClient(message="record stopped")
    gravity_stop = _TriggerClient(message="gravity stopped")
    client = TeachRecordClient(
        set_path_client=_SetPathClient(),
        start_client=_TriggerClient(),
        stop_client=record_stop,
        gravity_start_client=_TriggerClient(),
        gravity_stop_client=gravity_stop,
        record_path_request_factory=_RecordPathRequest,
    )

    result = client.stop()

    assert result["accepted"] is True
    assert result["state"] == "stopped"
    assert record_stop.calls == 1
    assert gravity_stop.calls == 1


def test_teach_record_client_does_not_start_recording_when_gravity_start_fails() -> None:
    gravity = _TriggerClient(success=False, message="gravity service unavailable")
    record = _TriggerClient()
    client = TeachRecordClient(
        set_path_client=_SetPathClient(),
        start_client=record,
        stop_client=_TriggerClient(),
        gravity_start_client=gravity,
        gravity_stop_client=_TriggerClient(),
        record_path_request_factory=_RecordPathRequest,
    )

    result = client.start({"record_path": "teach1.jsonl"})

    assert result["accepted"] is False
    assert result["state"] == "blocked"
    assert result["record_started"] is False
    assert record.calls == 0


def test_web_teleop_client_builds_and_sends_joint_trajectory() -> None:
    action_client = _ActionClient()
    client = WebTeleopClient(
        action_client=action_client,
        joint_names=("joint1", "joint2"),
        joint_limits={"joint1": (-1.0, 1.0), "joint2": (-1.0, 1.0)},
        joint_velocity_limits={"joint1": 2.0, "joint2": 2.0},
        trajectory_factory=_Trajectory,
        trajectory_point_factory=_TrajectoryPoint,
        follow_goal_factory=_FollowGoal,
    )

    result = client.execute(
        {"confirm": "EXECUTE", "joint_positions": {"joint1": 0.2, "joint2": -0.1}, "duration": 1.0},
        current_positions={"joint1": 0.0, "joint2": 0.0},
        max_delta_rad=1.0,
        min_duration=0.2,
        max_duration=5.0,
        max_joint_speed_rad_s=2.0,
    )

    assert result["accepted"] is True
    assert result["status"]["state"] == "active"
    assert result["goal_future"] is not None
    assert len(action_client.goals) == 1
    trajectory = action_client.goals[0].trajectory
    assert trajectory.joint_names == ["joint1", "joint2"]
    assert trajectory.points[0].positions == [0.0, 0.0]
    assert trajectory.points[-1].positions == [0.2, -0.1]


def test_web_teleop_client_rejects_when_action_server_unavailable() -> None:
    client = WebTeleopClient(
        action_client=_ActionClient(available=False),
        joint_names=("joint1",),
        joint_limits={"joint1": (-1.0, 1.0)},
        joint_velocity_limits={"joint1": 2.0},
        trajectory_factory=_Trajectory,
        trajectory_point_factory=_TrajectoryPoint,
        follow_goal_factory=_FollowGoal,
    )

    result = client.execute(
        {"confirm": "EXECUTE", "joint_positions": {"joint1": 0.2}, "duration": 1.0},
        current_positions={"joint1": 0.0},
        max_delta_rad=1.0,
        min_duration=0.2,
        max_duration=5.0,
        max_joint_speed_rad_s=2.0,
    )

    assert result["accepted"] is False
    assert result["status"]["state"] == "unavailable"


def test_web_teleop_client_stop_requests_cancel_and_controller_stop() -> None:
    stop = _TriggerClient(message="stopped")
    goal_handle = _GoalHandle()
    client = WebTeleopClient(
        action_client=_ActionClient(),
        joint_names=("joint1",),
        joint_limits={"joint1": (-1.0, 1.0)},
        joint_velocity_limits={"joint1": 2.0},
        trajectory_factory=_Trajectory,
        trajectory_point_factory=_TrajectoryPoint,
        follow_goal_factory=_FollowGoal,
    )

    result = client.stop(goal_handle, trajectory_stop_client=stop)

    assert result["accepted"] is True
    assert result["status"]["state"] == "cancel_requested"
    assert result["clear_goal_handle"] is True
    assert result["cancel_future"] is not None
    assert goal_handle.cancel_calls == 1
    assert stop.calls == 1


def test_web_teleop_client_stop_without_goal_uses_controller_stop_fallback() -> None:
    stop = _TriggerClient(message="stopped")
    client = WebTeleopClient(
        action_client=_ActionClient(),
        joint_names=("joint1",),
        joint_limits={"joint1": (-1.0, 1.0)},
        joint_velocity_limits={"joint1": 2.0},
        trajectory_factory=_Trajectory,
        trajectory_point_factory=_TrajectoryPoint,
        follow_goal_factory=_FollowGoal,
    )

    result = client.stop(None, trajectory_stop_client=stop)

    assert result["accepted"] is True
    assert result["status"]["state"] == "cancel_requested"
    assert result["trajectory_stop_requested"] is True
    assert stop.calls == 1


def test_web_teleop_client_simulates_gripper_without_action_goal() -> None:
    action_client = _ActionClient()
    client = WebTeleopClient(
        action_client=_ActionClient(),
        joint_names=("joint1",),
        joint_limits={"joint1": (-1.0, 1.0)},
        joint_velocity_limits={"joint1": 2.0},
        trajectory_factory=_Trajectory,
        trajectory_point_factory=_TrajectoryPoint,
        follow_goal_factory=_FollowGoal,
        gripper_action_client=action_client,
        gripper_goal_factory=_GripperGoal,
    )

    result = client.set_gripper(
        {"confirm": "SET_GRIPPER", "position": 0.03, "max_effort": 0.4},
        use_hardware=False,
        gripper_limits=(0.0, 0.045),
        default_max_effort=0.3,
        max_effort_limit=1.5,
    )

    assert result["accepted"] is True
    assert result["status"]["state"] == "done"
    assert result["simulated_position"] == 0.03
    assert result["goal_future"] is None
    assert action_client.goals == []


def test_web_teleop_client_sends_gripper_goal_for_hardware() -> None:
    action_client = _ActionClient()
    client = WebTeleopClient(
        action_client=_ActionClient(),
        joint_names=("joint1",),
        joint_limits={"joint1": (-1.0, 1.0)},
        joint_velocity_limits={"joint1": 2.0},
        trajectory_factory=_Trajectory,
        trajectory_point_factory=_TrajectoryPoint,
        follow_goal_factory=_FollowGoal,
        gripper_action_client=action_client,
        gripper_goal_factory=_GripperGoal,
    )

    result = client.set_gripper(
        {"confirm": "SET_GRIPPER", "position": 0.02, "max_effort": 0.5},
        use_hardware=True,
        gripper_limits=(0.0, 0.045),
        default_max_effort=0.3,
        max_effort_limit=1.5,
    )

    assert result["accepted"] is True
    assert result["status"]["state"] == "active"
    assert result["goal_future"] is not None
    assert len(action_client.goals) == 1
    assert action_client.goals[0].command.position == 0.02
    assert action_client.goals[0].command.max_effort == 0.5


def test_teach_replay_client_stop_cancels_goal_and_requests_controller_stop() -> None:
    stop = _TriggerClient(message="stopped")
    goal_handle = _GoalHandle()
    client = TeachReplayClient()

    result = client.stop(goal_handle, trajectory_stop_client=stop)

    assert result["accepted"] is True
    assert result["state"] == "cancel_requested"
    assert result["cancel_future"] is not None
    assert goal_handle.cancel_calls == 1
    assert stop.calls == 1


def test_teach_replay_client_stop_without_goal_uses_controller_stop() -> None:
    stop = _TriggerClient(message="stopped")
    client = TeachReplayClient()

    result = client.stop(None, trajectory_stop_client=stop)

    assert result["accepted"] is True
    assert result["state"] == "stop_requested"
    assert result["cancel_future"] is None
    assert stop.calls == 1


def test_teach_replay_coordinator_builds_blocked_dry_run_payload() -> None:
    coordinator = TeachReplayCoordinator()
    decision = type(
        "Decision",
        (),
        {"accepted": True, "state": "dry_run", "message": "dry-run accepted"},
    )()

    result = coordinator.build_dry_run_result(
        info_payload={
            "path": "teleop_records/teach1.jsonl",
            "start_band": "align",
            "max_error": 0.2,
            "worst_joint": "joint1",
            "samples": 100,
            "duration_sec": 2.0,
            "quality": {"risk_level": "green"},
        },
        settings={"replay_speed": 0.5, "align_duration": 3.0, "align_steps": 20, "final_hold_sec": 1.0},
        decision=decision,
        prepared_payload={
            "prepared_samples": 120,
            "after_quality": {
                "risk_level": "green",
                "max_jump_rad": 0.01,
                "max_acceleration_rad_s2": 2.0,
                "max_jerk_rad_s3": 10.0,
            },
        },
        prepared_record_path="teleop_records/teach1.prepared.jsonl",
        moveit_align={"state": "ready", "message": "MoveIt ready"},
        collision_precheck={"state": "unknown", "message": "state validity service unavailable"},
        trajectory_points=0,
        limits=TeachReplayLimits(
            max_prepared_jump_rad=0.02,
            max_replay_acceleration_rad_s2=5.0,
            max_replay_jerk_rad_s3=20.0,
        ),
        target_runtime="simulation",
    )

    assert result["accepted"] is False
    assert result["state"] == "blocked"
    assert "MoveIt/collision precheck blocked real replay" in result["message"]
    assert result["trajectory_points"] == 120
    assert result["estimated_duration_sec"] == 8.0
    assert result["prepared_risk_level"] == "green"
    assert result["target_runtime"] == "simulation"
    assert result["dry_run"] is True


def test_teach_replay_coordinator_builds_execute_rejection_payload() -> None:
    coordinator = TeachReplayCoordinator()
    decision = type(
        "Decision",
        (),
        {"accepted": False, "state": "blocked", "message": "real replay requires a successful dry-run first"},
    )()

    result = coordinator.build_execute_result(
        info_payload={
            "path": "teleop_records/teach1.jsonl",
            "start_band": "direct",
            "max_error": 0.01,
            "samples": 5,
            "duration_sec": 0.5,
            "quality": {"risk_level": "green"},
        },
        settings={"replay_speed": 1.0, "align_duration": 3.0, "align_steps": 20, "final_hold_sec": 1.0},
        decision=decision,
        prepared_payload={"after_quality": {"risk_level": "green", "max_jump_rad": 0.01}},
        prepared_record_path="teleop_records/teach1.prepared.jsonl",
        moveit_align={"state": "ready", "message": "MoveIt ready"},
        collision_precheck={"state": "pass", "message": "ok"},
        trajectory_points=0,
        limits=TeachReplayLimits(
            max_prepared_jump_rad=0.02,
            max_replay_acceleration_rad_s2=5.0,
            max_replay_jerk_rad_s3=20.0,
        ),
        target_runtime="hardware",
    )

    assert result["accepted"] is False
    assert result["state"] == "blocked"
    assert result["dry_run"] is False
    assert result["record_path"] == "teleop_records/teach1.jsonl"
    assert result["target_runtime"] == "hardware"


def test_teach_replay_coordinator_accepts_execute_when_dry_run_token_matches() -> None:
    coordinator = TeachReplayCoordinator()
    settings = {"replay_speed": 1.0, "align_duration": 3.0, "align_steps": 20, "final_hold_sec": 1.0}

    decision = coordinator.evaluate_execute_request(
        info_payload={"path": "teleop_records/teach1.jsonl", "start_band": "direct", "quality": {"risk_level": "green"}},
        settings=settings,
        prepared_quality={"risk_level": "green", "max_jump_rad": 0.01},
        dry_run_token={
            "accepted": True,
            "record_path": "teleop_records/teach1.jsonl",
            "prepared_risk_level": "green",
            "settings": settings,
        },
        limits=TeachReplayLimits(
            max_prepared_jump_rad=0.02,
            max_replay_acceleration_rad_s2=5.0,
            max_replay_jerk_rad_s3=20.0,
        ),
        yellow_max_speed=0.6,
    )

    assert decision.accepted is True
    assert decision.state == "replaying"


def test_teach_replay_coordinator_blocks_execute_when_dry_run_token_is_stale() -> None:
    coordinator = TeachReplayCoordinator()
    settings = {"replay_speed": 1.0, "align_duration": 3.0, "align_steps": 20, "final_hold_sec": 1.0}

    decision = coordinator.evaluate_execute_request(
        info_payload={"path": "teleop_records/new.jsonl", "start_band": "direct", "quality": {"risk_level": "green"}},
        settings=settings,
        prepared_quality={"risk_level": "green", "max_jump_rad": 0.01},
        dry_run_token={
            "accepted": True,
            "record_path": "teleop_records/old.jsonl",
            "prepared_risk_level": "green",
            "settings": settings,
        },
        limits=TeachReplayLimits(
            max_prepared_jump_rad=0.02,
            max_replay_acceleration_rad_s2=5.0,
            max_replay_jerk_rad_s3=20.0,
        ),
        yellow_max_speed=0.6,
    )

    assert decision.accepted is False
    assert decision.message == "real replay requires a successful dry-run first"


def test_teach_replay_trajectory_builder_appends_soft_start_retimed_points_and_final_hold() -> None:
    builder = TeachReplayTrajectoryBuilder(
        trajectory_factory=_Trajectory,
        trajectory_point_factory=_TrajectoryPoint,
    )

    result = builder.build(
        prepared=_PreparedReplay(),
        current_positions={"joint1": 0.0, "joint2": 0.0},
        start_band="direct",
        settings={"align_duration": 3.0, "align_steps": 20, "final_hold_sec": 1.0},
        config=TeachReplayTrajectoryConfig(
            use_moveit_start_align=False,
            start_hold_sec=0.1,
            soft_start_duration=0.2,
            soft_start_steps=2,
            first_hold_sec=0.0,
            yellow_max_speed=0.6,
            initial_replay_delay_sec=0.05,
            max_velocity_rad_s=2.0,
            max_acceleration_rad_s2=5.0,
            max_jerk_rad_s3=20.0,
        ),
    )

    trajectory = result.trajectory

    assert trajectory.joint_names == ["joint1", "joint2"]
    assert [round(point.time_from_start.sec + point.time_from_start.nanosec * 1e-9, 3) for point in trajectory.points] == [
        0.1,
        0.3,
        0.35,
        0.85,
        1.85,
    ]
    assert trajectory.points[-1].positions == [0.2, -0.2]
    assert trajectory.points[-1].velocities == [0.0, 0.0]


def test_teach_replay_trajectory_builder_calls_moveit_alignment_with_keywords() -> None:
    builder = TeachReplayTrajectoryBuilder(
        trajectory_factory=_Trajectory,
        trajectory_point_factory=_TrajectoryPoint,
    )
    calls = []

    def _align(trajectory, *, current_positions, first_positions, reference_grade):
        assert reference_grade == 'q/v'
        calls.append((trajectory, current_positions, first_positions))
        point = _TrajectoryPoint()
        point.positions = list(first_positions)
        point.velocities = [0.]*len(first_positions)
        point.accelerations = []
        trajectory.points.append(point)
        return 0.25

    result = builder.build(
        prepared=_PreparedReplay(),
        current_positions={"joint1": 0.0, "joint2": 0.0},
        start_band="align",
        settings={"align_duration": 3.0, "align_steps": 20, "final_hold_sec": 1.0},
        config=TeachReplayTrajectoryConfig(
            use_moveit_start_align=True,
            start_hold_sec=0.1,
            soft_start_duration=0.2,
            soft_start_steps=2,
            first_hold_sec=0.0,
            yellow_max_speed=0.6,
            initial_replay_delay_sec=0.05,
            max_velocity_rad_s=2.0,
            max_acceleration_rad_s2=5.0,
            max_jerk_rad_s3=20.0,
        ),
        moveit_start_alignment=_align,
    )

    assert calls[0][1] == (0.0, 0.0)
    assert calls[0][2] == (0.1, -0.1)
    assert result.trajectory.points[0].positions == [0.1, -0.1]


def test_moveit_start_aligner_appends_hold_plan_and_first_hold_with_joint_remap() -> None:
    plan = _Trajectory()
    plan.joint_names = ["joint2", "joint1"]
    point = _TrajectoryPoint()
    point.positions = [-0.2, 0.2]
    point.velocities = [-0.1, 0.1]
    point.time_from_start.sec = 2
    plan.points.append(point)
    planner = _Planner(plan)
    trajectory = _Trajectory()
    trajectory.joint_names = ["joint1", "joint2"]
    aligner = MoveItStartAligner(
        planner=planner,
        trajectory_point_factory=_TrajectoryPoint,
    )

    elapsed = aligner.append(
        trajectory,
        current_positions=(0.0, 0.0),
        first_positions=(0.2, -0.2),
        config=MoveItStartAlignmentConfig(
            start_hold_sec=0.5,
            first_hold_sec=0.25,
            skip_threshold=0.01,
            joint_goal_tolerance=0.02,
            velocity_scaling=0.4,
            acceleration_scaling=0.3,
        ),
    )

    assert round(elapsed, 3) == 2.75
    assert planner.calls[0]["joint_names"] == ("joint1", "joint2")
    assert planner.calls[0]["target_positions"] == (0.2, -0.2)
    assert trajectory.points[0].positions == [0.0, 0.0]
    assert trajectory.points[1].positions == [0.2, -0.2]
    # 本例唯一规划点是终点，明确连接静止保持；缺 a 仍保持 C1。
    assert trajectory.points[1].velocities == [0.0, 0.0]
    assert trajectory.points[1].accelerations == []
    assert trajectory.points[-1].positions == [0.2, -0.2]
    assert trajectory.points[-1].velocities == [0.0, 0.0]


def test_moveit_start_align_prechecker_reports_ready_without_planning() -> None:
    prechecker = MoveItStartAlignPrechecker(
        planner=_Planner(_Trajectory()),
        service_client=_MoveItServiceClient(available=True),
    )

    result = prechecker.summary(
        {"max_error": 0.2},
        config=MoveItStartAlignPrecheckConfig(
            enabled=True,
            service="/plan_kinematic_path",
            skip_threshold=0.05,
            joint_goal_tolerance=0.02,
            velocity_scaling=0.4,
            acceleration_scaling=0.3,
        ),
        samples=[],
        plan=False,
    )

    assert result == {
        "state": "ready",
        "message": "MoveIt planning service ready",
        "max_error": 0.2,
        "skip_threshold": 0.05,
        "service": "/plan_kinematic_path",
    }


def test_moveit_start_align_prechecker_plans_to_first_teach_sample() -> None:
    plan = _Trajectory()
    plan.points.append(_TrajectoryPoint())
    planner = _Planner(plan)
    prechecker = MoveItStartAlignPrechecker(
        planner=planner,
        service_client=_MoveItServiceClient(available=True),
    )

    result = prechecker.summary(
        {"max_error": 0.2},
        config=MoveItStartAlignPrecheckConfig(
            enabled=True,
            service="/plan_kinematic_path",
            skip_threshold=0.05,
            joint_goal_tolerance=0.02,
            velocity_scaling=0.4,
            acceleration_scaling=0.3,
        ),
        samples=[_TeachSample(stamp=0.0, positions=(0.1, -0.1))],
        plan=True,
    )

    assert result["state"] == "planned"
    assert result["message"] == "planned"
    assert result["points"] == 1
    assert planner.calls[0]["joint_names"] == ("joint1", "joint2")
    assert planner.calls[0]["target_positions"] == (0.1, -0.1)


def test_moveit_start_align_prechecker_reports_unavailable_service() -> None:
    prechecker = MoveItStartAlignPrechecker(
        planner=_Planner(_Trajectory()),
        service_client=_MoveItServiceClient(available=False),
    )

    result = prechecker.summary(
        {"max_error": 0.2},
        config=MoveItStartAlignPrecheckConfig(
            enabled=True,
            service="/plan_kinematic_path",
            skip_threshold=0.05,
            joint_goal_tolerance=0.02,
            velocity_scaling=0.4,
            acceleration_scaling=0.3,
        ),
        samples=[],
        plan=False,
    )

    assert result["state"] == "unavailable"
    assert result["service"] == "/plan_kinematic_path"


def test_teach_replay_settings_provider_uses_auto_align_duration() -> None:
    provider = TeachReplaySettingsProvider(
        replay_speed=1.0,
        align_duration=3.0,
        align_duration_auto=True,
        align_target_speed_rad_s=0.1,
        align_min_duration=2.0,
        align_max_duration=8.0,
        align_steps=30,
    )

    settings = provider.from_payload({"settings": {"replay_speed": 1.2}}, max_error=0.5)

    assert settings["replay_speed"] == 1.0
    assert settings["align_duration"] == 5.0
    assert settings["align_steps"] == 30
    assert settings["final_hold_sec"] == 1.0


def test_teach_replay_settings_provider_honors_manual_align_duration_when_disabled() -> None:
    provider = TeachReplaySettingsProvider(
        replay_speed=1.0,
        align_duration=3.0,
        align_duration_auto=False,
        align_target_speed_rad_s=0.1,
        align_min_duration=2.0,
        align_max_duration=8.0,
        align_steps=30,
    )

    settings = provider.from_payload({"settings": {"align_duration": 6.0}}, max_error=0.5)

    assert settings["align_duration"] == 6.0

from rebotarm_vision.visual_grasp_service_gateway import VisualGraspServiceGateway
"""Pure-planning preview: virtual start must never leak into real execution."""

from types import SimpleNamespace

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rebotarm_msgs.srv import ExecutePose, PublishTrajectoryPreview

from rebotarm_motion.moveit_planner import MoveItMotionPlanner
from rebotarm_motion.pose_execution_node import PoseExecutionNode
from rebotarm_vision.nodes.visual_grasp_executor_node import VisualGraspExecutorNode
from rebotarm_vision.visual_grasp_sequence import PoseTarget, VisualGraspStage
from rebotarm_msgs.msg import GraspPlan
import pytest


NAMES = [f"joint{i}" for i in range(1, 7)]


def _trajectory(start=0.0, end=0.2):
    trajectory = JointTrajectory()
    trajectory.joint_names = NAMES
    for position in (start, end):
        point = JointTrajectoryPoint()
        point.positions = [position] * 6
        trajectory.points.append(point)
    return trajectory


def _fake_motion_node():
    published = []
    planner_calls = []
    def plan_pose(pose, **kwargs):
        planner_calls.append(kwargs)
        return SimpleNamespace(success=True, message="planned", trajectory=_trajectory())
    node = SimpleNamespace(
        _planner=SimpleNamespace(plan_pose=plan_pose),
        _preview_publisher=SimpleNamespace(publish=published.append),
        get_parameter=lambda name: SimpleNamespace(value=0.1),
    )
    return node, planner_calls, published


def test_preview_publishes_only_display_trajectory_and_passes_virtual_start():
    node, calls, published = _fake_motion_node()
    request = ExecutePose.Request()
    request.preview_start_joint_state.name = NAMES
    request.preview_start_joint_state.position = [0.1] * 6
    response = PoseExecutionNode._execute_pose_impl(node, request, ExecutePose.Response())

    assert response.success and response.stage == "planning"
    assert list(response.planned_trajectory.points[-1].positions) == [0.2] * 6
    assert list(calls[0]["start_joint_state"].position) == [0.1] * 6
    assert len(published) == 1
    assert list(published[0].trajectory_start.joint_state.position) == [0.0] * 6
    assert list(published[0].trajectory[0].joint_trajectory.points[-1].positions) == [0.2] * 6


def test_preview_sequence_is_published_as_one_display_message():
    published = []
    node = SimpleNamespace(_preview_publisher=SimpleNamespace(publish=published.append))
    first = _trajectory(start=0.0, end=0.2)
    second = _trajectory(start=0.2, end=0.4)
    request = PublishTrajectoryPreview.Request()
    request.trajectories = [first, second]

    response = PoseExecutionNode._publish_trajectory_preview(
        node, request, PublishTrajectoryPreview.Response()
    )

    assert response.success
    assert len(published) == 1
    assert len(published[0].trajectory) == 2
    assert list(published[0].trajectory_start.joint_state.position) == [0.0] * 6
    assert list(published[0].trajectory[1].joint_trajectory.points[-1].positions) == [0.4] * 6


def test_preview_sequence_rejects_discontinuous_stages():
    published = []
    node = SimpleNamespace(_preview_publisher=SimpleNamespace(publish=published.append))
    request = PublishTrajectoryPreview.Request()
    request.trajectories = [_trajectory(start=0.0, end=0.2), _trajectory(start=0.3, end=0.4)]

    response = PoseExecutionNode._publish_trajectory_preview(
        node, request, PublishTrajectoryPreview.Response()
    )

    assert not response.success
    assert "discontinuous" in response.message
    assert published == []


def test_real_execution_rejects_virtual_start_before_planning_or_action():
    node, calls, published = _fake_motion_node()
    request = ExecutePose.Request()
    request.execute = True
    request.preview_start_joint_state.name = NAMES
    request.preview_start_joint_state.position = [0.1] * 6
    response = PoseExecutionNode._execute_pose_impl(node, request, ExecutePose.Response())

    assert not response.success
    assert "forbidden" in response.message
    assert calls == [] and published == []


def test_bad_virtual_state_fails_closed():
    node, calls, published = _fake_motion_node()
    request = ExecutePose.Request()
    request.preview_start_joint_state.name = NAMES
    request.preview_start_joint_state.position = [float("nan")] * 6
    response = PoseExecutionNode._execute_pose_impl(node, request, ExecutePose.Response())

    assert not response.success
    assert calls == [] and published == []


def test_moveit_request_uses_explicit_full_preview_start():
    planner = object.__new__(MoveItMotionPlanner)
    planner._client = SimpleNamespace(wait_for_service=lambda **kwargs: True)
    planner._group_name = "arm"
    planner._planning_pipeline = "ompl"
    planner._planner_id = ""
    planner._num_attempts = 1
    planner._planning_time = 2.0
    planner._build_pose_constraints = lambda pose: SimpleNamespace()
    planner._call_plan_service = lambda request: request
    start = JointState()
    start.name = NAMES
    start.position = [0.2] * 6

    request = planner.plan_pose(PoseStamped(), start_joint_state=start)
    assert not request.motion_plan_request.start_state.is_diff
    assert list(request.motion_plan_request.start_state.joint_state.position) == [0.2] * 6


def test_visual_stages_chain_last_trajectory_point_without_executing():
    requests = []
    class Future:
        def done(self):
            return True
        def result(self):
            response = ExecutePose.Response()
            response.success = True
            response.stage = "planning"
            response.planned_trajectory = _trajectory(end=0.3 if len(requests) == 1 else 0.6)
            return response

    def call_async(request):
        requests.append(request)
        return Future()

    node = SimpleNamespace(
        _execution_mode="plan_only",
        _preview_start_joint_state=None,
        _preview_trajectories=[],
        _target_frame="base_link",
        _motion_result_timeout_sec=5.0,
        _service_timeout_sec=1.0,
        _execute_pose_client=SimpleNamespace(call_async=call_async),
        _wait_for_future=lambda *args: True,
        get_parameter=lambda name: SimpleNamespace(value=0.1),
        _velocity_scaling_for_stage=lambda name: 0.1,
        _execution_enabled=lambda: False,
    )
    stage = VisualGraspStage(
        name="move_to_pregrasp", kind="move",
        pose=PoseTarget(position=(0.4, 0.0, 0.2), orientation=(0.0, 0.0, 0.0, 1.0)),
    )
    assert VisualGraspServiceGateway._send_execute_pose(node, stage, execute=False)[0]
    assert VisualGraspServiceGateway._send_execute_pose(node, stage, execute=False)[0]
    assert not requests[0].execute and not requests[1].execute
    assert requests[0].suppress_preview and requests[1].suppress_preview
    assert requests[0].preview_start_joint_state.name == []
    assert list(requests[1].preview_start_joint_state.position) == [0.3] * 6
    assert list(node._state.preview_start_joint_state.position) == [0.6] * 6
    assert len(node._state.preview_trajectories) == 2


def test_visual_preview_sequence_calls_motion_owned_batch_publisher_once():
    requests = []

    class Future:
        def done(self):
            return True
        def result(self):
            response = PublishTrajectoryPreview.Response()
            response.success = True
            response.message = "published"
            return response

    node = SimpleNamespace(
        _preview_trajectories=[_trajectory(start=0.0, end=0.2), _trajectory(start=0.2, end=0.4)],
        _service_timeout_sec=1.0,
        _publish_preview_client=SimpleNamespace(
            wait_for_service=lambda **kwargs: True,
            call_async=lambda request: requests.append(request) or Future(),
        ),
        _wait_for_future=lambda *args: True,
    )

    ok, message = VisualGraspServiceGateway._publish_preview_sequence(node)

    assert ok and message == "published"
    assert len(requests) == 1
    assert len(requests[0].trajectories) == 2


def test_gripper_preview_opens_closes_and_holds_without_control_clients():
    published = []
    node = SimpleNamespace(_preview_publisher=SimpleNamespace(publish=published.append))
    request = PublishTrajectoryPreview.Request()
    request.trajectories = [_trajectory(0.0, 0.2), _trajectory(0.2, 0.4), _trajectory(0.4, 0.0)]
    request.gripper_before_trajectory = [0, 2]
    request.gripper_openings_m = [0.08, 0.02]
    response = PoseExecutionNode._publish_trajectory_preview(node, request, PublishTrajectoryPreview.Response())
    assert response.success
    segments = [t.joint_trajectory for t in published[0].trajectory]
    assert len(segments) == 5  # open, pregrasp, approach, close, retreat
    assert list(segments[0].points[0].positions[-2:]) == [0.0, 0.0]
    assert list(segments[0].points[-1].positions[-2:]) == [0.04, -0.04]
    assert list(segments[3].points[-1].positions[-2:]) == pytest.approx([0.01, -0.01])
    assert all(list(p.positions[-2:]) == pytest.approx([0.01, -0.01]) for p in segments[4].points)
    assert all(len(p.positions) == 8 for t in segments for p in t.points)
    assert list(request.trajectories[0].joint_names) == NAMES  # no input mutation
    assert list(published[0].trajectory_start.joint_state.name) == NAMES + ["left_finger_joint", "right_finger_joint"]
    for before, after in zip(segments, segments[1:]):
        assert list(after.points[0].positions) == pytest.approx(list(before.points[-1].positions))


@pytest.mark.parametrize("indices,widths", [([0], []), ([4], [0.02]), ([2, 0], [0.02, 0.03]), ([0], [float('nan')]), ([0], [0.1])])
def test_invalid_gripper_preview_never_publishes(indices, widths):
    published = []
    node = SimpleNamespace(_preview_publisher=SimpleNamespace(publish=published.append))
    request = PublishTrajectoryPreview.Request()
    request.trajectories = [_trajectory()]
    request.gripper_before_trajectory = indices
    request.gripper_openings_m = widths
    response = PoseExecutionNode._publish_trajectory_preview(node, request, PublishTrajectoryPreview.Response())
    assert not response.success
    assert published == []


def test_visual_execute_reports_plan_expired_on_arrival(monkeypatch):
    monkeypatch.setattr("rebotarm_vision.nodes.visual_grasp_executor_node.rclpy.ok", lambda: True)
    plan = GraspPlan()
    plan.valid = True
    plan.header.stamp.sec = 100
    warnings = []
    node = SimpleNamespace(
        _latest_plan=None, _last_plan_rejection="", _plan_revision=0,
        _max_plan_age_sec=10.0,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=112_000_000_000)),
        get_logger=lambda: SimpleNamespace(warn=warnings.append),
    )
    node._plan_is_fresh = lambda value: VisualGraspExecutorNode._plan_is_fresh(node, value)
    VisualGraspExecutorNode._on_plan(node, plan)
    assert node._state.latest_plan is None
    assert node._state.plan_revision == 1
    node._io_gateway = SimpleNamespace(stop_pending=False)
    node._running = False
    node._refresh_config = lambda: None
    node._service_timeout_sec = 0.0
    node._wait_for_fresh_plan = lambda: VisualGraspExecutorNode._wait_for_fresh_plan(node)
    response = SimpleNamespace(success=None, message="")
    VisualGraspExecutorNode._execute_visual_grasp(node, None, response)
    assert not response.success
    assert "expired on arrival" in response.message
    assert "max_plan_age_sec=10.0" in response.message

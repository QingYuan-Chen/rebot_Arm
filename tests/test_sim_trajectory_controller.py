from __future__ import annotations

from pathlib import Path
import sys
import threading
from types import SimpleNamespace

import pytest

ROS2_ROOT = Path(__file__).resolve().parents[1]
PREVIEW_SRC = ROS2_ROOT / "src" / "rebotarm_preview"
if str(PREVIEW_SRC) not in sys.path:
    sys.path.insert(0, str(PREVIEW_SRC))


def test_gripper_width_maps_to_symmetric_finger_joints():
    from rebotarm_preview.sim_gripper import gripper_joint_positions_for_width

    left, right, reached = gripper_joint_positions_for_width(0.06, min_width=0.0, max_width=0.09)

    assert left == pytest.approx(0.03)
    assert right == pytest.approx(-0.03)
    assert reached == pytest.approx(0.06)


def test_gripper_width_is_clamped_before_mapping():
    from rebotarm_preview.sim_gripper import gripper_joint_positions_for_width

    left, right, reached = gripper_joint_positions_for_width(0.12, min_width=0.0, max_width=0.09)

    assert left == pytest.approx(0.045)
    assert right == pytest.approx(-0.045)
    assert reached == pytest.approx(0.09)


def _point(time, positions):
    return SimpleNamespace(time_from_start=SimpleNamespace(sec=int(time), nanosec=round((time % 1) * 1e9)),
                           positions=positions)


def _owner():
    return SimpleNamespace(_joint_names=[*(f"joint{i}" for i in range(1, 7)),
                                         "left_finger_joint", "right_finger_joint"])


def test_preview_validates_time_and_preserves_reordered_joint_positions():
    from rebotarm_preview.rviz_preview_controller_node import SimTrajectoryControllerNode as Node

    owner = _owner()
    names = list(reversed(owner._joint_names[:6]))
    trajectory = SimpleNamespace(joint_names=names, points=[_point(0, range(6)), _point(1, range(6, 12))])
    names_out, points, times = Node._normalize_trajectory(owner, trajectory)
    assert names_out == names
    assert times == (0.0, 1.0)
    assert Node._interpolate(owner, points, times, 0.5) == pytest.approx([3, 4, 5, 6, 7, 8])
    assert Node._interpolate(owner, points, times, 0.0) == list(range(6))


@pytest.mark.parametrize("points", [[], [_point(0, [0] * 5)], [_point(0, [float('nan')] * 6)],
    [_point(-1, [0] * 6)], [_point(0, [0] * 6), _point(0, [0] * 6)],
    [_point(2, [0] * 6), _point(1, [0] * 6)]])
def test_preview_rejects_damaged_trajectory(points):
    from rebotarm_preview.rviz_preview_controller_node import SimTrajectoryControllerNode as Node

    owner = _owner()
    with pytest.raises(ValueError):
        Node._normalize_trajectory(owner, SimpleNamespace(joint_names=owner._joint_names[:6], points=points))


def test_preview_rejects_nonfinite_gripper_without_state_change():
    from rebotarm_preview.rviz_preview_controller_node import SimTrajectoryControllerNode as Node

    owner = SimpleNamespace(_last_gripper_width_m=0.04, _lock=threading.Lock())
    response = SimpleNamespace(success=True, reached_position=0.0)
    result = Node._set_gripper_service(owner, SimpleNamespace(position=float('nan')), response)
    assert not result.success
    assert result.reached_position == 0.04


def test_preview_shutdown_aborts_instead_of_succeeding(monkeypatch):
    from rebotarm_preview import rviz_preview_controller_node as module

    events = []
    owner = SimpleNamespace(_stop_requested=threading.Event(),
                            _normalize_trajectory=lambda _: (['joint1'], [(0.0,), (1.0,)], (0.0, 1.0)))
    goal = SimpleNamespace(request=SimpleNamespace(trajectory=None), abort=lambda: events.append('abort'))
    monkeypatch.setattr(module.rclpy, 'ok', lambda: False)
    result = module.SimTrajectoryControllerNode._execute_owned_goal(owner, goal)
    assert events == ['abort']
    assert result.error_code == module.FollowJointTrajectory.Result.INVALID_GOAL


def test_preview_stop_after_acceptance_is_not_cleared(monkeypatch):
    from rclpy.action import GoalResponse
    from rebotarm_preview import rviz_preview_controller_node as module

    node = module.SimTrajectoryControllerNode
    owner = _owner()
    owner._recovery_lock, owner._motion_lock = threading.Lock(), threading.Lock()
    owner._stop_requested = threading.Event()
    owner.get_logger = lambda: SimpleNamespace(warn=lambda _: None)
    owner._normalize_trajectory = lambda _: (owner._joint_names[:6], [(0.,) * 6], (1.,))
    owner._execute_owned_goal = lambda goal: node._execute_owned_goal(owner, goal)
    request = SimpleNamespace(trajectory=SimpleNamespace(joint_names=owner._joint_names[:6]))
    assert node._goal_callback(owner, request) == GoalResponse.ACCEPT
    assert node._goal_callback(owner, request) == GoalResponse.REJECT
    owner._stop_requested.set()
    events = []
    goal = SimpleNamespace(request=request, is_cancel_requested=False, abort=lambda: events.append('abort'))
    monkeypatch.setattr(module.rclpy, 'ok', lambda: True)
    result = node._execute_goal(owner, goal)
    assert events == ['abort']
    assert result.error_code == module.FollowJointTrajectory.Result.INVALID_GOAL
    assert not owner._motion_lock.locked()

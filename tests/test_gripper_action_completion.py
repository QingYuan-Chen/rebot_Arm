from __future__ import annotations

import sys
from types import SimpleNamespace
import types

import rebotarmcontroller.ros_actions as ros_actions_module
from rebotarmcontroller.ros_actions import ArmActions


class _LongMoveHardware:
    def __init__(self, clock: list[float]) -> None:
        self._clock = clock
        self.active = True

    def set_gripper_target(self, _position: float, _max_effort: float):
        return 1

    def gripper_execution_status(self, generation):
        assert generation == 1
        if self.gripper_command_error:
            self.active = False
            return 'failed', self.gripper_command_error
        if self.gripper_reached_target():
            self.active = False
            return 'succeeded', None
        if self._clock[0] >= self.gripper_target_timeout_sec():
            self.active = False
            return 'failed', 'gripper target timeout'
        return 'active', None

    def gripper_target_timeout_sec(self) -> float:
        return 10.0

    def gripper_position_m(self) -> float:
        return 0.08 if self._clock[0] >= 8.0 else 0.02

    def get_gripper_state(self):
        return 0.0, 0.0, 0.2, 1

    def gripper_reached_target(self) -> bool:
        return self._clock[0] >= 8.0

    def wait_gripper_target(self, timeout: float | None = None) -> bool:
        del timeout
        if not self.gripper_reached_target():
            return False
        self.active = False
        return True

    def cancel_gripper_position_command(self, _reason: str, *, expected_generation=None) -> bool:
        return False

    @property
    def gripper_command_error(self):
        return None


class _FailedMoveHardware(_LongMoveHardware):
    @property
    def gripper_command_error(self):
        return "gripper feedback stale: age=0.300s"


class _NeverMoveHardware(_LongMoveHardware):
    def __init__(self, clock: list[float]) -> None:
        super().__init__(clock)
        self.active = True

    def gripper_target_timeout_sec(self) -> float:
        return 0.1

    def gripper_reached_target(self) -> bool:
        return False

    def cancel_gripper_position_command(self, _reason: str, *, expected_generation=None) -> bool:
        self.active = False
        return True


class _GoalHandle:
    def __init__(self) -> None:
        self.request = SimpleNamespace(
            command=SimpleNamespace(position=0.08, max_effort=1.0)
        )
        self.is_cancel_requested = False
        self.state = "active"
        self.feedback = []

    def publish_feedback(self, feedback) -> None:
        self.feedback.append(feedback)

    def succeed(self) -> None:
        self.state = "succeeded"

    def abort(self) -> None:
        self.state = "aborted"

    def canceled(self) -> None:
        self.state = "canceled"


class _Logger:
    def error(self, _message: str) -> None:
        return None


class _Node:
    def get_logger(self):
        return _Logger()


def test_gripper_action_uses_distance_related_hardware_timeout(monkeypatch) -> None:
    clock = [0.0]
    monkeypatch.setattr(ros_actions_module.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(
        ros_actions_module.time,
        "sleep",
        lambda duration: clock.__setitem__(0, clock[0] + duration),
    )
    actions = ArmActions.__new__(ArmActions)
    actions._hardware = _LongMoveHardware(clock)
    actions._node = _Node()
    goal_handle = _GoalHandle()

    result = actions.execute_gripper_command(goal_handle)

    assert result.reached_goal is True
    assert result.position == 0.08
    assert goal_handle.state == "succeeded"
    assert clock[0] >= 8.0
    assert actions._hardware.active is False


def test_gripper_action_aborts_immediately_on_feedback_failure(monkeypatch) -> None:
    clock = [0.0]
    monkeypatch.setattr(ros_actions_module.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(
        ros_actions_module.time,
        "sleep",
        lambda duration: clock.__setitem__(0, clock[0] + duration),
    )
    hardware = _FailedMoveHardware(clock)
    actions = ArmActions.__new__(ArmActions)
    actions._hardware = hardware
    actions._node = _Node()
    goal_handle = _GoalHandle()

    result = actions.execute_gripper_command(goal_handle)

    assert result.reached_goal is False
    assert goal_handle.state == "aborted"
    assert clock[0] < 0.1


def test_gripper_action_timeout_stops_active_hardware_command(monkeypatch) -> None:
    clock = [0.0]
    monkeypatch.setattr(ros_actions_module.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(
        ros_actions_module.time,
        "sleep",
        lambda duration: clock.__setitem__(0, clock[0] + duration),
    )
    hardware = _NeverMoveHardware(clock)
    actions = ArmActions.__new__(ArmActions)
    actions._hardware = hardware
    actions._node = _Node()
    goal_handle = _GoalHandle()

    result = actions.execute_gripper_command(goal_handle)

    assert result.reached_goal is False
    assert goal_handle.state == "aborted"
    assert hardware.active is False

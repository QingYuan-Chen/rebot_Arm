from __future__ import annotations

from types import SimpleNamespace
import threading

import numpy as np
import pytest
from builtin_interfaces.msg import Time

from rebotarmcontroller.ros_publishers import JointStatePublisher
import rebotarmcontroller.ros_publishers as ros_publishers


@pytest.fixture(autouse=True)
def feedback_clock(monkeypatch):
    monkeypatch.setattr(ros_publishers.time, "monotonic", lambda: 100.0)


class _Recorder:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, message) -> None:
        self.messages.append(message)


class _Clock:
    def now(self):
        return SimpleNamespace(to_msg=lambda: Time(sec=1))


class _Node:
    def __init__(self) -> None:
        self.warnings: list[str] = []

    def get_clock(self):
        return _Clock()

    def get_logger(self):
        return SimpleNamespace(warn=self.warnings.append)


class _CacheOnlyHardware:
    joint_names = [f"joint{i}" for i in range(1, 7)]

    def __init__(self) -> None:
        self.refresh_calls = 0
        self.identity = (1,) * 6
        self.mode = "mit"
        self.enabled = False
        self.control_loop_active = False
        self.state_machine = "IDLE"
        self.codes = [0] * 6
        self.error_codes = []
        self.observed_at = 100.0
        self.gripper_identity = 1
        self.gripper_observed_at = 100.0
        self.gripper_status = 1

    def refresh_feedback_if_due(self) -> None:
        self.refresh_calls += 1

    def get_cached_joint_sample(self):
        return np.zeros(6), np.zeros(6), np.zeros(6), self.codes, self.identity, self.observed_at

    def get_joint_state(self):
        raise AssertionError("publisher attempted synchronous serial feedback")

    def get_joint_status_codes(self):
        raise AssertionError("publisher split status and error snapshots")

    def get_feedback_status(self):
        return self.codes, self.error_codes

    def get_cached_gripper_sample(self):
        return 0.018, 0.1, 0.2, self.gripper_status, self.gripper_identity, self.gripper_observed_at

    def get_gripper_state(self):
        raise AssertionError("publisher split gripper values and status snapshots")

    def gripper_position_m(self):
        raise AssertionError("publisher read gripper position a second time")


def test_joint_state_publisher_reads_validated_cache_without_serial_io() -> None:
    """Calling the old synchronous getter from the 100 Hz timer is the bus bug."""
    publisher = JointStatePublisher.__new__(JointStatePublisher)
    publisher._publish_lock = threading.Lock()
    publisher._last_feedback_identity = None
    publisher._last_gripper_identity = None
    publisher._last_gripper_stamp = Time()
    publisher._node = _Node()
    publisher._hardware = _CacheOnlyHardware()
    publisher._publisher = _Recorder()
    publisher._joint_state_publishers = {
        name: _Recorder() for name in publisher._hardware.joint_names
    }
    publisher._gripper_state_publisher = _Recorder()
    status_calls: list[int] = []
    publisher.publish_status = lambda **kwargs: status_calls.append(kwargs)

    publisher.publish()

    assert publisher._hardware.refresh_calls == 1
    assert len(publisher._publisher.messages) == 1
    assert len(publisher._gripper_state_publisher.messages) == 1
    assert status_calls == [{"force": False}]

    first_stamp = publisher._publisher.messages[-1].header.stamp
    publisher._node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=2))
    )
    publisher.publish()
    assert publisher._publisher.messages[-1].header.stamp == first_stamp
    publisher._hardware.identity = (2,) * 6
    publisher.publish()
    assert publisher._publisher.messages[-1].header.stamp.sec == 2
    assert status_calls == [{"force": False}] * 3
    with publisher._publish_lock:
        before = publisher._hardware.refresh_calls
        publisher.publish()
        assert publisher._hardware.refresh_calls == before
        assert status_calls == [{"force": False}] * 3


def test_joint_state_publisher_reports_status_when_feedback_cache_is_invalid() -> None:
    publisher = JointStatePublisher.__new__(JointStatePublisher)
    publisher._publish_lock = threading.Lock()
    publisher._last_feedback_identity = None
    publisher._last_gripper_identity = None
    publisher._last_gripper_stamp = Time()
    publisher._node = _Node()
    publisher._hardware = _CacheOnlyHardware()
    publisher._hardware.get_cached_joint_sample = lambda: (_ for _ in ()).throw(
        RuntimeError("arm feedback stale")
    )
    publisher._publisher = _Recorder()
    publisher._joint_state_publishers = {
        name: _Recorder() for name in publisher._hardware.joint_names
    }
    publisher._gripper_state_publisher = _Recorder()
    status_calls: list[int] = []
    publisher.publish_status = lambda **kwargs: status_calls.append(kwargs)

    publisher.publish()

    assert status_calls == [{"force": False}]
    assert publisher._publisher.messages == []
    assert publisher._node.warnings == ["joint state read failed: arm feedback stale"]


def test_joint_state_publisher_refreshes_latched_status_after_feedback_recovers() -> None:
    publisher = JointStatePublisher.__new__(JointStatePublisher)
    publisher._publish_lock = threading.Lock()
    publisher._last_feedback_identity = None
    publisher._last_gripper_identity = None
    publisher._last_gripper_stamp = Time()
    publisher._last_feedback_stamp = None
    publisher._node = _Node()
    publisher._hardware = _CacheOnlyHardware()
    publisher._publisher = _Recorder()
    publisher._joint_state_publishers = {
        name: _Recorder() for name in publisher._hardware.joint_names
    }
    publisher._gripper_state_publisher = _Recorder()
    status_calls: list[str] = []
    publisher.publish_status = lambda **kwargs: status_calls.append(kwargs)

    original_get_sample = publisher._hardware.get_cached_joint_sample
    publisher._hardware.get_cached_joint_sample = lambda: (_ for _ in ()).throw(
        RuntimeError("arm feedback stale")
    )
    publisher.publish()

    publisher._hardware.get_cached_joint_sample = original_get_sample
    publisher.publish()

    assert status_calls == [{"force": False}] * 2
    assert len(publisher._publisher.messages) == 1


def test_arm_status_change_is_immediate_with_five_hz_heartbeat(monkeypatch) -> None:
    publisher = JointStatePublisher.__new__(JointStatePublisher)
    publisher._node = _Node()
    publisher._hardware = _CacheOnlyHardware()
    publisher._status_publisher = _Recorder()
    publisher._status_lock = threading.Lock()
    publisher._last_status_identity = None
    publisher._last_status_published_at = None
    publisher._status_heartbeat_sec = 0.2
    clock = [0.0]
    monkeypatch.setattr(ros_publishers.time, "monotonic", lambda: clock[0])

    publisher.publish_status(force=False)
    assert len(publisher._status_publisher.messages) == 1
    clock[0] = 0.1
    publisher.publish_status(force=False)
    assert len(publisher._status_publisher.messages) == 1

    publisher._hardware.codes = [255] * 6
    publisher._hardware.error_codes = ["ARM_FEEDBACK: arm feedback stale"]
    publisher.publish_status(force=False)
    assert list(publisher._status_publisher.messages[-1].per_joint_status_code) == [255] * 6
    assert len(publisher._status_publisher.messages) == 2

    clock[0] = 0.11
    publisher._hardware.codes = [0] * 6
    publisher._hardware.error_codes = []
    publisher.publish_status(force=False)
    assert list(publisher._status_publisher.messages[-1].per_joint_status_code) == [0] * 6
    assert list(publisher._status_publisher.messages[-1].error_codes) == []
    assert len(publisher._status_publisher.messages) == 3

    clock[0] = 0.3
    publisher.publish_status(force=False)
    assert len(publisher._status_publisher.messages) == 3
    clock[0] = 0.31
    publisher.publish_status(force=False)
    assert len(publisher._status_publisher.messages) == 4
    publisher._hardware.enabled = True
    publisher.publish_status(force=False)
    assert publisher._status_publisher.messages[-1].enabled is True
    publisher.publish_status()
    assert len(publisher._status_publisher.messages) == 6


def _feedback_publisher():
    publisher = JointStatePublisher.__new__(JointStatePublisher)
    publisher._publish_lock = threading.Lock()
    publisher._last_feedback_identity = None
    publisher._last_gripper_identity = None
    publisher._last_gripper_stamp = Time()
    publisher._node = _Node()
    publisher._hardware = _CacheOnlyHardware()
    publisher._publisher = _Recorder()
    publisher._joint_state_publishers = {
        name: _Recorder() for name in publisher._hardware.joint_names
    }
    publisher._gripper_state_publisher = _Recorder()
    publisher.publish_status = lambda **kwargs: None
    return publisher


def test_gripper_timestamp_tracks_its_own_feedback_and_preserves_age(monkeypatch) -> None:
    publisher = _feedback_publisher()
    hardware = publisher._hardware
    hardware.observed_at = 99.75
    hardware.gripper_observed_at = 99.5
    publisher.publish()
    arm_stamp = publisher._publisher.messages[-1].header.stamp
    gripper_stamp = publisher._gripper_state_publisher.messages[-1].header.stamp
    assert arm_stamp == Time(sec=0, nanosec=750_000_000)
    assert gripper_stamp == Time(sec=0, nanosec=500_000_000)

    monkeypatch.setattr(ros_publishers.time, "monotonic", lambda: 101.0)
    publisher._node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=2))
    )
    hardware.identity = (2,) * 6
    hardware.observed_at = 101.0
    hardware.gripper_status = 255
    publisher.publish()
    assert publisher._publisher.messages[-1].header.stamp == Time(sec=2)
    assert publisher._gripper_state_publisher.messages[-1].header.stamp == gripper_stamp
    assert publisher._gripper_state_publisher.messages[-1].status_code == 255

    hardware.gripper_identity = 2
    hardware.gripper_observed_at = 100.75
    hardware.gripper_status = 11
    publisher.publish()
    msg = publisher._gripper_state_publisher.messages[-1]
    assert msg.header.stamp == Time(sec=1, nanosec=750_000_000)
    assert (msg.position, msg.velocity, msg.torque, msg.status_code) == (0.018, 0.1, 0.2, 11)


def test_gripper_still_publishes_when_arm_cache_is_unavailable() -> None:
    publisher = _feedback_publisher()
    publisher._hardware.get_cached_joint_sample = lambda: (_ for _ in ()).throw(
        RuntimeError("joint3 [INVALID_COORDINATE]")
    )
    publisher.publish()
    assert publisher._publisher.messages == []
    assert publisher._gripper_state_publisher.messages[-1].status_code == 1


def test_gripper_waiting_for_first_frame_has_no_fabricated_timestamp() -> None:
    publisher = _feedback_publisher()
    publisher._hardware.gripper_identity = None
    publisher._hardware.gripper_observed_at = None
    publisher._hardware.gripper_status = 255
    publisher.publish()
    assert publisher._gripper_state_publisher.messages[-1].header.stamp == Time()
    assert publisher._gripper_state_publisher.messages[-1].status_code == 255


def test_motor_fault_codes_are_published_without_remapping() -> None:
    publisher = _feedback_publisher()
    publisher._hardware.codes = [0, 1, 8, 9, 11, 14]
    publisher.publish()
    assert [publisher._joint_state_publishers[name].messages[-1].status_code
            for name in publisher._hardware.joint_names] == [0, 1, 8, 9, 11, 14]

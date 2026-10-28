"""生命周期组件的独立事务测试。

直接装配设备、反馈与执行器接口，不构造 manager、ROS 或 SDK。用事件记录和
故障注入核对设备回执、状态迁移及清理顺序，保留跨语言复用所需的行为断言。
"""
from dataclasses import replace
from types import SimpleNamespace

import pytest

from rebotarmcontroller.lifecycle import ControllerLifecycle, LifecyclePorts


class Session:
    """最小设备会话；反馈状态与命令状态分离，便于模拟确认失败。"""

    def __init__(self):
        self.calls = []
        self.status = 0
        self.failure = None
        self.mode = "mit"
        self.control_loop_active = False
        self.positions = (0.1,) * 6

    def record(self, name):
        self.calls.append(name)
        if name == self.failure:
            raise RuntimeError(f"injected {name}")

    def connect(self):
        self.record("connect")

    def disconnect(self):
        self.record("disconnect")

    def release_disabled(self):
        self.record("release")

    def set_target(self, positions):
        assert tuple(positions) == self.positions
        self.record("target")

    def enable(self):
        self.record("enable")
        self.status = 1

    def disable(self):
        self.record("disable")
        self.status = 0

    def mode_pos_vel(self):
        self.record("mode_pos_vel")
        self.mode = "pos_vel"
        return True

    def mode_mit(self):
        self.record("mode_mit")
        self.mode = "mit"
        return True

    def mode_vel(self):
        self.record("mode_vel")
        self.mode = "vel"
        return True

    def set_zero(self):
        self.record("zero_all")

    def set_zero_single(self, name):
        self.record(f"zero_{name}")
        return True

    def feedback(self, *, expected_status=None):
        self.record(f"feedback_{expected_status}")
        if expected_status is not None and self.status != expected_status:
            raise RuntimeError("feedback status mismatch")
        return self.positions, (0.,) * 6, (0.,) * 6, [self.status] * 6


def lifecycle():
    device = Session()
    ports = LifecyclePorts(
        device=device,
        joint_feedback=device.feedback,
        gripper_status=lambda status: device.record(f"gripper_feedback_{status}"),
        gripper_feedback_status=lambda: None,
        gripper_motor=lambda: None,
        gripper_coordinate_failure=lambda: None,
        prepare_gripper_enable=lambda: device.record("prepare_gripper"),
        clear_gripper_transport_error=lambda: device.record("clear_transport"),
        reset_session=lambda: device.record("reset"),
        initialize_gripper=lambda: device.record("init_gripper"),
        stop_gripper=lambda: device.record("stop_gripper"),
        stop_control=lambda: device.record("stop_control"),
        stop_gravity=lambda: device.record("stop_gravity"),
        clear_passthrough=lambda **kw: device.record("clear_arm" if kw else "clear_streams"),
        hold=lambda: device.record("hold"),
        start_position=lambda **kw: device.record("start_position"),
        zero_gripper=lambda: device.record("zero_gripper") or True,
        confirm_disabled=lambda: device.record("confirmed_disabled"),
        protect=lambda *args, **kw: device.record("protect"),
    )
    return ControllerLifecycle(ports), device


def test_session_enable_disable_and_release_have_verified_order():
    owner, device = lifecycle()
    owner.connect()
    assert owner.connected and not owner.enabled
    assert owner.state == "CONNECTED_DISABLED"
    assert device.calls == ["clear_streams", "connect", "reset", "init_gripper", "feedback_None",
                            "confirmed_disabled"]
    device.calls.clear()
    owner.enable()
    assert owner.enabled and owner.state == "ENABLED_HOLD"
    assert device.calls == ["feedback_0", "gripper_feedback_0", "target",
                            "mode_pos_vel", "prepare_gripper", "enable",
                            "feedback_1", "gripper_feedback_1", "clear_transport",
                            "start_position"]
    device.calls.clear()
    owner.disable()
    assert not owner.enabled and owner.state == "CONNECTED_DISABLED"
    assert device.calls == ["clear_streams", "stop_gravity", "stop_gripper",
                            "stop_control", "disable", "feedback_0",
                            "gripper_feedback_0", "confirmed_disabled"]
    owner.shutdown()
    assert not owner.connected and owner.state == "DISCONNECTED"
    assert device.calls[-3:] == ["feedback_0", "gripper_feedback_0", "release"]


@pytest.mark.parametrize("failure", [
    "feedback_0", "mode_pos_vel", "prepare_gripper", "enable", "feedback_1",
    "clear_transport", "start_position",
])
def test_each_enable_failure_rolls_back_or_preserves_unknown_state(failure):
    owner, device = lifecycle()
    owner.connect()
    device.failure = failure
    with pytest.raises(RuntimeError, match="enable failed"):
        owner.enable()
    assert "disable" in device.calls
    # 失能确认也失败时不能把输出未知标记为安全失能。
    if failure == "feedback_0":
        assert owner.enabled and owner.state == "DISABLING"
        assert "ENABLE_ROLLBACK_FAILED" in owner.errors
    else:
        assert not owner.enabled and owner.state == "CONNECTED_DISABLED"


def test_failed_cleanup_still_attempts_disable_and_retains_unknown_state():
    owner, device = lifecycle()
    owner.connect()
    device.failure = "stop_gripper"
    owner.ports = replace(owner.ports, prepare_gripper_enable=lambda: (_ for _ in ()).throw(
        RuntimeError("enable preflight failed")))
    with pytest.raises(RuntimeError, match="rollback verification failed"):
        owner.enable()
    assert device.calls[-2:] == ["stop_control", "disable"]
    assert owner.enabled and owner.state == "DISABLING"
    device.failure = None
    owner.disable()
    assert not owner.enabled and owner.state == "CONNECTED_DISABLED"


def test_failed_connect_cleans_all_resources_without_enabling():
    owner, device = lifecycle()
    device.failure = "init_gripper"
    with pytest.raises(RuntimeError, match="init_gripper"):
        owner.connect()
    assert device.calls[-4:] == ["stop_gripper", "stop_control", "disable", "disconnect"]
    assert not owner.connected and not owner.enabled
    assert "enable" not in device.calls


def test_disable_failure_does_not_permit_enable_or_mode_change():
    owner, device = lifecycle()
    owner.connect()
    owner.enable()
    device.failure = "feedback_0"
    with pytest.raises(RuntimeError):
        owner.disable()
    assert owner.enabled and owner.state == "DISABLING"
    before = list(device.calls)
    with pytest.raises(RuntimeError, match="verify disable"):
        owner.enable()
    with pytest.raises(RuntimeError, match="verify disable"):
        owner.set_mode("mit")
    assert device.calls == before


def test_shutdown_rejects_enabled_and_keeps_connection_if_verification_fails():
    owner, device = lifecycle()
    owner.connect()
    owner.enable()
    with pytest.raises(RuntimeError, match="explicit disable"):
        owner.shutdown()
    assert "release" not in device.calls
    owner.disable()
    device.failure = "feedback_0"
    with pytest.raises(RuntimeError, match="disabled-state verification"):
        owner.shutdown()
    assert owner.connected and owner.enabled and owner.state == "DISABLING"
    assert "release" not in device.calls


def test_both_devices_attempt_disable_even_if_arm_command_fails():
    owner, device = lifecycle()
    device.failure = "disable"
    motor = SimpleNamespace(disable=lambda: device.record("gripper_disable"))
    owner.ports = replace(owner.ports, gripper_motor=lambda: motor)
    with pytest.raises(RuntimeError, match="arm disable"):
        owner.disable_all()
    assert device.calls == ["disable", "gripper_disable"]


def test_mode_and_zero_keep_explicit_enable_boundary():
    owner, device = lifecycle()
    owner.connect()
    assert owner.set_mode(" VEL ")
    assert device.mode == "vel" and not owner.enabled
    assert owner.set_zero("joint2")
    assert owner.set_zero("gripper")
    assert "enable" not in device.calls
    owner.enable()
    count = len(device.calls)
    with pytest.raises(RuntimeError, match="CONNECTED_DISABLED"):
        owner.set_zero()
    assert len(device.calls) == count

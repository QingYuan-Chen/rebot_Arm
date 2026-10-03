from __future__ import annotations

from pathlib import Path
import threading
import time
from types import SimpleNamespace
from xml.etree import ElementTree

import numpy as np
import pytest

import rebotarmcontroller.hardware_manager as hardware_manager_module
from rebotarmcontroller.hardware_manager import (
    HardwareManager,
    _JOINT_POSITION_LIMITS_RAD,
    _G_ANGLE_OPEN,
    _G_GRASP_HOLD_TIMEOUT_MAX_SEC,
    _G_LARGE_MOVE_MAX_TAU_CAP_NM,
    _G_LARGE_MOVE_MAX_TAU_NM,
    _G_MAX_DIST_M,
    _G_POSITION_MAX_SPEED_RAD_S,
    _G_TAU_MAX,
    _G_VERIFIED_OPEN_LIMIT_M,
)


JOINT_NAMES = [f"joint{i}" for i in range(1, 7)]
UINT64_MAX = (1 << 64) - 1


class FakeMotor:
    def __init__(self, position: float = 0.0) -> None:
        self.request_feedback_calls = 0
        self.feedback_sequence = 0
        self.advance_feedback_on_poll = True
        self._feedback_requested = False
        self.state = SimpleNamespace(
            pos=float(position),
            vel=0.0,
            torq=0.0,
            status_code=0,
        )
        self.set_zero_position_calls = 0

    def get_state(self):
        return self.state

    def get_state_with_sequence(self):
        return self.state, self.feedback_sequence

    def request_feedback(self) -> None:
        self.request_feedback_calls += 1
        self._feedback_requested = True

    def deliver_requested_feedback(self) -> None:
        if self._feedback_requested and self.advance_feedback_on_poll:
            self.feedback_sequence = (
                1 if self.feedback_sequence == UINT64_MAX else self.feedback_sequence + 1
            )
            self._feedback_requested = False

    def set_zero_position(self) -> None:
        self.set_zero_position_calls += 1


class FakeController:
    def __init__(self, motors) -> None:
        self._motors = list(motors)
        self._bus_lock = threading.RLock()
        self.poll_feedback_calls = 0

    def poll_feedback_once(self) -> None:
        self.poll_feedback_calls += 1
        for motor in self._motors:
            motor.deliver_requested_feedback()


class FakeArm:
    def __init__(self, *, fail_enable_joint: str | None = None) -> None:
        self.joint_names = list(JOINT_NAMES)
        self.num_joints = len(self.joint_names)
        self._joints = [
            SimpleNamespace(name=name, vendor="fake") for name in self.joint_names
        ]
        self._motor_map = {name: FakeMotor() for name in self.joint_names}
        self._ctrl_map = {"fake": FakeController(self._motor_map.values())}
        self.mode = "mit"
        self.control_loop_active = False
        self.connect_calls = 0
        self.enable_calls = 0
        self.disable_calls = 0
        self.disconnect_calls = 0
        self.mode_pos_vel_calls = 0
        self.start_control_loop_calls = 0
        self.control_callback = None
        self.fail_enable_joint = fail_enable_joint

    def connect(self) -> None:
        self.connect_calls += 1

    def disconnect(self) -> None:
        self.disconnect_calls += 1
        self.control_loop_active = False

    def _request_and_poll(self) -> None:
        for motor in self._motor_map.values():
            motor.request_feedback()

    def get_state(self):
        states = [self._motor_map[name].state for name in self.joint_names]
        return (
            np.array([state.pos for state in states], dtype=np.float64),
            np.array([state.vel for state in states], dtype=np.float64),
            np.array([state.torq for state in states], dtype=np.float64),
        )

    def get_positions(self, request: bool = False):
        del request
        return self.get_state()[0]

    def enable(self) -> None:
        self.enable_calls += 1
        for name, motor in self._motor_map.items():
            motor.state.status_code = 0 if name == self.fail_enable_joint else 1

    def disable(self) -> None:
        self.disable_calls += 1
        for motor in self._motor_map.values():
            motor.state.status_code = 0
        self.control_loop_active = False

    def mode_pos_vel(self) -> bool:
        self.mode_pos_vel_calls += 1
        self.mode = "pos_vel"
        return True

    def start_control_loop(self, callback, rate=None) -> None:
        del rate
        self.start_control_loop_calls += 1
        self.control_callback = callback
        self.control_loop_active = True

    def stop_control_loop(self) -> None:
        self.control_loop_active = False


def make_manager(
    *,
    fail_enable_joint: str | None = None,
    gripper_position_torque_cap_nm: float = _G_LARGE_MOVE_MAX_TAU_NM,
    gripper_position_max_speed_rad_s: float = _G_POSITION_MAX_SPEED_RAD_S,
    gripper_position_timeout_margin_sec: float = 1.5,
    gripper_feedback_stale_timeout_sec: float = 0.15,
    hardware_feedback_rate_hz: float = 50.0,
) -> HardwareManager:
    # Mirror the real constructor's validation without running hardware setup.
    requested_cap = float(gripper_position_torque_cap_nm)
    if not 0.05 <= requested_cap <= _G_LARGE_MOVE_MAX_TAU_CAP_NM:
        raise ValueError(
            "gripper_position_torque_cap_nm must be within "
            f"[0.05, {_G_LARGE_MOVE_MAX_TAU_CAP_NM:g}] N.m"
        )
    manager = HardwareManager.__new__(HardwareManager)
    manager._gripper_position_torque_cap_nm = requested_cap
    manager._gripper_position_max_speed_rad_s = float(gripper_position_max_speed_rad_s)
    manager._gripper_position_timeout_margin_sec = float(gripper_position_timeout_margin_sec)
    manager._gripper_feedback_stale_timeout_sec = float(gripper_feedback_stale_timeout_sec)
    manager._hardware_feedback_rate_hz = float(hardware_feedback_rate_hz)
    manager._hardware_feedback_period_sec = 1.0 / float(hardware_feedback_rate_hz)
    manager._feedback_next_refresh_monotonic = None
    manager._verified_feedback_by_label = {}
    manager._feedback_request_baseline_by_label = {}
    manager._feedback_request_deadline_by_label = {}
    manager._feedback_error_by_label = {}
    manager._arm_feedback_updated_monotonic = None
    manager._arm_feedback_error = "arm feedback not received"
    manager._grasp_hold_timeout_sec = 30.0
    manager._gripper_hold_deadline = None
    manager._gripper_hold_release_reason = None
    manager._gripper_target_timeout_sec = 0.0
    manager._gripper_target_deadline_monotonic = None
    manager._gripper_last_tick_monotonic = None
    manager._gripper_status_code = 255
    manager._gripper_feedback_updated_monotonic = None
    manager._gripper_feedback_error = "gripper feedback not received"
    manager._gripper_zero_error = None
    manager._gripper_command_error = None
    manager._gripper_position_result = "idle"
    manager._gripper_neutral_pending = None
    manager._arm = FakeArm(fail_enable_joint=fail_enable_joint)
    manager._endpos_ctrl = SimpleNamespace(
        _q_target=np.full(6, 99.0, dtype=np.float64),
        _running=False,
        _moving=False,
        _stop_send=SimpleNamespace(set=lambda: None),
        _loop_cb=lambda _arm, _dt: None,
    )
    manager._connected = False
    manager._enabled = False
    manager._lifecycle_state = "DISCONNECTED"
    manager._state_machine = "IDLE"
    manager._error_codes = []
    manager._gripper_cfg_path = Path("unused-gripper.yaml")
    manager._gripper_cfg = None
    manager._gripper_mot = None
    manager._gripper_ctrl = None
    manager._gripper_active = False
    manager._gripper_mode = "idle"
    manager._gripper_pos = 0.0
    manager._gripper_vel = 0.0
    manager._gripper_torque = 0.0
    manager._gripper_lock = threading.RLock()
    manager._feedback_lock = threading.RLock()
    manager._motor_lifecycle_lock = threading.RLock()
    manager._gravity_comp_active = False
    manager._gravity_comp_q_target = None
    manager._gravity_comp_integral = None
    manager._gravity_comp_lock_counter = 0
    manager._gravity_comp_q_last = None
    manager.init_gripper = lambda _path: None
    manager._stop_gripper_loop = lambda: None
    manager._start_gripper_loop = lambda: None
    return manager


def seed_verified_feedback(
    manager: HardwareManager,
    *,
    observed_at: float | None = None,
    arm_status: int = 0,
    gripper_state=None,
) -> dict[str, object]:
    """Seed the controller-owned cache without touching a MotorBridge getter."""
    timestamp = time.monotonic() if observed_at is None else float(observed_at)
    with manager._feedback_lock:
        for sequence, name in enumerate(JOINT_NAMES, start=11):
            motor_state = manager.arm._motor_map[name].state
            motor_state.status_code = int(arm_status)
            manager._record_verified_feedback(
                name,
                motor_state,
                sequence,
                timestamp,
            )
        if gripper_state is not None:
            manager._record_verified_feedback(
                "gripper",
                gripper_state,
                31,
                timestamp,
            )
        manager._sync_feedback_health()
        return dict(manager._verified_feedback_by_label)


def forbid_raw_feedback_reads(manager: HardwareManager) -> None:
    def fail() -> None:
        raise AssertionError("connected consumer performed a raw feedback read")

    for motor in manager.arm._motor_map.values():
        motor.get_state = fail
        motor.get_state_with_sequence = fail
    if manager._gripper_mot is not None:
        manager._gripper_mot.get_state = fail
        manager._gripper_mot.get_state_with_sequence = fail


def assert_verified_feedback_unchanged(
    manager: HardwareManager,
    before: dict[str, object],
) -> None:
    with manager._feedback_lock:
        assert manager._verified_feedback_by_label.keys() == before.keys()
        for label, sample in before.items():
            current = manager._verified_feedback_by_label[label]
            assert current is sample
            assert current.sequence == sample.sequence
            assert current.observed_at == sample.observed_at


def configure_gravity_compensation_test(
    manager: HardwareManager,
    *,
    positions: np.ndarray,
    velocities: np.ndarray,
) -> list[dict[str, object]]:
    manager._connected = True
    manager._enabled = True
    for name, position, velocity in zip(JOINT_NAMES, positions, velocities):
        state = manager.arm._motor_map[name].state
        state.pos = float(position)
        state.vel = float(velocity)
        state.status_code = 1
    seed_verified_feedback(manager, arm_status=1)

    def forbidden_sdk_getter(*_args, **_kwargs):
        raise AssertionError("gravity compensation used an SDK aggregate getter")

    for motor in manager.arm._motor_map.values():
        motor.get_state = forbidden_sdk_getter
    manager.arm.get_positions = forbidden_sdk_getter
    manager.arm.get_velocities = forbidden_sdk_getter
    manager.arm._rate = 500.0
    manager.arm.mode_mit = lambda **_kwargs: True
    mit_commands: list[dict[str, object]] = []
    manager._send_gravity_mit = lambda **kwargs: mit_commands.append(dict(kwargs))
    from rebotarmcontroller.gravity_control import GravitySettings
    manager._gc_settings = GravitySettings()
    manager._gc_torque_limits = np.array([27., 27., 27., 7., 7., 7.])
    manager._gravity_comp_last_total = np.zeros(6)
    manager._gravity_comp_fault = None
    manager._gravity_comp_hold_feedforward = None
    manager._gc_compute_generalized_gravity = lambda q: np.zeros_like(q)
    manager._gc_model = object()
    manager._gc_data = object()
    manager._gc_ee_frame_id = 0

    class FakePin:
        class ReferenceFrame:
            WORLD = object()
            LOCAL_WORLD_ALIGNED = object()

        @staticmethod
        def computeJointJacobians(_model, _data, _q) -> None:
            return None

        @staticmethod
        def updateFramePlacements(_model, _data) -> None:
            return None

        @staticmethod
        def getFrameJacobian(_model, _data, _frame_id, _reference):
            return np.zeros((6, 6), dtype=np.float64)

    manager._gc_pin = FakePin
    return mit_commands


def test_gravity_integral_has_same_effect_at_different_control_rates():
    def integrate(dt):
        manager = make_manager(gripper_feedback_stale_timeout_sec=2.)
        q = np.array([.2, -.3, -.4, .1, .2, -.1])
        configure_gravity_compensation_test(manager, positions=q, velocities=np.zeros(6))
        manager._gravity_comp_active = True
        manager._gravity_comp_q_target = q + .01
        manager._gravity_comp_integral = np.zeros(6)
        for _ in range(round(.1 / dt)):
            manager._gravity_comp_tick(manager.arm, dt)
        return manager._gravity_comp_integral
    np.testing.assert_allclose(integrate(.01), integrate(.002), atol=1e-12)
    np.testing.assert_allclose(integrate(.01), .001, atol=1e-12)


def test_gravity_mode_failure_rolls_back_to_verified_position_hold():
    manager = make_manager()
    q = np.array([.2, -.3, -.4, .1, .2, -.1])
    commands = configure_gravity_compensation_test(manager, positions=q, velocities=np.zeros(6))
    manager.arm.mode_mit = lambda **kwargs: False
    with pytest.raises(RuntimeError, match='MIT mode'):
        manager.start_gravity_compensation()
    assert commands == []
    assert not manager.gravity_compensation_active()
    assert manager.enabled
    assert manager.arm.mode_pos_vel_calls == 1
    np.testing.assert_allclose(manager._endpos_ctrl._q_target, q)


def test_gravity_sender_does_not_swallow_motor_send_failure():
    manager = make_manager()
    configure_gravity_compensation_test(manager, positions=np.zeros(6), velocities=np.zeros(6))
    del manager._send_gravity_mit
    for motor in manager.arm._motor_map.values():
        motor.send_mit = lambda *args: None
    manager.arm._motor_map['joint3'].send_mit = lambda *args: (_ for _ in ()).throw(
        RuntimeError('serial write failed'))
    with pytest.raises(RuntimeError, match='joint3.*serial write failed'):
        manager._send_gravity_mit(pos=np.zeros(6), vel=np.zeros(6), kp=np.ones(6),
                                  kd=np.ones(6), tau=np.zeros(6))


def test_gravity_model_exception_keeps_healthy_arm_enabled_in_guarded_hold(monkeypatch):
    manager = make_manager()
    q = np.array([.2, -.3, -.4, .1, .2, -.1])
    commands = configure_gravity_compensation_test(manager, positions=q, velocities=np.zeros(6))
    manager.start_gravity_compensation()
    manager._gc_compute_generalized_gravity = lambda **kwargs: (_ for _ in ()).throw(
        RuntimeError('invalid model output'))
    monkeypatch.setattr(manager, 'refresh_feedback_if_due', lambda **kwargs: False)
    manager._gravity_hardware_tick(manager.arm, .002)
    assert manager.enabled and manager.gravity_compensation_active()
    assert manager._gravity_comp_fault is not None
    np.testing.assert_allclose(commands[-1]['pos'], q)
    assert manager.arm.disable_calls == 0


def test_gravity_preflight_model_failure_leaves_position_hold_running():
    manager = make_manager()
    q = np.array([.2, -.3, -.4, .1, .2, -.1])
    configure_gravity_compensation_test(manager, positions=q, velocities=np.zeros(6))
    manager._gc_compute_generalized_gravity = lambda **kwargs: np.full(6, np.nan)
    with pytest.raises(ValueError, match='finite'):
        manager.start_gravity_compensation()
    assert manager.arm.control_loop_active and manager.enabled
    assert not manager.gravity_compensation_active()
    np.testing.assert_allclose(manager._endpos_ctrl._q_target, q)


def test_gravity_deadline_fault_keeps_healthy_arm_enabled(monkeypatch):
    manager = make_manager()
    q = np.array([.2, -.3, -.4, .1, .2, -.1])
    configure_gravity_compensation_test(manager, positions=q, velocities=np.zeros(6))
    manager.start_gravity_compensation()
    from rebotarmcontroller.gravity_control import LoopTiming
    manager._gravity_timing = LoopTiming(100.)
    monkeypatch.setattr(manager, 'refresh_feedback_if_due', lambda **kwargs: False)
    for i in range(4):
        now = 10. + i * .02
        monkeypatch.setattr(hardware_manager_module.time, 'monotonic', lambda: now)
        seed_verified_feedback(manager, observed_at=now, arm_status=1)
        manager._gravity_hardware_tick(manager.arm, .01)
    assert manager._gravity_comp_fault == 'three consecutive control deadline misses'
    assert manager.enabled and manager.arm.disable_calls == 0


def test_gravity_send_failure_requests_protective_disable(monkeypatch):
    manager = make_manager()
    q = np.array([.2, -.3, -.4, .1, .2, -.1])
    configure_gravity_compensation_test(manager, positions=q, velocities=np.zeros(6))
    manager.start_gravity_compensation()
    from rebotarmcontroller.hardware_manager import GravityCommandError
    manager._send_gravity_mit = lambda **kwargs: (_ for _ in ()).throw(
        GravityCommandError('serial write failed'))
    monkeypatch.setattr(manager, 'refresh_feedback_if_due', lambda **kwargs: False)
    disables = []
    manager.arm._ctrl_map['fake'].disable_all = lambda: disables.append(True)
    manager._gravity_hardware_tick(manager.arm, .002)
    assert disables == [True]
    assert not manager.gravity_compensation_active()
    assert 'GRAVITY_COMMAND_PROTECTIVE_DISABLE' in manager.error_codes
    assert 'FEEDBACK_PROTECTIVE_DISABLE' not in manager.error_codes


class FakeThread:
    def __init__(self, *, target, name: str, daemon: bool) -> None:
        self.target = target
        self.name = name
        self.daemon = daemon

    def start(self) -> None:
        pass

    def join(self, timeout: float | None = None) -> None:
        del timeout


def test_validated_joint_feedback_uses_verified_cache_only() -> None:
    manager = make_manager()
    manager._connected = True
    manager._enabled = True
    expected = np.array([0.1, -0.2, -0.3, 0.4, 0.5, -0.6])
    for name, position in zip(JOINT_NAMES, expected):
        manager.arm._motor_map[name].state.pos = float(position)
    before = seed_verified_feedback(manager, arm_status=1)
    forbid_raw_feedback_reads(manager)

    positions, velocities, torques, statuses = manager._validated_joint_feedback(
        expected_status=1,
        refresh=False,
    )

    assert np.allclose(positions, expected)
    assert np.allclose(velocities, 0.0)
    assert np.allclose(torques, 0.0)
    assert statuses == [1] * 6
    assert_verified_feedback_unchanged(manager, before)


def test_joint_status_codes_use_verified_cache_only() -> None:
    manager = make_manager()
    manager._connected = True
    before = seed_verified_feedback(manager, arm_status=1)
    forbid_raw_feedback_reads(manager)

    assert manager.get_joint_status_codes() == [1] * 6
    assert_verified_feedback_unchanged(manager, before)


def test_cached_joint_sample_exposes_verified_identity_without_serial_reads() -> None:
    manager = make_manager()
    manager._connected = True
    manager._enabled = True
    before = seed_verified_feedback(manager, arm_status=1)
    forbid_raw_feedback_reads(manager)
    positions, velocities, torques, statuses, identity = manager.get_cached_joint_sample()
    assert len(positions) == len(velocities) == len(torques) == 6
    assert statuses == [1] * 6
    assert identity == tuple(manager._verified_feedback_sample(name).sequence for name in JOINT_NAMES)
    assert_verified_feedback_unchanged(manager, before)


def test_validated_gripper_status_uses_verified_cache_after_refresh(
    monkeypatch,
) -> None:
    manager = make_manager()
    manager._connected = True
    gripper_state = SimpleNamespace(pos=-1.25, vel=0.1, torq=0.2, status_code=1)
    manager._gripper_mot = FakeMotor(position=gripper_state.pos)
    manager._gripper_ctrl = SimpleNamespace(poll_feedback_once=lambda: None)
    before = seed_verified_feedback(
        manager,
        arm_status=1,
        gripper_state=gripper_state,
    )
    forbid_raw_feedback_reads(manager)
    monkeypatch.setattr(manager, "refresh_feedback_if_due", lambda **_kwargs: False)

    manager._validated_gripper_status(expected_status=1)

    assert_verified_feedback_unchanged(manager, before)


def test_gripper_publication_uses_verified_cache_only() -> None:
    manager = make_manager()
    manager._connected = True
    gripper_state = SimpleNamespace(pos=-1.25, vel=0.1, torq=0.2, status_code=1)
    manager._gripper_mot = FakeMotor(position=gripper_state.pos)
    manager._gripper_ctrl = SimpleNamespace(poll_feedback_once=lambda: None)
    before = seed_verified_feedback(
        manager,
        arm_status=1,
        gripper_state=gripper_state,
    )
    forbid_raw_feedback_reads(manager)

    assert manager.get_gripper_state() == pytest.approx((-1.25, 0.1, 0.2, 1))
    assert_verified_feedback_unchanged(manager, before)


def test_gripper_target_start_uses_verified_cache_only() -> None:
    manager = make_manager()
    manager._connected = True
    manager._enabled = True
    manager.arm.control_loop_active = True
    gripper_state = SimpleNamespace(pos=-1.25, vel=0.1, torq=0.2, status_code=1)
    manager._gripper_mot = FakeMotor(position=gripper_state.pos)
    manager._gripper_ctrl = SimpleNamespace(poll_feedback_once=lambda: None)
    before = seed_verified_feedback(
        manager,
        arm_status=1,
        gripper_state=gripper_state,
    )
    forbid_raw_feedback_reads(manager)

    manager.set_gripper_target(0.04, max_effort=0.4)

    assert manager._gripper_target_angle == pytest.approx(-1.25)
    assert manager.gripper_active is True
    assert_verified_feedback_unchanged(manager, before)


def test_joint_motor_command_defaults_use_verified_cache_only() -> None:
    manager = make_manager()
    manager._connected = True
    manager._enabled = True
    state = manager.arm._motor_map["joint1"].state
    state.pos = 0.25
    state.vel = -0.5
    state.status_code = 1
    commands: list[tuple[float, ...]] = []
    manager.arm._motor_map["joint1"].send_mit = (
        lambda *values: commands.append(tuple(float(value) for value in values))
    )
    before = seed_verified_feedback(manager, arm_status=1)
    forbid_raw_feedback_reads(manager)
    cmd = SimpleNamespace(
        mode=0,
        use_pos=False,
        pos=99.0,
        use_vel=False,
        vel=99.0,
        use_kp=True,
        kp=2.0,
        use_kd=True,
        kd=0.3,
        use_tau=False,
        tau=99.0,
        use_vlim=True,
        vlim=1.0,
    )

    manager.send_joint_motor_cmd("joint1", cmd)

    assert commands == [(0.25, -0.5, 2.0, 0.3, 0.0)]
    assert_verified_feedback_unchanged(manager, before)


def test_gripper_motor_command_defaults_use_verified_cache_only() -> None:
    manager = make_manager()
    manager._connected = True
    manager._enabled = True
    gripper_state = SimpleNamespace(pos=-1.25, vel=0.1, torq=0.2, status_code=1)
    commands: list[tuple[float, ...]] = []
    manager._gripper_mot = FakeMotor(position=gripper_state.pos)
    manager._gripper_mot.send_mit = (
        lambda *values: commands.append(tuple(float(value) for value in values))
    )
    manager._gripper_ctrl = SimpleNamespace(poll_feedback_once=lambda: None)
    manager._gripper_cfg = SimpleNamespace(kp=3.0, kd=0.4, vlim=1.5)
    before = seed_verified_feedback(
        manager,
        arm_status=1,
        gripper_state=gripper_state,
    )
    forbid_raw_feedback_reads(manager)
    cmd = SimpleNamespace(
        mode=0,
        use_pos=False,
        pos=99.0,
        use_vel=False,
        vel=99.0,
        use_kp=False,
        kp=99.0,
        use_kd=False,
        kd=99.0,
        use_tau=False,
        tau=99.0,
        use_vlim=False,
        vlim=99.0,
    )

    manager.send_gripper_motor_cmd(cmd)

    assert commands == [(-1.25, 0.1, 3.0, 0.4, 0.0)]
    assert_verified_feedback_unchanged(manager, before)


def test_gripper_tick_uses_verified_cache_only() -> None:
    manager = make_manager()
    commands = _configure_gripper_position_move(
        manager,
        position=-0.5,
        goal=-4.0,
        effort=1.0,
    )
    manager._connected = True
    manager._enabled = True
    gripper_state = SimpleNamespace(pos=-0.5, vel=0.0, torq=0.0, status_code=1)
    before = seed_verified_feedback(
        manager,
        arm_status=1,
        gripper_state=gripper_state,
    )
    forbid_raw_feedback_reads(manager)

    manager._gripper_tick()

    assert manager.gripper_active is True
    assert len(commands) == 1
    assert commands[0][2:4] == (5.0, 1.0)
    assert_verified_feedback_unchanged(manager, before)


def test_gravity_start_and_tick_use_one_verified_joint_snapshot() -> None:
    manager = make_manager()
    positions = np.array([0.2, -0.3, -0.4, 0.1, 0.2, -0.1])
    velocities = np.array([0.01, -0.02, 0.03, -0.04, 0.05, -0.06])
    mit_commands = configure_gravity_compensation_test(
        manager,
        positions=positions,
        velocities=velocities,
    )

    manager.start_gravity_compensation()

    assert np.allclose(manager.gravity_compensation_target(), positions)
    assert len(mit_commands) == 1
    assert np.allclose(mit_commands[0]["pos"], positions)

    mit_commands.clear()
    manager._gravity_comp_tick(manager.arm, 1.0 / 500.0)

    assert len(mit_commands) == 1
    assert np.allclose(mit_commands[0]["pos"], positions)


@pytest.mark.parametrize(
    ("failure_kind", "expected_error"),
    (
        ("error", "arm feedback unavailable"),
        ("stale", "arm feedback stale"),
    ),
)
def test_gravity_tick_rejects_unhealthy_verified_feedback_before_torque(
    monkeypatch,
    failure_kind: str,
    expected_error: str,
) -> None:
    manager = make_manager()
    positions = np.array([0.2, -0.3, -0.4, 0.1, 0.2, -0.1])
    velocities = np.zeros(6, dtype=np.float64)
    mit_commands = configure_gravity_compensation_test(
        manager,
        positions=positions,
        velocities=velocities,
    )
    manager._gravity_comp_active = True
    manager._gravity_comp_q_target = positions.copy()
    manager._gravity_comp_integral = np.zeros(6, dtype=np.float64)
    if failure_kind == "error":
        manager._arm_feedback_error = "serial feedback timeout"
    else:
        manager._arm_feedback_error = None
        manager._arm_feedback_updated_monotonic = 10.0
        monkeypatch.setattr(hardware_manager_module.time, "monotonic", lambda: 11.0)

    with pytest.raises(RuntimeError, match=expected_error):
        manager._gravity_comp_tick(manager.arm, 1.0 / 500.0)

    assert mit_commands == []


@pytest.mark.parametrize("failure_kind", ("error", "stale"))
def test_gripper_tick_does_not_copy_unhealthy_sample_before_neutral(
    monkeypatch,
    failure_kind: str,
) -> None:
    manager = make_manager()
    commands = _configure_gripper_position_move(
        manager,
        position=-0.5,
        goal=-4.0,
        effort=1.0,
    )
    original = (
        manager._gripper_pos,
        manager._gripper_vel,
        manager._gripper_torque,
        manager._gripper_status_code,
    )
    with manager._feedback_lock:
        previous = manager._verified_feedback_by_label["gripper"]
        manager._verified_feedback_by_label["gripper"] = type(previous)(
            state=SimpleNamespace(pos=-3.0, vel=0.3, torq=0.7, status_code=1),
            sequence=previous.sequence + 1,
            observed_at=10.0,
        )
    forbid_raw_feedback_reads(manager)
    if failure_kind == "error":
        manager._gripper_feedback_error = "serial feedback timeout"
    else:
        manager._gripper_feedback_error = None
        manager._gripper_feedback_updated_monotonic = 10.0
        monkeypatch.setattr(hardware_manager_module.time, "monotonic", lambda: 11.0)

    manager._gripper_tick()

    assert manager.gripper_active is False
    assert (
        manager._gripper_pos,
        manager._gripper_vel,
        manager._gripper_torque,
        manager._gripper_status_code,
    ) == original
    assert commands == [(-0.5, 0.0, 0.0, 0.0, 0.0)]


@pytest.mark.parametrize("failure_kind", ("error", "stale"))
def test_outer_hardware_tick_protects_on_arm_feedback_loss(
    monkeypatch,
    failure_kind: str,
) -> None:
    manager = make_manager()
    positions = np.array([0.2, -0.3, -0.4, 0.1, 0.2, -0.1])
    mit_commands = configure_gravity_compensation_test(
        manager,
        positions=positions,
        velocities=np.zeros(6, dtype=np.float64),
    )
    gripper_commands = _configure_gripper_position_move(
        manager,
        position=-0.5,
        goal=-4.0,
        effort=1.0,
    )
    manager._gravity_comp_active = True
    manager._gravity_comp_q_target = positions.copy()
    manager._gravity_comp_integral = np.zeros(6, dtype=np.float64)
    manager._lifecycle_state = "ENABLED_HOLD"
    manager.arm._running = True
    manager._endpos_ctrl._stop_send = threading.Event()
    manager._endpos_ctrl._moving = True
    disable_calls: list[str] = []
    shared_controller = manager.arm._ctrl_map["fake"]
    manager.arm._ctrl_map["duplicate"] = shared_controller
    manager._gripper_ctrl = shared_controller
    shared_controller.disable_all = lambda: disable_calls.append("arm")
    monkeypatch.setattr(manager, "refresh_feedback_if_due", lambda **_kwargs: False)
    if failure_kind == "error":
        manager._arm_feedback_error = "serial feedback timeout"
    else:
        manager._arm_feedback_error = None
        manager._arm_feedback_updated_monotonic = 10.0
        monkeypatch.setattr(hardware_manager_module.time, "monotonic", lambda: 11.0)
        with manager._gripper_lock:
            manager._queue_gripper_neutral_locked(
                "pre-existing operator release",
                marks_success=False,
            )

    manager._gravity_hardware_tick(manager.arm, 1.0 / 500.0)

    assert mit_commands == []
    assert gripper_commands == [(-0.5, 0.0, 0.0, 0.0, 0.0)]
    assert manager._gripper_neutral_pending is None
    assert manager.gripper_active is False
    assert disable_calls == ["arm"]
    assert manager.arm._running is False
    assert manager._endpos_ctrl._stop_send.is_set() is True
    assert manager._endpos_ctrl._moving is False
    assert manager.lifecycle_state == "DISABLING"
    assert manager.enabled is True
    assert manager.gravity_compensation_active() is False
    assert "FEEDBACK_PROTECTIVE_DISABLE" in manager.error_codes


def test_outer_hardware_tick_records_neutral_and_disable_failures(
    monkeypatch,
) -> None:
    manager = make_manager()
    positions = np.array([0.2, -0.3, -0.4, 0.1, 0.2, -0.1])
    configure_gravity_compensation_test(
        manager,
        positions=positions,
        velocities=np.zeros(6, dtype=np.float64),
    )
    _configure_gripper_position_move(manager, position=-0.5, goal=-4.0)
    manager._gravity_comp_active = True
    manager._gravity_comp_q_target = positions.copy()
    manager.arm._running = True
    manager._arm_feedback_error = "serial feedback timeout"
    manager._gripper_mot.send_mit = lambda *_args: (_ for _ in ()).throw(
        RuntimeError("neutral write failed")
    )
    controller = manager.arm._ctrl_map["fake"]
    manager._gripper_ctrl = controller
    controller.disable_all = lambda: (_ for _ in ()).throw(
        RuntimeError("disable write failed")
    )
    monkeypatch.setattr(manager, "refresh_feedback_if_due", lambda **_kwargs: False)

    manager._gravity_hardware_tick(manager.arm, 1.0 / 500.0)

    assert manager.arm._running is False
    assert manager.enabled is True
    assert manager.lifecycle_state == "DISABLING"
    assert any("NEUTRAL_FAILED" in code for code in manager.error_codes)
    assert any("DISABLE_FAILED" in code for code in manager.error_codes)


def test_outer_hardware_tick_protects_when_scheduler_raises_feedback_error(
    monkeypatch,
) -> None:
    manager = make_manager()
    manager._connected = True
    manager._enabled = True
    manager.arm._running = True
    manager._arm_feedback_error = None

    def fail_feedback_refresh(**_kwargs) -> None:
        manager._arm_feedback_error = "scheduler serial timeout"
        raise RuntimeError("scheduler transaction failed")

    disable_calls: list[int] = []
    manager.arm._ctrl_map["fake"].disable_all = lambda: disable_calls.append(1)
    monkeypatch.setattr(manager, "refresh_feedback_if_due", fail_feedback_refresh)
    callback_calls: list[int] = []

    manager._hardware_control_tick(
        manager.arm,
        1.0 / 500.0,
        lambda _arm, _dt: callback_calls.append(1),
    )

    assert callback_calls == []
    assert disable_calls == [1]
    assert manager.arm._running is False
    assert manager.lifecycle_state == "DISABLING"


def test_outer_hardware_tick_reraises_non_feedback_callback_failure(
    monkeypatch,
) -> None:
    manager = make_manager()
    manager._connected = True
    manager._enabled = True
    seed_verified_feedback(manager, arm_status=1)
    monkeypatch.setattr(manager, "refresh_feedback_if_due", lambda **_kwargs: True)

    with pytest.raises(RuntimeError, match="controller bug"):
        manager._hardware_control_tick(
            manager.arm,
            1.0 / 500.0,
            lambda _arm, _dt: (_ for _ in ()).throw(RuntimeError("controller bug")),
        )

    assert manager.enabled is True
    assert manager.lifecycle_state != "DISABLING"

    monkeypatch.setattr(
        manager,
        "refresh_feedback_if_due",
        lambda **_kwargs: (_ for _ in ()).throw(RuntimeError("scheduler bug")),
    )
    with pytest.raises(RuntimeError, match="scheduler bug"):
        manager._hardware_control_tick(
            manager.arm,
            1.0 / 500.0,
            lambda _arm, _dt: None,
        )

    assert manager.lifecycle_state != "DISABLING"


def test_outer_hardware_tick_orders_feedback_before_arm_and_gripper(
    monkeypatch,
) -> None:
    manager = make_manager()
    manager._connected = True
    manager._enabled = True
    seed_verified_feedback(manager, arm_status=1)
    events: list[str] = []
    monkeypatch.setattr(
        manager,
        "refresh_feedback_if_due",
        lambda **_kwargs: events.append("feedback") or True,
    )
    monkeypatch.setattr(manager, "_gripper_tick", lambda: events.append("gripper"))

    manager._hardware_control_tick(
        manager.arm,
        1.0 / 500.0,
        lambda _arm, _dt: events.append("arm"),
    )

    assert events == ["feedback", "arm", "gripper"]


def test_connect_keeps_all_motors_disabled_and_does_not_start_control() -> None:
    manager = make_manager()

    manager.connect()

    assert manager.connected is True
    assert manager.enabled is False
    assert manager.lifecycle_state == "CONNECTED_DISABLED"
    assert manager.arm.connect_calls == 1
    assert manager.arm.enable_calls == 0
    assert manager.arm.disable_calls == 0
    assert manager.arm.mode_pos_vel_calls == 0
    assert manager.arm.start_control_loop_calls == 0
    assert manager.get_joint_status_codes() == [0] * 6


def test_disabled_joint_state_reads_request_fresh_feedback() -> None:
    manager = make_manager()
    manager.connect()
    requests_before = manager.arm._motor_map["joint1"].request_feedback_calls

    manager.get_joint_state()

    assert manager.arm._motor_map["joint1"].request_feedback_calls == requests_before + 1


def test_enabled_joint_state_reads_use_control_loop_cache() -> None:
    manager = make_manager()
    manager.connect()
    manager.enable()
    requests_before = manager.arm._motor_map["joint1"].request_feedback_calls

    manager.get_joint_state()

    assert manager.arm._motor_map["joint1"].request_feedback_calls == requests_before


def test_connect_forces_unexpected_enabled_motors_back_to_disabled() -> None:
    manager = make_manager()
    manager.arm._motor_map["joint3"].state.status_code = 1

    manager.connect()

    assert manager.enabled is False
    assert manager.lifecycle_state == "CONNECTED_DISABLED"
    assert manager.arm.disable_calls == 1
    assert manager.get_joint_status_codes() == [0] * 6


def test_enable_holds_current_position_before_starting_control_loop() -> None:
    manager = make_manager()
    current = np.array([0.2, -0.4, -0.3, 0.1, 0.0, 0.3])
    for name, position in zip(JOINT_NAMES, current):
        manager.arm._motor_map[name].state.pos = float(position)
    manager.connect()

    manager.enable()

    assert manager.enabled is True
    assert manager.lifecycle_state == "ENABLED_HOLD"
    assert manager.arm.enable_calls == 1
    assert manager.arm.mode_pos_vel_calls == 1
    assert manager.arm.start_control_loop_calls == 1
    assert np.allclose(manager.endpos_ctrl._q_target, current)
    assert manager.get_joint_status_codes() == [1] * 6
    assert manager.ready_for_motion is True


def test_enable_failure_disables_every_motor_and_returns_to_disabled_state() -> None:
    manager = make_manager(fail_enable_joint="joint4")
    manager.connect()
    disable_calls_before_enable = manager.arm.disable_calls

    with pytest.raises(RuntimeError, match="joint4"):
        manager.enable()

    assert manager.enabled is False
    assert manager.lifecycle_state == "CONNECTED_DISABLED"
    assert manager.arm.disable_calls > disable_calls_before_enable
    assert manager.arm.control_loop_active is False
    assert manager.get_joint_status_codes() == [0] * 6


def test_motion_entrypoints_do_not_implicitly_enable() -> None:
    manager = make_manager()
    manager.connect()

    with pytest.raises(RuntimeError, match="explicit /rebotarm/enable"):
        manager.ensure_pos_vel_control()
    with pytest.raises(RuntimeError, match="explicit /rebotarm/enable"):
        manager.start_gravity_compensation()

    assert manager.arm.enable_calls == 0
    assert manager.ready_for_motion is False


def test_stop_active_motion_is_safe_during_concurrent_disable() -> None:
    manager = make_manager()
    manager.connect()
    manager.enable()
    disable_started = threading.Event()
    allow_disable = threading.Event()
    original_disable_all_motors = manager._disable_all_motors
    errors: list[BaseException] = []

    def delayed_disable_all_motors() -> None:
        disable_started.set()
        assert allow_disable.wait(timeout=1.0)
        original_disable_all_motors()

    def record_errors(operation) -> None:
        try:
            operation()
        except BaseException as exc:  # pragma: no cover - assertion reports details
            errors.append(exc)

    manager._disable_all_motors = delayed_disable_all_motors
    disable_thread = threading.Thread(
        target=record_errors,
        args=(manager.disable,),
    )
    stop_thread = threading.Thread(
        target=record_errors,
        args=(manager.stop_active_motion,),
    )

    disable_thread.start()
    assert disable_started.wait(timeout=1.0)
    stop_thread.start()
    allow_disable.set()
    disable_thread.join(timeout=1.0)
    stop_thread.join(timeout=1.0)

    assert disable_thread.is_alive() is False
    assert stop_thread.is_alive() is False
    assert errors == []
    assert manager.enabled is False
    assert manager.lifecycle_state == "CONNECTED_DISABLED"
    assert manager.state_machine == "IDLE"
    assert manager.get_joint_status_codes() == [0] * 6


def test_joint_state_read_waits_for_disable_transition() -> None:
    manager = make_manager()
    manager.connect()
    manager.enable()
    disable_started = threading.Event()
    allow_disable = threading.Event()
    original_disable_all_motors = manager._disable_all_motors
    errors: list[BaseException] = []
    reads: list[tuple[np.ndarray, np.ndarray, np.ndarray]] = []

    def delayed_disable_all_motors() -> None:
        disable_started.set()
        assert allow_disable.wait(timeout=1.0)
        original_disable_all_motors()

    def disable() -> None:
        try:
            manager.disable()
        except BaseException as exc:  # pragma: no cover - assertion reports details
            errors.append(exc)

    def read_joint_state() -> None:
        try:
            reads.append(manager.get_joint_state())
        except BaseException as exc:  # pragma: no cover - assertion reports details
            errors.append(exc)

    manager._disable_all_motors = delayed_disable_all_motors
    disable_thread = threading.Thread(target=disable)
    read_thread = threading.Thread(target=read_joint_state)

    disable_thread.start()
    assert disable_started.wait(timeout=1.0)
    read_thread.start()
    allow_disable.set()
    disable_thread.join(timeout=1.0)
    read_thread.join(timeout=1.0)

    assert disable_thread.is_alive() is False
    assert read_thread.is_alive() is False
    assert errors == []
    assert len(reads) == 1
    assert manager.enabled is False
    assert manager.get_joint_status_codes() == [0] * 6


def test_action_goal_callback_rejects_motion_until_explicit_enable() -> None:
    source = (
        Path(__file__).resolve().parents[1]
        / "src/rebotarmcontroller/rebotarmcontroller/ros_actions.py"
    ).read_text(encoding="utf-8")

    assert "if not self._hardware.ready_for_motion:" in source
    assert "return GoalResponse.REJECT" in source


def test_connect_rejects_missing_or_out_of_limit_feedback_and_disconnects() -> None:
    manager = make_manager()
    manager.arm._motor_map["joint2"].state = None

    with pytest.raises(RuntimeError, match="joint2 feedback unavailable"):
        manager.connect()
    assert manager.connected is False
    assert manager.lifecycle_state == "DISCONNECTED"
    assert manager.arm.disconnect_calls == 1

    manager = make_manager()
    manager.arm._motor_map["joint5"].state.pos = 2.0
    with pytest.raises(RuntimeError, match="joint5 position"):
        manager.connect()
    assert manager.connected is False
    assert manager.lifecycle_state == "DISCONNECTED"


@pytest.mark.parametrize("joint_name", ["joint2", "joint3"])
@pytest.mark.parametrize("position", [0.000478, 0.019, 0.02])
def test_zero_margin_accepts_feedback_but_rejects_larger_positive_feedback(joint_name, position) -> None:
    manager = make_manager()
    manager.arm._motor_map[joint_name].state.pos = position

    manager.connect()

    assert manager.connected is True
    manager.shutdown()

    manager = make_manager()
    manager.arm._motor_map[joint_name].state.pos = 0.020001

    with pytest.raises(RuntimeError, match=f"{joint_name} position"):
        manager.connect()

    assert manager.connected is False
    assert manager.lifecycle_state == "DISCONNECTED"


def test_connect_does_not_swallow_active_feedback_refresh_failure() -> None:
    manager = make_manager()
    controller = SimpleNamespace(poll_feedback_once=lambda: None)
    manager.arm._ctrl_map = {"fake": controller}
    for joint in manager.arm._joints:
        joint.vendor = "fake"

    def fail_request() -> None:
        raise RuntimeError("feedback timeout")

    manager.arm._motor_map["joint3"].request_feedback = fail_request

    with pytest.raises(RuntimeError, match="shared feedback batch failed"):
        manager.connect()

    assert manager.connected is False
    assert manager.lifecycle_state == "DISCONNECTED"
    assert manager.arm.disconnect_calls == 1


def test_feedback_refresh_batches_requests_before_one_controller_poll() -> None:
    manager = make_manager()
    events: list[str] = []
    controller = SimpleNamespace(
        poll_feedback_once=lambda: events.append("poll"),
    )
    manager.arm._ctrl_map = {"fake": controller}
    for joint in manager.arm._joints:
        joint.vendor = "fake"
        motor = manager.arm._motor_map[joint.name]
        original_request = motor.request_feedback
        motor.request_feedback = (
            lambda name=joint.name, request=original_request: (
                events.append(f"request:{name}"), request()
            )[-1]
        )
    original_poll = controller.poll_feedback_once
    controller.poll_feedback_once = lambda: (
        original_poll(),
        [motor.deliver_requested_feedback() for motor in manager.arm._motor_map.values()],
    )[0]

    manager._refresh_arm_feedback()

    assert events == [*(f"request:{name}" for name in JOINT_NAMES), "poll"]


def test_feedback_refresh_holds_controller_lock_for_full_transaction() -> None:
    manager = make_manager()
    events: list[str] = []

    class RecordingLock:
        def __enter__(self):
            events.append("lock:enter")

        def __exit__(self, _exc_type, _exc, _traceback):
            events.append("lock:exit")

    controller = SimpleNamespace(
        _bus_lock=RecordingLock(),
        poll_feedback_once=lambda: events.append("poll"),
    )
    manager.arm._ctrl_map = {"fake": controller}
    for joint in manager.arm._joints:
        joint.vendor = "fake"
        motor = manager.arm._motor_map[joint.name]
        original_request = motor.request_feedback
        motor.request_feedback = (
            lambda name=joint.name, request=original_request: (
                events.append(f"request:{name}"), request()
            )[-1]
        )
    original_poll = controller.poll_feedback_once
    controller.poll_feedback_once = lambda: (
        original_poll(),
        [motor.deliver_requested_feedback() for motor in manager.arm._motor_map.values()],
    )[0]

    manager._refresh_arm_feedback()

    assert events[0] == "lock:enter"
    assert events[-1] == "lock:exit"
    assert events.count("lock:enter") == 1
    assert events.count("lock:exit") == 1


def test_forced_lifecycle_feedback_refresh_retries_transient_failure() -> None:
    manager = make_manager()
    calls = {"request": 0, "poll": 0}
    state_holder = {"state": None, "sequence": 0}

    def request_feedback() -> None:
        calls["request"] += 1

    def poll_feedback_once() -> None:
        calls["poll"] += 1
        if calls["poll"] == 2:
            state_holder["state"] = SimpleNamespace(
                pos=0.0,
                vel=0.0,
                torq=0.0,
                status_code=0,
            )
            state_holder["sequence"] += 1

    manager._gripper_mot = SimpleNamespace(
        request_feedback=request_feedback,
        get_state=lambda: state_holder["state"],
        get_state_with_sequence=lambda: (
            state_holder["state"], state_holder["sequence"]
        ),
    )
    manager._gripper_ctrl = SimpleNamespace(
        poll_feedback_once=poll_feedback_once,
    )

    manager._refresh_all_feedback()

    assert calls == {"request": 2, "poll": 2}


def test_gripper_mode_is_set_before_explicit_gripper_enable() -> None:
    manager = make_manager()
    events: list[str] = []
    state = SimpleNamespace(
        status_code=0,
        pos=-1.0,
        vel=0.0,
        torq=0.0,
    )

    def ensure_mode(_mode, _timeout_ms: int) -> None:
        events.append("gripper_mode")

    def gripper_enable() -> None:
        events.append("gripper_enable")
        state.status_code = 1

    def gripper_disable() -> None:
        state.status_code = 0

    sequence = {"value": 0}
    manager._gripper_mot = SimpleNamespace(
        request_feedback=lambda: None,
        get_state=lambda: state,
        get_state_with_sequence=lambda: (state, sequence["value"]),
        ensure_mode=ensure_mode,
        enable=gripper_enable,
        disable=gripper_disable,
    )
    manager._gripper_ctrl = SimpleNamespace(
        poll_feedback_once=lambda: sequence.update(value=sequence["value"] + 1)
    )
    original_arm_enable = manager.arm.enable

    def arm_enable() -> None:
        events.append("arm_enable")
        original_arm_enable()

    manager.arm.enable = arm_enable
    manager._start_gripper_loop = lambda: events.append("gripper_loop")
    manager.connect()

    manager.enable()

    assert events == ["gripper_mode", "arm_enable", "gripper_enable"]
    assert manager.ready_for_motion is True
    manager.set_gripper_target(0.05)
    assert events[-1] == "gripper_loop"
    manager.disable()


def test_gripper_target_does_not_create_an_independent_control_thread(
    monkeypatch,
) -> None:
    """Removing the shared-loop backport must recreate the extra 500 Hz writer."""
    manager = make_manager()
    manager._connected = True
    manager._enabled = True
    manager.arm.control_loop_active = True
    manager._gripper_loop_running = False
    manager._gripper_loop_thread = None
    del manager._start_gripper_loop
    manager._gripper_mot = FakeMotor(position=-1.0)
    manager._gripper_mot.state.status_code = 1
    manager._gripper_ctrl = SimpleNamespace(poll_feedback_once=lambda: None)
    seed_verified_feedback(
        manager,
        arm_status=1,
        gripper_state=manager._gripper_mot.state,
    )
    created_threads: list[FakeThread] = []

    def make_thread(**kwargs):
        thread = FakeThread(**kwargs)
        created_threads.append(thread)
        return thread

    monkeypatch.setattr(hardware_manager_module.threading, "Thread", make_thread)

    manager.set_gripper_target(0.08, max_effort=1.0)

    assert created_threads == []


def test_arm_control_callback_also_emits_active_gripper_command() -> None:
    """The installed arm loop must be the sole arm-and-gripper command writer."""
    manager = make_manager()
    arm_events: list[str] = []
    manager._endpos_ctrl._loop_cb = lambda _arm, _dt: arm_events.append("arm")
    commands = _configure_gripper_position_move(
        manager,
        position=-1.0,
        goal=-4.0,
        effort=1.0,
    )
    manager._connected = True
    manager._enabled = True
    seed_verified_feedback(manager, arm_status=1)

    manager._start_pos_vel_loop(target=np.zeros(6))
    assert manager.arm.control_callback is not None
    manager.arm.control_callback(manager.arm, 1.0 / 500.0)

    assert arm_events == ["arm"]
    assert len(commands) == 1


def test_shared_feedback_scheduler_batches_all_seven_motors_at_50hz() -> None:
    """A regression to per-command feedback would turn 50 batches into 500."""
    manager = make_manager(hardware_feedback_rate_hz=50.0)
    poll_calls: list[int] = []
    controller = SimpleNamespace(
        poll_feedback_once=lambda: poll_calls.append(1),
    )
    manager.arm._ctrl_map = {"fake": controller}
    manager._gripper_mot = FakeMotor(position=-1.0)
    manager._gripper_mot.state.status_code = 1
    manager._gripper_ctrl = controller

    for tick in range(500):
        manager.refresh_feedback_if_due(now=tick / 500.0)

    for motor in manager.arm._motor_map.values():
        assert motor.request_feedback_calls == 50
    assert manager._gripper_mot.request_feedback_calls == 50
    assert len(poll_calls) == 50


def test_unchanged_feedback_sequence_does_not_advance_timestamp() -> None:
    manager = make_manager()
    controller = manager.arm._ctrl_map["fake"]

    assert manager.refresh_feedback_if_due(now=10.0) is True
    for motor in manager.arm._motor_map.values():
        motor.advance_feedback_on_poll = False
    assert manager.refresh_feedback_if_due(now=10.02) is True
    accepted_at = manager._arm_feedback_updated_monotonic

    assert accepted_at == pytest.approx(10.02)
    assert manager.refresh_feedback_if_due(now=10.04) is True
    assert controller.poll_feedback_calls == 3
    assert manager._arm_feedback_updated_monotonic == accepted_at


def test_identical_numeric_feedback_with_advanced_sequence_is_accepted() -> None:
    manager = make_manager()
    expected_positions = {
        name: motor.state.pos for name, motor in manager.arm._motor_map.items()
    }

    assert manager.refresh_feedback_if_due(now=20.0) is True
    assert manager._arm_feedback_updated_monotonic is None
    assert manager.refresh_feedback_if_due(now=20.02) is True

    assert manager._arm_feedback_updated_monotonic == pytest.approx(20.02)
    assert manager._arm_feedback_failure_reason(now=20.02) is None
    assert {
        name: manager._verified_feedback_by_label[name].state.pos
        for name in JOINT_NAMES
    } == expected_positions
    assert {
        manager._verified_feedback_by_label[name].sequence for name in JOINT_NAMES
    } == {1}


def test_arm_feedback_can_recover_while_gripper_sequence_stays_unchanged() -> None:
    manager = make_manager()
    gripper = FakeMotor(position=-1.0)
    gripper.advance_feedback_on_poll = False
    controller = manager.arm._ctrl_map["fake"]
    controller._motors.append(gripper)
    manager._gripper_mot = gripper
    manager._gripper_ctrl = controller

    assert manager.refresh_feedback_if_due(now=30.0) is True
    assert manager.refresh_feedback_if_due(now=30.02) is True

    assert manager._arm_feedback_updated_monotonic == pytest.approx(30.02)
    assert manager._arm_feedback_failure_reason(now=30.02) is None
    assert manager._gripper_feedback_updated_monotonic is None
    assert manager._gripper_feedback_failure_reason_locked(now=30.02) is not None
    assert "gripper" not in manager._verified_feedback_by_label


def test_delayed_sequence_within_pending_window_is_accepted_later() -> None:
    manager = make_manager()
    controller = manager.arm._ctrl_map["fake"]
    for motor in manager.arm._motor_map.values():
        motor.advance_feedback_on_poll = False

    assert manager.refresh_feedback_if_due(now=40.0) is True
    assert manager._arm_feedback_updated_monotonic is None
    for motor in manager.arm._motor_map.values():
        motor.feedback_sequence = 1

    assert manager.refresh_feedback_if_due(now=40.02) is True

    assert controller.poll_feedback_calls == 2
    assert manager._arm_feedback_updated_monotonic == pytest.approx(40.02)
    assert manager._arm_feedback_failure_reason(now=40.02) is None


def test_feedback_deadline_expiry_retains_diagnostic_sample_and_reports_unknown() -> None:
    manager = make_manager()
    gripper = FakeMotor(position=-1.0)
    controller = manager.arm._ctrl_map["fake"]
    controller._motors.append(gripper)
    manager._gripper_mot = gripper
    manager._gripper_ctrl = controller

    assert manager.refresh_feedback_if_due(now=50.0) is True
    gripper.advance_feedback_on_poll = False
    assert manager.refresh_feedback_if_due(now=50.02) is True
    accepted = manager._verified_feedback_by_label["gripper"]
    deadline = manager._feedback_request_deadline_by_label["gripper"]
    gripper.state = SimpleNamespace(
        pos=-2.0,
        vel=0.0,
        torq=0.0,
        status_code=1,
    )

    assert manager.refresh_feedback_if_due(now=deadline + 0.001) is True

    retained = manager._verified_feedback_by_label["gripper"]
    assert retained is accepted
    assert retained.state.pos == pytest.approx(-1.0)
    assert retained.observed_at == pytest.approx(50.02)
    assert manager._gripper_feedback_updated_monotonic == pytest.approx(50.02)
    assert "gripper" in manager._gripper_feedback_error
    assert "baseline=1" in manager._gripper_feedback_error
    assert manager.get_gripper_state()[3] == 255


def test_runtime_sequence_read_failure_is_scoped_and_recovers() -> None:
    manager = make_manager()
    gripper = FakeMotor(position=-1.0)
    gripper_controller = FakeController([gripper])
    manager._gripper_mot = gripper
    manager._gripper_ctrl = gripper_controller
    joint1 = manager.arm._motor_map["joint1"]
    real_getter = joint1.get_state_with_sequence
    fail = {"active": True}

    def getter():
        if fail["active"]:
            raise RuntimeError("joint1 isolated read failure")
        return real_getter()

    joint1.get_state_with_sequence = getter
    assert manager.refresh_feedback_if_due(now=60.0) is False
    assert "joint1 isolated read failure" in manager._feedback_error_by_label["joint1"]
    assert "gripper" not in manager._feedback_error_by_label
    assert manager.refresh_feedback_if_due(now=60.02) is False
    assert manager._gripper_feedback_updated_monotonic == pytest.approx(60.02)

    fail["active"] = False
    assert manager.refresh_feedback_if_due(now=60.04) is True
    assert manager.refresh_feedback_if_due(now=60.06) is True
    assert "joint1" not in manager._feedback_error_by_label
    assert manager._arm_feedback_failure_reason(now=60.06) is None


def test_active_feedback_accepts_uint64_max_wrap_to_one() -> None:
    manager = make_manager()
    for motor in manager.arm._motor_map.values():
        motor.feedback_sequence = UINT64_MAX
        motor.advance_feedback_on_poll = False

    assert manager.refresh_feedback_if_due(now=70.0) is True
    for motor in manager.arm._motor_map.values():
        motor.feedback_sequence = 1
    assert manager.refresh_feedback_if_due(now=70.02) is True

    assert manager._arm_feedback_updated_monotonic == pytest.approx(70.02)
    assert {manager._verified_feedback_by_label[name].sequence for name in JOINT_NAMES} == {1}


def test_forced_feedback_accepts_uint64_max_wrap_to_one() -> None:
    manager = make_manager()
    for motor in manager.arm._motor_map.values():
        motor.feedback_sequence = UINT64_MAX

    manager._refresh_all_feedback()

    assert {manager._verified_feedback_by_label[name].sequence for name in JOINT_NAMES} == {1}
    assert manager._arm_feedback_failure_reason() is None


def test_feedback_sequence_advance_rejects_equal_backward_and_half_range() -> None:
    advanced = HardwareManager._feedback_sequence_advanced

    assert advanced(8, 7) is True
    assert advanced(1, UINT64_MAX) is True
    assert advanced(7, 7) is False
    assert advanced(6, 7) is False
    assert advanced((7 + (1 << 63)) & UINT64_MAX, 7) is False
    assert advanced(0, UINT64_MAX) is False
    with pytest.raises(RuntimeError, match="uint64"):
        advanced(UINT64_MAX + 1, 7)


def test_forced_refresh_rejects_old_sample_after_raw_sequence_reset() -> None:
    manager = make_manager()
    manager._refresh_all_feedback()
    old_samples = dict(manager._verified_feedback_by_label)
    old_updated = manager._arm_feedback_updated_monotonic
    for motor in manager.arm._motor_map.values():
        motor.feedback_sequence = 0
        motor.advance_feedback_on_poll = False

    with pytest.raises(RuntimeError, match="feedback"):
        manager._refresh_all_feedback()

    assert manager._arm_feedback_updated_monotonic == old_updated
    assert all(manager._verified_feedback_by_label[name] is old_samples[name] for name in JOINT_NAMES)


def test_forced_refresh_rejects_poll_that_delivers_then_raises() -> None:
    manager = make_manager()
    motors = list(manager.arm._motor_map.values())

    def poll() -> None:
        for motor in motors:
            motor.deliver_requested_feedback()
        raise RuntimeError("recv failed after frames")

    manager.arm._ctrl_map["fake"] = SimpleNamespace(
        _bus_lock=threading.RLock(), poll_feedback_once=poll
    )

    with pytest.raises(RuntimeError, match="recv failed after frames"):
        manager._refresh_all_feedback()

    assert manager._verified_feedback_by_label == {}
    assert "recv failed after frames" in manager._arm_feedback_error


def test_forced_refresh_requires_success_after_latest_group_failure() -> None:
    manager = make_manager()
    arm_motors = list(manager.arm._motor_map.values())
    gripper = FakeMotor(position=-1.0)
    manager._gripper_mot = gripper
    arm_attempts = 0
    gripper_attempts = 0

    def poll_arm() -> None:
        nonlocal arm_attempts
        arm_attempts += 1
        if arm_attempts <= 2:
            for motor in arm_motors:
                motor.deliver_requested_feedback()
        if arm_attempts == 2:
            raise RuntimeError("arm transient bus error after frames")

    def poll_gripper() -> None:
        nonlocal gripper_attempts
        gripper_attempts += 1
        if gripper_attempts == 1:
            raise RuntimeError("gripper transient bus error")
        gripper.deliver_requested_feedback()

    manager.arm._ctrl_map["fake"] = SimpleNamespace(
        _bus_lock=threading.RLock(), poll_feedback_once=poll_arm
    )
    manager._gripper_ctrl = SimpleNamespace(
        _bus_lock=threading.RLock(), poll_feedback_once=poll_gripper
    )

    with pytest.raises(RuntimeError, match="arm transient bus error"):
        manager._refresh_all_feedback()

    assert arm_attempts == 3
    assert gripper_attempts == 3
    assert all(name in manager._feedback_error_by_label for name in JOINT_NAMES)
    assert manager._arm_feedback_failure_reason() is not None
    assert manager._feedback_request_baseline_by_label == {}
    assert manager._feedback_request_deadline_by_label == {}

    def poll_arm_with_fresh_frame() -> None:
        nonlocal arm_attempts
        arm_attempts += 1
        for motor in arm_motors:
            motor.deliver_requested_feedback()

    manager.arm._ctrl_map["fake"].poll_feedback_once = poll_arm_with_fresh_frame

    manager._refresh_all_feedback()

    assert all(name not in manager._feedback_error_by_label for name in JOINT_NAMES)
    assert manager._arm_feedback_failure_reason() is None


def test_forced_refresh_accepts_staggered_motor_feedback() -> None:
    """Startup must tolerate a serial bridge returning motor frames in batches."""
    manager = make_manager()
    motors = list(manager.arm._motor_map.values())
    polls = 0

    def poll_staggered() -> None:
        nonlocal polls
        polls += 1
        for motor in motors[:2] if polls == 1 else motors:
            motor.deliver_requested_feedback()

    manager.arm._ctrl_map["fake"].poll_feedback_once = poll_staggered

    manager._refresh_all_feedback()

    assert polls >= 2
    assert set(manager._verified_feedback_by_label) == set(JOINT_NAMES)
    assert manager._arm_feedback_error is None


def test_forced_initial_getter_failure_records_group_before_any_request() -> None:
    manager = make_manager()
    manager._refresh_all_feedback()
    requests_before = {
        name: motor.request_feedback_calls
        for name, motor in manager.arm._motor_map.items()
    }
    joint1 = manager.arm._motor_map["joint1"]
    joint1.get_state_with_sequence = lambda: (_ for _ in ()).throw(
        RuntimeError("initial baseline read failed")
    )

    with pytest.raises(RuntimeError, match="initial baseline read failed"):
        manager._refresh_all_feedback()

    assert "initial baseline read failed" in manager._arm_feedback_error
    assert {
        name: motor.request_feedback_calls
        for name, motor in manager.arm._motor_map.items()
    } == requests_before


def test_ros_thread_cannot_refresh_bus_while_hardware_loop_is_active() -> None:
    manager = make_manager()
    controller = SimpleNamespace(poll_feedback_once=lambda: None)
    manager.arm._ctrl_map = {"fake": controller}
    manager.arm.control_loop_active = True
    manager.arm._ctrl_thread = object()

    assert manager.refresh_feedback_if_due(now=1.0) is False
    assert all(
        motor.request_feedback_calls == 0
        for motor in manager.arm._motor_map.values()
    )


def test_failed_feedback_batch_stops_gripper_before_next_target() -> None:
    manager = make_manager()
    commands = _configure_gripper_position_move(
        manager,
        position=-0.5,
        goal=-4.0,
        effort=1.0,
    )
    controller = SimpleNamespace(
        poll_feedback_once=lambda: (_ for _ in ()).throw(
            RuntimeError("serial feedback timeout")
        )
    )
    manager.arm._ctrl_map = {"fake": controller}
    manager._gripper_ctrl = controller

    assert manager.refresh_feedback_if_due(now=1.0) is False
    manager._gripper_tick()

    assert commands == [(-0.5, 0.0, 0.0, 0.0, 0.0)]
    assert "serial feedback timeout" in manager.gripper_command_error
    before = len(commands)
    manager._gripper_tick()
    assert len(commands) == before


def _configure_gripper_position_move(
    manager: HardwareManager,
    *,
    position: float = -4.72,
    goal: float = -4.72,
    mode: str = "position",
    effort: float = _G_LARGE_MOVE_MAX_TAU_NM,
) -> list[tuple[float, ...]]:
    commands: list[tuple[float, ...]] = []
    state = SimpleNamespace(pos=position, vel=0.0, torq=0.0, status_code=1)

    def send_mit(pos: float, vel: float, kp: float, kd: float, tau: float) -> None:
        commands.append((pos, vel, kp, kd, tau))

    manager._gripper_mot = SimpleNamespace(
        get_state=lambda: state,
        get_state_with_sequence=lambda: (state, 0),
        send_mit=send_mit,
        request_feedback=lambda: None,
    )
    manager._gripper_ctrl = SimpleNamespace(poll_feedback_once=lambda: None)
    manager._gripper_pos = state.pos
    manager._gripper_vel = state.vel
    manager._gripper_torque = state.torq
    manager._gripper_target_angle = goal
    manager._gripper_goal_angle = goal
    manager._gripper_target_effort = float(effort)
    manager._gripper_close_force = 0.4
    manager._gripper_hold_force = 0.4
    manager._gripper_hold_angle = state.pos
    manager._gripper_hold_deadline = None
    manager._gripper_hold_release_reason = None
    manager._gripper_mode = mode
    manager._gripper_active = True
    observed_at = time.monotonic()
    with manager._feedback_lock:
        manager._record_verified_feedback(
            "gripper",
            state,
            1,
            observed_at,
        )
    manager._gripper_position_result = "active"
    return commands


def _configure_arrived_gripper_position_move(manager: HardwareManager) -> list[tuple[float, ...]]:
    return _configure_gripper_position_move(manager)


def test_completed_gripper_position_move_neutralizes_and_goes_idle() -> None:
    manager = make_manager()
    commands = _configure_arrived_gripper_position_move(manager)

    manager._gripper_tick()
    assert manager.wait_gripper_target(timeout=0.01) is True

    # Neutral MIT: zero stiffness, zero damping, zero feed-forward torque.
    assert commands == [(-4.72, 0.0, 0.0, 0.0, 0.0)]
    assert manager.gripper_active is False
    assert manager.gripper_mode == "idle"

    # No further position command may be emitted once the move is released.
    manager._gripper_tick()
    assert commands == [(-4.72, 0.0, 0.0, 0.0, 0.0)]


def test_waiter_accepts_in_tolerance_feedback_without_rewriting_goal() -> None:
    manager = make_manager()
    commands = _configure_gripper_position_move(
        manager,
        position=-4.328030586242676,
        goal=-4.444444444444445,
    )
    result: list[bool] = []
    waiter = threading.Thread(
        target=lambda: result.append(manager.wait_gripper_target(timeout=0.5))
    )

    waiter.start()
    deadline = time.monotonic() + 0.2
    while manager._gripper_neutral_pending is None and time.monotonic() < deadline:
        time.sleep(0.001)
    assert manager._gripper_neutral_pending is not None
    manager._gripper_tick()
    waiter.join(timeout=0.5)

    assert result == [True]
    assert manager._gripper_goal_angle == pytest.approx(-4.444444444444445)
    assert commands == [(-4.328030586242676, 0.0, 0.0, 0.0, 0.0)]


def test_waiter_queues_release_but_only_hardware_loop_emits_neutral() -> None:
    """A ROS service/action waiter must never become a second bus writer."""
    manager = make_manager()
    commands = _configure_arrived_gripper_position_move(manager)
    manager.arm.control_loop_active = True
    manager._gripper_neutral_pending = None

    assert manager.wait_gripper_target(timeout=0.01) is False
    assert commands == []
    assert manager._gripper_neutral_pending is not None

    manager._gripper_tick()

    assert commands == [(-4.72, 0.0, 0.0, 0.0, 0.0)]
    assert manager.gripper_reached_target() is True


def test_gripper_tick_neutralizes_arrived_position_move_without_waiter() -> None:
    manager = make_manager()
    commands = _configure_arrived_gripper_position_move(manager)

    manager._gripper_tick()

    assert commands == [(-4.72, 0.0, 0.0, 0.0, 0.0)]
    assert manager.gripper_active is False
    assert manager.gripper_mode == "idle"

    manager._gripper_tick()
    assert commands == [(-4.72, 0.0, 0.0, 0.0, 0.0)]


def test_timed_out_gripper_position_move_neutralizes_and_goes_idle() -> None:
    manager = make_manager()
    # Feedback stays far from the goal, so the arrival tolerance is never met.
    commands = _configure_gripper_position_move(manager, position=-0.5, goal=-4.72)

    assert manager.wait_gripper_target(timeout=0.05) is False

    assert commands == []
    manager._gripper_tick()
    assert commands[-1] == (-0.5, 0.0, 0.0, 0.0, 0.0)
    assert manager.gripper_active is False
    assert manager.gripper_mode == "idle"

    before = len(commands)
    manager._gripper_tick()
    assert len(commands) == before


def test_gripper_position_move_applies_configured_torque_cap() -> None:
    # A configured cap below _G_TAU_MAX must still bound the move-phase torque.
    manager = make_manager(gripper_position_torque_cap_nm=0.40)
    commands = _configure_gripper_position_move(
        manager, position=-0.5, goal=-4.72, effort=0.40
    )

    manager._gripper_tick()

    assert len(commands) == 1
    pos, vel, kp, kd, tau_safe = commands[0]
    assert (pos, vel, kp, kd) == (-4.72, 0.0, 5.0, 1.0)
    pos_term = kp * (pos - (-0.5)) + kd * 0.0
    # _gripper_safe_mit sends tau_safe such that pos_term + tau_safe lands on
    # the effective limit; assert that limit is the configured cap.
    assert pos_term + tau_safe == pytest.approx(-0.40)


def test_gripper_position_target_advances_by_configured_speed_ramp(monkeypatch) -> None:
    manager = make_manager(gripper_position_torque_cap_nm=1.0)
    manager._connected = True
    manager._enabled = True
    manager.arm.control_loop_active = True
    commands = _configure_gripper_position_move(
        manager,
        position=-1.0,
        goal=-1.0,
        effort=1.0,
    )
    manager._start_gripper_loop = lambda: None

    now = [10.0]
    monkeypatch.setattr(hardware_manager_module.time, "monotonic", lambda: now[0])
    manager.set_gripper_target(0.08, max_effort=1.0)

    # 0.5 rad/s is the approved teleop-aligned limit.  After 0.2 s the
    # commanded target may advance by at most 0.1 rad, never jump to the final
    # approximately -4.44 rad goal in one control tick.
    now[0] = 10.2
    manager._gripper_tick()

    assert commands[0][0] == pytest.approx(-1.1)
    assert manager._gripper_goal_angle == pytest.approx((0.08 / _G_MAX_DIST_M) * _G_ANGLE_OPEN)


def test_gripper_position_timeout_scales_with_command_distance() -> None:
    manager = make_manager(gripper_position_max_speed_rad_s=0.5)
    manager._connected = True
    manager._enabled = True
    manager.arm.control_loop_active = True
    _configure_gripper_position_move(manager, position=-1.0, goal=-1.0)
    manager._start_gripper_loop = lambda: None

    manager.set_gripper_target(0.08, max_effort=1.0)

    goal = (0.08 / _G_MAX_DIST_M) * _G_ANGLE_OPEN
    assert manager.gripper_target_timeout_sec() == pytest.approx(
        abs(goal - (-1.0)) / 0.5 + 1.5
    )


def test_gripper_position_command_logs_command_feedback_and_timing(monkeypatch) -> None:
    manager = make_manager(gripper_position_torque_cap_nm=1.0)
    manager._connected = True
    manager._enabled = True
    manager.arm.control_loop_active = True
    _configure_gripper_position_move(manager, position=-1.0, goal=-1.0)
    manager._start_gripper_loop = lambda: None

    messages: list[str] = []
    monkeypatch.setattr(
        hardware_manager_module._LOG,
        "info",
        lambda template, *args: messages.append(template % args),
    )
    manager.set_gripper_target(0.08, max_effort=1.0)

    message = "\n".join(messages)
    for field in (
        "requested=0.080000m",
        "clamped=0.080000m",
        "start=-1.000000rad",
        "goal=-4.444444rad",
        "effort=1.000000Nm",
        "speed=0.500000rad/s",
        "timeout=",
        "feedback_updated=",
    ):
        assert field in message


def test_gripper_feedback_exception_stops_active_position_command() -> None:
    manager = make_manager()
    commands = _configure_gripper_position_move(
        manager,
        position=-0.5,
        goal=-4.0,
        effort=1.0,
    )

    with manager._feedback_lock:
        manager._record_feedback_label_error("gripper", "feedback read failed")
        manager._sync_feedback_health()
    manager._gripper_tick()

    assert manager.gripper_active is False
    assert manager.gripper_mode == "idle"
    assert "feedback read failed" in manager.gripper_command_error
    # The only allowable command after feedback loss is zero-gain/zero-torque
    # neutral at the last trusted position; no Kp=5 target may be emitted.
    manager._gripper_tick()
    assert commands == [(-0.5, 0.0, 0.0, 0.0, 0.0)]


def test_gripper_non_enabled_status_stops_before_next_target() -> None:
    manager = make_manager()
    commands = _configure_gripper_position_move(
        manager,
        position=-0.5,
        goal=-4.0,
        effort=1.0,
    )
    with manager._feedback_lock:
        previous = manager._verified_feedback_by_label["gripper"]
        manager._record_verified_feedback(
            "gripper",
            SimpleNamespace(pos=-0.5, vel=0.0, torq=0.0, status_code=0),
            previous.sequence + 1,
            time.monotonic(),
        )
    forbid_raw_feedback_reads(manager)

    manager._gripper_tick()

    assert commands == []
    assert "status_code=0" in manager.gripper_command_error
    manager._gripper_tick()
    assert commands == [(-0.5, 0.0, 0.0, 0.0, 0.0)]


def test_stale_gripper_feedback_stops_ramp_before_next_target(monkeypatch) -> None:
    manager = make_manager()
    commands = _configure_gripper_position_move(
        manager,
        position=-0.5,
        goal=-4.0,
        effort=1.0,
    )
    manager._gripper_feedback_updated_monotonic = 10.0
    manager._gripper_feedback_error = None
    manager._gripper_last_tick_monotonic = 10.9
    monkeypatch.setattr(hardware_manager_module.time, "monotonic", lambda: 11.0)

    manager._gripper_tick()

    assert manager.gripper_active is False
    assert manager.gripper_mode == "idle"
    assert "stale" in manager.gripper_command_error
    assert commands == [(-0.5, 0.0, 0.0, 0.0, 0.0)]


def test_grasp_rejects_stale_feedback_before_sending_close_command() -> None:
    manager = make_manager()
    manager._connected = True
    manager._enabled = True
    manager.arm.control_loop_active = True
    commands = _configure_gripper_position_move(
        manager,
        position=-1.0,
        goal=-1.0,
        mode="idle",
    )
    manager._gripper_active = False
    manager._gripper_feedback_updated_monotonic = time.monotonic() - 1.0

    with pytest.raises(RuntimeError, match="feedback stale"):
        manager.grasp_gripper(close_timeout_sec=0.1)

    assert commands == []


def test_get_gripper_state_reports_unknown_status_without_verified_sample() -> None:
    manager = make_manager()
    manager._gripper_mot = FakeMotor()

    position, velocity, torque, status = manager.get_gripper_state()

    assert (position, velocity, torque) == (0.0, 0.0, 0.0)
    assert status == 255
    assert "verified feedback unavailable" in manager.gripper_feedback_error


def test_get_gripper_state_reports_unknown_status_when_feedback_is_stale(monkeypatch) -> None:
    manager = make_manager()
    manager._gripper_mot = FakeMotor(position=-1.0)
    manager._gripper_mot.state.status_code = 1
    before = seed_verified_feedback(
        manager,
        observed_at=10.0,
        gripper_state=manager._gripper_mot.state,
    )
    monkeypatch.setattr(hardware_manager_module.time, "monotonic", lambda: 11.0)

    position, velocity, torque, status = manager.get_gripper_state()

    assert (position, velocity, torque) == (-1.0, 0.0, 0.0)
    assert status == 255
    assert "stale" in manager.gripper_feedback_error
    assert_verified_feedback_unchanged(manager, before)


def test_get_gripper_state_reports_unknown_status_after_shared_refresh_error() -> None:
    manager = make_manager()
    manager._gripper_mot = FakeMotor(position=-1.0)
    manager._gripper_mot.state.status_code = 1
    before = seed_verified_feedback(
        manager,
        gripper_state=manager._gripper_mot.state,
    )
    manager._gripper_feedback_error = "shared feedback batch failed: serial timeout"

    position, velocity, torque, status = manager.get_gripper_state()

    assert (position, velocity, torque) == (-1.0, 0.0, 0.0)
    assert status == 255
    assert manager.gripper_feedback_error == "shared feedback batch failed: serial timeout"
    assert_verified_feedback_unchanged(manager, before)


def test_joint_statuses_report_unknown_after_shared_refresh_error() -> None:
    manager = make_manager()
    manager._arm_feedback_error = "shared feedback batch failed: serial timeout"

    assert manager.get_joint_status_codes() == [255] * 6


def test_missing_verified_joint_status_is_not_reported_as_disabled() -> None:
    manager = make_manager()
    seed_verified_feedback(manager)
    with manager._feedback_lock:
        manager._verified_feedback_by_label.pop("joint3")

    assert manager.get_joint_status_codes() == [0, 0, 255, 0, 0, 0]


def test_feedback_failures_are_exposed_in_arm_status_error_codes() -> None:
    manager = make_manager()
    manager._gripper_mot = FakeMotor(position=-1.0)
    manager._gripper_feedback_updated_monotonic = time.monotonic()
    manager._arm_feedback_error = "shared feedback batch failed: serial timeout"
    manager._gripper_feedback_error = "shared feedback batch failed: serial timeout"

    assert manager.error_codes == [
        "ARM_FEEDBACK: arm feedback unavailable: shared feedback batch failed: serial timeout",
        "GRIPPER_FEEDBACK: gripper feedback unavailable: shared feedback batch failed: serial timeout",
    ]


def test_wait_gripper_target_rejects_stale_arrival(monkeypatch) -> None:
    manager = make_manager()
    commands = _configure_gripper_position_move(
        manager,
        position=-4.0,
        goal=-4.0,
    )
    manager._gripper_target_deadline_monotonic = 12.0
    manager._gripper_feedback_updated_monotonic = 10.0
    monkeypatch.setattr(hardware_manager_module.time, "monotonic", lambda: 11.0)

    assert manager.wait_gripper_target() is False
    assert manager.gripper_reached_target() is False
    assert "stale" in manager.gripper_command_error
    manager._gripper_tick()
    assert commands == [(-4.0, 0.0, 0.0, 0.0, 0.0)]


def test_gripper_arrival_is_not_success_when_neutral_command_fails() -> None:
    manager = make_manager()
    commands = _configure_arrived_gripper_position_move(manager)
    original_send = manager._gripper_mot.send_mit

    def fail_neutral(pos, vel, kp, kd, tau) -> None:
        if kp == 0.0 and kd == 0.0 and tau == 0.0:
            raise RuntimeError("neutral command failed")
        original_send(pos, vel, kp, kd, tau)

    manager._gripper_mot.send_mit = fail_neutral

    manager._gripper_tick()

    assert manager.gripper_reached_target() is False
    assert "neutral command failed" in manager.gripper_command_error
    assert commands == []


def test_gripper_position_move_default_cap_matches_web_teleop() -> None:
    # Operator decision 2026-09-04: use the known-good Web teleop default for
    # normal position moves while retaining 1.5 N.m as the configurable ceiling.
    manager = make_manager()
    commands = _configure_gripper_position_move(
        manager, position=-0.5, goal=-4.72, effort=_G_LARGE_MOVE_MAX_TAU_NM
    )

    manager._gripper_tick()

    assert len(commands) == 1
    pos, vel, kp, kd, tau_safe = commands[0]
    pos_term = kp * (pos - (-0.5)) + kd * 0.0
    assert pos_term + tau_safe == pytest.approx(-1.0)


@pytest.mark.parametrize("bad_cap", [0.04, _G_TAU_MAX + 0.01])
def test_gripper_position_torque_cap_rejects_out_of_range(bad_cap: float) -> None:
    # Exercise the real constructor validation, not the test helper's mirror.
    # It runs before any hardware setup, so no serial channel is opened.
    with pytest.raises(ValueError, match="gripper_position_torque_cap_nm"):
        HardwareManager(gripper_position_torque_cap_nm=bad_cap)


def test_gripper_position_torque_cap_accepts_hard_max() -> None:
    assert _G_LARGE_MOVE_MAX_TAU_CAP_NM == _G_TAU_MAX
    manager = make_manager(gripper_position_torque_cap_nm=_G_TAU_MAX)
    assert manager._gripper_position_torque_cap_nm == pytest.approx(_G_TAU_MAX)


@pytest.mark.parametrize("bad_rate", [19.9, 100.1])
def test_hardware_feedback_rate_rejects_out_of_range(bad_rate: float) -> None:
    with pytest.raises(ValueError, match="hardware_feedback_rate_hz"):
        HardwareManager(hardware_feedback_rate_hz=bad_rate)


def test_gripper_tick_emits_no_command_once_arrival_is_detected() -> None:
    # Regression for the observed ~1.2-1.33 mm of extra closing travel after
    # SetGripper returned: a tick that saw the arrival condition must not also
    # emit a torque-carrying command.  Closing to zero is the exposed direction
    # because abs(target) < 1e-6 turns effort into feed-forward torque.
    manager = make_manager()
    commands = _configure_gripper_position_move(manager, position=0.0, goal=0.0)

    manager._gripper_tick()

    # Only the neutral release, never a kp=5 command carrying tau_ff.
    assert commands == [(0.0, 0.0, 0.0, 0.0, 0.0)]
    assert manager.gripper_mode == "idle"


def test_grasp_hold_is_released_after_timeout() -> None:
    manager = make_manager()
    commands = _configure_gripper_position_move(manager, mode="grasp_holding")
    manager._gripper_hold_deadline = time.monotonic() + 60.0

    manager._gripper_tick()
    assert manager.gripper_mode == "grasp_holding"
    assert commands[-1][:4] == (-4.72, 0.0, 5.0, 1.0)

    # Deadline reached: neutralize and stop loading the motor.
    manager._gripper_hold_deadline = time.monotonic() - 0.01
    manager._gripper_tick()

    assert manager.gripper_active is False
    assert manager.gripper_mode == "idle"
    assert commands[-1] == (-4.72, 0.0, 0.0, 0.0, 0.0)
    assert manager.grasp_hold_release_reason == "hold timeout"

    before = len(commands)
    manager._gripper_tick()
    assert len(commands) == before


def test_grasp_hold_without_deadline_is_not_released() -> None:
    manager = make_manager()
    _configure_gripper_position_move(manager, mode="grasp_holding")
    manager._gripper_hold_deadline = None

    manager._gripper_tick()

    assert manager.gripper_mode == "grasp_holding"


def test_release_grasp_hold_neutralizes_on_request() -> None:
    manager = make_manager()
    commands = _configure_gripper_position_move(manager, mode="grasp_holding")
    manager._gripper_hold_deadline = time.monotonic() + 60.0

    assert manager.release_grasp_hold("external release") is True

    assert commands == []
    assert manager.gripper_active is False
    assert manager.gripper_mode == "idle"
    assert manager.grasp_hold_release_reason == "external release"

    manager._gripper_tick()
    assert commands == [(-4.72, 0.0, 0.0, 0.0, 0.0)]

    # Idempotent: nothing active to release.
    assert manager.release_grasp_hold() is False


def test_release_grasp_hold_ignores_normal_position_move() -> None:
    manager = make_manager()
    _configure_gripper_position_move(manager, mode="position")

    assert manager.release_grasp_hold() is False
    assert manager.gripper_mode == "position"


def test_grasp_hold_timeout_rejects_out_of_range() -> None:
    for bad in (0.05, _G_GRASP_HOLD_TIMEOUT_MAX_SEC + 1.0):
        with pytest.raises(ValueError, match="grasp_hold_timeout_sec"):
            HardwareManager(grasp_hold_timeout_sec=bad)


def test_grasp_modes_are_not_released_by_position_neutralization() -> None:
    for mode, expected in (
        ("grasp_closing", (0.0, 0.0, 0.0, 0.5)),
        ("grasp_holding", (-4.72, 0.0, 5.0, 1.0)),
    ):
        manager = make_manager()
        commands = _configure_gripper_position_move(manager, mode=mode)

        manager._gripper_tick()
        manager._gripper_tick()

        assert manager.gripper_active is True
        assert manager.gripper_mode == mode
        assert len(commands) == 2
        for command in commands:
            assert command[:4] == expected


def test_gripper_position_target_cannot_exceed_verified_85mm_limit() -> None:
    manager = make_manager()
    manager._connected = True
    manager._enabled = True
    manager.arm.control_loop_active = True
    manager._gripper_pos = 0.0
    manager._gripper_mot = SimpleNamespace(
        get_state=lambda: SimpleNamespace(pos=0.0, vel=0.0, torq=0.0, status_code=1),
        request_feedback=lambda: None,
    )
    manager._gripper_ctrl = SimpleNamespace(poll_feedback_once=lambda: None)
    manager._start_gripper_loop = lambda: None
    seed_verified_feedback(
        manager,
        arm_status=1,
        gripper_state=SimpleNamespace(pos=0.0, vel=0.0, torq=0.0, status_code=1),
    )

    manager.set_gripper_target(0.09, max_effort=0.15)

    assert manager._gripper_goal_angle == pytest.approx(
        (_G_VERIFIED_OPEN_LIMIT_M / _G_MAX_DIST_M) * _G_ANGLE_OPEN
    )


def test_hardware_soft_limits_match_moveit_urdf() -> None:
    root = Path(__file__).resolve().parents[1]
    urdf_root = ElementTree.parse(
        root / "src/rebotarm_moveit_config/config/rebotarm.urdf"
    ).getroot()
    urdf_limits = {
        joint.attrib["name"]: (
            float(joint.find("limit").attrib["lower"]),
            float(joint.find("limit").attrib["upper"]),
        )
        for joint in urdf_root.findall("joint")
        if joint.attrib.get("name") in _JOINT_POSITION_LIMITS_RAD
    }

    assert urdf_limits == _JOINT_POSITION_LIMITS_RAD


def test_gripper_initialization_defers_background_loop_until_first_command(
    monkeypatch,
) -> None:
    HardwareManager._ensure_rebot_sdk_in_syspath()
    from reBotArm_control_py.actuator import gripper as gripper_module

    manager = make_manager()
    del manager.init_gripper
    gripper_motor = FakeMotor()
    controller = SimpleNamespace(
        add_damiao_motor=lambda _motor_id, _feedback_id, _model: gripper_motor,
        poll_feedback_once=lambda: None,
    )
    manager.arm._ctrl_map["damiao"] = controller
    monkeypatch.setattr(
        gripper_module,
        "load_cfg",
        lambda _path: {
            "gripper": SimpleNamespace(
                vendor="damiao",
                motor_id=0x07,
                feedback_id=0x17,
                model="4310",
            )
        },
    )
    manager._gripper_loop_running = False
    manager._start_gripper_loop = lambda: setattr(
        manager, "_gripper_loop_running", True
    )

    manager.init_gripper("unused-gripper.yaml")

    assert manager._gripper_mot is gripper_motor
    assert manager._gripper_loop_running is False


def test_set_zero_gripper_only_calibrates_disabled_gripper_motor() -> None:
    manager = make_manager()
    gripper = FakeMotor(position=0.0)
    manager._gripper_mot = gripper
    manager._gripper_ctrl = FakeController([gripper])
    manager.connect()

    assert manager.set_zero("gripper") is True
    assert gripper.set_zero_position_calls == 1
    assert manager.enabled is False
    assert manager.lifecycle_state == "CONNECTED_DISABLED"
    assert manager.arm.disable_calls == 0


def test_set_zero_gripper_rejects_enabled_arm() -> None:
    manager = make_manager()
    manager._gripper_mot = FakeMotor(position=0.0)
    manager._gripper_ctrl = FakeController([manager._gripper_mot])
    manager.connect()
    manager._enabled = True

    with pytest.raises(RuntimeError, match="CONNECTED_DISABLED"):
        manager.set_zero("gripper")


@pytest.mark.parametrize("position", [2 * np.pi, 0.001 * 5 / 0.09 + 1e-6, 0.074, -5.02])
def test_invalid_gripper_coordinate_preserves_raw_but_reports_unknown(position) -> None:
    manager = make_manager()
    gripper = FakeMotor(position=position)
    manager._gripper_mot = gripper
    manager._gripper_ctrl = FakeController([gripper])
    manager.connect()

    assert manager.get_gripper_state()[0] == pytest.approx(position)
    assert manager.get_gripper_state()[3] == 255
    assert np.isnan(manager.gripper_position_m())
    assert any("coordinate invalid" in reason for reason in manager.error_codes)
    assert manager.arm.disable_calls == 0
    assert gripper.set_zero_position_calls == 0
    with pytest.raises(RuntimeError, match="coordinate invalid"):
        manager.enable()
    assert manager.arm.enable_calls == 0
    assert manager.arm.mode_pos_vel_calls == 0


@pytest.mark.parametrize(
    "position,width",
    [(0.000190735, 0.0), (0.00743866, 0.0), (0.01, 0.0),
     (0.01583099365234375, 0.0), (0.001 * 5 / 0.09, 0.0),
     (-0.000190735, 0.00000343323),
     (-5.000190735, 0.09), (-4.8, 0.0864)],
)
def test_gripper_width_allows_closed_tolerance_and_open_quantization(position, width) -> None:
    manager = make_manager()
    manager._gripper_pos = position
    assert manager.gripper_position_m() == pytest.approx(width)


@pytest.mark.parametrize("mode", ["position", "grasp_closing", "grasp_holding"])
def test_coordinate_jump_stops_gripper_without_disabling_healthy_arm(mode) -> None:
    manager = make_manager()
    manager.connect()
    manager.enable()
    commands = _configure_gripper_position_move(manager, position=-1.0, goal=-2.0, mode=mode)
    state = SimpleNamespace(pos=2 * np.pi, vel=0.0, torq=0.0, status_code=1)
    seed_verified_feedback(manager, arm_status=1, gripper_state=state)
    manager._feedback_next_refresh_monotonic = time.monotonic() + 1.0
    arm_ticks = []
    manager._gripper_cfg = SimpleNamespace()

    manager._hardware_control_tick(manager.arm, 0.002, lambda *_: arm_ticks.append(True))

    assert arm_ticks == [True]
    assert manager.enabled is True
    assert manager.control_loop_active is True
    assert manager.arm.disable_calls == 0
    assert manager.gripper_active is False
    assert manager.gripper_mode == "idle"
    assert manager.gripper_reached_target() is False
    assert commands == [(0.0, 0.0, 0.0, 0.0, 0.0)]
    for command in [
        lambda: manager.set_gripper_target(0.01),
        manager.grasp_gripper,
        lambda: manager.send_gripper_motor_cmd(SimpleNamespace()),
    ]:
        with pytest.raises(RuntimeError, match="coordinate invalid"):
            command()


def _manager_for_zero_verification(position=0.0):
    manager = make_manager()
    gripper = FakeMotor(position=position)
    manager._gripper_mot = gripper
    manager._gripper_ctrl = FakeController([gripper])
    manager.connect()
    return manager, gripper


def test_zero_verifies_new_frames_and_repairs_invalid_coordinate() -> None:
    manager, gripper = _manager_for_zero_verification(2 * np.pi)

    def zero():
        gripper.set_zero_position_calls += 1
        gripper.state = SimpleNamespace(pos=-0.000190735, vel=0.0, torq=0.0, status_code=0)

    gripper.set_zero_position = zero
    before = gripper.feedback_sequence
    assert manager.set_zero("gripper") is True
    assert gripper.set_zero_position_calls == 1
    assert gripper.feedback_sequence >= before + 4  # preflight plus three post-zero frames
    assert manager.get_gripper_state()[3] == 0
    assert manager.gripper_position_m() == pytest.approx(0.00000343323)
    assert manager.arm.enable_calls == manager.arm.disable_calls == 0


def test_zero_rejects_old_zero_cache_and_retains_failure_until_verified() -> None:
    manager, gripper = _manager_for_zero_verification()

    def no_new_frames():
        gripper.set_zero_position_calls += 1
        gripper.advance_feedback_on_poll = False

    gripper.set_zero_position = no_new_frames
    with pytest.raises(RuntimeError, match="set_zero failed"):
        manager.set_zero("gripper")
    assert gripper.set_zero_position_calls == 1
    assert np.isnan(manager.gripper_position_m())
    gripper.advance_feedback_on_poll = True
    manager.refresh_feedback_if_due(force=True)
    assert np.isnan(manager.gripper_position_m())  # acquisition alone is not calibration
    assert manager.get_gripper_state()[3] == 255
    gripper.set_zero_position = lambda: None
    assert manager.set_zero("gripper") is True
    assert manager.gripper_position_m() == 0.0


@pytest.mark.parametrize("result", ["nonzero", "closed_tolerance", "enabled", "nonfinite", "write_error"])
def test_zero_failure_never_returns_success_or_retries_write(result, monkeypatch) -> None:
    manager, gripper = _manager_for_zero_verification(-1.0)
    monkeypatch.setattr(hardware_manager_module, "_G_ZERO_VERIFY_TIMEOUT_SEC", 0.03)

    def zero():
        gripper.set_zero_position_calls += 1
        if result == "enabled":
            gripper.state.status_code = 1
        elif result == "closed_tolerance":
            gripper.state.pos = 0.00743866
        elif result == "nonfinite":
            gripper.state.pos = float("nan")
        elif result == "write_error":
            raise RuntimeError("serial write failed")

    gripper.set_zero_position = zero
    with pytest.raises(RuntimeError, match="set_zero failed"):
        manager.set_zero("gripper")
    assert gripper.set_zero_position_calls == 1
    assert np.isnan(manager.gripper_position_m())
    assert manager.arm.enable_calls == manager.arm.disable_calls == 0


def test_zero_requires_consecutive_near_zero_samples(monkeypatch) -> None:
    manager, gripper = _manager_for_zero_verification(-1.0)
    monkeypatch.setattr(hardware_manager_module, "_G_ZERO_VERIFY_TIMEOUT_SEC", 0.04)
    deliver = gripper.deliver_requested_feedback
    samples = iter([0.0, 0.0, -1.0])

    def intermittent_zero():
        gripper.state.pos = next(samples, -1.0)
        deliver()

    def zero():
        gripper.set_zero_position_calls += 1
        gripper.deliver_requested_feedback = intermittent_zero

    gripper.set_zero_position = zero
    with pytest.raises(RuntimeError, match="zero verification timed out"):
        manager.set_zero("gripper")
    assert gripper.set_zero_position_calls == 1
    assert np.isnan(manager.gripper_position_m())


def test_invalid_coordinate_publishes_unknown_without_losing_arm_feedback() -> None:
    from builtin_interfaces.msg import Time
    from rebotarmcontroller.ros_publishers import JointStatePublisher
    from rebotarm_dashboard.status_panel_state import TeleopStatusStore

    manager, _gripper = _manager_for_zero_verification(2 * np.pi)
    messages = {}

    def publisher(key):
        return SimpleNamespace(publish=lambda msg: messages.setdefault(key, []).append(msg))

    node = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=Time)),
    )
    output = JointStatePublisher.__new__(JointStatePublisher)
    output._publish_lock = threading.Lock()
    output._status_lock = threading.Lock()
    output._last_status_identity = None
    output._last_status_published_at = None
    output._status_heartbeat_sec = 0.2
    output._last_feedback_identity = None
    output._node = node
    output._hardware = manager
    output._publisher = publisher("joints")
    output._status_publisher = publisher("status")
    output._joint_state_publishers = {name: publisher(name) for name in JOINT_NAMES}
    output._gripper_state_publisher = publisher("gripper")

    output.publish()
    output.publish_status()

    gripper = messages["gripper"][-1]
    assert np.isnan(gripper.position)
    assert gripper.status_code == 255
    assert len(messages["joints"][-1].position) == 6
    assert list(messages["status"][-1].per_joint_status_code) == [0] * 6
    assert any("coordinate invalid" in reason for reason in messages["status"][-1].error_codes)
    store = TeleopStatusStore()
    store.update_motor_state(
        joint_name=gripper.joint_name, position=gripper.position,
        velocity=gripper.velocity, torque=gripper.torque, status_code=gripper.status_code,
    )
    assert store.snapshot_dict()["joints"]["gripper"]["position"] is None

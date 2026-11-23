"""终端键盘力控分模式路由,单关节阻抗调参草稿与参数隔离单元测试."""
from __future__ import annotations

import time
from types import SimpleNamespace

import pytest

from rebotarm_teleop.keyboard_force_tuning import (
    ImpedanceTuningDraft,
    KeyboardImpedanceProfile,
    KeyboardModeRouter,
    PROTOCOL_KD_MAX,
    PROTOCOL_KP_MAX,
    apply_draft_tuning,
    format_telemetry,
)
from rebotarm_teleop.teleop_keyboard_node import TeleopKeyboardNode


JOINTS = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
LIMITS = {name: (-2.8, 2.8) for name in JOINTS}
SIM_KP = (45.0, 45.0, 45.0, 17.5, 17.5, 17.5)
SIM_KD = (3.32, 5.0, 5.0, 1.28, 0.78, 0.34)


def _sim_profile() -> KeyboardImpedanceProfile:
    profile = KeyboardImpedanceProfile.from_parameters(
        use_hardware=False,
        profile_name="simulation",
        initial_stiffness=SIM_KP,
        initial_damping=SIM_KD,
        max_stiffness=(PROTOCOL_KP_MAX,) * 6,
        max_damping=(PROTOCOL_KD_MAX,) * 6,
        kp_step=2.0,
        kd_step=0.1,
        q_step_rad=0.02,
    )
    assert profile is not None
    return profile


def test_impedance_profile_enforces_sim_hardware_isolation_and_no_implicit_defaults() -> None:
    assert (
        KeyboardImpedanceProfile.from_parameters(
            use_hardware=True,
            profile_name="",
            initial_stiffness=(),
            initial_damping=(),
            max_stiffness=(),
            max_damping=(),
            kp_step=0.0,
            kd_step=0.0,
            q_step_rad=0.0,
        )
        is None
    )

    with pytest.raises(ValueError, match="mismatch"):
        KeyboardImpedanceProfile.from_parameters(
            use_hardware=True,
            profile_name="simulation",
            initial_stiffness=SIM_KP,
            initial_damping=SIM_KD,
            max_stiffness=(500.0,) * 6,
            max_damping=(5.0,) * 6,
            kp_step=2.0,
            kd_step=0.1,
            q_step_rad=0.02,
        )

    with pytest.raises(ValueError, match="mismatch"):
        KeyboardImpedanceProfile.from_parameters(
            use_hardware=False,
            profile_name="hardware",
            initial_stiffness=(10.0,) * 6,
            initial_damping=(0.5,) * 6,
            max_stiffness=(500.0,) * 6,
            max_damping=(5.0,) * 6,
            kp_step=1.0,
            kd_step=0.05,
            q_step_rad=0.01,
        )

    with pytest.raises(ValueError, match="500.0"):
        KeyboardImpedanceProfile.from_parameters(
            use_hardware=False,
            profile_name="simulation",
            initial_stiffness=SIM_KP,
            initial_damping=SIM_KD,
            max_stiffness=(501.0,) * 6,
            max_damping=(5.0,) * 6,
            kp_step=2.0,
            kd_step=0.1,
            q_step_rad=0.02,
        )

    with pytest.raises(ValueError, match="5.0"):
        KeyboardImpedanceProfile.from_parameters(
            use_hardware=False,
            profile_name="simulation",
            initial_stiffness=SIM_KP,
            initial_damping=SIM_KD,
            max_stiffness=(500.0,) * 6,
            max_damping=(5.1,) * 6,
            kp_step=2.0,
            kd_step=0.1,
            q_step_rad=0.02,
        )


def test_mode_router_enforces_mode_scoped_keys_and_stop_priority() -> None:
    router = KeyboardModeRouter(joint_names=JOINTS)

    assert router.classify_key("1", state_machine="IDLE", has_impedance_profile=True).kind == "jog_fjt"
    assert router.classify_key("q", state_machine="IDLE", has_impedance_profile=True).kind == "jog_fjt"
    assert router.classify_key("g", state_machine="IDLE", has_impedance_profile=True).kind == "start_gravity_comp"
    assert router.classify_key("i", state_machine="IDLE", has_impedance_profile=True).kind == "start_impedance"
    assert router.classify_key("i", state_machine="IDLE", has_impedance_profile=False).kind == "reject"
    assert router.classify_key("[", state_machine="IDLE", has_impedance_profile=True).kind == "reject"

    # GRAVITY_COMP 下禁止 FJT,调参与直跳阻抗;允许 h/空格 和 p.
    assert router.classify_key("1", state_machine="GRAVITY_COMP", has_impedance_profile=True).kind == "reject"
    assert router.classify_key("i", state_machine="GRAVITY_COMP", has_impedance_profile=True).kind == "reject"
    assert router.classify_key("h", state_machine="GRAVITY_COMP", has_impedance_profile=True).kind == "stop"
    assert router.classify_key("p", state_machine="GRAVITY_COMP", has_impedance_profile=True).kind == "print_telemetry"

    # JOINT_IMPEDANCE 下 1..6 仅选轴,q..y 禁止,[]/,. /-=/0/l 调参,g 直跳禁止.
    sel = router.classify_key("6", state_machine="JOINT_IMPEDANCE", has_impedance_profile=True)
    assert sel.kind == "select_joint" and sel.joint_index == 5
    assert router.classify_key("q", state_machine="JOINT_IMPEDANCE", has_impedance_profile=True).kind == "reject"
    assert router.classify_key("g", state_machine="JOINT_IMPEDANCE", has_impedance_profile=True).kind == "reject"
    assert router.classify_key("i", state_machine="JOINT_IMPEDANCE", has_impedance_profile=True).kind == "reject"
    for k, op, s in (("[", "kp", -1.0), ("]", "kp", 1.0), (",", "kd", -1.0), (".", "kd", 1.0),
                     ("-", "q_ref", -1.0), ("=", "q_ref", 1.0), ("0", "align", 0.0), ("l", "align", 0.0)):
        act = router.classify_key(k, state_machine="JOINT_IMPEDANCE", has_impedance_profile=True)
        assert act.kind == "tune_impedance" and act.tune_op == op and act.sign == s

    # 未收到 arm_status 时拦截运动与力控切模式,放行急停和遥测打印.
    assert router.classify_key("1", state_machine=None, has_impedance_profile=True).kind == "reject"
    assert router.classify_key("h", state_machine=None, has_impedance_profile=True).kind == "stop"
    assert router.classify_key("p", state_machine=None, has_impedance_profile=True).kind == "print_telemetry"


def test_draft_tuning_is_immutable_clamped_and_formats_telemetry() -> None:
    profile = _sim_profile()
    live = {name: 0.1 * i for i, name in enumerate(JOINTS)}
    draft = ImpedanceTuningDraft.create_initial(
        joint_names=JOINTS,
        current_positions=live,
        joint_limits=LIMITS,
        profile=profile,
    )
    selected = draft.select_joint(5)
    assert draft.selected_index == 0 and selected.selected_index == 5

    kp_up, msg = apply_draft_tuning(
        selected,
        tune_op="kp",
        sign=1.0,
        profile=profile,
        current_positions=live,
        joint_limits=LIMITS,
    )
    assert kp_up.stiffness[5] == pytest.approx(19.5) and selected.stiffness[5] == pytest.approx(17.5)
    assert "joint6 kp -> 19.50" in msg
    assert "Kp=19.50" in msg and "Kd=0.34" in msg

    kd_maxed, msg = apply_draft_tuning(
        kp_up.select_joint(1),
        tune_op="kd",
        sign=1.0,
        profile=profile,
        current_positions=live,
        joint_limits=LIMITS,
    )
    assert kd_maxed.damping[1] == pytest.approx(5.0) and "clamped" in msg
    assert "Kp=45.00" in msg and "Kd=5.00" in msg

    near_limit = kd_maxed.select_joint(0).adjust_position(5.0, joint_limits=LIMITS)[0]
    assert near_limit.positions[0] == pytest.approx(2.8)

    aligned, _ = apply_draft_tuning(
        near_limit,
        tune_op="align",
        sign=0.0,
        profile=profile,
        current_positions=live,
        joint_limits=LIMITS,
    )
    assert aligned.positions == pytest.approx(tuple(live[name] for name in JOINTS))

    text = format_telemetry(
        joint_names=JOINTS,
        state_machine="JOINT_IMPEDANCE",
        arm_mode="mit",
        enabled=True,
        current_positions=live,
        current_velocities={name: 0.0 for name in JOINTS},
        current_efforts={name: 0.5 for name in JOINTS},
        draft=aligned.select_joint(2),
    )
    assert "state_machine=JOINT_IMPEDANCE" in text
    assert "*joint3:" in text


def _fake_keyboard_owner(*, state_machine="IDLE", enabled=True):
    class _DoneFuture:
        def __init__(self, result):
            self._result = result
            self.canceled = False

        def add_done_callback(self, cb):
            cb(self)

        def result(self):
            return self._result

        def cancel(self):
            self.canceled = True

    class _FakeClient:
        def __init__(self, *, ready=True, success=True, message="ok"):
            self.ready = ready
            self.success = success
            self.message = message
            self.requests = []

        def service_is_ready(self):
            return self.ready

        def call_async(self, req):
            self.requests.append(req)
            return _DoneFuture(SimpleNamespace(success=self.success, message=self.message))

    events = []
    keys = []
    owner = SimpleNamespace(
        _joint_names=JOINTS,
        _joint_limits=LIMITS,
        _busy=False,
        _step_generation=0,
        _retime_pending=None,
        _service_pending=None,
        _feedback_at=time.monotonic(),
        _arm_status=SimpleNamespace(state_machine=state_machine, mode="pos_vel", enabled=enabled),
        _arm_status_at=time.monotonic(),
        _active_draft=None,
        _input_timeout=0.5,
        _deadman_required=False,
        _deadman_key="z",
        _deadman_until=0.0,
        _last_input_time=0.0,
        _current_positions={name: 0.0 for name in JOINTS},
        _current_velocities={name: 0.0 for name in JOINTS},
        _current_efforts={name: 0.0 for name in JOINTS},
        _impedance_profile=_sim_profile(),
        _router=KeyboardModeRouter(joint_names=JOINTS),
        _planner=SimpleNamespace(apply_delta=lambda **kw: SimpleNamespace(accepted=True, joint_names=JOINTS, positions=(0.02,) + (0.0,) * 5, message="jog")),
        _stop_client=_FakeClient(message="traj stop"),
        _gravity_start_client=_FakeClient(message="gravity started"),
        _gravity_stop_client=_FakeClient(message="gravity stopped"),
        _impedance_start_client=_FakeClient(message="impedance started"),
        _impedance_stop_client=_FakeClient(message="impedance stopped"),
        _read_key=lambda: keys.pop(0) if keys else None,
        _publish_status=lambda state, message, key=None, log=True: events.append((state, message, key)),
        get_logger=lambda: SimpleNamespace(info=lambda *_args: None),
        _send_target=lambda *args: events.append(("fjt_sent", args)),
    )
    owner._handle_stop = lambda key, sm: TeleopKeyboardNode._handle_stop(owner, key, sm)
    owner._dispatch_stop_service = lambda c, l, k, clear_draft: TeleopKeyboardNode._dispatch_stop_service(owner, c, l, k, clear_draft=clear_draft)
    owner._on_stop_reply = lambda f, l, k, clear_draft: TeleopKeyboardNode._on_stop_reply(owner, f, l, k, clear_draft=clear_draft)
    owner._call_trigger_async = lambda c, l, k: TeleopKeyboardNode._call_trigger_async(owner, c, l, k)
    owner._on_trigger_reply = lambda f, g, d, l, k: TeleopKeyboardNode._on_trigger_reply(owner, f, g, d, l, k)
    owner._call_impedance_async = lambda cand, l, k: TeleopKeyboardNode._call_impedance_async(owner, cand, l, k)
    owner._on_impedance_reply = lambda f, g, d, cand, l, k: TeleopKeyboardNode._on_impedance_reply(owner, f, g, d, cand, l, k)
    owner._print_telemetry = lambda key: TeleopKeyboardNode._print_telemetry(owner, key)
    return owner, keys, events


def test_keyboard_node_rejects_draft_commit_on_service_failure_and_blocks_jog_in_force_modes() -> None:
    owner, keys, events = _fake_keyboard_owner(state_machine="IDLE")
    # Enter JOINT_IMPEDANCE via 'i'.
    keys.append("i")
    TeleopKeyboardNode._poll_key(owner)
    assert len(owner._impedance_start_client.requests) == 1
    assert owner._active_draft is not None
    assert owner._active_draft.stiffness == SIM_KP

    # Backend transitions to JOINT_IMPEDANCE; 'q' must NOT call _send_target.
    owner._arm_status.state_machine = "JOINT_IMPEDANCE"
    keys.append("q")
    TeleopKeyboardNode._poll_key(owner)
    assert not any(ev[0] == "fjt_sent" for ev in events)
    assert events[-1][0] == "rejected"

    # Reject from backend keeps previous draft unmodified.
    owner._impedance_start_client.success = False
    owner._impedance_start_client.message = "out of range"
    keys.extend(["2", "]"])
    TeleopKeyboardNode._poll_key(owner)
    assert owner._active_draft.selected_index == 1
    TeleopKeyboardNode._poll_key(owner)
    assert owner._active_draft.stiffness[1] == pytest.approx(SIM_KP[1])
    assert events[-1] == ("rejected", "out of range", "]")

    # 'h' invokes joint_impedance/stop and clears draft cleanly.
    keys.append("h")
    TeleopKeyboardNode._poll_key(owner)
    assert len(owner._impedance_stop_client.requests) == 1
    assert owner._active_draft is None
    assert events[-1][0] == "stopped"

    # When disabled on hardware, 'g' and 'i' are rejected locally without touching services.
    owner._arm_status.state_machine = "IDLE"
    owner._arm_status.enabled = False
    keys.extend(["g", "i"])
    TeleopKeyboardNode._poll_key(owner)
    TeleopKeyboardNode._poll_key(owner)
    assert owner._gravity_start_client.requests == []
    assert len(owner._impedance_start_client.requests) == 2
    assert events[-1] == ("rejected", "arm disabled; enable arm first", "i")


def test_keyboard_node_logs_info_for_key_events_and_preserves_draft_across_stale_idle() -> None:
    logs = []
    published = []
    owner = SimpleNamespace(
        _joint_names=JOINTS,
        _current_positions={name: 0.0 for name in JOINTS},
        _status_pub=SimpleNamespace(publish=lambda msg: published.append(msg.data)),
        get_logger=lambda: SimpleNamespace(info=lambda text: logs.append(text)),
        _arm_status=SimpleNamespace(state_machine="IDLE", mode="pos_vel", enabled=True),
        _arm_status_at=0.0,
        _last_arm_status_key=("IDLE", "pos_vel", True),
        _active_draft=ImpedanceTuningDraft.create_initial(
            joint_names=JOINTS,
            current_positions={name: 0.0 for name in JOINTS},
            joint_limits=LIMITS,
            profile=_sim_profile(),
        ),
    )

    TeleopKeyboardNode._publish_status(
        owner,
        "active",
        "joint6 kp -> 19.50 [Kp=19.50, Kd=0.34, q_ref=+0.0000 rad]",
        key="]",
    )
    assert any("key=']'" in line and "Kp=19.50" in line and "Kd=0.34" in line for line in logs)

    # timeout 状态只发 JSON, 不刷屏 INFO 日志.
    before_logs = len(logs)
    TeleopKeyboardNode._publish_status(owner, "timeout", "keyboard input timeout")
    assert len(logs) == before_logs

    # 刚进入阻抗收到一条排队中的旧 IDLE 帧时, 不得误清空本地草稿.
    TeleopKeyboardNode._on_arm_status(
        owner, SimpleNamespace(state_machine="IDLE", mode="pos_vel", enabled=True)
    )
    assert owner._active_draft is not None

    # 真正进入 JOINT_IMPEDANCE 再切回 IDLE 时, 应记录状态切换 INFO 并清空草稿.
    TeleopKeyboardNode._on_arm_status(
        owner, SimpleNamespace(state_machine="JOINT_IMPEDANCE", mode="mit", enabled=True)
    )
    assert owner._active_draft is not None
    assert any("state_machine=JOINT_IMPEDANCE" in line for line in logs)
    TeleopKeyboardNode._on_arm_status(
        owner, SimpleNamespace(state_machine="IDLE", mode="pos_vel", enabled=True)
    )
    assert owner._active_draft is None
    assert any("state_machine=IDLE" in line for line in logs)

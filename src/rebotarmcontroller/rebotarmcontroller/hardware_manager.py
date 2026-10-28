"""硬件管理核心：真实电机的唯一访问点与最后一道执行安全。

本模块属于硬件包，向上层（服务/动作/话题适配层）暴露一个
``HardwareManager``；它协调本地驱动、串口/总线通道与唯一的硬件控制循环，
向上提供设备生命周期和执行接口。

主要职责：
- 装配生命周期组件并转发连接、使能、失能、模式、置零及关闭请求；
- 协调反馈组件采集与校验位置、速度、力矩、状态码、时效性及接收序号；
- 位置-速度（pos_vel）轨迹跟踪、safe_home、trajectory_stop 的落地点；
- 夹爪的位置移动、抓取闭合与有界保持；
- 重力补偿（MIT 模式 + 广义重力前馈）与低层 JointMotorCmd 直通；
- 拒绝不安全的低层命令：未连接/未使能、反馈过期、越限、状态码异常。

并发的关键约定：
- 所有设备命令、反馈采集和句柄生命周期均经有界请求队列交唯一设备线程；
  驱动的命令锁及共享总线锁继续保护单次事务，端口断言禁止外部线程访问；
- ``ControllerLifecycle.lock`` 保护连接/使能/失能/置零等生命周期迁移，
  与运动命令互斥；
- ``_gripper_lock`` 保护夹爪状态机；``_feedback_lock`` 保护已验证反馈样本。

安全约束（务必保持）：
- 真机启动后处于失能态，只有新鲜反馈 + 显式使能才允许运动；
- 反馈过期（超过 ``_gripper_feedback_stale_timeout_sec``）或状态码异常时，
  控制循环会执行保护性失能；
- 任何安全校验失败都必须抛出异常，不得静默降级为"继续运动"。
"""

from __future__ import annotations

import logging
import threading
import time
from pathlib import Path
from typing import Optional, Sequence

import numpy as np

from .configuration import HardwareSettings, default_config, load_configuration
from .feedback_acquisition import FeedbackAcquisition, _FeedbackValidationError, _JOINT_POSITION_LIMITS_RAD
from .gripper_execution import GripperExecution
from .lifecycle import ControllerLifecycle, LifecyclePorts
from .controller_contracts import (VerifiedFeedbackSample as _VerifiedFeedbackSample,
                               CachedMotorFeedback as _CachedMotorFeedback)
from .controller_kinematics import ArmKinematics
from .controller_runtime import ControllerRuntime
from .device_requests import device_request
from .command_guard import CommandGuard, passthrough_decision
from .arm_trajectory import ArmTrajectory
from .sdk_driver import CommandTransportError
from .conversions import fk_to_pose
from .gravity_control import (GravitySettings, GravityExecutionState, LoopTiming, bounded_mit_command,
                              load_gravity_model, package_file, vector)


class GravityCommandError(RuntimeError):
    """电机命令传输失败，无法确认保持力矩。"""


_LOG = logging.getLogger(__name__)

from .gripper_control import (GripperControl,
    _G_MAX_DIST_M,
    _G_VERIFIED_OPEN_LIMIT_M,
    _G_ANGLE_OPEN,
    _G_COORDINATE_TOL_RAD,
    _G_CLOSED_FEEDBACK_TOL_RAD,
    _G_ZERO_VERIFY_TIMEOUT_SEC,
    _G_ZERO_VERIFY_SAMPLES,
    _G_ARRIVE_TOL,
    _G_TAU_MAX,
    _G_POSITION_MAX_SPEED_RAD_S,
    _G_POSITION_MAX_SPEED_MIN_RAD_S,
    _G_POSITION_MAX_SPEED_MAX_RAD_S,
    _G_POSITION_TIMEOUT_MARGIN_SEC,
    _G_POSITION_TIMEOUT_MARGIN_MAX_SEC,
    _G_FEEDBACK_STALE_TIMEOUT_SEC,
    _G_FEEDBACK_STALE_TIMEOUT_MAX_SEC,
    _HARDWARE_FEEDBACK_RATE_HZ,
    _HARDWARE_FEEDBACK_RATE_MIN_HZ,
    _HARDWARE_FEEDBACK_RATE_MAX_HZ,
    _G_LARGE_MOVE_MAX_TAU_NM,
    _G_LARGE_MOVE_MAX_TAU_CAP_NM,
    _G_KP_MOVE,
    _G_KD_MOVE,
    _G_DEFAULT_FORCE,
    _G_GRASP_CLOSE_KP,
    _G_GRASP_CLOSE_KD,
    _G_GRASP_HOLD_KP,
    _G_GRASP_HOLD_KD,
    _G_GRASP_CLOSE_FORCE_DEFAULT,
    _G_GRASP_CLOSE_FORCE_MAX,
    _G_GRASP_HOLD_FORCE_DEFAULT,
    _G_GRASP_HOLD_TIMEOUT_DEFAULT_SEC,
    _G_GRASP_HOLD_TIMEOUT_MAX_SEC,
    _G_GRASP_VEL_THRESHOLD,
    _G_GRASP_MIN_CLOSE_TIME,
    _G_GRASP_MIN_CLOSURE_M,
    _G_GRASP_TIMEOUT,
    _G_CTRL_RATE)

_GC_VEL_THRESHOLD = 0.04
_GC_W_VEL_THRESHOLD = 0.08
_GC_EE_FRAME = "end_link"
_GC_KP = 7.0
_GC_KD = 0.8
_GC_TAU_SCALE = np.ones(6, dtype=np.float64)
# DM 串口桥可能分多个轮询周期才把六帧电机反馈取回。5 ms 短于实测的帧节拍，
# 会把首帧晚于 joint1/joint2 到达的健康关节误判为失败。
_FEEDBACK_RETRY_INTERVAL_SEC = 0.05

# 厂商 ArmEndPos.safe_home 会把每个关节都驱到 0 rad。对本机械臂而言那是"折叠"
# 姿态而非张开姿态：joint3 原点在 x 方向为 -0.264 m，joint4 为 +0.2426 m，
# 因此当 2-5 关节接近零时，前臂会折回贴上臂，link2/link5 间距缩到约 1.6 mm。
# MoveIt 的 CheckStartStateCollision 随后会否决从该处发出的每一个规划请求
# （"1 contact(s) detected : link2 - link5"），即零位不可用于规划，也就无法作为
# 停放姿态。这里改为停在"视觉就绪"姿态：link2/link5 实测间隙 33.2 mm，且机械
# 上与零位同样是紧凑站姿（end_link 水平伸展 0.254 m 对 0.260 m，高 24 mm），
# 因此不增加机械代价，也不会让手臂在重力下长期外伸。该姿态同时是
# tests/test_paired_trajectory_protocol.py 视为标准工作位姿的姿态，并与 MoveIt
# SRDF 中的 "safe_home" 命名状态以及 MuJoCo keyframe 一致。
_SAFE_HOME_JOINT_POSITIONS = (0.0, 0.0, -0.017453292519943295, 0.0, 0.0, 0.0)
# safe_home 到位判据：所有关节残差的最大值需小于该阈值（rad）。
_SAFE_HOME_ARRIVE_TOL_RAD = 0.02
_SAFE_HOME_TIMEOUT_SEC = 30.0


def apply_gravity_compensation_tau_scale(tau: np.ndarray) -> np.ndarray:
    """对六轴广义重力力矩施加补偿增益，返回独立副本。"""
    scaled = np.array(tau, dtype=np.float64, copy=True)
    if scaled.shape == _GC_TAU_SCALE.shape:
        scaled *= _GC_TAU_SCALE
    return scaled


class HardwareManager:
    """ROS 硬件门面，本地驱动和运行时持有唯一总线与周期线程。

    生命周期：``connect()`` -> ``enable()`` -> 运动/夹爪命令 -> ``disable()``
    -> ``shutdown()``。构造仅加载配置和规范模型；设备打开发生在 ``connect()``，
    连接后默认失能，运动必须显式 enable。

    并发的回调模型：运动、生命周期与透传均跑在 ``ControllerRuntime`` 的唯一
    设备线程；同步公共入口在生命周期锁外排队等待结果。ROS 定时器线程只读
    已验证缓存，即使失能也不得同步刷新总线。
    因此任何跨线程状态都由 ``ControllerLifecycle.lock`` / ``_gripper_lock`` /
    ``_feedback_lock`` 保护。
    """

    def __init__(
        self,
        arm_cfg: Optional[str] = None,
        gripper_cfg: Optional[str] = None,
        channel: str = "",
        hardware_feedback_rate_hz: float = _HARDWARE_FEEDBACK_RATE_HZ,
        gripper_position_torque_cap_nm: float = _G_LARGE_MOVE_MAX_TAU_NM,
        gripper_position_max_speed_rad_s: float = _G_POSITION_MAX_SPEED_RAD_S,
        gripper_position_timeout_margin_sec: float = _G_POSITION_TIMEOUT_MARGIN_SEC,
        gripper_feedback_stale_timeout_sec: float = _G_FEEDBACK_STALE_TIMEOUT_SEC,
        grasp_hold_timeout_sec: float = _G_GRASP_HOLD_TIMEOUT_DEFAULT_SEC,
        gravity_config: Optional[str] = None,
        *,
        runtime_factory=ControllerRuntime,
    ) -> None:
        settings = HardwareSettings(hardware_feedback_rate_hz,gripper_position_torque_cap_nm,
                                    gripper_position_max_speed_rad_s,gripper_position_timeout_margin_sec,
                                    gripper_feedback_stale_timeout_sec,grasp_hold_timeout_sec)
        for name in settings.__dataclass_fields__:
            setattr(self,'_'+name,getattr(settings,name))
        self._hardware_feedback_period_sec=1. / settings.hardware_feedback_rate_hz
        self._configuration=load_configuration(arm_cfg,gripper_cfg,channel)
        self._command_guard = CommandGuard()
        import pinocchio as pin

        settings_path = (Path(gravity_config).expanduser() if gravity_config else
                         package_file('rebotarmcontroller', 'config/gravity_compensation.yaml'))
        self._gc_settings = GravitySettings.from_yaml(settings_path)
        self._gravity_model = load_gravity_model(
            package_file('rebotarm_moveit_config', 'config/rebotarm.urdf'), self._gc_settings)
        self._gc_torque_limits = self._gravity_model.effort
        self._backend = runtime_factory(self._configuration, ArmKinematics(self._gravity_model.model))
        self._backend.idle_callback = self._runtime_feedback_tick
        self._arm_trajectory = ArmTrajectory(self._backend.execution)
        self._lifecycle = self._assemble_lifecycle()
        self._backend.on_error = lambda exc: self._protective_disable_from_hardware_loop(str(exc),category="CONTROLLER")
        # 重力补偿用的动力学模型：pinocchio 模型 + 数据缓冲 + 末端帧 id，
        # 计算函数和数据缓冲来自同一个权威模型，不再使用 SDK 的另一份 URDF。
        self._gc_model = self._gravity_model.model
        self._gc_data = self._gc_model.createData()
        self._gc_ee_frame_id = self._gc_model.getFrameId(_GC_EE_FRAME)
        self._gc_compute_generalized_gravity = self._gravity_model.gravity
        self._gc_pin = pin

        self._gripper_cfg_path = (
            Path(gripper_cfg).expanduser() if gripper_cfg else self.default_gripper_cfg()
        )
        self._gripper_cfg = None
        self._gripper_mot = None
        self._gripper_ctrl = None
        # 组件在启动任何线程前一次初始化；生产与离线替身使用同一装配路径。
        self._gravity_execution = GravityExecutionState()
        self._gripper_execution = self._assemble_gripper()
        self._feedback_component = self._assemble_feedback()
        self._gripper_feedback_updated_monotonic = None
        self._gripper_feedback_error = None
        self._gripper_zero_error = None
        self._gripper_transport_error = None
        self._gravity_timing = None
        self._gravity_feedback_metrics = {}
        self._gravity_next_log = 0.
        self._arm_passthrough_until = self._gripper_passthrough_until = None


    def default_arm_cfg(self) -> Path:
        return default_config('arm')

    def default_gripper_cfg(self) -> Path:
        return default_config('gripper')

    @property
    def joint_names(self) -> list[str]:
        return list(self._backend.joint_names)

    @property
    def mode(self) -> str:
        return str(self._backend.mode)

    @property
    def enabled(self) -> bool:
        return self._lifecycle.enabled

    @property
    def connected(self) -> bool:
        return self._lifecycle.connected

    @property
    def control_loop_active(self) -> bool:
        return bool(self._backend.control_loop_active)

    @property
    def has_gripper(self) -> bool:
        return self._gripper_mot is not None

    @property
    def state_machine(self) -> str:
        return self._lifecycle.execution

    @property
    def lifecycle_state(self) -> str:
        return self._lifecycle.state

    @property
    def ready_for_motion(self) -> bool:
        # 只有"已连接 + 已使能 + 位于保持或轨迹执行状态"才允许运动；
        # 上层必须以此为准，不得只看 enabled。
        return bool(
            self._lifecycle.connected
            and self._lifecycle.enabled
            and self._lifecycle.state in ("ENABLED_HOLD", "TRAJECTORY_RUNNING")
        )

    @property
    def gripper_active(self) -> bool:
        with self._gripper_execution._gripper_lock:
            return bool(self._gripper_execution._gripper_active)

    @property
    def gripper_mode(self) -> str:
        with self._gripper_execution._gripper_lock:
            return str(self._gripper_execution._gripper_mode)

    @property
    def error_codes(self) -> list[str]:
        """累计错误码 + 当前反馈健康状态派生出的错误项。

        反馈类错误是在读取时按最新时效性实时派生的，不会写入 ``_error_codes``；
        这样"反馈恢复"后无需清理历史即可自动消失。
        """
        return self.get_feedback_status()[1]

    def get_feedback_status(self) -> tuple[list[int], list[str]]:
        """一次只读快照生成 ArmStatus 的状态码与诊断，避免恢复帧混用旧错误。"""
        with self._feedback_component._feedback_lock, self._gripper_execution._gripper_lock:
            now = time.monotonic()
            codes = list(self._lifecycle.errors)
            failure = self._arm_feedback_failure_reason(now=now)
            statuses = [
                self._cached_motor_feedback_locked(
                    name, failure_reason=failure, now=now,
                ).status_code
                for name in self.joint_names
            ]
            if failure is not None:
                codes.append(f"ARM_FEEDBACK: {failure}")
            if self._gripper_mot is not None:
                failure = self._cached_gripper_feedback(now=now).failure_reason
                if failure is not None:
                    codes.append(f"GRIPPER_FEEDBACK: {failure}")
            return statuses, codes


    def _assemble_lifecycle(self):
        """只装配明确的组件能力；事务不能反向访问整个 manager。"""
        return ControllerLifecycle(LifecyclePorts(
            device=self._backend,
            joint_feedback=lambda **kw: self._validated_joint_feedback(**kw),
            gripper_status=lambda status: self._validated_gripper_status(status),
            gripper_feedback_status=lambda: (None if self._gripper_mot is None else
                                            int(self._refresh_gripper_feedback().status_code)),
            gripper_motor=lambda: self._gripper_mot,
            gripper_coordinate_failure=self._lifecycle_gripper_coordinate_failure,
            prepare_gripper_enable=self._prepare_gripper_enable,
            clear_gripper_transport_error=lambda: setattr(self, '_gripper_transport_error', None),
            reset_session=self._reset_device_session,
            initialize_gripper=lambda: self.init_gripper(str(self._gripper_cfg_path)),
            stop_gripper=lambda: self._stop_gripper_loop(),
            stop_control=lambda: self._stop_control_loop(),
            stop_gravity=lambda: self.stop_gravity_compensation(),
            clear_passthrough=self._clear_passthrough,
            hold=lambda: self.hold_current_position(),
            start_position=lambda **kw: self._start_pos_vel_loop(**kw),
            zero_gripper=lambda: self._set_gripper_zero(),
            confirm_disabled=lambda: self._command_guard.confirm_disabled(),
            protect=lambda *args, **kw: self._protective_disable_from_hardware_loop(*args, **kw),
        ))

    def _lifecycle_gripper_coordinate_failure(self):
        if self._gripper_mot is not None:
            with self._gripper_execution._gripper_lock:
                return self._gripper_coordinate_failure_reason_locked()
        return None

    def _prepare_gripper_enable(self):
        if self._gripper_mot is not None:
            from motorbridge import Mode
            self._gripper_mot.ensure_mode(Mode.MIT, 1000)

    def _clear_passthrough(self, *, arm_only=False):
        self._arm_passthrough_until = None
        if not arm_only:
            self._gripper_passthrough_until = None

    def _reset_device_session(self):
        """会话装配协调；旧缓存与旧夹爪代次不得跨重连复用。"""
        with self._feedback_component._feedback_lock:
            self._feedback_component._verified_feedback_by_label.clear()
            self._feedback_component._feedback_request_baseline_by_label.clear()
            self._feedback_component._feedback_request_deadline_by_label.clear()
            self._feedback_component._feedback_error_by_label.clear()
            self._feedback_component._feedback_next_refresh_monotonic = None
            self._feedback_component._arm_feedback_updated_monotonic = None
            self._feedback_component._arm_feedback_error = None
            self._gripper_feedback_updated_monotonic = None
            self._gripper_feedback_error = None
        with self._gripper_execution._gripper_lock:
            self._gripper_execution.supersede_locked()
            self._gripper_execution._gripper_active = False
            self._gripper_execution._gripper_mode = "idle"
            self._gripper_execution._gripper_neutral_pending = None
            self._gripper_execution._gripper_hold_deadline = None
            self._gripper_execution._gripper_target_deadline_monotonic = None
            self._gripper_transport_error = None


    def set_state_machine(self, state: str) -> None:
        """更新执行状态并同步生命周期；禁用过渡期间不能重新标记为保持。"""
        self._lifecycle.set_execution(state, enabled=self._lifecycle.enabled)


    def _require_connected(self) -> None:
        """读取生命周期准入，拒绝尚未连接的设备会话。"""
        return self._lifecycle.require_connected()

    def _require_enabled(self) -> None:
        """运动入口须已显式使能，且不处于失能状态未知的过渡。"""
        return self._lifecycle.require_enabled()

    def _refresh_all_feedback(self) -> None:
        self.refresh_feedback_if_due(force=True)

    def _feedback_controller_groups(self):
        """把关节与夹爪按所属控制器分组，返回 [(ctrl, [(label, motor), ...]), ...]。

        同一控制器下的电机共享一条总线，因此每个控制器只需一次
        request_feedback + poll_feedback_once 事务即可取回其全部电机反馈。
        缺少电机/控制器映射时抛错，宁可失败也不要静默漏采反馈。
        """
        groups: list[tuple[object, list[tuple[str, object]]]] = []

        def add(ctrl, label: str, motor) -> None:
            for existing_ctrl, entries in groups:
                if existing_ctrl is ctrl:
                    entries.append((label, motor))
                    return
            groups.append((ctrl, [(label, motor)]))

        for ctrl, entries in self._backend.feedback_groups():
            for label,motor in entries: add(ctrl,label,motor)
        if self._gripper_mot is not None and self._gripper_ctrl is None:
            raise RuntimeError("gripper feedback controller unavailable")
        if self._gripper_mot is not None:
            add(self._gripper_ctrl, "gripper", self._gripper_mot)
        if not groups:
            raise RuntimeError("hardware feedback controller map unavailable")
        return groups

    def _assemble_gripper(self):
        """构造时一次装配组件，运行期间不惰性补建状态。"""
        return GripperExecution(
            motor=lambda: self._gripper_mot,
            verified=lambda: self._verified_feedback_sample('gripper').state,
            feedback_failure=lambda: self._gripper_feedback_failure_reason_locked(),
            record_feedback_error=lambda message: self._record_gripper_feedback_error(message),
            validate=lambda state: self._validated_gripper_feedback_values(state),
            feedback_age=lambda: self._gripper_feedback_age_sec(),
            feedback_updated=lambda: self._gripper_feedback_updated_monotonic,
            speed=lambda: self._gripper_position_max_speed_rad_s,
            send=lambda *args, **kwargs: self._gripper_safe_mit(*args, **kwargs),
        )


    def _assemble_feedback(self):
        """构造时一次装配组件，运行期间不惰性补建状态。"""
        return FeedbackAcquisition(
            groups=self._feedback_controller_groups, joint_names=lambda: self.joint_names,
            gripper_motor=lambda: self._gripper_mot,
            gripper_values=self._validated_gripper_feedback_values,
            gripper_sample=self._record_gripper_feedback,
            gripper_error=self._record_gripper_feedback_error,
            period=lambda: self._hardware_feedback_period_sec,
            stale_timeout=lambda: self._gripper_feedback_stale_timeout_sec,
            backend=lambda: self._backend, loop_active=lambda: self.control_loop_active,
        )


    _feedback_state_with_sequence = staticmethod(FeedbackAcquisition._feedback_state_with_sequence)

    _feedback_sequence_advanced = staticmethod(FeedbackAcquisition._feedback_sequence_advanced)

    def _verified_feedback_sample(self, *args, **kwargs):
        return self._feedback_component._verified_feedback_sample(*args, **kwargs)

    def _verified_feedback_samples(self, *args, **kwargs):
        return self._feedback_component._verified_feedback_samples(*args, **kwargs)

    def _feedback_response_window_sec(self, *args, **kwargs):
        return self._feedback_component._feedback_response_window_sec(*args, **kwargs)

    _classified_feedback_error = staticmethod(FeedbackAcquisition._classified_feedback_error)

    def _feedback_failure_reason(self, *args, **kwargs):
        return self._feedback_component._feedback_failure_reason(*args, **kwargs)

    def _cached_motor_feedback_locked(self, *args, **kwargs):
        return self._feedback_component._cached_motor_feedback_locked(*args, **kwargs)

    def _validate_feedback_sample(self, *args, **kwargs):
        return self._feedback_component._validate_feedback_sample(*args, **kwargs)

    def _record_feedback_label_error(self, *args, **kwargs):
        return self._feedback_component._record_feedback_label_error(*args, **kwargs)

    def _record_verified_feedback(self, *args, **kwargs):
        return self._feedback_component._record_verified_feedback(*args, **kwargs)

    def _sync_feedback_health(self, *args, **kwargs):
        return self._feedback_component._sync_feedback_health(*args, **kwargs)

    def _inspect_pending_feedback(self, *args, **kwargs):
        return self._feedback_component._inspect_pending_feedback(*args, **kwargs)

    def _read_feedback_observations(self, *args, **kwargs):
        return self._feedback_component._read_feedback_observations(*args, **kwargs)

    _require_feedback_sequence_api = staticmethod(FeedbackAcquisition._require_feedback_sequence_api)

    def _record_arm_feedback_success(self, *args, **kwargs):
        return self._feedback_component._record_arm_feedback_success(*args, **kwargs)

    def _record_arm_feedback_error(self, *args, **kwargs):
        return self._feedback_component._record_arm_feedback_error(*args, **kwargs)

    def _arm_feedback_failure_reason(self, *args, **kwargs):
        return self._feedback_component._arm_feedback_failure_reason(*args, **kwargs)

    def _refresh_feedback_batch(self, *args, **kwargs):
        return self._feedback_component._refresh_feedback_batch(*args, **kwargs)

    def _force_feedback_refresh(self, *args, **kwargs):
        return self._feedback_component._force_feedback_refresh(*args, **kwargs)

    def refresh_feedback_if_due(self, *args, **kwargs):
        return self._feedback_component.refresh_feedback_if_due(*args, **kwargs)


    @staticmethod
    def _validated_gripper_feedback_values(state) -> tuple[float, float, float, int]:
        """拆出夹爪反馈的 (位置 rad, 速度 rad/s, 力矩 N.m, 状态码)，并做有限性检查。"""
        if state is None:
            raise RuntimeError("gripper feedback unavailable")
        position = float(state.pos)
        velocity = float(state.vel)
        torque = float(state.torq)
        if not all(np.isfinite(value) for value in (position, velocity, torque)):
            raise _FeedbackValidationError(
                "INVALID_VALUES", "gripper feedback contains non-finite values"
            )
        return position, velocity, torque, int(state.status_code)

    def _record_gripper_feedback(self, state, *, observed_at: float | None = None) -> None:
        position, velocity, torque, status = self._validated_gripper_feedback_values(state)
        timestamp = time.monotonic() if observed_at is None else float(observed_at)
        with self._gripper_execution._gripper_lock:
            recovered = self._gripper_feedback_error is not None
            self._gripper_execution._gripper_pos = position
            self._gripper_execution._gripper_vel = velocity
            self._gripper_execution._gripper_torque = torque
            self._gripper_feedback_updated_monotonic = timestamp
            # 有限的原始坐标仍保留供显式置零使用，健康判定和控制入口必须拒绝越限。
            coordinate_error = self._gripper_coordinate_failure_reason_locked(
                position=position, check_zero=False,
            )
            if coordinate_error is not None:
                self._record_gripper_feedback_error(coordinate_error)
                return
            self._gripper_feedback_error = None
        if recovered:
            _LOG.info(
                "gripper feedback recovered pos=%.6frad vel=%.6frad/s "
                "torque=%.6fNm status=%d updated=%.6f",
                position,
                velocity,
                torque,
                status,
                timestamp,
            )

    def _record_gripper_feedback_error(self, reason: str) -> None:
        message = str(reason)
        with self._gripper_execution._gripper_lock:
            changed = message != self._gripper_feedback_error
            self._gripper_feedback_error = message
        if changed:
            _LOG.error("gripper feedback error: %s", message)

    def _gripper_feedback_age_sec(self, *, now: float | None = None) -> float:
        with self._gripper_execution._gripper_lock:
            updated = self._gripper_feedback_updated_monotonic
        if updated is None:
            return float("inf")
        current = time.monotonic() if now is None else float(now)
        return max(current - updated, 0.0)

    def _gripper_feedback_failure_reason_locked(self, *, now: float | None = None) -> str | None:
        """夹爪反馈失效原因（调用方需持有 ``_gripper_lock``）。

        依次检查：显式错误 -> 数据过期 -> 坐标合法性（置零错误或原始角越界）。
        """
        return self._feedback_failure_reason(
            "gripper", error=self._gripper_feedback_error,
            updated=self._gripper_feedback_updated_monotonic, now=now,
        ) or self._gripper_coordinate_failure_reason_locked()

    def _gripper_coordinate_failure_reason_locked(
        self, *, position: float | None = None, check_zero: bool = True
    ) -> str | None:
        """夹爪原始角度坐标的合法性检查（调用方需持有 ``_gripper_lock``）。

        有效区间是 [_G_ANGLE_OPEN - 量化容差, _G_CLOSED_FEEDBACK_TOL_RAD]，
        即"从全开到闭合零点附近"。越界说明零点未标定或编码器异常，必须拒绝
        使用该反馈（例如合拢方向越界会让力矩变成前馈而持续顶紧）。
        """
        if check_zero and self._gripper_zero_error is not None:
            return f"[ZERO_INVALID] {self._gripper_zero_error}"
        if position is None:
            position = self._gripper_execution._gripper_pos
        if not np.isfinite(position) or not (
            _G_ANGLE_OPEN - _G_COORDINATE_TOL_RAD
            <= position <= _G_CLOSED_FEEDBACK_TOL_RAD
        ):
            return (
                f"[INVALID_COORDINATE] gripper coordinate invalid: raw={position:.6f} rad outside "
                f"[{_G_ANGLE_OPEN:.6f}, 0.000000] rad "
                f"(open_tolerance={_G_COORDINATE_TOL_RAD:.6f}, "
                f"closed_tolerance={_G_CLOSED_FEEDBACK_TOL_RAD:.6f}); verify closed zero"
            )
        return None

    def _refresh_gripper_feedback(self):
        """同步强制刷新并返回夹爪的已验证状态对象（拿不到新帧即报错）。"""
        if self._gripper_mot is None or self._gripper_ctrl is None:
            raise RuntimeError("gripper feedback unavailable: motor/controller not initialized")
        self.refresh_feedback_if_due(force=True)
        return self._verified_feedback_sample("gripper").state

    def _validated_joint_feedback(
        self,
        *,
        expected_status: int | None = None,
        refresh: bool = True,
        check_freshness: bool = True,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, list[int]]:
        """取出全部关节的 (位置 rad, 速度 rad/s, 力矩 N.m, 状态码列表)。

        ``refresh`` 为真时先做一次同步强制刷新；``check_freshness`` 为真时要求
        整体反馈未过期；``expected_status`` 非空时要求每个电机的状态码完全一致
        （0 = 失能，1 = 使能）。任一关节缺失、非有限、越软限位或状态码不符都会
        抛错，调用方不得继续运动。
        """
        if refresh:
            self.refresh_feedback_if_due(force=True)
        if check_freshness:
            feedback_failure = self._arm_feedback_failure_reason()
            if feedback_failure is not None:
                raise RuntimeError(feedback_failure)
        joint_names = self.joint_names
        if len(joint_names) != len(set(joint_names)):
            raise RuntimeError("joint names contain duplicates")
        if len(joint_names) != len(_JOINT_POSITION_LIMITS_RAD):
            raise RuntimeError(
                f"joint count mismatch: got {len(joint_names)}, "
                f"expected {len(_JOINT_POSITION_LIMITS_RAD)}"
            )

        positions: list[float] = []
        velocities: list[float] = []
        torques: list[float] = []
        statuses: list[int] = []
        samples = self._verified_feedback_samples(joint_names)
        for name, sample in zip(joint_names, samples):
            if name not in _JOINT_POSITION_LIMITS_RAD:
                raise RuntimeError(f"no hardware soft limit configured for {name}")
            state = sample.state
            values = (float(state.pos), float(state.vel), float(state.torq))
            if not all(np.isfinite(value) for value in values):
                raise RuntimeError(f"{name} feedback contains non-finite values")
            lower, upper = _JOINT_POSITION_LIMITS_RAD[name]
            if values[0] < lower or values[0] > upper:
                raise RuntimeError(
                    f"{name} position {values[0]:.6f} rad outside "
                    f"hardware soft limit [{lower:.6f}, {upper:.6f}]"
                )
            status = int(state.status_code)
            if expected_status is not None and status != expected_status:
                raise RuntimeError(
                    f"{name} status_code={status}, expected {expected_status}"
                )
            positions.append(values[0])
            velocities.append(values[1])
            torques.append(values[2])
            statuses.append(status)

        return (
            np.array(positions, dtype=np.float64),
            np.array(velocities, dtype=np.float64),
            np.array(torques, dtype=np.float64),
            statuses,
        )

    def _validated_gripper_status(self, expected_status: int) -> None:
        if self._gripper_mot is None:
            return
        state = self._refresh_gripper_feedback()
        status = int(state.status_code)
        if status != expected_status:
            raise RuntimeError(
                f"gripper status_code={status}, expected {expected_status}"
            )

    @device_request(priority=5, ttl=10., invalidate=('device',))
    def connect(self) -> None:
        """建立新会话并确认默认失能，不启动运动输出。"""
        return self._lifecycle.connect()

    @device_request(priority=5, ttl=10., invalidate=('device',))
    def shutdown(self) -> None:
        """关闭已确认失能的会话；仍使能或状态未知时保留设备连接。"""
        return self._lifecycle.shutdown()

    def get_joint_state(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """返回 (位置, 速度, 力矩)；未连接时拒绝伪造首帧。

        已连接时要求状态码与使能状态一致（使能 -> 1，失能 -> 0）。正式运行时
        的采集始终归唯一设备线程，本入口只读取缓存。
        """
        with self._lifecycle.lock:
            if not self._lifecycle.connected:
                raise RuntimeError('hardware is not connected; waiting for verified first frame')
            expected_status = 1 if self._lifecycle.enabled else 0
            positions, velocities, torques, _statuses = self._validated_joint_feedback(
                expected_status=expected_status,
                refresh=False,
            )
            return positions, velocities, torques

    def get_cached_joint_state(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """与 ``get_joint_state`` 相同，但不触发新的总线事务（只读缓存）。"""
        with self._lifecycle.lock:
            if not self._lifecycle.connected:
                raise RuntimeError('hardware is not connected; waiting for verified first frame')
            expected_status = 1 if self._lifecycle.enabled else 0
            positions, velocities, torques, _statuses = self._validated_joint_feedback(
                expected_status=expected_status,
                refresh=False,
            )
            return positions, velocities, torques

    def get_cached_joint_sample(self):
        """返回同一快照的数值、原始状态码、接收序号及最旧反馈观测时间。

        发布允许保留真实电机故障码；动作入口仍通过 expected_status 拒绝故障。
        这里只读已验证缓存，不依赖生命周期锁；否则发布回调会在慢模式切换
        中阻塞，并因占有发布锁而阻止下一轮反馈刷新。
        """
        with self._feedback_component._feedback_lock:
            now = time.monotonic()
            failure = self._arm_feedback_failure_reason(now=now)
            if failure is not None:
                raise RuntimeError(failure)
            feedback = [self._cached_motor_feedback_locked(name, now=now) for name in self.joint_names]
            for item in feedback:
                if item.failure_reason is not None:
                    raise RuntimeError(item.failure_reason)
            positions, velocities, torques = (
                np.array([item.values[index] for item in feedback], dtype=np.float64)
                for index in range(3)
            )
            return (
                positions, velocities, torques,
                [item.status_code for item in feedback],
                tuple(item.sample.sequence for item in feedback),
                min(item.sample.observed_at for item in feedback),
            )

    @device_request(resource='arm', priority=0, invalidate=('arm',))
    def hold_current_position(self) -> np.ndarray:
        """把当前关节角写成保持目标，使机械臂停在原地。"""
        q, _, _ = self.get_joint_state()
        current = np.array(q, dtype=np.float64, copy=True)
        self._backend.set_target(current)
        return current

    @device_request(resource='arm', priority=0, ttl=5.)
    def stop_active_motion(self, task=None) -> None:
        """撤销当前任务并确认保持；旧任务的取消不能停止接替任务。

        只有保持命令成功发送且后续新帧持续静止 0.1 s 才返回成功。超时报告
        未确认，保留使能和保持目标；这里绝不把普通取消变成自动失能。
        """
        owner = getattr(self, '_arm_trajectory', None)
        active = None if owner is None else owner.task
        if task is not None and (active is not task or task.event.is_set()):
            return
        with self._lifecycle.lock:
            self._backend.requests.invalidate('arm')
            if active is not None:
                owner.stopping(active)
            self._backend.cancel_trajectory()
            self._arm_passthrough_until = None
            if self._lifecycle.enabled:
                self.stop_gravity_compensation()
                self.ensure_pos_vel_control()
                current = self.hold_current_position()
                try:
                    self._confirm_arm_hold(current)
                except Exception as exc:
                    if active is not None:
                        owner.finish(active, 'aborted', f'stop/hold unconfirmed: {exc}')
                    raise
            self.set_state_machine('IDLE')
            if active is not None:
                owner.finish(active, 'canceled', 'stop/hold confirmed')

    def _confirm_arm_hold(self, current):
        """运行时独占的停止确认事务；保持发送与反馈确认使用同一写入者。"""
        deadline = time.monotonic() + 2.
        quiet_since = None
        sent = time.monotonic()
        while time.monotonic() < deadline:
            self._backend.position_tick(self._backend, 1./self._backend.rate)
            self.refresh_feedback_if_due(force=True)
            q, qd, _, _, _, stamp = self.get_cached_joint_sample()
            quiet = (stamp > sent and np.max(np.abs(q-current)) <= .03
                     and np.max(np.abs(qd)) <= .05)
            if quiet:
                quiet_since = stamp if quiet_since is None else quiet_since
                if stamp - quiet_since >= .10:
                    return
            else:
                quiet_since = None
            self._backend.device_wait(.01)
        raise RuntimeError('fresh stationary hold feedback not confirmed')

    @device_request(resource='arm')
    def submit_arm_trajectory(self, joint_names, points, times, *, home=False,
                              velocity=None, timeout=2.):
        """原子检查六轴资源并提交轨迹；返回只属于本请求的任务句柄。"""
        self._require_enabled()
        if len(joint_names) != 6 or set(joint_names) != set(self.joint_names):
            raise ValueError('trajectory joints must match hardware exactly')
        if not points or len(points) != len(times):
            raise ValueError('trajectory must contain matching points and times')
        if any(len(point) != 6 for point in points):
            raise ValueError('each trajectory point must contain six positions')
        if not np.isfinite(timeout) or timeout <= 0:
            raise ValueError('trajectory settling timeout must be positive and finite')
        order = [joint_names.index(name) for name in self.joint_names]
        ordered = [self._backend.execution.validate_target([point[i] for i in order])
                   for point in points]
        stamps = [float(value) for value in times]
        if (not np.isfinite(stamps).all() or min(stamps) < 0
                or any(b < a for a, b in zip(stamps, stamps[1:]))):
            raise ValueError('trajectory times must be finite, nonnegative and nondecreasing')
        active = self._arm_trajectory.task
        if active is not None and not active.event.is_set():
            raise RuntimeError('arm resource busy')
        if getattr(self, '_arm_passthrough_until', None) is not None:
            raise RuntimeError('arm resource owned by passthrough')
        current, _, _ = self.get_joint_state()
        if np.max(np.abs(np.asarray(ordered[0])-current)) > .10:
            raise ValueError('first trajectory point is too far from current joint state')
        self.ensure_pos_vel_control()
        # 同时刻的点合并为最后一个；t=0 首点按既有接口仅作为对齐占位。
        samples = {0.: tuple(current)}
        for index, (stamp, point) in enumerate(zip(stamps, ordered)):
            if index == 0 and stamp <= 1e-9:
                continue
            samples[stamp] = point
        task = self._arm_trajectory.submit(list(samples.values()), list(samples), current,
            tolerance=.02 if home else .03, settle=timeout,
            velocity=velocity, hold_on_success=home)
        self.set_state_machine('TRAJ_RUNNING')
        return task

    def arm_execution_status(self, task):
        return self._arm_trajectory.snapshot(task)

    @device_request(resource='arm')
    def set_joint_target(self, positions) -> None:
        self._require_enabled()
        self._arm_passthrough_until = None
        self._backend.set_target(positions)

    def joint_target(self):
        return self._backend.target()

    @device_request(resource='arm', priority=0, invalidate=('arm',))
    def cancel_arm_trajectory(self) -> None:
        self.stop_active_motion()

    @property
    def trajectory_moving(self):
        return self._backend.trajectory_moving

    @property
    def trajectory_progress(self):
        return self._backend.trajectory_progress

    def safe_home_target(self) -> np.ndarray:
        return np.array(_SAFE_HOME_JOINT_POSITIONS, dtype=np.float64)

    def _validated_safe_home_target(
        self, target: Optional[Sequence[float]] = None
    ) -> np.ndarray:
        """校验 safe_home 目标：长度、有限性，以及逐关节是否在软限位内。"""
        if target is None:
            return self.safe_home_target()
        values = np.array([float(value) for value in target], dtype=np.float64)
        expected = len(_SAFE_HOME_JOINT_POSITIONS)
        if values.shape != (expected,):
            raise ValueError(
                f"safe_home target must hold {expected} joint positions, got {values.shape[0]}"
            )
        if not np.all(np.isfinite(values)):
            raise ValueError("safe_home target must be finite")
        for name, value in zip(self.joint_names, values):
            limits = _JOINT_POSITION_LIMITS_RAD.get(name)
            if limits is None:
                continue
            low, high = limits
            if not low <= value <= high:
                raise ValueError(
                    f"safe_home target for {name} is {value:.4f} rad, "
                    f"outside the allowed range [{low}, {high}]"
                )
        return values

    def safe_home(self, target=None, vlim=None, timeout_sec=None) -> np.ndarray:
        """提交有速度覆盖的停靠任务并等待；超时/取消由周期执行器处理。"""
        goal = self._validated_safe_home_target(target)
        speed = self._backend.home_velocity if vlim is None else float(vlim)
        timeout = _SAFE_HOME_TIMEOUT_SEC if timeout_sec is None else float(timeout_sec)
        if not np.isfinite(timeout) or timeout <= 0:
            raise ValueError('safe_home timeout must be positive and finite')
        if not 0 < speed <= self._backend.home_velocity:
            raise ValueError('safe_home velocity exceeds reviewed limit')
        current, _, _ = self.get_joint_state()
        task = self.submit_arm_trajectory(self.joint_names, [current, goal], [0., .001],
            home=True, velocity=[speed]*6, timeout=timeout)
        task.event.wait()
        status = self.arm_execution_status(task)
        if status.state != 'succeeded':
            raise RuntimeError(status.message)
        return self.get_joint_state()[0]

    @device_request(priority=5, ttl=5., invalidate=('device',))
    def enable(self) -> None:
        """显式使能并以当前位置保持；失败由生命周期组件验证回滚。"""
        return self._lifecycle.enable()

    @device_request(priority=1, ttl=10., invalidate=('device',))
    def disable(self) -> None:
        """显式失能并验证新反馈，验证失败不谎报已安全失能。"""
        return self._lifecycle.disable()

    @device_request(priority=5, ttl=5., invalidate=('device',))
    def set_mode(self, mode: str) -> bool:
        """串行交接 mit、pos_vel 或 vel 模式，传输故障进入保护。"""
        return self._lifecycle.set_mode(mode)

    @device_request(priority=5, ttl=5., invalidate=('device',))
    def set_zero(self, joint_name: str = "") -> bool:
        """仅失能时允许置零；空名称为六轴，gripper 为夹爪。"""
        return self._lifecycle.set_zero(joint_name)

    def _set_gripper_zero(self) -> bool:
        """夹爪置零并验证：要求在超时内连续若干帧都接近零位。

        验证必须基于"置零之后新到"的帧（``_refresh_gripper_feedback`` 会重建
        接收序号基线，置零前的缓存样本无法满足）。校验失败时保留
        ``_gripper_zero_error``：仅凭"有新帧"不能证明标定成功，必须等到下一次
        显式置零验证通过才清除。
        """
        if self._gripper_mot is None:
            raise RuntimeError("gripper is not initialized")
        self._stop_gripper_loop()
        with self._feedback_component._feedback_lock:
            self._validated_gripper_status(expected_status=0)
            with self._gripper_execution._gripper_lock:
                self._gripper_zero_error = "gripper zero verification pending"
            try:
                self._gripper_mot.set_zero_position()
                deadline = time.monotonic() + _G_ZERO_VERIFY_TIMEOUT_SEC
                consecutive = 0
                position = float("nan")
                while time.monotonic() < deadline:
                    # 强制刷新会重建接收序号基线；置零前缓存下来的样本无法满足
                    # 本次验证请求。
                    state = self._refresh_gripper_feedback()
                    position, _velocity, _torque, status = (
                        self._validated_gripper_feedback_values(state)
                    )
                    if status != 0:
                        raise RuntimeError(
                            f"gripper status_code={status}, expected 0 after zero"
                        )
                    if time.monotonic() >= deadline:
                        break
                    # 必须是"连续"多帧接近零位，单帧偶然命中不算通过。
                    consecutive = (
                        consecutive + 1 if abs(position) <= _G_COORDINATE_TOL_RAD else 0
                    )
                    if consecutive >= _G_ZERO_VERIFY_SAMPLES:
                        with self._gripper_execution._gripper_lock:
                            self._gripper_zero_error = None
                        self._lifecycle.enabled = False
                        self.set_state_machine("IDLE")
                        return True
                    time.sleep(_FEEDBACK_RETRY_INTERVAL_SEC)
                raise RuntimeError(
                    f"zero verification timed out: raw={position:.6f} rad; "
                    f"need {_G_ZERO_VERIFY_SAMPLES} fresh disabled samples within "
                    f"+/-{_G_COORDINATE_TOL_RAD:.6f} rad"
                )
            except Exception as exc:
                message = f"gripper set_zero failed: {exc}"
                with self._gripper_execution._gripper_lock:
                    # 仅凭新帧并不能证明标定失败已消除；在显式置零验证通过之前
                    # 一直保留该失败标记。
                    self._gripper_zero_error = message
                raise RuntimeError(message) from exc

    @device_request(resource='arm', priority=5)
    def ensure_pos_vel_control(self) -> None:
        """确保处于位置-速度模式且统一控制循环在运行（否则保持当前位置）。"""
        with self._lifecycle.lock:
            self._require_enabled()
            if self.mode != "pos_vel":
                self._stop_control_loop()
                self.set_mode("pos_vel")
            if not self.control_loop_active:
                self._start_pos_vel_loop()
            else:
                self.hold_current_position()

    @staticmethod
    def _validate_passthrough_values(pos, vel, kp, kd, tau, vlim, bounds, max_velocity, max_effort, mode):
        passthrough_decision(pos, vel, kp, kd, tau, vlim, bounds,
                             max_velocity, max_effort, mode).require_allowed()

    @device_request(resource='arm', ttl=.25)
    def send_joint_motor_cmd(self, joint_name: str, cmd, arbitration="reject") -> None:
        """低层单关节直通命令（JointMotorCmd）。

        未显式给出的字段回落到当前反馈值或关节默认增益，避免用零值覆盖。命令
        送入后状态机切到 LOWLEVEL_STREAMING，提示上层这不是轨迹跟踪。
        mode：0 = MIT，1 = 位置-速度，2 = 速度（需电机支持 send_vel）。
        """
        with self._lifecycle.lock:
            self._require_enabled()
            if joint_name not in self.joint_names:
                raise KeyError(f"unknown joint: {joint_name}")

            task = getattr(getattr(self, '_arm_trajectory', None), 'task', None)
            busy = ((task is not None and not task.event.is_set())
                    or self.gravity_compensation_active())
            if busy and arbitration != 'preempt':
                raise RuntimeError('arm resource busy')
            mot = self._backend.joint_device(joint_name)
            jc = self._backend.joint_configuration(joint_name)
            feedback_failure = self._arm_feedback_failure_reason()
            if feedback_failure is not None:
                raise RuntimeError(feedback_failure)
            state = self._verified_feedback_sample(joint_name).state

            def values_from(sample):
                return (float(cmd.pos) if cmd.use_pos else float(sample.pos),
                        float(cmd.vel) if cmd.use_vel else float(sample.vel),
                        float(cmd.kp) if cmd.use_kp else float(jc.kp),
                        float(cmd.kd) if cmd.use_kd else float(jc.kd),
                        float(cmd.tau) if cmd.use_tau else 0.,
                        float(cmd.vlim) if cmd.use_vlim else float(jc.vlim))

            values = values_from(state)
            passthrough_decision(*values,
                _JOINT_POSITION_LIMITS_RAD[joint_name], jc.vlim,
                float(self._gc_torque_limits[self.joint_names.index(joint_name)]),
                int(cmd.mode), device_mode='pos_vel' if busy else self.mode).require_allowed()
            if busy:
                # 无效命令不得取消原任务；完成准入后才交接资源并确认保持。
                self.stop_active_motion(task)
                # 停止可能耗时且切回 POS_VEL；默认反馈值须取停止后的新快照。
                values = values_from(self._verified_feedback_sample(joint_name).state)
                passthrough_decision(*values,
                    _JOINT_POSITION_LIMITS_RAD[joint_name], jc.vlim,
                    float(self._gc_torque_limits[self.joint_names.index(joint_name)]),
                    int(cmd.mode), device_mode=self.mode).require_allowed()
            pos, vel, kp, kd, tau, vlim = values
            try:
                if int(cmd.mode) == 0:
                    mot.send_mit(pos, vel, kp, kd, tau)
                elif int(cmd.mode) == 1:
                    mot.send_pos_vel(pos, vlim)
                elif int(cmd.mode) == 2:
                    if not hasattr(mot, "send_vel"):
                        raise RuntimeError(f"{joint_name} does not support send_vel")
                    mot.send_vel(vel)
                else:
                    raise ValueError(f"unsupported JointMotorCmd mode: {cmd.mode}")
            except CommandTransportError as exc:
                self._protective_disable_from_hardware_loop(str(exc),category="COMMAND")
                raise
            # 一次透传租约最长 0.25 s；后续周期不覆盖它，断流后在当前位置保持。
            self._arm_passthrough_until = time.monotonic() + .25
            self.set_state_machine("LOWLEVEL_STREAMING")

    @device_request(resource='arm', priority=5, invalidate=('arm',))
    def start_gravity_compensation(self) -> None:
        with self._lifecycle.lock:
            self._start_gravity_compensation_locked()

    def _start_gravity_compensation_locked(self) -> None:
        """进入重力补偿：MIT 模式 + 广义重力前馈 + 目标位置 PI 锁定。

        流程：停掉位置-速度循环与发送线程 -> 刷新反馈并把当前角度设为目标 ->
        以配置的 Kp/Kd 进入 MIT 模式 -> 启动统一控制循环执行
        ``_gravity_hardware_tick``。补偿期间操作员可以推动机械臂（目标会跟随），
        停止时再回到位置-速度保持。
        """
        self._require_enabled()
        self._arm_passthrough_until = None
        timing = LoopTiming(float(self._backend.rate))
        self.stop_gravity_compensation()
        self._stop_control_loop()
        self._backend.cancel_trajectory()
        try:
            self._refresh_arm_feedback()
            self._gravity_execution.target = self._read_gravity_comp_positions()
        except Exception as exc:
            self._protective_disable_from_hardware_loop(str(exc))
            raise
        self._gravity_execution.last_position = self._gravity_execution.target.copy()
        # 切模式前先拒绝无法支撑当前姿态的模型计算结果。
        try:
            gravity = vector(self._gc_compute_generalized_gravity(q=self._gravity_execution.target),
                             6, 'gravity torque')
            if (np.abs(gravity) > self._gc_torque_limits).any():
                raise RuntimeError('gravity torque exceeds model effort limits')
        except Exception:
            self._start_pos_vel_loop(target=self._gravity_execution.target)
            raise
        self._gravity_execution.fault = None
        self._gravity_execution.hold_feedforward = None
        self._gravity_execution.integral = np.zeros_like(self._gravity_execution.target)
        self._gravity_execution.lock_counter = 0
        try:
            if not self._backend.mode_mit(
                kp=np.full(6, self._gc_settings.kp), kd=np.full(6, self._gc_settings.kd)
            ):
                raise RuntimeError('MIT mode switch did not succeed on all joints')
            # 模式交接可能持续数百毫秒，交接后必须重新取得新帧。
            self._refresh_arm_feedback()
            self._gravity_execution.target = self._read_gravity_comp_positions()
            samples = self._verified_feedback_samples(self.joint_names)
            self._gravity_execution.last_total = np.clip(
                [float(s.state.torq) for s in samples],
                -self._gc_torque_limits, self._gc_torque_limits)
            self._gravity_execution.active = True
            self._gravity_comp_tick(self._backend, 1. / float(self._backend.rate))
        except (GravityCommandError, CommandTransportError) as exc:
            self._gravity_execution.active = False
            self._protective_disable_from_hardware_loop(str(exc),category="GRAVITY_COMMAND")
            raise
        except Exception:
            self._gravity_execution.active = False
            # 健康任务失败恢复已验证的当前位置保持；恢复失败按设备故障处理。
            try:
                self._refresh_arm_feedback()
                target = self._read_gravity_comp_positions()
                if not self._backend.mode_pos_vel():
                    raise GravityCommandError('POS_VEL restore failed')
                self._start_pos_vel_loop(target=target)
                self.set_state_machine('IDLE')
            except Exception as restore_error:
                self._protective_disable_from_hardware_loop(str(restore_error))
            raise
        self._gravity_timing = timing
        self._gravity_feedback_metrics = {}
        self._gravity_next_log = 0.
        self.set_state_machine("GRAVITY_COMP")
        self._backend.start_control_loop(self._gravity_hardware_tick, rate=self._backend.rate)

    @device_request(resource='arm', priority=5, invalidate=('arm',))
    def stop_gravity_compensation(self) -> None:
        with self._lifecycle.lock:
            self._stop_gravity_compensation_locked()

    def _stop_gravity_compensation_locked(self) -> None:
        """退出重力补偿：用最新验证角度接管位置-速度保持。"""
        if not self._gravity_execution.active:
            return
        self._stop_control_loop()
        self._gravity_execution.active = False
        self._gravity_execution.target = None
        self._gravity_execution.integral = None
        self._gravity_execution.lock_counter = 0
        self._gravity_execution.last_position = None
        if self._lifecycle.enabled:
            try:
                self._refresh_arm_feedback()
                hold_target = self._read_gravity_comp_positions()
                if not self._backend.mode_pos_vel():
                    raise GravityCommandError('gravity POS_VEL restore failed')
                self._start_pos_vel_loop(target=hold_target)
            except Exception as exc:
                self._protective_disable_from_hardware_loop(str(exc))
                raise
        self.set_state_machine("IDLE")

    def gravity_compensation_active(self) -> bool:
        return self._gravity_execution.active

    def gravity_compensation_target(self) -> np.ndarray | None:
        if self._gravity_execution.target is None:
            return None
        return self._gravity_execution.target.copy()

    def _refresh_arm_feedback(self) -> None:
        self.refresh_feedback_if_due(force=True)

    @staticmethod
    def _angles_near_reference(values: np.ndarray, reference: np.ndarray) -> np.ndarray:
        """把角度解到与参考值最接近的等价分支上。

        差值先按模 2*pi 折到 (-pi, pi]，再加回参考值；用于抵消编码器在
        +/-pi 处的跳变，避免重力补偿把跨圈跳变误判为大幅运动。
        """
        delta = values - reference
        delta = (delta + np.pi) % (2.0 * np.pi) - np.pi
        return reference + delta

    def _read_gravity_comp_positions(
        self,
        *,
        request: bool = False,
        reference: np.ndarray | None = None,
    ) -> np.ndarray:
        # request 参数保留为兼容接口：实际采集一律通过统一反馈批次完成。
        del request
        q, _qd = self._read_gravity_comp_feedback(reference=reference)
        return q

    def _read_gravity_comp_feedback(
        self,
        *,
        reference: np.ndarray | None = None,
    ) -> tuple[np.ndarray, np.ndarray]:
        """重力补偿回路的反馈读取：(角度 rad, 角速度 rad/s)。

        要求全部电机状态码为 1（补偿期间必须使能），并做软限位/有限性校验；
        角度会解到与上一帧最接近的分支，结果同时更新 ``_gravity_comp_q_last``。
        """
        joint_names = self.joint_names
        with self._feedback_component._feedback_lock:
            feedback_failure = self._arm_feedback_failure_reason()
            if feedback_failure is not None:
                raise RuntimeError(feedback_failure)
            samples = self._verified_feedback_samples(joint_names)

        positions: list[float] = []
        velocities: list[float] = []
        for name, sample in zip(joint_names, samples):
            self._validate_feedback_sample(name, sample.state)
            status = int(sample.state.status_code)
            if status != 1:
                raise RuntimeError(
                    f"{name} status_code={status}, expected 1 during gravity compensation"
                )
            positions.append(float(sample.state.pos))
            velocities.append(float(sample.state.vel))

        q = np.array(positions, dtype=np.float64)
        qd = np.array(velocities, dtype=np.float64)
        ref = reference if reference is not None else self._gravity_execution.last_position
        if ref is not None:
            q = self._angles_near_reference(q, ref)
        self._gravity_execution.last_position = np.array(q, dtype=np.float64, copy=True)
        return self._gravity_execution.last_position.copy(), qd

    def _gravity_comp_tick(self, arm, dt: float) -> None:
        """重力补偿的单周期控制律（运行在统一控制循环里）。

        1) 读反馈；2) 用动力学模型算广义重力力矩并按增益缩放，作为前馈 tau；
        3) 目标角误差按实际 dt 积分，以 N.m 限幅用于抵消稳态摩擦/模型误差；
        4) 用末端雅可比把关节速度映到笛卡尔空间：一旦线速度/角速度超过阈值，
        说明人正在拖动，于是把目标角更新为当前角并衰减积分（"跟随"模式）；
        否则累加锁定计数（"锁定"模式）。
        下发时位置/速度目标固定，仅用 tau 项补偿重力，因此机械臂呈现"轻"的手感。
        """
        if not self._gravity_execution.active or self._gravity_execution.target is None:
            return

        q, qd = self._read_gravity_comp_feedback()
        if self._gravity_execution.fault is not None:
            self._emit_bounded_gravity_command(q, qd, self._gravity_execution.hold_feedforward, dt)
            return
        tau_g = vector(self._gc_compute_generalized_gravity(q=q), 6, 'gravity torque')
        tau_g = apply_gravity_compensation_tau_scale(tau_g)

        self._gc_pin.computeJointJacobians(self._gc_model, self._gc_data, q)
        self._gc_pin.updateFramePlacements(self._gc_model, self._gc_data)
        jacobian = self._gc_pin.getFrameJacobian(
            self._gc_model, self._gc_data, self._gc_ee_frame_id,
            self._gc_pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )
        # 计算状态不直接写设备；发送成功后才更新上一周期总力矩。
        feedforward = self._gravity_execution.step(
            q, dt, tau_g, jacobian @ qd, self._gc_settings,
        )
        self._emit_bounded_gravity_command(q, qd, feedforward, dt)

    def _emit_bounded_gravity_command(self, q, qd, feedforward, dt):
        settings = self._gc_settings
        kp, kd, tau, total = bounded_mit_command(
            self._gravity_execution.target, q, qd, np.full(6, settings.kp),
            np.full(6, settings.kd), feedforward, self._gc_torque_limits,
            settings.torque_rate_limits_nm_s, self._gravity_execution.last_total, dt)
        self._send_gravity_mit(
            pos=self._gravity_execution.target,
            vel=np.zeros(6), kp=kp, kd=kd, tau=tau,
        )
        self._gravity_execution.last_total = total

    def _send_gravity_mit(self, *, pos, vel, kp, kd, tau):
        try:
            self._backend.send_mit_batch(pos=pos,vel=vel,kp=kp,kd=kd,tau=tau)
        except Exception as exc:
            raise GravityCommandError(f'MIT batch failed: {exc}') from exc

    def _gravity_fault_hold(self, reason):
        q, _ = self._read_gravity_comp_feedback()
        if self._gravity_execution.fault is None:
            self._gravity_execution.fault = str(reason)
            self._gravity_execution.target = q.copy()
            self._gravity_execution.integral.fill(0.)
            self._gravity_execution.hold_feedforward = self._gravity_execution.last_total.copy()
            self._lifecycle.errors.append(f'GRAVITY_HOLD: {reason}')
            _LOG.error('gravity compensation stopped; enabled guarded hold: %s', reason)

    def gravity_compensation_diagnostics(self):
        timing = getattr(self, '_gravity_timing', None)
        model = getattr(self, '_gravity_model', None)
        settings = getattr(self, '_gc_settings', None)
        return dict(timing=timing.snapshot() if timing else {},
                    feedback={k: dict(v) for k, v in getattr(self, '_gravity_feedback_metrics', {}).items()},
                    configured_feedback_rate_hz=self._hardware_feedback_rate_hz,
                    fault=getattr(self._gravity_execution, 'fault', None),
                    model_path=model.path if model else None,
                    model_sha256=model.sha256 if model else None,
                    gravity_xyz_m_s2=list(settings.gravity_xyz_m_s2) if settings else None,
                    payload_mass_kg=settings.payload_mass_kg if settings else None,
                    payload_com_xyz_m=list(settings.payload_com_xyz_m) if settings else None)

    def current_pose(self):
        """用规范模型的纯数值 FK 换算成 ROS Pose(供状态发布使用)。"""
        q, _, _ = self.get_joint_state()
        position, rotation = self._backend.current_pose_values(q)
        return fk_to_pose(position, rotation)

    def get_joint_status_codes(self) -> list[int]:
        """返回各关节状态码；反馈整体失效时统一返回 255(未知)。"""
        with self._feedback_component._feedback_lock:
            now = time.monotonic()
            failure = self._arm_feedback_failure_reason(now=now)
            return [
                self._cached_motor_feedback_locked(
                    name, failure_reason=failure, now=now,
                ).status_code
                for name in self.joint_names
            ]

    def init_gripper(self, cfg_path: str) -> None:
        """按配置初始化夹爪电机，并挂到机械臂同一控制器/总线上。

        夹爪必须与机械臂共用同一个控制器对象（同一串口），否则无法保证总线
        事务的原子性。初始化阶段保持单线程：首次新鲜反馈校验由本方法同步完成，
        位置/抓取入口再按需启动统一控制循环。
        """
        gc=self._configuration.gripper
        self._gripper_cfg=gc
        self._gripper_mot,self._gripper_ctrl=self._backend.add_gripper(gc)

    @device_request(resource='gripper')
    def set_gripper_target(self, position_m: float, max_effort: float = 0.0) -> None:
        """下发夹爪位置目标（异步）：只登记目标，实际下发由 统一硬件循环完成。

        ``position_m``：0 表示完全闭合、正值为张开距离（m），会被夹到
        [0, _G_VERIFIED_OPEN_LIMIT_M]；``max_effort`` <= 0 时使用默认力矩
        ``_G_DEFAULT_FORCE``，否则夹到 [0.05, 位置力矩上限]。
        超时按"角度行程 / 最大速度 + 余量"估算，供 ``wait_gripper_target`` 使用。
        """
        self._require_enabled()
        if not self.control_loop_active:
            raise RuntimeError(
                "gripper command requires the unified hardware control loop"
            )
        if self._gripper_mot is None:
            raise RuntimeError("gripper is not initialized")
        with self._gripper_execution._gripper_lock:
            feedback_failure = self._gripper_feedback_failure_reason_locked()
        if feedback_failure is not None:
            raise RuntimeError(feedback_failure)
        state = self._verified_feedback_sample("gripper").state
        start_angle, _velocity, _torque, status = self._validated_gripper_feedback_values(state)
        if status != 1:
            raise RuntimeError(f"gripper status_code={status}, expected 1 before position command")
        if getattr(self, '_gripper_passthrough_until', None) is not None:
            raise RuntimeError('gripper resource owned by passthrough')
        if not np.isfinite([position_m, max_effort]).all():
            raise ValueError('gripper target and effort must be finite')
        distance = float(np.clip(position_m, 0.0, _G_VERIFIED_OPEN_LIMIT_M))
        # 使用完整标称行程：90 mm 对应 -5 rad，不再额外裁到 -4.9 rad。
        target = (distance / _G_MAX_DIST_M) * _G_ANGLE_OPEN
        effort = _G_DEFAULT_FORCE if max_effort <= 0.0 else float(max_effort)
        now = time.monotonic()
        target_timeout_sec = (
            abs(target - start_angle) / self._gripper_position_max_speed_rad_s
            + self._gripper_position_timeout_margin_sec
        )
        with self._gripper_execution._gripper_lock:
            self._gripper_execution.supersede_locked()
            self._gripper_execution._gripper_target_angle = start_angle
            self._gripper_execution._gripper_goal_angle = target
            self._gripper_execution._gripper_target_effort = float(
                np.clip(
                    effort,
                    0.05,
                    getattr(
                        self,
                        "_gripper_position_torque_cap_nm",
                        _G_LARGE_MOVE_MAX_TAU_NM,
                    ),
                )
            )
            self._gripper_execution._gripper_mode = "position"
            self._gripper_execution._gripper_active = True
            self._gripper_execution._gripper_position_result = "active"
            self._gripper_execution._gripper_command_error = None
            self._gripper_execution._gripper_last_tick_monotonic = now
            self._gripper_execution._gripper_target_timeout_sec = target_timeout_sec
            self._gripper_execution._gripper_target_deadline_monotonic = now + target_timeout_sec
            feedback_updated = self._gripper_feedback_updated_monotonic
            applied_effort = self._gripper_execution._gripper_target_effort
        _LOG.info(
            "gripper position command requested=%.6fm clamped=%.6fm "
            "start=%.6frad goal=%.6frad effort=%.6fNm speed=%.6frad/s "
            "timeout=%.3fs feedback_updated=%s",
            float(position_m),
            distance,
            start_angle,
            target,
            applied_effort,
            self._gripper_position_max_speed_rad_s,
            target_timeout_sec,
            feedback_updated,
        )
        self._start_gripper_loop()
        return self._gripper_execution._generation

    def gripper_target_timeout_sec(self) -> float:
        with self._gripper_execution._gripper_lock:
            return float(self._gripper_execution._gripper_target_timeout_sec)

    @property
    def gripper_command_error(self) -> str | None:
        with self._gripper_execution._gripper_lock:
            return self._gripper_execution._gripper_command_error

    @property
    def gripper_feedback_error(self) -> str | None:
        """当前反馈不可用原因；查询不写入或锁存派生错误。"""
        return self._cached_gripper_feedback().failure_reason

    def gripper_execution_status(self, generation):
        """返回本目标的状态，旧代次永远不能读成新目标的成功。"""
        with self._gripper_execution._gripper_lock:
            if generation != self._gripper_execution._generation:
                return 'superseded', 'gripper target superseded'
            state = self._gripper_execution._gripper_position_result
            if self._gripper_execution._gripper_neutral_pending is not None:
                state = 'releasing'
            return state, self._gripper_execution._gripper_command_error

    def wait_gripper_target(self, timeout=None, generation=None) -> bool:
        """兼容等待入口；只读快照，显式超时通过队列取消自己的代次。"""
        if generation is None:
            generation = self._gripper_execution._generation
        deadline = float('inf') if timeout is None else time.monotonic()+max(0., timeout)
        while True:
            state, _ = self.gripper_execution_status(generation)
            if state in ('succeeded', 'failed', 'superseded'):
                return state == 'succeeded'
            if time.monotonic() >= deadline:
                self.cancel_gripper_position_command('gripper target timeout',
                    expected_generation=generation)
                return False
            time.sleep(.02)

    def set_gripper_position(self, position_m: float, max_effort: float = 0.0) -> tuple[bool, float]:
        """同步版位置命令：下发并等待到位，返回 (是否到位, 实际位置 m)。"""
        generation = self.set_gripper_target(position_m, max_effort)
        reached = self.wait_gripper_target(generation=generation)
        return reached, self.gripper_position_m()

    @device_request(resource='gripper')
    def submit_grasp(
        self,
        close_force: float = _G_GRASP_CLOSE_FORCE_DEFAULT,
        hold_force: float = _G_GRASP_HOLD_FORCE_DEFAULT,
        close_timeout_sec: float = _G_GRASP_TIMEOUT,
        min_close_time_sec: float = _G_GRASP_MIN_CLOSE_TIME,
        velocity_threshold: float = _G_GRASP_VEL_THRESHOLD,
        min_closure_distance_m: float = _G_GRASP_MIN_CLOSURE_M,
        hold_timeout_sec: float | None = None,
    ) -> tuple[bool, bool, float, float, float, str]:
        """闭合抓取：检测"堵转"即认为接触，随后进入有界保持。

        判据（三者同时满足才算抓住）：已过 ``min_close_time_sec``、闭合行程
        >= ``min_closure_distance_m``、速度绝对值 <= ``velocity_threshold``。
        这里没有力传感器，因此这些条件表示"闭合推进 + 速度堵转"，即堵转检测，
        而不是实测接触力。
        返回 (是否检测到堵转, 是否进入保持, 接触位置 m, 当前位置 m, 保持力矩 N.m, 说明)。
        保持阶段有时限（``hold_timeout_sec``，默认取配置值，上限
        ``_G_GRASP_HOLD_TIMEOUT_MAX_SEC``），到时或超时路径都会中性释放，
        不会让电机长期带电顶紧。
        """
        self._require_enabled()
        if not self.control_loop_active:
            raise RuntimeError(
                "gripper grasp requires the unified hardware control loop"
            )
        if self._gripper_mot is None:
            raise RuntimeError("gripper is not initialized")
        with self._gripper_execution._gripper_lock:
            feedback_failure = self._gripper_feedback_failure_reason_locked()
        if feedback_failure is not None:
            raise RuntimeError(feedback_failure)

        if getattr(self, '_gripper_passthrough_until', None) is not None:
            raise RuntimeError('gripper resource owned by passthrough')
        close_effort = float(np.clip(close_force, 0.05, _G_GRASP_CLOSE_FORCE_MAX))
        hold_effort = float(np.clip(hold_force, 0.05, _G_TAU_MAX))
        # 抓取保持必须有界；无界保持会让电机在控制循环中持续加载且没有退出路径。
        requested_hold = (
            self._grasp_hold_timeout_sec
            if hold_timeout_sec is None
            else float(hold_timeout_sec)
        )
        hold_timeout = float(np.clip(requested_hold, 0.1, _G_GRASP_HOLD_TIMEOUT_MAX_SEC))
        timeout = max(float(close_timeout_sec), 0.1)
        min_time = max(float(min_close_time_sec), 0.0)
        velocity_limit = max(float(velocity_threshold), 0.0)
        min_closure = max(float(min_closure_distance_m), 0.0)
        start_position_m = self.gripper_position_m()
        start = time.monotonic()
        request = self._gripper_execution.start_grasp(
            close_force=close_effort, hold_force=hold_effort, timeout=timeout,
            min_time=min_time, velocity_limit=velocity_limit, min_closure=min_closure,
            hold_timeout=hold_timeout, start_position=start_position_m, now=start,
        )
        self._start_gripper_loop()
        return request

    def grasp_gripper(self, **kwargs):
        """同步兼容入口：提交后只等待周期执行器的完成事件。"""
        request = self.submit_grasp(**kwargs)
        request['event'].wait()
        return request['result']

    def _cached_gripper_feedback(self, *, now: float | None = None) -> _CachedMotorFeedback:
        # 采集路径使用反馈锁 -> 夹爪锁；查询保持相同顺序，避免与控制循环互锁。
        with self._feedback_component._feedback_lock, self._gripper_execution._gripper_lock:
            now = time.monotonic() if now is None else now
            sample = self._feedback_component._verified_feedback_by_label.get("gripper")
            failure = self._feedback_failure_reason(
                "gripper", error=self._gripper_feedback_error,
                updated=sample.observed_at if sample is not None else None, now=now,
            )
            feedback = self._cached_motor_feedback_locked(
                "gripper", failure_reason=failure, now=now,
            )
            if feedback.failure_reason is not None or feedback.values is None:
                return feedback
            coordinate_failure = self._gripper_coordinate_failure_reason_locked(
                position=feedback.values[0],
            )
            if coordinate_failure is not None:
                return _CachedMotorFeedback(
                    feedback.values, feedback.raw_status_code, coordinate_failure, feedback.sample,
                )
            return feedback

    def get_cached_gripper_sample(self):
        """返回夹爪同一快照的米制开口、速度、力矩、状态、序号与观测时间。

        不可用时开口为 NaN;保留旧样本身份和时间,不把错误查询当作新反馈。
        """
        with self._feedback_component._feedback_lock, self._gripper_execution._gripper_lock:
            feedback = self._cached_gripper_feedback()
            values = feedback.values or (self._gripper_execution._gripper_pos, self._gripper_execution._gripper_vel, self._gripper_execution._gripper_torque)
            distance = float("nan") if feedback.failure_reason is not None else float(
                np.clip(values[0] / _G_ANGLE_OPEN * _G_MAX_DIST_M, 0.0, _G_MAX_DIST_M)
            )
            sample = feedback.sample
            return (
                distance, values[1], values[2], feedback.status_code,
                sample.sequence if sample is not None else None,
                sample.observed_at if sample is not None else None,
            )

    def get_gripper_state(self) -> tuple[float, float, float, int]:
        """返回同一缓存快照的 (位置 rad, 速度 rad/s, 力矩 N.m, 状态码)。

        与关节共用可用性判定；可信反馈返回电机原码，否则派生 255。
        读取不刷新总线、不登记错误、不改变反馈时间。无样本时沿用最后缓存数值，
        这些回退数值必须结合 255 解读，不能视为当前有效反馈。
        """
        with self._feedback_component._feedback_lock:
            feedback = self._cached_gripper_feedback()
            values = feedback.values
            if values is None:
                with self._gripper_execution._gripper_lock:
                    values = (self._gripper_execution._gripper_pos, self._gripper_execution._gripper_vel, self._gripper_execution._gripper_torque)
            return (*values, feedback.status_code)

    def gripper_position_m(self) -> float:
        """把夹爪原始角度换算为开口距离(m,0 表示闭合）。"""
        with self._gripper_execution._gripper_lock:
            if self._gripper_coordinate_failure_reason_locked() is not None:
                # 坐标非法时返回 NaN，让上层无法把无效值当距离使用。
                return float("nan")
            distance = (self._gripper_execution._gripper_pos / _G_ANGLE_OPEN) * _G_MAX_DIST_M
        # 只有落在已验证反馈容差内的端点越界才会到这里被夹紧。
        return float(np.clip(distance, 0.0, _G_MAX_DIST_M))

    def gripper_reached_target(self) -> bool:
        """查询位置命令是否已到位且中性释放发送成功。"""
        with self._gripper_execution._gripper_lock:
            if self._gripper_execution._gripper_position_result == "succeeded":
                return True
            if self._gripper_execution._gripper_position_result == "failed" or not self._gripper_execution._gripper_active:
                return False
            feedback_failure = self._gripper_feedback_failure_reason_locked()
            if feedback_failure is not None:
                self._fail_active_gripper_command_locked(feedback_failure)
                return False
            return False

    @device_request(resource='gripper', ttl=.25)
    def send_gripper_motor_cmd(self, cmd, arbitration="reject") -> None:
        """低层夹爪直通命令（JointMotorCmd）：未给出的字段回落到反馈或配置增益。

        直通后夹爪状态机被置为 idle，避免与高层位置/抓取命令互相干扰。
        mode：0 = MIT，1 = 位置-速度，2 = 速度。
        """
        self._require_enabled()
        if self._gripper_mot is None or self._gripper_cfg is None:
            raise RuntimeError("gripper is not initialized")
        with self._gripper_execution._gripper_lock:
            feedback_failure = self._gripper_feedback_failure_reason_locked()
        if feedback_failure is not None:
            raise RuntimeError(feedback_failure)
        state = self._verified_feedback_sample("gripper").state
        pos = float(cmd.pos) if cmd.use_pos else float(state.pos if state is not None else 0.0)
        vel = float(cmd.vel) if cmd.use_vel else float(state.vel if state is not None else 0.0)
        kp = float(cmd.kp) if cmd.use_kp else float(self._gripper_cfg.kp)
        kd = float(cmd.kd) if cmd.use_kd else float(self._gripper_cfg.kd)
        tau = float(cmd.tau) if cmd.use_tau else 0.0
        vlim = float(cmd.vlim) if cmd.use_vlim else float(self._gripper_cfg.vlim)

        mode = int(cmd.mode)
        if mode not in (0, 1, 2):
            raise ValueError(f"unsupported JointMotorCmd mode: {cmd.mode}")
        if mode == 2 and not hasattr(self._gripper_mot, "send_vel"):
            raise RuntimeError("gripper does not support send_vel")
        self._validate_passthrough_values(pos, vel, kp, kd, tau, vlim,
            (_G_ANGLE_OPEN, 0.), self._gripper_cfg.vlim, _G_TAU_MAX, mode)
        with self._gripper_execution._gripper_lock:
            if (self._gripper_execution._gripper_active or self._gripper_execution._gripper_neutral_pending is not None) and arbitration != 'preempt':
                raise RuntimeError('gripper resource busy')
            # 先确认旧命令中性化，再交接透传；旧等待者不能释放新命令。
            if self._gripper_execution._gripper_active:
                self._queue_gripper_neutral_locked('passthrough preempt', marks_success=False)
            if self._gripper_execution._gripper_neutral_pending is not None:
                if not self._emit_pending_gripper_neutral_locked():
                    raise RuntimeError('gripper preempt neutral failed')
            self._gripper_execution.supersede_locked()
            self._gripper_execution._gripper_active = False
            self._gripper_execution._gripper_mode = "idle"
            self._gripper_execution._gripper_position_result = "failed"
            self._gripper_execution._gripper_neutral_pending = None
            try:
                from motorbridge import Mode
                self._gripper_mot.ensure_mode({0:Mode.MIT, 1:Mode.POS_VEL, 2:Mode.VEL}[mode], 1000)
                if mode == 0:
                    self._gripper_mot.send_mit(pos, vel, kp, kd, tau)
                elif mode == 1:
                    self._gripper_mot.send_pos_vel(pos, vlim)
                else:
                    self._gripper_mot.send_vel(vel)
            except Exception as exc:
                self._protective_disable_from_hardware_loop(str(exc), category='COMMAND')
                raise
            self._gripper_passthrough_until = time.monotonic() + .25

    def _start_pos_vel_loop(self, target: np.ndarray | None = None) -> None:
        """启动统一硬件控制循环（位置-速度跟踪），并把控制器标记为运行中。

        ``target`` 为空时以当前角度为目标（保持不动）。
        """
        if self.control_loop_active:
            return
        if target is None:
            self.hold_current_position()
        else:
            self._backend.set_target(target)
        self._backend.set_execution_running(True)
        try:
            self._backend.start_control_loop(self._endpos_hardware_tick)
        except Exception:
            self._backend.set_execution_running(False)
            raise

    def _expire_passthrough(self):
        """透传租约到期后交回保持/中性输出；没有消息也必须有本地终止路径。"""
        now = time.monotonic()
        expires = getattr(self, '_arm_passthrough_until', None)
        if expires is not None and now >= expires:
            self._arm_passthrough_until = None
            self.ensure_pos_vel_control()
            self.hold_current_position()
            self.set_state_machine('IDLE')
        expires = getattr(self, '_gripper_passthrough_until', None)
        if expires is not None and now >= expires:
            self._gripper_passthrough_until = None
            from motorbridge import Mode
            self._gripper_mot.ensure_mode(Mode.MIT, 1000)
            with self._gripper_execution._gripper_lock:
                self._queue_gripper_neutral_locked('passthrough lease expired', marks_success=False)
                if not self._emit_pending_gripper_neutral_locked():
                    raise RuntimeError('passthrough neutral release failed')

    def _runtime_feedback_tick(self):
        """设备事务等待和失能期间只采集；发布线程不再轮询总线。"""
        if self._lifecycle.connected and not self._feedback_component._feedback_force_refresh_active:
            self.refresh_feedback_if_due()
        if self._lifecycle.enabled and not self._backend._waiting:
            failure = self._arm_feedback_failure_reason()
            if failure is not None:
                self._protective_disable_from_hardware_loop(failure)
                return
            self._expire_passthrough()
        owner = getattr(self, '_arm_trajectory', None)
        task = None if owner is None else owner.task
        if (task is not None and task.result.state != 'stopping'
                and task.generation != self._backend.target_generation):
            owner.finish(task, 'aborted', 'execution canceled or superseded')

    def _endpos_hardware_tick(self, arm, dt: float) -> None:
        self._hardware_control_tick(arm, dt, self._backend.position_tick)

    def _gravity_hardware_tick(self, arm, dt: float) -> None:
        timing = getattr(self, '_gravity_timing', None)
        if timing is None:
            timing = self._gravity_timing = LoopTiming(float(self._backend.rate))
        started = time.monotonic()
        actual_dt = timing.begin(started)
        try:
            self._hardware_control_tick(arm, actual_dt, self._gravity_comp_tick)
        except GravityCommandError as exc:
            self._protective_disable_from_hardware_loop(str(exc), category='GRAVITY_COMMAND')
            return
        except Exception as exc:
            # 模型或计算异常不能直接终止唯一线程，让健康电机持续旧 MIT 目标。
            try:
                self._gravity_fault_hold(str(exc))
                q, qd = self._read_gravity_comp_feedback()
                self._emit_bounded_gravity_command(
                    q, qd, self._gravity_execution.hold_feedforward, actual_dt)
            except Exception as hold_error:
                self._protective_disable_from_hardware_loop(str(hold_error))
                return
        finally:
            timing.finish(time.monotonic())
        if not self._gravity_execution.active:
            return
        if timing.faulted:
            self._gravity_fault_hold('three consecutive control deadline misses')
        metrics = getattr(self, '_gravity_feedback_metrics', {})
        with self._feedback_component._feedback_lock:
            for name, sample in zip(self.joint_names, self._verified_feedback_samples(self.joint_names)):
                entry = metrics.setdefault(name, dict(sequence=sample.sequence, first=sample.observed_at,
                                                       last=sample.observed_at, updates=0))
                if sample.sequence != entry['sequence']:
                    entry.update(sequence=sample.sequence, last=sample.observed_at,
                                 updates=entry['updates'] + 1)
                elapsed = entry['last'] - entry['first']
                entry['effective_rate_hz'] = entry['updates'] / elapsed if elapsed > 0 else 0.
                entry['age_sec'] = max(0., time.monotonic() - sample.observed_at)
        self._gravity_feedback_metrics = metrics
        if started >= getattr(self, '_gravity_next_log', 0.):
            _LOG.info('gravity diagnostics: %s', self.gravity_compensation_diagnostics())
            self._gravity_next_log = started + 5.

    def _protective_disable_from_hardware_loop(self, reason: str, *, category: str = 'FEEDBACK') -> None:
        """每个故障会话只请求一次保护失能；确认失能前保留未知状态。

        先锁存，避免空闲采集或重入错误路径每周期重复写总线。即使发送失败也不
        自动重试；操作员可显式调用 disable 重试，只有新鲜失能反馈才解除锁存。
        """
        if not self._command_guard.begin_protection(reason, category):
            return
        message = str(reason)
        self._arm_passthrough_until = self._gripper_passthrough_until = None
        # 先停止输出。任务收尾出错也必须继续尝试失能，不能因锁存而漏发首个保护命令。
        preparation_errors = []
        def attempt(label, operation):
            try:
                operation()
            except Exception as exc:
                preparation_errors.append(f'{label}: {exc}')
        attempt('stop output', self._backend.request_loop_stop)
        attempt('invalidate requests', lambda: self._backend.requests.invalidate('device', 'arm', 'gripper'))
        try:
            with self._gripper_execution._gripper_lock:
                if self._gripper_execution._gripper_active:
                    self._fail_active_gripper_command_locked(message)
                if self._gripper_execution._gripper_neutral_pending is not None:
                    if not self._emit_pending_gripper_neutral_locked():
                        self._lifecycle.errors.append("FEEDBACK_PROTECTIVE_NEUTRAL_FAILED")
        except Exception as exc:
            preparation_errors.append(f'gripper stop: {exc}')
        owner = getattr(self, '_arm_trajectory', None)
        if owner is not None and owner.task is not None:
            attempt('finish arm task', lambda: owner.finish(owner.task, 'aborted', message))
        attempt('stop execution', lambda: self._backend.set_execution_running(False))
        attempt('cancel trajectory', self._backend.cancel_trajectory)
        self._gravity_execution.active = False
        self._gravity_execution.target = None
        self._gravity_execution.integral = None
        self._lifecycle.execution = "IDLE"
        self._lifecycle.set_state("DISABLING")
        code = f'{category}_PROTECTIVE_DISABLE'
        if code not in self._lifecycle.errors:
            self._lifecycle.errors.append(code)
        if preparation_errors:
            self._lifecycle.errors.append(f'{category}_PROTECTIVE_PREPARE_FAILED: ' + '; '.join(preparation_errors))

        controllers: list[object] = []
        disable_errors: list[str] = []
        try:
            for controller in self._backend.controllers():
                if all(controller is not existing for existing in controllers):
                    controllers.append(controller)
        except Exception as exc:
            disable_errors.append(f'controller lookup: {exc}')
        if self._gripper_ctrl is not None and all(
            self._gripper_ctrl is not controller for controller in controllers
        ):
            controllers.append(self._gripper_ctrl)
        for controller in controllers:
            disable_all = getattr(controller, "disable_all", None)
            if not callable(disable_all):
                disable_errors.append(f'{type(controller).__name__}: disable_all unavailable')
                continue
            try:
                disable_all()
            except Exception as exc:
                disable_errors.append(f"{type(controller).__name__}: {exc}")
        if disable_errors:
            self._lifecycle.errors.append(
                f"{category}_PROTECTIVE_DISABLE_FAILED: " + "; ".join(disable_errors)
            )
        _LOG.error(
            "hardware writer stopped and controller protective disable requested: %s",
            message,
        )

    def _hardware_control_tick(self, arm, dt: float, arm_callback) -> None:
        """机械臂命令、反馈批次与夹爪命令的唯一所有者。

        每周期顺序：刷新反馈 -> 检查反馈健康 -> 执行模式回调（位置跟踪或重力
        补偿）-> 推进夹爪状态机。若反馈刷新或回调因反馈类错误失败，则执行保护性
        失能并结束本次 tick；命令发送错误立即保护，其余异常交本地运行时登记和保护，
        以免把编程错误当成"反馈故障"而静默失能。
        """
        try:
            self.refresh_feedback_if_due()
        except Exception:
            feedback_failure = self._arm_feedback_failure_reason()
            if feedback_failure is None:
                raise
            self._protective_disable_from_hardware_loop(feedback_failure)
            return
        feedback_failure = self._arm_feedback_failure_reason()
        if feedback_failure is not None:
            self._protective_disable_from_hardware_loop(feedback_failure)
            return
        try:
            self._expire_passthrough()
            if getattr(self, '_arm_passthrough_until', None) is None:
                arm_callback(arm, dt)
        except CommandTransportError as exc:
            self._protective_disable_from_hardware_loop(str(exc), category="COMMAND")
            return
        except Exception:
            feedback_failure = self._arm_feedback_failure_reason()
            if feedback_failure is None:
                raise
            self._protective_disable_from_hardware_loop(feedback_failure)
            return
        if hasattr(self, '_arm_trajectory'):
            q, _, _, _, _, stamp = self.get_cached_joint_sample()
            hold = self._arm_trajectory.update(q, stamp)
            if hold is not None:
                self._backend.set_target(hold)
            task = self._arm_trajectory.task
            if task is not None and task.event.is_set() and self.state_machine == 'TRAJ_RUNNING':
                self.set_state_machine('IDLE')
        self._gripper_tick()
        if getattr(self,"_gripper_transport_error",None) is not None:
            self._protective_disable_from_hardware_loop(self._gripper_transport_error,category="COMMAND")

    def _stop_control_loop(self) -> None:
        self._backend.stop_control_loop()
        self._backend.set_execution_running(False)
        owner = getattr(self, '_arm_trajectory', None)
        if owner is not None and owner.task is not None:
            owner.finish(owner.task, 'aborted', 'control mode or lifecycle changed')

    def _gripper_safe_mit(
        self,
        pos: float,
        vel: float,
        kp: float,
        kd: float,
        tau_ff: float = 0.0,
        tau_limit: float = _G_TAU_MAX,
    ) -> None:
        """下发夹爪 MIT 命令，并保证总力矩不超限。

        位置命令先夹到 [_G_ANGLE_OPEN, 0]（不允许命令超过标称全开或越过
        闭合零点）。力矩处理是"先算位置项，再把总力矩夹到 +/-limit，最后反解出
        等效前馈"：这样即使 tau_ff 很大，实际输出也不会超过 limit。
        """
        if self._gripper_mot is None or self._gripper_ctrl is None:
            raise RuntimeError("gripper motor or controller unavailable")
        pos_cmd, vel, kp, kd, tau_safe = GripperControl.bounded_mit(
            pos, vel, kp, kd, tau_ff, tau_limit, self._gripper_execution._gripper_pos, self._gripper_execution._gripper_vel,
        )
        lock = getattr(self._gripper_ctrl, "bus_lock", None)
        try:
            if lock:
                with lock:
                    self._gripper_mot.send_mit(pos_cmd, vel, kp, kd, tau_safe)
            else:
                self._gripper_mot.send_mit(pos_cmd, vel, kp, kd, tau_safe)
        except Exception as exc:
            if isinstance(exc,CommandTransportError):
                self._gripper_transport_error = str(exc)
            raise RuntimeError(f"gripper MIT command failed: {exc}") from exc

    def _fail_active_gripper_command_locked(self, *args, **kwargs):
        return self._gripper_execution._fail_active_gripper_command_locked(*args, **kwargs)

    def _queue_gripper_neutral_locked(self, *args, **kwargs):
        return self._gripper_execution._queue_gripper_neutral_locked(*args, **kwargs)

    def _emit_pending_gripper_neutral_locked(self, *args, **kwargs):
        return self._gripper_execution._emit_pending_gripper_neutral_locked(*args, **kwargs)

    @device_request(resource='gripper', priority=0)
    def cancel_gripper_position_command(self, *args, **kwargs):
        canceled = self._gripper_execution.cancel_gripper_position_command(*args, **kwargs)
        if canceled:
            self._backend.requests.invalidate('gripper')
        return canceled

    @device_request(resource='gripper', priority=0)
    def _release_gripper_position_target(self, *args, **kwargs):
        return self._gripper_execution._release_gripper_position_target(*args, **kwargs)

    def _release_gripper_position_target_locked(self, *args, **kwargs):
        return self._gripper_execution._release_gripper_position_target_locked(*args, **kwargs)

    def _grasp_hold_expired_locked(self, *args, **kwargs):
        return self._gripper_execution._grasp_hold_expired_locked(*args, **kwargs)

    def _release_grasp_hold_locked(self, *args, **kwargs):
        return self._gripper_execution._release_grasp_hold_locked(*args, **kwargs)

    @device_request(resource='gripper', priority=0, invalidate=('gripper',))
    def release_grasp_hold(self, *args, **kwargs):
        return self._gripper_execution.release_grasp_hold(*args, **kwargs)

    @property
    def grasp_hold_release_reason(self):
        return self._gripper_execution.grasp_hold_release_reason

    def _gripper_tick(self, *args, **kwargs):
        return self._gripper_execution._gripper_tick(*args, **kwargs)

    def _start_gripper_loop(self) -> None:
        """夹爪命令的入口校验：夹爪没有独立循环，必须依赖统一硬件循环。"""
        if not self.control_loop_active:
            raise RuntimeError(
                "gripper command requires the unified hardware control loop"
            )

    def _stop_gripper_loop(self) -> None:
        # 使能/连接会话结束时使旧目标和等待者失效，防止重启循环后续发旧命令。
        # 此处不发送命令；调用方负责随后停止周期线程并执行生命周期设备操作。
        with self._gripper_execution._gripper_lock:
            self._gripper_execution.supersede_locked()
            self._gripper_execution._gripper_active = False
            self._gripper_execution._gripper_mode = "idle"
            self._gripper_execution._gripper_position_result = "failed"
            self._gripper_execution._gripper_neutral_pending = None
            self._gripper_execution._gripper_hold_deadline = None
            self._gripper_execution._gripper_target_deadline_monotonic = None
            self._gripper_loop_running = False
            self._gripper_passthrough_until = None

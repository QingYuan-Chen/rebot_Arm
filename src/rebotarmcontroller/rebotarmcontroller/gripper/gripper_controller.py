"""夹爪坐标, 零位验证, 目标接纳与安全发送.

职责与位置
    夹爪设备映射, 坐标, 反馈健康和请求准入; 执行状态由 GripperExecution 持有.

对外接口
    set_gripper_target 返回代次; 状态查询读缓存;
    set_zero 仅供生命周期在失能时调用并以置零后的连续新帧验证.

安全与并发约束
    七路共享一条总线; 设备操作经唯一运行时. 查询不伪造时间, 释放失败不能报成功.
    目标开口使用 m, 内部电机角使用 rad; max_effort / force 历史字段实际表示电机力矩 N·m.

文件结构(按定义顺序)
    GripperController: 夹爪坐标, 设备绑定和反馈准入所有者; 独立执行器持有目标代次, 抓取与释放状态.
"""

from __future__ import annotations
from ..contracts.component_ports import GripperFeedbackPort, ProtectionPort
from ..contracts.controller_values import GripperTarget
from ..runtime.command_guard import passthrough_decision
from ..hardware.sdk_driver import CommandTransportError
from .gripper_control import GripperControl
from .gripper_execution import GripperExecution
from ..contracts.controller_contracts import CachedMotorFeedback as _CachedMotorFeedback
from ..hardware.feedback_acquisition import _FeedbackValidationError
from .gripper_control import _G_ANGLE_OPEN
from .gripper_control import _G_KP_MOVE, _G_KD_MOVE
from .gripper_control import _G_CLOSED_FEEDBACK_TOL_RAD
from .gripper_control import _G_COORDINATE_TOL_RAD
from .gripper_control import _G_DEFAULT_FORCE
from .gripper_control import _G_LARGE_MOVE_MAX_TAU_NM
from .gripper_control import _G_MAX_DIST_M
from .gripper_control import _G_TAU_MAX
from .gripper_control import _G_VERIFIED_OPEN_LIMIT_M
from .gripper_control import _G_ZERO_VERIFY_SAMPLES
from .gripper_control import _G_ZERO_VERIFY_TIMEOUT_SEC
from ..runtime.device_requests import device_request
from ..runtime.stationary_evidence import StationaryEvidence
import numpy as np
import time
from ..controller_parameters import (
    _FEEDBACK_RETRY_INTERVAL_SEC, _LOG
)


class GripperController:
    """夹爪坐标, 设备绑定和反馈准入所有者; 独立执行器持有目标代次, 抓取与释放状态."""

    def clear_transport_error(self):
        """仅在显式使能成功后清除上个会话的发送错误."""
        self._gripper_transport_error = None

    def feedback_transaction(self):
        """采集锁内查询夹爪的事务上下文; 锁顺序固定为反馈 -> 夹爪."""
        return self._gripper_execution._gripper_lock

    def feedback_binding(self):
        """仅供采集/生命周期/保护装配的设备绑定, 不返回可变执行状态."""
        return self._gripper_ctrl, self._gripper_mot

    def feedback_values(self, state):
        """校验并拆分原始反馈数值, 供采集器使用同一夹爪规则."""
        return self._validated_gripper_feedback_values(state)

    def record_feedback(self, state, **options):
        """由采集事务交接已验证反馈与原观测时间, 不重新锚定时间."""
        return self._record_gripper_feedback(state, **options)

    def record_feedback_error(self, reason):
        """登记采集传来的夹爪错误, 保留分类与原因."""
        return self._record_gripper_feedback_error(reason)

    def cached_feedback(self, **options):
        """查询只读夹爪缓存及可用性; 不刷新总线或登记派生错误."""
        return self._cached_gripper_feedback(**options)

    def transport_error(self):
        """读取本会话发送错误, 供周期协调决定是否保护."""
        return self._gripper_transport_error

    def feedback_status(self):
        """生命周期使用的新帧状态查询; 无夹爪返回 None, 刷新失败抛异常."""
        return None if self._gripper_mot is None else int(self._refresh_gripper_feedback().status_code)

    def initialize(self):
        """使用构造时已经校验的配置建立夹爪绑定."""
        return self.init_gripper(str(self._gripper_cfg_path))

    def clear_passthrough(self):
        """清除夹爪透传租约标记, 不在此发送中性命令."""
        self._gripper_passthrough_until = None

    def reset_session(self):
        """连接前重置反馈时效及执行代次, 唤醒旧会话等待者."""
        self._gripper_feedback_updated_monotonic = None
        self._gripper_feedback_error = None
        self._stationary.reset()
        with self._gripper_execution._gripper_lock:
            self._gripper_execution.supersede_locked()
            self._gripper_execution._gripper_active = False
            self._gripper_execution._gripper_mode = "idle"
            self._gripper_execution._gripper_neutral_pending = None
            self._gripper_execution._gripper_target_deadline_monotonic = None
            self._gripper_transport_error = None

    def expire_passthrough(self, now):
        """空载或透传断流均通过中性输出结束, 失败交给周期保护."""
        if self._gripper_passthrough_until is not None and now >= self._gripper_passthrough_until:
            self.clear_passthrough()
            from motorbridge import Mode
            self._gripper_mot.ensure_mode(Mode.MIT, 1000)
            with self._gripper_execution._gripper_lock:
                self._queue_gripper_neutral_locked('passthrough lease expired', marks_success=False)
                if not self._emit_pending_gripper_neutral_locked():
                    raise RuntimeError('passthrough neutral release failed')

    def prepare_protection(self, message):
        """先终止本夹爪任务; 中性输出失败仍须继续执行总线保护失能."""
        with self._gripper_execution._gripper_lock:
            if self._gripper_execution._gripper_active:
                self._fail_active_gripper_command_locked(message)
            if self._gripper_execution._gripper_neutral_pending is not None:
                if not self._emit_pending_gripper_neutral_locked():
                    self._lifecycle.errors.append("FEEDBACK_PROTECTIVE_NEUTRAL_FAILED")

    def __init__(self, runtime, lifecycle, settings, configuration, gripper_config):
        """保存共享运行时和配置, 建立坐标/健康标记并装配独立夹爪执行器; 不连接设备."""
        self._backend = runtime
        self._lifecycle = lifecycle
        self.settings = settings
        self._bound = False
        self._configuration = configuration
        self._gripper_cfg_path = gripper_config
        self._gripper_cfg = None
        self._gripper_mot = self._gripper_ctrl = None
        self._gripper_feedback_updated_monotonic = None
        self._gripper_feedback_error = self._gripper_zero_error = None
        self._gripper_transport_error = None
        self._gripper_passthrough_until = None
        self._gripper_loop_running = False
        self._gripper_execution = self._assemble_execution()
        self._stationary = StationaryEvidence(joints=1)

    def bind(self, *, feedback: GripperFeedbackPort, protection: ProtectionPort):
        """装配阶段一次注入明确组件接口; 运行后不替换依赖."""
        if self._bound:
            raise RuntimeError("component already bound")
        self.feedback = feedback
        self.protection = protection
        self._bound = True

    def _assemble_execution(self):
        """构造时一次装配组件, 运行期间不惰性补建状态."""
        return GripperExecution(
            motor=lambda: self._gripper_mot,
            verified=lambda: self.feedback.verified_sample('gripper').state,
            feedback_failure=lambda: self._gripper_feedback_failure_reason_locked(),
            record_feedback_error=lambda message: self._record_gripper_feedback_error(message),
            validate=lambda state: self._validated_gripper_feedback_values(state),
            feedback_age=lambda: self._gripper_feedback_age_sec(),
            feedback_updated=lambda: self._gripper_feedback_updated_monotonic,
            speed=lambda: self.settings.gripper_position_max_speed_rad_s,
            send=lambda *args, **kwargs: self._gripper_safe_mit(*args, **kwargs),
        )

    @property
    def has_gripper(self) -> bool:
        """查询是否已绑定夹爪设备, 不代表反馈健康或已经使能."""
        return self._gripper_mot is not None

    def preparation_snapshot(self):
        """只复制新鲜、连续静止且空闲的夹爪状态，不能在准备时停止原有夹爪任务。"""
        sample = self.feedback.verified_sample('gripper')
        with self._gripper_execution._gripper_lock:
            if (self._gripper_execution._gripper_active
                    or self._gripper_execution._gripper_neutral_pending is not None
                    or self._gripper_passthrough_until is not None
                    or sample.state.status_code != 1
                    or not self._stationary.confirmed(time.monotonic())):
                raise RuntimeError('fresh stationary idle gripper required')
            width = float(np.clip(sample.state.pos / _G_ANGLE_OPEN * _G_MAX_DIST_M, 0., _G_MAX_DIST_M))
            # 碰撞模型使用左右手指的米制坐标，原始电机角保留在快照中供诊断。
            return dict(positions=(width/2, -width/2), raw_position=sample.state.pos,
                        generation=self._gripper_execution._generation)

    @property
    def gripper_active(self) -> bool:
        """在执行锁内查询位置, 抓取或保持是否活动."""
        with self._gripper_execution._gripper_lock:
            return bool(self._gripper_execution._gripper_active)

    def start_reserved_position(self, token, position_m, max_effort):
        """仅供设备事务内的组合所有者接纳；令牌身份及目标均须与原预约一致。"""
        self._backend.assert_writer()
        if (self._backend.execution.reservation is not token or token.collision_policy != 1
                or token.gripper_command != (position_m, max_effort)
                or self._gripper_execution._generation != token.gripper_generation):
            raise RuntimeError('independent gripper command does not own the combination reservation')
        return self._accept_position_target(position_m, max_effort)

    @property
    def gripper_mode(self) -> str:
        """在执行锁内读取夹爪业务模式, 区别于电机协议模式."""
        with self._gripper_execution._gripper_lock:
            return str(self._gripper_execution._gripper_mode)

    def coordinate_failure(self):
        """查询夹爪坐标或零位锁存错误, 无设备时返回 None."""
        if self._gripper_mot is not None:
            with self._gripper_execution._gripper_lock:
                return self._gripper_coordinate_failure_reason_locked()
        return None

    def prepare_enable(self):
        """在生命周期显式使能前确保夹爪处于 MIT 模式, 不自行使能."""
        if self._gripper_mot is not None:
            from motorbridge import Mode
            self._gripper_mot.ensure_mode(Mode.MIT, 1000)

    @staticmethod
    def _validated_gripper_feedback_values(state) -> tuple[float, float, float, int]:
        """拆出夹爪反馈的 (位置 rad, 速度 rad/s, 力矩 N.m, 状态码), 并做有限性检查."""
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
        """校验有限数值并在执行锁内更新反馈镜像, 保留原时刻并登记越界原因."""
        position, velocity, torque, status = self._validated_gripper_feedback_values(state)
        timestamp = time.monotonic() if observed_at is None else float(observed_at)
        with self._gripper_execution._gripper_lock:
            recovered = self._gripper_feedback_error is not None
            self._gripper_execution._gripper_pos = position
            self._gripper_execution._gripper_vel = velocity
            self._gripper_execution._gripper_torque = torque
            self._gripper_feedback_updated_monotonic = timestamp
            # 有限的原始坐标仍保留供显式置零使用, 健康判定和控制入口必须拒绝越限.
            coordinate_error = self._gripper_coordinate_failure_reason_locked(
                position=position, check_zero=False,
            )
            if coordinate_error is not None:
                self._record_gripper_feedback_error(coordinate_error)
                return
            self._gripper_feedback_error = None
            if (status != 1 or self._gripper_execution._gripper_active
                    or abs(velocity) > .05):
                self._stationary.reset((position,))
            self._stationary.observe((position,), (velocity,), timestamp, time.monotonic())
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
        """持执行锁保存当前反馈错误, 只在原因变化时记录日志."""
        message = str(reason)
        with self._gripper_execution._gripper_lock:
            changed = message != self._gripper_feedback_error
            self._gripper_feedback_error = message
        if changed:
            _LOG.error("gripper feedback error: %s", message)

    def _gripper_feedback_age_sec(self, *, now: float | None = None) -> float:
        """用单调时钟计算反馈年龄 s, 尚无样本时返回无穷大."""
        with self._gripper_execution._gripper_lock:
            updated = self._gripper_feedback_updated_monotonic
        if updated is None:
            return float("inf")
        current = time.monotonic() if now is None else float(now)
        return max(current - updated, 0.0)

    def _gripper_feedback_failure_reason_locked(self, *, now: float | None = None) -> str | None:
        """夹爪反馈失效原因(调用方需持有 ``_gripper_lock``).

        依次检查: 显式错误 -> 数据过期 -> 坐标合法性(置零错误或原始角越界).
        """
        return self.feedback.failure_reason(
            "gripper", error=self._gripper_feedback_error,
            updated=self._gripper_feedback_updated_monotonic, now=now,
        ) or self._gripper_coordinate_failure_reason_locked()

    def _gripper_coordinate_failure_reason_locked(
        self, *, position: float | None = None, check_zero: bool = True
    ) -> str | None:
        """夹爪原始角度坐标的合法性检查(调用方需持有 ``_gripper_lock``).

        有效区间是 [_G_ANGLE_OPEN - 量化容差, _G_CLOSED_FEEDBACK_TOL_RAD],
        即"从全开到闭合零点附近". 越界说明零点未标定或编码器异常, 必须拒绝
        使用该反馈(例如合拢方向越界会让力矩变成前馈而持续顶紧).
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
        """同步强制刷新并返回夹爪的已验证状态对象(拿不到新帧即报错)."""
        if self._gripper_mot is None or self._gripper_ctrl is None:
            raise RuntimeError("gripper feedback unavailable: motor/controller not initialized")
        self.feedback.refresh_feedback_if_due(force=True)
        return self.feedback.verified_sample("gripper").state

    def validate_status(self, expected_status: int) -> None:
        """同步强制刷新夹爪并核对期望状态码; 不能用旧帧确认使能或失能."""
        if self._gripper_mot is None:
            return
        state = self._refresh_gripper_feedback()
        status = int(state.status_code)
        if status != expected_status:
            raise RuntimeError(
                f"gripper status_code={status}, expected {expected_status}"
            )

    def set_zero(self) -> bool:
        """夹爪置零并验证: 要求在超时内连续若干帧都接近零位.

        验证必须基于"置零之后新到"的帧(``_refresh_gripper_feedback`` 会重建
        接收序号基线, 置零前的缓存样本无法满足). 校验失败时保留
        ``_gripper_zero_error``: 仅凭"有新帧"不能证明标定成功, 必须等到下一次
        显式置零验证通过才清除.
        """
        if self._gripper_mot is None:
            raise RuntimeError("gripper is not initialized")
        self.stop_loop()
        with self.feedback.transaction():
            self.validate_status(expected_status=0)
            with self._gripper_execution._gripper_lock:
                self._gripper_zero_error = "gripper zero verification pending"
            try:
                self._gripper_mot.set_zero_position()
                deadline = time.monotonic() + _G_ZERO_VERIFY_TIMEOUT_SEC
                consecutive = 0
                position = float("nan")
                while time.monotonic() < deadline:
                    # 强制刷新会重建接收序号基线; 置零前缓存下来的样本无法满足
                    # 本次验证请求.
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
                    # 必须是"连续"多帧接近零位, 单帧偶然命中不算通过.
                    consecutive = (
                        consecutive + 1 if abs(position) <= _G_COORDINATE_TOL_RAD else 0
                    )
                    if consecutive >= _G_ZERO_VERIFY_SAMPLES:
                        with self._gripper_execution._gripper_lock:
                            self._gripper_zero_error = None
                        self._lifecycle.enabled = False
                        self._lifecycle.set_execution_state("IDLE")
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
                    # 仅凭新帧并不能证明标定失败已消除; 在显式置零验证通过之前
                    # 一直保留该失败标记.
                    self._gripper_zero_error = message
                raise RuntimeError(message) from exc

    def init_gripper(self, cfg_path: str) -> None:
        """按配置初始化夹爪电机, 并挂到机械臂同一控制器/总线上.

        夹爪必须与机械臂共用同一个控制器对象(同一串口), 否则无法保证总线
        事务的原子性. 本方法只绑定已注册设备; 首次新鲜反馈由生命周期连接事务验证,
        位置/抓取入口要求统一硬件控制循环已运行, 不另建夹爪线程.
        """
        gc=self._configuration.gripper
        self._gripper_cfg=gc
        self._gripper_mot,self._gripper_ctrl=self._backend.add_gripper(gc)

    @device_request
    def set_gripper_target(self, position_m: float, max_effort: float = 0.0) -> None:
        """下发夹爪位置目标(异步): 只登记目标, 实际下发由 统一硬件循环完成.

        ``position_m``: 0 表示完全闭合, 正值为张开距离(m), 会被夹到
        [0, _G_VERIFIED_OPEN_LIMIT_M]; ``max_effort`` <= 0 时使用默认力矩
        ``_G_DEFAULT_FORCE``, 否则夹到 [0.05, 位置力矩上限].
        超时按"角度行程 / 最大速度 + 余量"估算, 供 ``wait_gripper_target`` 使用.
        """
        self._lifecycle.require_enabled()
        if self._backend.arm_resource_reserved:
            raise RuntimeError('gripper reserved by prepared arm trajectory')
        return self._accept_position_target(position_m, max_effort)

    def _accept_position_target(self, position_m, max_effort):
        """两类入口共用原位置逻辑；调用者已核验预约，实际输出仍由统一周期完成。"""
        self._backend.assert_writer()
        self._lifecycle.require_enabled()
        if not self._backend.control_loop_active:
            raise RuntimeError(
                "gripper command requires the unified hardware control loop"
            )
        if self._gripper_mot is None:
            raise RuntimeError("gripper is not initialized")
        with self._gripper_execution._gripper_lock:
            feedback_failure = self._gripper_feedback_failure_reason_locked()
        if feedback_failure is not None:
            raise RuntimeError(feedback_failure)
        state = self.feedback.verified_sample("gripper").state
        start_angle, _velocity, _torque, status = self._validated_gripper_feedback_values(state)
        if status != 1:
            raise RuntimeError(f"gripper status_code={status}, expected 1 before position command")
        if self._gripper_passthrough_until is not None:
            raise RuntimeError('gripper resource owned by passthrough')
        if not np.isfinite([position_m, max_effort]).all():
            raise ValueError('gripper target and effort must be finite')
        distance = float(np.clip(position_m, 0.0, _G_VERIFIED_OPEN_LIMIT_M))
        # 使用完整标称行程: 90 mm 对应 -5 rad, 不再额外裁到 -4.9 rad.
        target = (distance / _G_MAX_DIST_M) * _G_ANGLE_OPEN
        effort = _G_DEFAULT_FORCE if max_effort <= 0.0 else float(max_effort)
        now = time.monotonic()
        target_timeout_sec = (
            abs(target - start_angle) / self.settings.gripper_position_max_speed_rad_s
            + self.settings.gripper_position_timeout_margin_sec
        )
        applied_effort = float(np.clip(effort, 0.05, self.settings.gripper_position_torque_cap_nm))
        # 接纳时同时固定代次与总超时, ROS 等待者只能查询/取消这个代次, 不能改写周期目标.
        generation = self._gripper_execution.accept_position(
            GripperTarget(start_angle, target, applied_effort, target_timeout_sec), now)
        feedback_updated = self._gripper_feedback_updated_monotonic
        _LOG.info(
            "gripper position command requested=%.6fm clamped=%.6fm "
            "start=%.6frad goal=%.6frad effort=%.6fNm speed=%.6frad/s "
            "timeout=%.3fs feedback_updated=%s",
            float(position_m),
            distance,
            start_angle,
            target,
            applied_effort,
            self.settings.gripper_position_max_speed_rad_s,
            target_timeout_sec,
            feedback_updated,
        )
        self._start_gripper_loop()
        return generation

    def gripper_target_timeout_sec(self) -> float:
        """在执行锁内读取当前目标的总预算 s, 不重建超时起点."""
        with self._gripper_execution._gripper_lock:
            return float(self._gripper_execution._gripper_target_timeout_sec)

    @property
    def gripper_command_error(self) -> str | None:
        """在执行锁内读取当前命令失败原因, 不修改执行结果."""
        with self._gripper_execution._gripper_lock:
            return self._gripper_execution._gripper_command_error

    @property
    def gripper_feedback_error(self) -> str | None:
        """当前反馈不可用原因; 查询不写入或锁存派生错误."""
        return self._cached_gripper_feedback().failure_reason

    def gripper_execution_status(self, generation):
        """返回本目标的状态, 旧代次永远不能读成新目标的成功."""
        result = self._gripper_execution.snapshot(generation)
        return result.state, result.message

    def wait_gripper_target(self, timeout=None, generation=None) -> bool:
        """兼容等待入口; 只读快照, 显式超时通过队列取消自己的代次."""
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
        """同步版位置命令: 下发并等待到位, 返回 (是否到位, 实际位置 m)."""
        generation = self.set_gripper_target(position_m, max_effort)
        reached = self.wait_gripper_target(generation=generation)
        return reached, self.gripper_position_m()



    def _cached_gripper_feedback(self, *, now: float | None = None) -> _CachedMotorFeedback:
        """按反馈 -> 夹爪锁顺序取得同一快照, 叠加坐标错误而不改写样本时间."""
        # 采集路径使用反馈锁 -> 夹爪锁; 查询保持相同顺序, 避免与控制循环互锁.
        with self.feedback.transaction(), self._gripper_execution._gripper_lock:
            now = time.monotonic() if now is None else now
            sample = self.feedback.cached_motor("gripper", now=now).sample
            failure = self.feedback.failure_reason(
                "gripper", error=self._gripper_feedback_error,
                updated=sample.observed_at if sample is not None else None, now=now,
            )
            feedback = self.feedback.cached_motor(
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
        """返回夹爪同一快照的米制开口, 速度, 力矩, 状态, 序号与观测时间.

        不可用时开口为 NaN; 保留旧样本身份和时间, 不把错误查询当作新反馈.
        """
        with self.feedback.transaction(), self._gripper_execution._gripper_lock:
            feedback = self._cached_gripper_feedback()
            values = feedback.values or (float("nan"), float("nan"), float("nan"))
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
        """返回同一缓存快照的 (位置 rad, 速度 rad/s, 力矩 N.m, 状态码).

        与关节共用可用性判定; 可信反馈返回电机原码, 否则派生 255.
        读取不刷新总线, 不登记错误, 不改变反馈时间. 无可用样本时三个数值返回 NaN,
        调用方必须结合 255 判断其不可用, 不能把控制镜像当作反馈.
        """
        with self.feedback.transaction():
            feedback = self._cached_gripper_feedback()
            values = feedback.values or (float("nan"), float("nan"), float("nan"))
            return (*values, feedback.status_code)

    def gripper_position_m(self) -> float:
        """把夹爪原始角度换算为开口距离(m, 0 表示闭合)."""
        with self._gripper_execution._gripper_lock:
            if self._gripper_coordinate_failure_reason_locked() is not None:
                # 坐标非法时返回 NaN, 让上层无法把无效值当距离使用.
                return float("nan")
            distance = (self._gripper_execution._gripper_pos / _G_ANGLE_OPEN) * _G_MAX_DIST_M
        # 只有落在已验证反馈容差内的端点越界才会到这里被夹紧.
        return float(np.clip(distance, 0.0, _G_MAX_DIST_M))

    def gripper_reached_target(self) -> bool:
        """查询位置命令是否已到位且中性释放发送成功."""
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

    @device_request
    def send_gripper_motor_cmd(self, cmd, arbitration="reject") -> None:
        """低层夹爪直通命令(JointMotorCmd): 未给出的字段回落到反馈或配置增益.

        直通后夹爪状态机被置为 idle, 避免与高层位置命令互相干扰.
        mode: 0 = MIT, 1 = 位置-速度, 2 = 速度.
        """
        self._lifecycle.require_enabled()
        if self._backend.arm_resource_reserved:
            raise RuntimeError('gripper reserved by prepared arm trajectory')
        if self._gripper_mot is None or self._gripper_cfg is None:
            raise RuntimeError("gripper is not initialized")
        with self._gripper_execution._gripper_lock:
            feedback_failure = self._gripper_feedback_failure_reason_locked()
        if feedback_failure is not None:
            raise RuntimeError(feedback_failure)
        state = self.feedback.verified_sample("gripper").state
        pos = float(cmd.pos) if cmd.use_pos else float(state.pos)
        vel = float(cmd.vel) if cmd.use_vel else float(state.vel)
        kp = float(cmd.kp) if cmd.use_kp else float(self._gripper_cfg.kp)
        kd = float(cmd.kd) if cmd.use_kd else float(self._gripper_cfg.kd)
        tau = float(cmd.tau) if cmd.use_tau else 0.0
        vlim = float(cmd.vlim) if cmd.use_vlim else float(self._gripper_cfg.vlim)

        mode = int(cmd.mode)
        if mode not in (0, 1, 2):
            raise ValueError(f"unsupported JointMotorCmd mode: {cmd.mode}")
        passthrough_decision(pos, vel, kp, kd, tau, vlim,
            (_G_ANGLE_OPEN, 0.), self._gripper_cfg.vlim, _G_TAU_MAX, mode).require_allowed()
        with self._gripper_execution._gripper_lock:
            if (self._gripper_execution._gripper_active or self._gripper_execution._gripper_neutral_pending is not None) and arbitration != 'preempt':
                raise RuntimeError('gripper resource busy')
            # 先确认旧命令中性化, 再交接透传; 旧等待者不能释放新命令.
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
                self.protection.protect(str(exc), category='COMMAND')
                raise
            self._gripper_passthrough_until = time.monotonic() + .25

    def _gripper_safe_mit(
        self,
        pos: float,
        vel: float,
        kp: float,
        kd: float,
        tau_ff: float = 0.0,
        tau_limit: float = _G_TAU_MAX,
    ) -> None:
        """按当前反馈估算并约束夹爪 MIT 总力矩, 再发送命令.

        位置命令先夹到 [_G_ANGLE_OPEN, 0](不允许命令超过标称全开或越过
        闭合零点). 力矩处理是"先算位置项, 再把总力矩夹到 +/-limit, 最后反解出
        等效前馈": 当前反馈下的估算总力矩受 limit 约束, 反馈间隔内实际输出仍需验证.
        """
        if self._gripper_mot is None or self._gripper_ctrl is None:
            raise RuntimeError("gripper motor or controller unavailable")
        pos_cmd, vel, kp, kd, tau_safe = GripperControl.bounded_mit(
            pos, vel, kp, kd, tau_ff, tau_limit, self._gripper_execution._gripper_pos, self._gripper_execution._gripper_vel,
        )
        try:
            with self._gripper_ctrl.bus_lock:
                self._gripper_mot.send_mit(pos_cmd, vel, kp, kd, tau_safe)
        except Exception as exc:
            if isinstance(exc,CommandTransportError):
                self._gripper_transport_error = str(exc)
                raise
            raise RuntimeError(f"gripper MIT command failed: {exc}") from exc

    def _fail_active_gripper_command_locked(self, *args, **kwargs):
        """将活动目标判为失败并排队中性释放; 调用方持夹爪执行锁."""
        return self._gripper_execution._fail_active_gripper_command_locked(*args, **kwargs)

    def _queue_gripper_neutral_locked(self, *args, **kwargs):
        """委托执行器排队中性输出, 不在排队阶段宣告释放成功."""
        return self._gripper_execution._queue_gripper_neutral_locked(*args, **kwargs)

    def _emit_pending_gripper_neutral_locked(self, *args, **kwargs):
        """仅在设备线程和执行锁内发送待决中性输出, 返回真实发送结果."""
        return self._gripper_execution._emit_pending_gripper_neutral_locked(*args, **kwargs)

    @device_request
    def cancel_gripper_position_command(self, *args, **kwargs):
        """只取消匹配代次的位置目标, 成功撤销后作废排队夹爪请求."""
        canceled = self._gripper_execution.cancel_gripper_position_command(*args, **kwargs)
        if canceled:
            self._backend.requests.invalidate('gripper')
        return canceled

    @device_request
    def _release_gripper_position_target(self, *args, **kwargs):
        """请求结束匹配目标并排队中性释放, 设备线程入队策略仍适用."""
        return self._gripper_execution._release_gripper_position_target(*args, **kwargs)

    def _release_gripper_position_target_locked(self, *args, **kwargs):
        """持锁复用目标所有权与到位检查, 不允许旧收尾释放接替命令."""
        return self._gripper_execution._release_gripper_position_target_locked(*args, **kwargs)





    def _gripper_tick(self, *args, **kwargs):
        """委托独立执行器推进一拍夹爪状态, 调用方必须是统一设备周期."""
        return self._gripper_execution._gripper_tick(*args, **kwargs)

    def tick(self):
        """统一设备周期推进夹爪; 不允许 ROS 等待者调用设备发送."""
        return self._gripper_tick()

    def _start_gripper_loop(self) -> None:
        """夹爪命令的入口校验: 夹爪没有独立循环, 必须依赖统一硬件循环."""
        if not self._backend.control_loop_active:
            raise RuntimeError(
                "gripper command requires the unified hardware control loop"
            )

    def stop_loop(self) -> None:
        """作废本会话夹爪任务及等待者, 清待发状态; 不发送设备命令, 生命周期负责后续操作."""
        # 使能/连接会话结束时使旧目标和等待者失效, 防止重启循环后续发旧命令.
        # 此处不发送命令; 调用方负责随后停止周期线程并执行生命周期设备操作.
        with self._gripper_execution._gripper_lock:
            self._gripper_execution.supersede_locked()
            self._gripper_execution._gripper_active = False
            self._gripper_execution._gripper_mode = "idle"
            self._gripper_execution._gripper_position_result = "failed"
            self._gripper_execution._gripper_neutral_pending = None
            self._gripper_execution._gripper_target_deadline_monotonic = None
            self._gripper_loop_running = False
            self._gripper_passthrough_until = None

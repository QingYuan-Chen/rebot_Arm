"""夹爪坐标、零位验证、目标接纳与安全发送。

对外接口：set_gripper_target/submit_grasp 接纳请求；查询只读缓存；零位必须
在失能时以连续新帧验证。GripperExecution 在唯一设备周期中推进并确认中性释放。
约束：本组件拥有夹爪设备映射、坐标及反馈错误，执行状态归 GripperExecution。
反馈锁先于执行锁；设备写入经同一运行时，释放发送失败不得报告成功。
"""
from __future__ import annotations
from .controller_values import GripperTarget
from .sdk_driver import CommandTransportError
from .gripper_control import GripperControl
from .gripper_execution import GripperExecution
from .controller_contracts import CachedMotorFeedback as _CachedMotorFeedback
from .feedback_acquisition import _FeedbackValidationError
from .gripper_control import _G_ANGLE_OPEN
from .gripper_control import _G_CLOSED_FEEDBACK_TOL_RAD
from .gripper_control import _G_COORDINATE_TOL_RAD
from .gripper_control import _G_DEFAULT_FORCE
from .gripper_control import _G_GRASP_CLOSE_FORCE_DEFAULT
from .gripper_control import _G_GRASP_CLOSE_FORCE_MAX
from .gripper_control import _G_GRASP_HOLD_FORCE_DEFAULT
from .gripper_control import _G_GRASP_HOLD_TIMEOUT_MAX_SEC
from .gripper_control import _G_GRASP_MIN_CLOSE_TIME
from .gripper_control import _G_GRASP_MIN_CLOSURE_M
from .gripper_control import _G_GRASP_TIMEOUT
from .gripper_control import _G_GRASP_VEL_THRESHOLD
from .gripper_control import _G_LARGE_MOVE_MAX_TAU_NM
from .gripper_control import _G_MAX_DIST_M
from .gripper_control import _G_TAU_MAX
from .gripper_control import _G_VERIFIED_OPEN_LIMIT_M
from .gripper_control import _G_ZERO_VERIFY_SAMPLES
from .gripper_control import _G_ZERO_VERIFY_TIMEOUT_SEC
from .device_requests import device_request
import numpy as np
import time
from .controller_parameters import (
    _FEEDBACK_RETRY_INTERVAL_SEC, _LOG
)


class GripperController:
    """拥有本职责的状态和事务；ROS 门面仅提交请求或读取结果。"""

    def clear_transport_error(self):
        """仅在显式使能成功后清除上个会话的发送错误。"""
        self._gripper_transport_error = None

    def clear_passthrough(self):
        self._gripper_passthrough_until = None

    def reset_session(self):
        """连接前重置反馈时效及执行代次，唤醒旧会话等待者。"""
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

    def expire_passthrough(self, now):
        """空载或透传断流均通过中性输出结束，失败交给周期保护。"""
        if self._gripper_passthrough_until is not None and now >= self._gripper_passthrough_until:
            self.clear_passthrough()
            from motorbridge import Mode
            self._gripper_mot.ensure_mode(Mode.MIT, 1000)
            with self._gripper_execution._gripper_lock:
                self._queue_gripper_neutral_locked('passthrough lease expired', marks_success=False)
                if not self._emit_pending_gripper_neutral_locked():
                    raise RuntimeError('passthrough neutral release failed')

    def prepare_protection(self, message):
        """先终止本夹爪任务；中性输出失败仍须继续执行总线保护失能。"""
        with self._gripper_execution._gripper_lock:
            if self._gripper_execution._gripper_active:
                self._fail_active_gripper_command_locked(message)
            if self._gripper_execution._gripper_neutral_pending is not None:
                if not self._emit_pending_gripper_neutral_locked():
                    self._lifecycle.errors.append("FEEDBACK_PROTECTIVE_NEUTRAL_FAILED")

    def __init__(self, runtime, lifecycle, settings, configuration, gripper_config):
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

    def bind(self, *, arm, feedback, protection):
        """装配阶段一次注入明确组件接口；运行后不替换依赖。"""
        if self._bound:
            raise RuntimeError("component already bound")
        self.arm = arm
        self.feedback = feedback
        self.protection = protection
        self._bound = True

    def _assemble_execution(self):
        """构造时一次装配组件，运行期间不惰性补建状态。"""
        return GripperExecution(
            motor=lambda: self._gripper_mot,
            verified=lambda: self.feedback._verified_feedback_sample('gripper').state,
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
        return self._gripper_mot is not None


    @property
    def gripper_active(self) -> bool:
        with self._gripper_execution._gripper_lock:
            return bool(self._gripper_execution._gripper_active)


    @property
    def gripper_mode(self) -> str:
        with self._gripper_execution._gripper_lock:
            return str(self._gripper_execution._gripper_mode)


    def _lifecycle_gripper_coordinate_failure(self):
        if self._gripper_mot is not None:
            with self._gripper_execution._gripper_lock:
                return self._gripper_coordinate_failure_reason_locked()
        return None


    def _prepare_gripper_enable(self):
        if self._gripper_mot is not None:
            from motorbridge import Mode
            self._gripper_mot.ensure_mode(Mode.MIT, 1000)


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
        return self.feedback._feedback_failure_reason(
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
        self.feedback.refresh_feedback_if_due(force=True)
        return self.feedback._verified_feedback_sample("gripper").state


    def _validated_gripper_status(self, expected_status: int) -> None:
        if self._gripper_mot is None:
            return
        state = self._refresh_gripper_feedback()
        status = int(state.status_code)
        if status != expected_status:
            raise RuntimeError(
                f"gripper status_code={status}, expected {expected_status}"
            )


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
        with self.feedback._feedback_component._feedback_lock:
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
                    # 仅凭新帧并不能证明标定失败已消除；在显式置零验证通过之前
                    # 一直保留该失败标记。
                    self._gripper_zero_error = message
                raise RuntimeError(message) from exc


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
        state = self.feedback._verified_feedback_sample("gripper").state
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
            abs(target - start_angle) / self.settings.gripper_position_max_speed_rad_s
            + self.settings.gripper_position_timeout_margin_sec
        )
        applied_effort = float(np.clip(effort, 0.05, self.settings.gripper_position_torque_cap_nm))
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
        result = self._gripper_execution.snapshot(generation)
        return result.state, result.message


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
        self._lifecycle.require_enabled()
        if not self._backend.control_loop_active:
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
            self.settings.grasp_hold_timeout_sec
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
        with self.feedback._feedback_component._feedback_lock, self._gripper_execution._gripper_lock:
            now = time.monotonic() if now is None else now
            sample = self.feedback._feedback_component._verified_feedback_by_label.get("gripper")
            failure = self.feedback._feedback_failure_reason(
                "gripper", error=self._gripper_feedback_error,
                updated=sample.observed_at if sample is not None else None, now=now,
            )
            feedback = self.feedback._cached_motor_feedback_locked(
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
        with self.feedback._feedback_component._feedback_lock, self._gripper_execution._gripper_lock:
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
        with self.feedback._feedback_component._feedback_lock:
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
        self._lifecycle.require_enabled()
        if self._gripper_mot is None or self._gripper_cfg is None:
            raise RuntimeError("gripper is not initialized")
        with self._gripper_execution._gripper_lock:
            feedback_failure = self._gripper_feedback_failure_reason_locked()
        if feedback_failure is not None:
            raise RuntimeError(feedback_failure)
        state = self.feedback._verified_feedback_sample("gripper").state
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
        self.arm._validate_passthrough_values(pos, vel, kp, kd, tau, vlim,
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
                self.protection._protective_disable_from_hardware_loop(str(exc), category='COMMAND')
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
        if not self._backend.control_loop_active:
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

"""反馈采集、已验证缓存和只读健康判断。

``FeedbackAcquisition`` 通过注入的分组、时钟相关参数和夹爪回调采集反馈，
按每电机序号保存不可变样本，并维护请求基线、截止时间和当前错误。
``refresh_feedback_if_due`` 保持周期线程独占规则；同步强制刷新要求每路
得到请求之后的新帧。状态查询不访问设备、不刷新时间戳或改变错误。

锁顺序为反馈锁再进入夹爪回调及设备事务；共享总线的请求与轮询须原子
执行。此组件不管理使能、模式或夹爪命令；夹爪坐标状态仍由管理器维护。
"""
from __future__ import annotations
import logging
import threading
import time
from typing import Sequence
import numpy as np
from types import MappingProxyType
from .gravity_control import JOINT_NAMES, POSITION_LIMITS
from .controller_contracts import MotorFeedback, VerifiedFeedbackSample as _VerifiedFeedbackSample, CachedMotorFeedback as _CachedMotorFeedback

_LOG = logging.getLogger('rebotarmcontroller.hardware_manager')
_FEEDBACK_REFRESH_RETRIES = 3
_FEEDBACK_RETRY_INTERVAL_SEC = 0.05
_UINT64_MAX = (1 << 64) - 1
_UINT64_HALF_RANGE = 1 << 63
# 与驱动、六轴目标及重力模型共用规范 URDF；只读映射防止消费者修改安全边界。
_JOINT_POSITION_LIMITS_RAD = MappingProxyType(dict(zip(JOINT_NAMES, POSITION_LIMITS)))

class _FeedbackValidationError(RuntimeError):
    """保留采集失败分类，供状态发布与故障判断使用。"""
    def __init__(self, code: str, detail: str) -> None:
        super().__init__(detail)
        self.code = code

class FeedbackAcquisition:
    """持有一次设备会话的反馈状态；依赖通过窄回调提供。"""
    def __init__(self, *, groups, joint_names, gripper_motor, gripper_values,
                 gripper_sample, gripper_error, period, stale_timeout, backend, loop_active):
        self._feedback_controller_groups = groups
        self._joint_names = joint_names
        self._gripper_motor = gripper_motor
        self._validated_gripper_feedback_values = gripper_values
        self._record_gripper_feedback = gripper_sample
        self._record_gripper_feedback_error = gripper_error
        self._period = period
        self._stale_timeout = stale_timeout
        self._backend_getter = backend
        self._loop_active = loop_active
        self._feedback_lock = threading.RLock()
        self._feedback_next_refresh_monotonic = None
        self._verified_feedback_by_label = {}
        self._feedback_request_baseline_by_label = {}
        self._feedback_request_deadline_by_label = {}
        self._feedback_error_by_label = {}
        self._feedback_force_refresh_active = False
        self._arm_feedback_updated_monotonic = None
        self._arm_feedback_error = None

    @property
    def joint_names(self): return self._joint_names()
    @property
    def _gripper_mot(self): return self._gripper_motor()
    @property
    def _hardware_feedback_period_sec(self): return self._period()
    @property
    def _gripper_feedback_stale_timeout_sec(self): return self._stale_timeout()
    @property
    def _backend(self): return self._backend_getter()
    @property
    def control_loop_active(self): return self._loop_active()

    @staticmethod
    def _feedback_state_with_sequence(label: str, motor) -> tuple[object, int]:
        """读取 (状态, 接收序号)；依赖打过补丁的 MotorBridge 提供序号接口。

        没有 ``get_state_with_sequence`` 就无法证明反馈是"新帧"，因此在采集路径
        上直接报错，而不是退化为读缓存。
        """
        getter = getattr(motor, "get_state_with_sequence", None)
        if not callable(getter):
            raise RuntimeError(
                f"{label} feedback requires patched MotorBridge "
                "get_state_with_sequence()"
            )
        state, sequence_value = getter()
        sequence = int(sequence_value)
        if not 0 <= sequence <= _UINT64_MAX:
            raise RuntimeError(f"{label} feedback sequence outside uint64 range")
        return state, sequence

    @staticmethod
    def _feedback_sequence_advanced(sequence: int, baseline: int) -> bool:
        """判断接收序号是否相对基线"向前"推进，可容忍 uint64 回绕。

        采用模 2^64 差值：差值落在 (0, 2^63) 视为推进，落在 (2^63, 2^64) 视为
        落后（回绕的另一半）。序号从 0 且基线非 0 时判为未推进——那是计数器
        复位，不能当作新帧。
        """
        current = int(sequence)
        previous = int(baseline)
        if not 0 <= current <= _UINT64_MAX or not 0 <= previous <= _UINT64_MAX:
            raise RuntimeError("feedback sequence outside uint64 range")
        if current == 0 and previous != 0:
            return False
        delta = (current - previous) & _UINT64_MAX
        return 0 < delta < _UINT64_HALF_RANGE

    def _verified_feedback_sample(self, label: str) -> _VerifiedFeedbackSample:
        """返回一条不可变的、由控制循环写入的已验证反馈快照。

        读 MotorBridge 缓存只属于采集路径。消费者在 ``_feedback_lock`` 下取用
        已经验证过的样本；读取它不得改变其接收序号或观测时间。
        """
        with self._feedback_lock:
            sample = self._verified_feedback_by_label.get(label)
        if sample is None:
            raise RuntimeError(f"{label} verified feedback unavailable")
        return sample

    def _verified_feedback_samples(
        self,
        labels: Sequence[str],
    ) -> tuple[_VerifiedFeedbackSample, ...]:
        # 一次性原子读取多个标签，避免同一控制周期内取到不同批次的数据；
        # 任一标签缺失即整体失败（多关节命令不能基于部分反馈执行）。
        with self._feedback_lock:
            samples = tuple(
                self._verified_feedback_by_label.get(label) for label in labels
            )
        for label, sample in zip(labels, samples):
            if sample is None:
                raise RuntimeError(f"{label} verified feedback unavailable")
        return samples  # type: ignore[return-value]

    def _feedback_response_window_sec(self) -> float:
        """一次反馈请求允许的响应窗口：取两种节拍假设与重试次数的较大者。"""
        return max(
            self._hardware_feedback_period_sec * _FEEDBACK_REFRESH_RETRIES,
            _FEEDBACK_RETRY_INTERVAL_SEC * _FEEDBACK_REFRESH_RETRIES,
        )

    @staticmethod
    def _classified_feedback_error(reason: str, code: str = "READ_FAILED") -> str:
        message = str(reason)
        return message if message.startswith("[") else f"[{code}] {message}"

    def _feedback_failure_reason(
        self,
        label: str,
        *,
        error: str | None,
        updated: float | None,
        now: float | None = None,
    ) -> str | None:
        """机械臂与夹爪共用的只读可用性判定，不登记错误或刷新采样时间。"""
        if error is not None:
            return f"{label} feedback unavailable: {self._classified_feedback_error(error)}"
        if updated is None:
            return (
                f"{label} feedback unavailable: [WAITING_FIRST_FRAME] "
                f"{label} verified feedback unavailable; feedback not received"
            )
        current = time.monotonic() if now is None else float(now)
        age = max(current - updated, 0.0)
        if age > self._gripper_feedback_stale_timeout_sec:
            return (
                f"{label} feedback stale: [STALE] age={age:.3f}s "
                f"limit={self._gripper_feedback_stale_timeout_sec:.3f}s"
            )
        return None

    def _cached_motor_feedback_locked(
        self,
        label: str,
        *,
        failure_reason: str | None = None,
        now: float | None = None,
    ) -> _CachedMotorFeedback:
        """统一解析已验证缓存；调用方持有反馈锁，不访问 SDK 或修改健康状态。"""
        sample = self._verified_feedback_by_label.get(label)
        reason = failure_reason or self._feedback_failure_reason(
            label,
            error=self._feedback_error_by_label.get(label),
            updated=sample.observed_at if sample is not None else None,
            now=now,
        )
        if sample is None:
            return _CachedMotorFeedback(None, None, reason, None)
        try:
            self._validate_feedback_sample(label, sample.state)
            values = (float(sample.state.pos), float(sample.state.vel), float(sample.state.torq))
            status = int(sample.state.status_code)
        except Exception as exc:
            detail = self._classified_feedback_error(
                str(exc), getattr(exc, "code", "READ_FAILED")
            )
            return _CachedMotorFeedback(
                None, None, reason or f"{label} feedback unavailable: {detail}", sample,
            )
        return _CachedMotorFeedback(values, status, reason, sample)

    def _validate_feedback_sample(self, label: str, state) -> None:
        """校验单条反馈的合法性：有限数值 + 位于该关节硬件软限位内。

        关节越限或非有限值都说明编码器/通信异常，必须报错，绝不能当作有效
        位置参与运动命令。
        """
        if label == "gripper":
            self._validated_gripper_feedback_values(state)
            return
        if state is None:
            raise RuntimeError(f"{label} feedback unavailable")
        values = (float(state.pos), float(state.vel), float(state.torq))
        if not all(np.isfinite(value) for value in values):
            raise _FeedbackValidationError(
                "INVALID_VALUES", f"{label} feedback contains non-finite values"
            )
        limits = _JOINT_POSITION_LIMITS_RAD.get(label)
        if limits is None:
            raise RuntimeError(f"no hardware soft limit configured for {label}")
        lower, upper = limits
        if values[0] < lower or values[0] > upper:
            raise _FeedbackValidationError(
                "INVALID_COORDINATE",
                f"{label} position {values[0]:.6f} rad outside "
                f"hardware soft limit [{lower:.6f}, {upper:.6f}]"
            )

    def _record_feedback_label_error(
        self, label: str, reason: str, *, code: str = "READ_FAILED"
    ) -> None:
        self._feedback_error_by_label[label] = self._classified_feedback_error(reason, code)

    def _record_verified_feedback(
        self,
        label: str,
        state,
        sequence: int,
        observed_at: float,
    ) -> None:
        """校验通过后登记为"已验证反馈"，并清除该标签的错误标记。"""
        self._validate_feedback_sample(label, state)
        sample = _VerifiedFeedbackSample(
            state=MotorFeedback.copy_from(state),
            sequence=int(sequence),
            observed_at=float(observed_at),
        )
        self._verified_feedback_by_label[label] = sample
        # 与原始字节和运动守卫源时间关联；不改变采样时间或接收序号。
        logging.getLogger('rebotarm.audit').info('verified_feedback', extra={'audit': dict(
            name=label, sequence=sample.sequence, observed_at=sample.observed_at,
            position=sample.state.pos, velocity=sample.state.vel, torque=sample.state.torq,
            status=sample.state.status_code)})
        self._feedback_error_by_label.pop(label, None)
        if label == "gripper":
            self._record_gripper_feedback(state, observed_at=observed_at)

    def _sync_feedback_health(self) -> None:
        """把逐标签的反馈状态汇总为机械臂/夹爪级健康状态。

        机械臂侧只要任一关节有错误就报错；缺少样本且不在强制刷新窗口内时标为
        "verified feedback pending"。夹爪只在自己的标签有错误或样本缺失时报错。
        """
        arm_labels = list(self.joint_names)
        arm_errors = [
            self._feedback_error_by_label[label]
            for label in arm_labels
            if label in self._feedback_error_by_label
        ]
        missing_arm = [
            label for label in arm_labels if label not in self._verified_feedback_by_label
        ]
        if arm_errors:
            self._record_arm_feedback_error("; ".join(arm_errors))
        elif missing_arm:
            if not getattr(self, "_feedback_force_refresh_active", False):
                self._record_arm_feedback_error(
                    "[WAITING_FIRST_FRAME] verified feedback pending: " + ",".join(missing_arm)
                )
        else:
            # 机械臂新鲜度取所有关节中"最旧"的观测时刻，即短板决定整体时效。
            self._record_arm_feedback_success(
                min(
                    self._verified_feedback_by_label[label].observed_at
                    for label in arm_labels
                )
            )

        if self._gripper_mot is None:
            return
        gripper_error = self._feedback_error_by_label.get("gripper")
        if gripper_error is not None:
            self._record_gripper_feedback_error(gripper_error)
        elif "gripper" not in self._verified_feedback_by_label:
            self._record_gripper_feedback_error("[WAITING_FIRST_FRAME] gripper feedback not received")

    def _inspect_pending_feedback(
        self,
        observations: dict[str, tuple[object, int]],
        *,
        observed_at: float,
    ) -> None:
        """结算所有未完成的反馈请求：序号前进则登记样本，超时则记录错误。

        每个待决请求只结算一次；无论成功还是超时都会清掉 baseline/deadline，
        避免过期的截止时间影响下一条命令。
        """
        for label, baseline in list(
            self._feedback_request_baseline_by_label.items()
        ):
            if label not in observations:
                continue
            state, sequence = observations[label]
            if self._feedback_sequence_advanced(sequence, baseline):
                try:
                    self._record_verified_feedback(
                        label,
                        state,
                        sequence,
                        observed_at,
                    )
                except Exception as exc:
                    self._record_feedback_label_error(
                        label, f"{label} feedback invalid: {exc}",
                        code=getattr(exc, "code", "READ_FAILED"),
                    )
                self._feedback_request_baseline_by_label.pop(label, None)
                self._feedback_request_deadline_by_label.pop(label, None)
                continue
            deadline = self._feedback_request_deadline_by_label[label]
            if observed_at >= deadline:
                self._record_feedback_label_error(
                    label,
                    f"{label} feedback deadline expired: sequence did not advance "
                    f"beyond baseline={baseline}",
                )
                self._feedback_request_baseline_by_label.pop(label, None)
                self._feedback_request_deadline_by_label.pop(label, None)

    def _read_feedback_observations(
        self,
        groups,
    ) -> dict[str, tuple[object, int]]:
        observations: dict[str, tuple[object, int]] = {}
        for _ctrl, entries in groups:
            for label, motor in entries:
                observations[label] = self._feedback_state_with_sequence(label, motor)
        return observations

    @staticmethod
    def _require_feedback_sequence_api(groups) -> None:
        """确认全部电机都支持带序号的反馈读取，否则反馈无法被证明为"新帧"。"""
        for _ctrl, entries in groups:
            for label, motor in entries:
                if not callable(getattr(motor, "get_state_with_sequence", None)):
                    raise RuntimeError(
                        f"{label} feedback requires patched MotorBridge "
                        "get_state_with_sequence()"
                    )

    def _record_arm_feedback_success(self, observed_at: float) -> None:
        recovered = self._arm_feedback_error is not None
        self._arm_feedback_updated_monotonic = float(observed_at)
        self._arm_feedback_error = None
        if recovered:
            _LOG.info("arm feedback recovered updated=%.6f", observed_at)

    def _record_arm_feedback_error(self, reason: str) -> None:
        # 只在错误内容变化时打日志，避免硬件回路上刷屏。
        message = str(reason)
        changed = message != self._arm_feedback_error
        self._arm_feedback_error = message
        if changed:
            _LOG.error("arm feedback error: %s", message)

    def _arm_feedback_failure_reason(self, *, now: float | None = None) -> str | None:
        """机械臂反馈的失效原因；健康时返回 None。

        先看是否有显式错误；否则用单调时钟计算反馈年龄，超过
        ``_gripper_feedback_stale_timeout_sec`` 即判定为过期（运动命令必须被拒绝）。
        """
        with self._feedback_lock:
            return self._feedback_failure_reason(
                "arm", error=self._arm_feedback_error,
                updated=self._arm_feedback_updated_monotonic, now=now,
            )

    def _refresh_feedback_batch(
        self,
        *,
        observed_at: float,
        inspect_after_poll: bool = False,
        request_new_feedback: bool = True,
    ) -> None:
        """执行一轮共享总线反馈事务：先结算旧请求，再请求新帧。

        流程：按控制器分组读一次缓存观测 -> 结算上一轮待决请求 -> 为尚无基线
        的标签建立 baseline/deadline -> 持总线锁执行 request_feedback +
        poll_feedback_once。``inspect_after_poll`` 为真时在轮询结束后立即再读一次
        并结算（用于强制刷新），否则只结算轮询前的观测。任一控制器组失败都会
        记录该组电机错误，并在最后以异常抛出，绝不静默继续。
        ``request_new_feedback`` 为假时仅轮询并结算已有请求，不建立新截止时间。
        """
        groups = self._feedback_controller_groups()
        self._require_feedback_sequence_api(groups)
        observations: dict[str, tuple[object, int]] = {}
        group_errors: list[str] = []
        readable_groups = []
        for ctrl, entries in groups:
            try:
                for label, motor in entries:
                    observations[label] = self._feedback_state_with_sequence(label, motor)
                readable_groups.append((ctrl, entries))
            except Exception as exc:
                labels = ",".join(label for label, _motor in entries)
                message = f"controller={type(ctrl).__name__} motors={labels}: {exc}"
                group_errors.append(message)
                for label, _motor in entries:
                    self._record_feedback_label_error(
                        label, f"shared feedback batch failed before request: {message}"
                    )

        if request_new_feedback:
            self._inspect_pending_feedback(observations, observed_at=observed_at)
            response_window = self._feedback_response_window_sec()
            for _ctrl, entries in readable_groups:
                for label, _motor in entries:
                    if label not in self._feedback_request_baseline_by_label:
                        _state, sequence = observations[label]
                        self._feedback_request_baseline_by_label[label] = sequence
                        self._feedback_request_deadline_by_label[label] = (
                            observed_at + response_window
                        )

        successful_groups = []
        for ctrl, entries in readable_groups:
            lock = getattr(ctrl, "bus_lock", None)

            def transaction() -> None:
                # 同一控制器上的"请求 + 轮询"必须在一次持锁事务内完成，
                # 否则别的线程可能在两者之间插入自己的帧，导致错配。
                if request_new_feedback:
                    for _label, motor in entries:
                        motor.request_feedback()
                ctrl.poll_feedback_once()

            try:
                if lock is None:
                    transaction()
                else:
                    with lock:
                        transaction()
                successful_groups.append((ctrl, entries))
            except Exception as exc:
                labels = ",".join(label for label, _motor in entries)
                message = (
                    f"controller={type(ctrl).__name__} motors={labels}: {exc}"
                )
                group_errors.append(message)
                for label, _motor in entries:
                    self._record_feedback_label_error(
                        label,
                        f"shared feedback batch failed: {message}",
                    )
                    self._feedback_request_baseline_by_label.pop(label, None)
                    self._feedback_request_deadline_by_label.pop(label, None)

        if inspect_after_poll:
            completed_at = time.monotonic()
            observations = {}
            for ctrl, entries in successful_groups:
                try:
                    for label, motor in entries:
                        observations[label] = self._feedback_state_with_sequence(label, motor)
                except Exception as exc:
                    labels = ",".join(label for label, _motor in entries)
                    message = f"controller={type(ctrl).__name__} motors={labels}: {exc}"
                    group_errors.append(message)
                    for label, _motor in entries:
                        self._record_feedback_label_error(label, message)
            self._inspect_pending_feedback(
                observations,
                observed_at=completed_at,
            )
        self._sync_feedback_health()
        if group_errors:
            raise RuntimeError("shared feedback batch failed: " + "; ".join(group_errors))

    def _force_feedback_refresh(self) -> None:
        """同步强制刷新：必须拿到"请求之后新到"的反馈，否则报错。

        用于使能、置零、夹爪命令等需要当下真实状态的场合。先记录每个标签的
        基线序号与上一份样本，再最多重试 ``_FEEDBACK_REFRESH_RETRIES`` 次，
        直到样本对象被替换且序号相对基线前进；全部失败时抛出最后一个错误或
        汇总缺失原因。最后一轮非阻塞轮询之后，在原有响应窗口剩余时间内
        只轮询、不追加请求；不能把“请求已写出”当作“应答已接收”。
        """
        groups = self._feedback_controller_groups()
        self._require_feedback_sequence_api(groups)
        initial: dict[str, tuple[object, int]] = {}
        initial_errors: list[str] = []
        for ctrl, entries in groups:
            try:
                for label, motor in entries:
                    initial[label] = self._feedback_state_with_sequence(label, motor)
            except Exception as exc:
                labels = ",".join(label for label, _motor in entries)
                message = f"controller={type(ctrl).__name__} motors={labels}: {exc}"
                initial_errors.append(message)
                for label, _motor in entries:
                    self._record_feedback_label_error(label, message)
        if initial_errors:
            self._sync_feedback_health()
            raise RuntimeError(
                "forced feedback baseline failed before request: "
                + "; ".join(initial_errors)
            )
        required_baselines = {
            label: sequence
            for label, (_state, sequence) in initial.items()
        }
        # 记住旧样本对象：即使序号相同，样本对象未变也说明没收到新帧。
        prior_samples = {
            label: self._verified_feedback_by_label.get(label)
            for label in required_baselines
        }

        def forced_sample_satisfies(label: str, baseline: int) -> bool:
            sample = self._verified_feedback_by_label.get(label)
            return (
                label not in self._feedback_error_by_label
                and sample is not None
                and sample is not prior_samples[label]
                and self._feedback_sequence_advanced(sample.sequence, baseline)
            )

        for label in required_baselines:
            self._feedback_request_baseline_by_label.pop(label, None)
            self._feedback_request_deadline_by_label.pop(label, None)

        last_error: Exception | None = None
        refresh_deadline = time.monotonic() + min(
            self._feedback_response_window_sec(),
            _FEEDBACK_REFRESH_RETRIES * _FEEDBACK_RETRY_INTERVAL_SEC,
        )
        self._feedback_force_refresh_active = True
        try:
            for attempt in range(_FEEDBACK_REFRESH_RETRIES):
                attempt_error: Exception | None = None
                try:
                    self._refresh_feedback_batch(
                        observed_at=time.monotonic(),
                        inspect_after_poll=True,
                    )
                except Exception as exc:
                    last_error = exc
                    attempt_error = exc
                if attempt_error is None and all(
                    forced_sample_satisfies(label, baseline)
                    for label, baseline in required_baselines.items()
                ):
                    return
                if attempt + 1 < _FEEDBACK_REFRESH_RETRIES:
                    time.sleep(_FEEDBACK_RETRY_INTERVAL_SEC)
            # 三轮请求约在 0/50/100 ms 发出，原来在第三次立即轮询后就失败。
            # 剩余最多 50 ms 留给在途应答；总线错误沿用既有失败路径，不用迟到
            # 缓存冲掉异常。固定次数也约束测试时钟停滞或底层轮询异常的等待。
            if last_error is None and not self._feedback_error_by_label:
                for _ in range(10):
                    remaining = refresh_deadline - time.monotonic()
                    if remaining <= 0:
                        break
                    time.sleep(min(.005, remaining))
                    if time.monotonic() > refresh_deadline:
                        break
                    try:
                        self._refresh_feedback_batch(
                            observed_at=time.monotonic(), inspect_after_poll=True,
                            request_new_feedback=False,
                        )
                    except Exception as exc:
                        last_error = exc
                        break
                    if all(forced_sample_satisfies(label, baseline)
                           for label, baseline in required_baselines.items()):
                        return
        finally:
            self._feedback_force_refresh_active = False

        missing = []
        for label, baseline in required_baselines.items():
            if not forced_sample_satisfies(label, baseline):
                reason = self._feedback_error_by_label.get(label)
                if reason is None:
                    reason = (
                        f"{label} feedback timeout: sequence did not advance "
                        f"beyond baseline={baseline}"
                    )
                self._record_feedback_label_error(label, reason)
                missing.append(reason)
            self._feedback_request_baseline_by_label.pop(label, None)
            self._feedback_request_deadline_by_label.pop(label, None)
        self._sync_feedback_health()
        if last_error is not None:
            raise last_error
        raise RuntimeError("shared feedback batch failed: " + "; ".join(missing))

    def refresh_feedback_if_due(
        self,
        *,
        force: bool = False,
        now: float | None = None,
    ) -> bool:
        """按配置频率最多执行一轮共享总线反馈批次。

        本地控制循环运行时，只有该循环可以发起反馈事务。ROS 定时器也会
        调用本方法，但在命令循环停止前它们被降级为"只读缓存"：``force=True``
        时直接抛出 RuntimeError，否则返回 False。
        """
        observed_at = time.monotonic() if now is None else float(now)
        control_thread = self._backend.control_thread
        if (
            (self.control_loop_active or getattr(self._backend, "writer_active", False))
            and control_thread is not None
            and threading.current_thread() is not control_thread
        ):
            if force:
                raise RuntimeError(
                    "synchronous feedback refresh rejected while hardware loop owns bus"
                )
            return False
        with self._feedback_lock:
            due = self._feedback_next_refresh_monotonic
            # 1e-12 的容差用于抵消浮点加法误差，避免恰好到期的周期被推迟。
            if not force and due is not None and observed_at + 1e-12 < due:
                return False
            self._feedback_next_refresh_monotonic = (
                observed_at + self._hardware_feedback_period_sec
            )
            try:
                if force:
                    self._force_feedback_refresh()
                else:
                    self._refresh_feedback_batch(observed_at=observed_at)
                return True
            except Exception:
                if force:
                    raise
                return False

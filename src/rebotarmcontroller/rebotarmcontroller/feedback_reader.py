"""六轴反馈采集装配、数值快照与健康查询。

对外接口：refresh_feedback_if_due 委托采集；joint_snapshot 返回不可变数值；
get_cached_joint_sample 保留旧查询形式并提供数组副本，不触发总线轮询。
约束：缓存和源时间由 FeedbackAcquisition 拥有；查询不得刷新旧时间戳。
bind 在装配时一次注入夹爪能力，不接收 manager，不从发布线程访问设备。
"""
from __future__ import annotations
from .controller_values import JointSnapshot
from .feedback_acquisition import FeedbackAcquisition
from .feedback_acquisition import _JOINT_POSITION_LIMITS_RAD
import numpy as np
import time


class FeedbackReader:
    """拥有本职责的状态和事务；ROS 门面仅提交请求或读取结果。"""

    def __init__(self, runtime, lifecycle, settings):
        self._backend = runtime
        self._lifecycle = lifecycle
        self.settings = settings
        self._bound = False

    def bind(self, *, gripper):
        """装配阶段一次注入明确组件接口；运行后不替换依赖。"""
        if self._bound:
            raise RuntimeError("component already bound")
        self.gripper = gripper
        self._feedback_component = self._assemble_acquisition()
        self._bound = True

    def _assemble_acquisition(self):
        """构造时一次装配组件，运行期间不惰性补建状态。"""
        return FeedbackAcquisition(
            groups=self._feedback_controller_groups, joint_names=lambda: self._backend.joint_names,
            gripper_motor=lambda: self.gripper._gripper_mot,
            gripper_values=self.gripper._validated_gripper_feedback_values,
            gripper_sample=self.gripper._record_gripper_feedback,
            gripper_error=self.gripper._record_gripper_feedback_error,
            period=lambda: (1. / self.settings.hardware_feedback_rate_hz),
            stale_timeout=lambda: self.settings.gripper_feedback_stale_timeout_sec,
            backend=lambda: self._backend, loop_active=lambda: self._backend.control_loop_active,
        )

    def reset_session(self):
        """新连接清除旧会话缓存；不能把旧反馈当作新会话首帧。"""
        cache = self._feedback_component
        with cache._feedback_lock:
            cache._verified_feedback_by_label.clear()
            cache._feedback_request_baseline_by_label.clear()
            cache._feedback_request_deadline_by_label.clear()
            cache._feedback_error_by_label.clear()
            cache._feedback_next_refresh_monotonic = None
            cache._arm_feedback_updated_monotonic = None
            cache._arm_feedback_error = None


    def get_feedback_status(self) -> tuple[list[int], list[str]]:
        """一次只读快照生成 ArmStatus 的状态码与诊断，避免恢复帧混用旧错误。"""
        with self._feedback_component._feedback_lock, self.gripper._gripper_execution._gripper_lock:
            now = time.monotonic()
            codes = list(self._lifecycle.errors)
            failure = self._arm_feedback_failure_reason(now=now)
            statuses = [
                self._cached_motor_feedback_locked(
                    name, failure_reason=failure, now=now,
                ).status_code
                for name in self._backend.joint_names
            ]
            if failure is not None:
                codes.append(f"ARM_FEEDBACK: {failure}")
            if self.gripper._gripper_mot is not None:
                failure = self.gripper._cached_gripper_feedback(now=now).failure_reason
                if failure is not None:
                    codes.append(f"GRIPPER_FEEDBACK: {failure}")
            return statuses, codes


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
        if self.gripper._gripper_mot is not None and self.gripper._gripper_ctrl is None:
            raise RuntimeError("gripper feedback controller unavailable")
        if self.gripper._gripper_mot is not None:
            add(self.gripper._gripper_ctrl, "gripper", self.gripper._gripper_mot)
        if not groups:
            raise RuntimeError("hardware feedback controller map unavailable")
        return groups


    def _verified_feedback_sample(self, *args, **kwargs):
        return self._feedback_component._verified_feedback_sample(*args, **kwargs)


    def _verified_feedback_samples(self, *args, **kwargs):
        return self._feedback_component._verified_feedback_samples(*args, **kwargs)


    def _feedback_response_window_sec(self, *args, **kwargs):
        return self._feedback_component._feedback_response_window_sec(*args, **kwargs)


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


    def _refresh_all_feedback(self) -> None:
        self.refresh_feedback_if_due(force=True)


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
        joint_names = self._backend.joint_names
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
        sample = self.joint_snapshot()
        return (np.array(sample.positions), np.array(sample.velocities), np.array(sample.torques),
                list(sample.statuses), sample.sequences, sample.observed_at)

    def joint_snapshot(self) -> JointSnapshot:
        """周期组件使用的不可变反馈值；公共兼容查询另外生成数组副本。"""
        with self._feedback_component._feedback_lock:
            now = time.monotonic()
            failure = self._arm_feedback_failure_reason(now=now)
            if failure is not None:
                raise RuntimeError(failure)
            feedback = [self._cached_motor_feedback_locked(name, now=now) for name in self._backend.joint_names]
            for item in feedback:
                if item.failure_reason is not None:
                    raise RuntimeError(item.failure_reason)
            positions, velocities, torques = (
                tuple(item.values[index] for item in feedback)
                for index in range(3)
            )
            return JointSnapshot(
                positions, velocities, torques,
                tuple(item.status_code for item in feedback),
                tuple(item.sample.sequence for item in feedback),
                min(item.sample.observed_at for item in feedback),
            )


    def get_joint_status_codes(self) -> list[int]:
        """返回各关节状态码；反馈整体失效时统一返回 255(未知)。"""
        with self._feedback_component._feedback_lock:
            now = time.monotonic()
            failure = self._arm_feedback_failure_reason(now=now)
            return [
                self._cached_motor_feedback_locked(
                    name, failure_reason=failure, now=now,
                ).status_code
                for name in self._backend.joint_names
            ]


    _feedback_state_with_sequence = staticmethod(FeedbackAcquisition._feedback_state_with_sequence)
    _feedback_sequence_advanced = staticmethod(FeedbackAcquisition._feedback_sequence_advanced)
    _classified_feedback_error = staticmethod(FeedbackAcquisition._classified_feedback_error)
    _require_feedback_sequence_api = staticmethod(FeedbackAcquisition._require_feedback_sequence_api)

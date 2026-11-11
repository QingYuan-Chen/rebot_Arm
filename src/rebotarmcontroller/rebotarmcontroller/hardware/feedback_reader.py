"""六轴反馈采集装配，数值快照与健康查询。

职责与位置
    反馈能力的装配与查询适配，FeedbackAcquisition 仍是缓存的唯一所有者。

对外接口
    transaction 提供反馈锁事务；joint_snapshot，get_cached_joint_sample 提供数值副本；
    validated_joints 和 verified_* 供设备事务使用，refresh_feedback_if_due 委托采集器。

安全与并发约束
    发布线程仅读缓存；状态与时间必须来自同一次快照。 派生错误不改写旧反馈时间或电机原码。

文件结构（按定义顺序）
    FeedbackReader：反馈装配与查询适配；采集器唯一拥有验证缓存，本类按事务输出副本。
"""

from __future__ import annotations
from ..contracts.component_ports import GripperAcquisitionPort
from ..contracts.controller_values import JointSnapshot
from ..contracts.controller_contracts import FeedbackUnavailableError
from .feedback_acquisition import FeedbackAcquisition
import numpy as np
import time


class FeedbackReader:
    """反馈装配与查询适配；采集器唯一拥有验证缓存，本类按事务输出副本。"""

    def __init__(self, runtime, lifecycle, settings):
        """保存运行时与参数引用，等待 bind 一次性装配实际采集器和夹爪端口。"""
        self._backend = runtime
        self._lifecycle = lifecycle
        self.settings = settings
        self._bound = False

    def bind(self, *, gripper: GripperAcquisitionPort):
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
            gripper_motor=lambda: self.gripper.feedback_binding()[1],
            gripper_values=self.gripper.feedback_values,
            gripper_sample=self.gripper.record_feedback,
            gripper_error=self.gripper.record_feedback_error,
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

    def transaction(self):
        """成套查询的上下文；必须先于夹爪执行事务进入，可重入且不得等待 ROS。"""
        return self._feedback_component._feedback_lock

    def refresh_active(self):
        """强制刷新在途时禁止空闲回调重入；不暴露采集组件。"""
        return self._feedback_component._feedback_force_refresh_active

    def arm_failure(self, *, now=None):
        """只读健康原因，不改变观测时间或采集错误。"""
        return self._arm_feedback_failure_reason(now=now)

    def verified_sample(self, name):
        """返回采集时已校验的不可变单电机样本；没有首帧时抛错。

        不重新检查当前年龄，也不复制样本或发起采集；使用者须通过健康接口确认当前时效。
        """
        return self._verified_feedback_sample(name)

    def verified_samples(self, names):
        """一次事务内读取同组电机样本，保留各自接收序号。"""
        return self._verified_feedback_samples(names)

    def validate_sample(self, name, state):
        """委托采集器检查单帧有限性与坐标范围，不更新接收序号或时间。"""
        return self._validate_feedback_sample(name, state)

    def failure_reason(self, name, **options):
        """按给定错误，观测时间和当前时刻求只读失效原因。"""
        return self._feedback_failure_reason(name, **options)

    def cached_motor(self, name, **options):
        """允许无首帧的只读诊断；失败返回未知状态，不伪造新时间。"""
        with self.transaction():
            return self._cached_motor_feedback_locked(name, **options)

    def validated_joints(self, **options):
        """取得已验证六轴数值并按选项检查状态码，供生命周期事务使用。"""
        return self._validated_joint_feedback(**options)

    def get_feedback_status(self) -> tuple[list[int], list[str]]:
        """一次只读快照生成 ArmStatus 的状态码与诊断，避免恢复帧混用旧错误。"""
        with self.transaction(), self.gripper.feedback_transaction():
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
            if self.gripper.feedback_binding()[1] is not None:
                failure = self.gripper.cached_feedback(now=now).failure_reason
                if failure is not None:
                    codes.append(f"GRIPPER_FEEDBACK: {failure}")
            return statuses, codes

    def _feedback_controller_groups(self):
        """按共享控制器分组六轴与夹爪，返回控制器及其电机标签列表。

        同组电机在共享总线事务中发送反馈请求并轮询；应答异步返回，单次轮询不保证收齐。
        采集器仍须逐电机验证序号推进。 缺少电机或控制器映射时抛错，避免静默漏采。
        """
        groups: list[tuple[object, list[tuple[str, object]]]] = []

        def add(ctrl, label: str, motor) -> None:
            """按控制器身份合并电机标签，避免共享总线被拆成多个互相竞争的采集事务。"""
            for existing_ctrl, entries in groups:
                if existing_ctrl is ctrl:
                    entries.append((label, motor))
                    return
            groups.append((ctrl, [(label, motor)]))

        for ctrl, entries in self._backend.feedback_groups():
            for label,motor in entries: add(ctrl,label,motor)
        controller, motor = self.gripper.feedback_binding()
        if motor is not None and controller is None:
            raise RuntimeError("gripper feedback controller unavailable")
        if motor is not None:
            add(controller, "gripper", motor)
        if not groups:
            raise RuntimeError("hardware feedback controller map unavailable")
        return groups

    def _verified_feedback_sample(self, *args, **kwargs):
        """从唯一采集缓存取指定标签样本，不读取 SDK 缓存。"""
        return self._feedback_component._verified_feedback_sample(*args, **kwargs)

    def _verified_feedback_samples(self, *args, **kwargs):
        """在同一采集事务内取得多个标签，缺任一标签即失败。"""
        return self._feedback_component._verified_feedback_samples(*args, **kwargs)

    def _feedback_response_window_sec(self, *args, **kwargs):
        """返回采集器的请求响应预算 s，不改变新鲜度阈值。"""
        return self._feedback_component._feedback_response_window_sec(*args, **kwargs)

    def _feedback_failure_reason(self, *args, **kwargs):
        """委托采集器计算只读可用性，不登记派生错误。"""
        return self._feedback_component._feedback_failure_reason(*args, **kwargs)

    def _cached_motor_feedback_locked(self, *args, **kwargs):
        """在反馈锁内解析单电机缓存，保留旧样本身份及原始状态码。"""
        return self._feedback_component._cached_motor_feedback_locked(*args, **kwargs)

    def _validate_feedback_sample(self, *args, **kwargs):
        """复用采集所有者的数值与软限位校验，不复制判定规则。"""
        return self._feedback_component._validate_feedback_sample(*args, **kwargs)

    def _record_feedback_label_error(self, *args, **kwargs):
        """由采集所有者保存逐路错误；调用方须处于反馈事务。"""
        return self._feedback_component._record_feedback_label_error(*args, **kwargs)

    def _record_verified_feedback(self, *args, **kwargs):
        """委托采集所有者登记已验证新样本及原观测时间。"""
        return self._feedback_component._record_verified_feedback(*args, **kwargs)

    def _sync_feedback_health(self, *args, **kwargs):
        """将逐路错误和最旧时间汇总到六轴，夹爪健康状态。"""
        return self._feedback_component._sync_feedback_health(*args, **kwargs)

    def _inspect_pending_feedback(self, *args, **kwargs):
        """按请求序号基线结算应答或截止超时，具体状态仍由采集器拥有。"""
        return self._feedback_component._inspect_pending_feedback(*args, **kwargs)

    def _read_feedback_observations(self, *args, **kwargs):
        """委托读取设备缓存中的状态/序号，仅供采集事务使用。"""
        return self._feedback_component._read_feedback_observations(*args, **kwargs)

    def _record_arm_feedback_success(self, *args, **kwargs):
        """委托采集器记录六轴成功观测时间和恢复状态。"""
        return self._feedback_component._record_arm_feedback_success(*args, **kwargs)

    def _record_arm_feedback_error(self, *args, **kwargs):
        """委托采集器保存六轴采集错误并按变化记录。"""
        return self._feedback_component._record_arm_feedback_error(*args, **kwargs)

    def _arm_feedback_failure_reason(self, *args, **kwargs):
        """查询六轴错误与源年龄，不触发采集或刷新旧时间。"""
        return self._feedback_component._arm_feedback_failure_reason(*args, **kwargs)

    def _refresh_feedback_batch(self, *args, **kwargs):
        """委托采集器执行共享总线反馈批次，保持统一请求和错误处理。"""
        return self._feedback_component._refresh_feedback_batch(*args, **kwargs)

    def _force_feedback_refresh(self, *args, **kwargs):
        """同步取得请求后的新反馈；受运行时设备所有权约束，不能由发布线程调用。"""
        return self._feedback_component._force_feedback_refresh(*args, **kwargs)

    def refresh_feedback_if_due(self, *args, **kwargs):
        """按期委托采集器刷新；设备线程外不抢占总线，force 不能绕过所有权。"""
        return self._feedback_component.refresh_feedback_if_due(*args, **kwargs)

    def _refresh_all_feedback(self) -> None:
        """要求整组同步新反馈，任一路未推进即失败。"""
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
        (0 = 失能, 1 = 使能)。 任一关节缺失，非有限，越软限位或状态码不符都会
        抛错，调用方不得继续运动。
        """
        if refresh:
            self.refresh_feedback_if_due(force=True)
        if check_freshness:
            feedback_failure = self._arm_feedback_failure_reason()
            if feedback_failure is not None:
                raise RuntimeError(feedback_failure)
        joint_names = self._backend.joint_names
        positions: list[float] = []
        velocities: list[float] = []
        torques: list[float] = []
        statuses: list[int] = []
        samples = self._verified_feedback_samples(joint_names)
        for name, sample in zip(joint_names, samples):
            state = sample.state
            values = (state.pos, state.vel, state.torq)
            status = state.status_code
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

        已连接时要求状态码与使能状态一致(使能 -> 1, 失能 -> 0)。 正式运行时
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
        """返回同一快照的数值，原始状态码，接收序号及最旧反馈观测时间。

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
                raise FeedbackUnavailableError(failure)
            feedback = [self._cached_motor_feedback_locked(name, now=now) for name in self._backend.joint_names]
            for item in feedback:
                if item.failure_reason is not None:
                    raise FeedbackUnavailableError(item.failure_reason)
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
        """返回各关节状态码；反馈整体失效时统一返回 255（未知）。"""
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

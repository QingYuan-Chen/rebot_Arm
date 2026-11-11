"""网页六轴轨迹与独立夹爪的组合任务所有者.

文件职责:保存组合身份并汇总原六轴,夹爪执行器的结果;六轴期限,容差及发送游标
    继续由 ArmTrajectory 拥有,夹爪继续采用独立开合逻辑.
对外接口:submit/tick 登记及推进;snapshot/status 查询;fail/stopping/finish
    登记失败和停止收尾;cancel_gripper 只取消本任务代次.
安全约束:设备线程推进,查询以可重入锁串行化.跨线程保留句柄身份,旧句柄收尾
    仅作用于旧子任务;停止确认前保留预约,两侧发送仍由各自周期确认.
文件结构:组合句柄 → 登记 → 状态汇总 → 失败与停止收尾.
"""

from dataclasses import dataclass, field, replace
import threading
from ..arm.arm_trajectory import ArmResult, ArmTask


@dataclass
class CoordinatedTask:
    """组合任务身份及整体结果;六轴执行参数只存在于原子任务中."""

    arm_task: ArmTask
    gripper_generation: int | None
    gripper_state: str
    result: ArmResult = field(default_factory=ArmResult)
    event: threading.Event = field(default_factory=threading.Event)

    @property
    def generation(self):
        return self.arm_task.generation

    def __deepcopy__(self, memo):
        """请求队列传递同一身份和完成事件."""
        return self


class CoordinatedTrajectory:
    """委托两个执行器并拥有整体终态,不继承六轴执行行为."""

    def __init__(self, execution, arm_trajectory):
        self.execution, self._arm = execution, arm_trajectory
        self.task = None
        self._lock = threading.RLock()

    @property
    def active(self):
        with self._lock:
            return self.task is not None and not self.task.event.is_set() and self.task.result.state != 'stopping'

    def submit(self, curve, token, gripper):
        """同一设备事务登记两侧;登记不发总线命令,随后同一周期各自推进."""
        with self._lock:
            if self.active or len(curve.joint_names) != 6:
                raise ValueError('idle combination and six-joint curve required')
            generation = None
            if token.gripper_command:
                generation = gripper.start_position(token, *token.gripper_command)
            try:
                arm_task = self._arm.submit_prepared(curve, token.generation)
            except Exception:
                if generation is not None:
                    gripper.cancel_position('combination admission failed', expected_generation=generation)
                raise
            self.task = CoordinatedTask(arm_task, generation,
                'running' if generation is not None else 'not_requested')
            return self.task

    def snapshot(self, task):
        """旧句柄继续查询自身不可变结果快照."""
        with self._lock:
            return replace(task.result)

    def status(self, task, gripper):
        """停止期间也刷新本代夹爪的中性释放状态."""
        with self._lock:
            if task is self.task and task.gripper_generation is not None:
                task.gripper_state, _ = gripper.position_state(task.gripper_generation)
            return self._arm.snapshot(task.arm_task).state, task.gripper_state

    def fail(self, task, decision):
        """在停止事务前保存当前未完成任务的原因;接替后的旧句柄保持原结果."""
        with self._lock:
            if task is self.task and not task.event.is_set():
                task.result = replace(task.result, decision=decision)

    def tick(self, gripper, sample):
        """只观察原执行器;夹爪不随 FJT 采样."""
        with self._lock:
            if not self.active:
                return
            task = self.task
            if task.generation != self.execution.generation:
                raise RuntimeError('combination execution generation changed')
            self._arm.update(sample.positions, sample.observed_at)
            arm = self._arm.snapshot(task.arm_task)
            task.result = replace(task.result, progress=arm.progress, desired=arm.desired)
            if arm.state not in ('running', 'succeeded'):
                raise RuntimeError(arm.message or 'arm execution failed')
            if task.gripper_generation is not None:
                task.gripper_state, reason = gripper.position_state(task.gripper_generation)
                if task.gripper_state not in ('active', 'releasing', 'succeeded'):
                    raise RuntimeError(reason or 'independent gripper execution failed')
            if arm.state == 'succeeded' and task.gripper_state in ('succeeded', 'not_requested'):
                # 夹爪 releasing 仍在等待中性发送;两侧终态齐备后才唤醒组合 Action.
                self.finish(task, 'succeeded', 'arm and independent gripper completed')

    def cancel_gripper(self, gripper):
        """设备线程取消当前组合代次,中性发送及静止继续由夹爪周期确认."""
        with self._lock:
            task = self.task
            if task is not None and task.gripper_generation is not None:
                gripper.cancel_position('combination stopped', expected_generation=task.gripper_generation)

    def stopping(self, task):
        """冻结本句柄子任务,完成事件等待原停止事务确认."""
        with self._lock:
            if not task.event.is_set():
                self._arm.stopping(task.arm_task)
                task.result = replace(task.result, state='stopping')

    def finish(self, task, state, message):
        """先收尾原子任务,再一次性发布整体终态;旧句柄不会触及后继任务."""
        with self._lock:
            if not task.event.is_set():
                self._arm.finish(task.arm_task, state, message)
                task.result = replace(task.result, state=state, message=message)
                task.event.set()

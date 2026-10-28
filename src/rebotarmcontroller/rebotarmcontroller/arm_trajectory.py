"""六轴轨迹的准入、终点确认及独立任务结果。

``ArmTrajectory.submit`` 接收纯数值轨迹并绑定执行器代次；``update`` 由
唯一周期线程在发送后调用，以新鲜反馈判断收敛或超时。``snapshot`` 返回
不可变结果，ROS 仅映射该结果，不推进时间轴、不修改生命周期。

起点容差为 0.10 rad，终点为 0.03 rad，沉降预算 2 s。发送完成不代表
物理到位；成功反馈必须晚于末点发送。健康超差交回当前位置保持，不能失能。
"""
from dataclasses import dataclass, field, replace
import threading
import time
import numpy as np
from .command_guard import SafetyDecision, failure_decision


@dataclass(frozen=True)
class ArmResult:
    state: str = 'running'
    message: str = ''
    progress: float = 0.
    desired: tuple = ()
    decision: SafetyDecision | None = None


@dataclass
class ArmTask:
    generation: int
    target: tuple
    deadline: float
    tolerance: float
    hold_on_success: bool = False
    result: ArmResult = field(default_factory=ArmResult)
    event: threading.Event = field(default_factory=threading.Event)
    sent_at: float | None = None

    def __deepcopy__(self, memo):
        # 任务是执行器颁发的身份令牌，跨线程传递不能复制其事件和代次。
        return self


class ArmTrajectory:
    def __init__(self, execution, clock=time.monotonic):
        self.execution, self.clock = execution, clock
        self.task = None
        self.lock = threading.RLock()

    def submit(self, points, times, current, *, tolerance=.03, settle=2., velocity=None,
               hold_on_success=False):
        with self.lock:
            if self.task is not None and not self.task.event.is_set():
                raise RuntimeError('arm resource busy')
            if np.max(np.abs(np.asarray(points[0])-current)) > .10:
                raise ValueError('first trajectory point is too far from current joint state')
            now = self.clock()
            if not self.execution.submit(points, times, self.execution.generation, now=now):
                raise RuntimeError('trajectory rejected by inactive execution')
            if velocity is not None:
                self.execution.set_speed_override(velocity)
            task = ArmTask(self.execution.generation, tuple(points[-1]),
                           now + times[-1] + settle, tolerance, hold_on_success)
            self.task = task
            return task

    def snapshot(self, task):
        with self.lock:
            return replace(task.result)

    def stopping(self, task):
        with self.lock:
            if not task.event.is_set():
                task.result = replace(task.result, state='stopping')

    def finish(self, task, state, message):
        with self.lock:
            if not task.event.is_set():
                task.result = replace(task.result, state=state, message=message)
                task.event.set()

    def update(self, positions, observed_at):
        """成功后保留终点；超差时返回需要接管保持的实测角度。"""
        with self.lock:
            task = self.task
            if task is None or task.event.is_set() or task.result.state == 'stopping':
                return None
            if task.generation != self.execution.generation:
                self.finish(task, 'aborted', 'execution superseded')
                return None
            now = self.clock()
            task.result = replace(task.result, progress=self.execution.progress,
                                  desired=tuple(self.execution.target()))
            if not self.execution.moving and task.sent_at is None:
                task.sent_at = now
            if (task.sent_at is not None and observed_at > task.sent_at
                    and np.max(np.abs(np.asarray(positions)-task.target)) <= task.tolerance):
                self.finish(task, 'succeeded', 'trajectory goal reached')
                return positions if task.hold_on_success else None
            if now >= task.deadline:
                decision = failure_decision('trajectory goal not reached within tolerance', 'TRAJECTORY')
                task.result = replace(task.result, decision=decision)
                self.finish(task, 'tolerance', decision.reason)
                return positions
            return None

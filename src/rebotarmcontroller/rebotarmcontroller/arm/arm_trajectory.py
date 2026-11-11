"""六轴轨迹的准入，终点确认及独立任务结果。

职责与位置
    六轴任务所有者，将数值执行器的发送状态与反馈收敛组合为任务终态。

对外接口
    ArmTask 保存身份与事件，ArmResult 是查询快照；ArmTrajectory 负责连续曲线接纳、停靠、update 和 finish。

安全与并发约束
    任务句柄跨线程保留身份；成功要求末点发送后的新反馈。 健康超差保留使能并交回当前位置保持。

文件结构（按定义顺序）
    ArmResult：六轴任务的不可变查询结果；状态，消息，期望角度和进度不暴露执行器内部缓存。
    ArmTask：绑定执行代次的任务身份，持有终点，沉降期限和完成事件；跨线程不复制身份。
    ArmTrajectory：协调一项六轴任务的接纳，发送进度，反馈收敛和一次性终态通知。
"""

from dataclasses import dataclass, field, replace
import threading
import time
import numpy as np
from ..runtime.command_guard import SafetyDecision, failure_decision


@dataclass(frozen=True)
class ArmResult:
    """六轴任务的不可变查询结果；状态，消息，期望角度和进度不暴露执行器内部缓存。"""

    state: str = 'running'
    message: str = ''
    progress: float = 0.
    desired: tuple = ()
    decision: SafetyDecision | None = None


@dataclass
class ArmTask:
    """绑定执行代次的任务身份，持有终点，沉降期限和完成事件；跨线程不复制身份。"""

    generation: int
    target: tuple
    deadline: float
    tolerance: float
    hold_on_success: bool = False
    result: ArmResult = field(default_factory=ArmResult)
    event: threading.Event = field(default_factory=threading.Event)
    sent_at: float | None = None

    def __deepcopy__(self, memo):
        """保留执行器颁发的身份令牌，避免排队复制把同一任务拆成不同事件或代次。"""
        # 任务是执行器颁发的身份令牌，跨线程传递不能复制其事件和代次。
        return self


class ArmTrajectory:
    """协调一项六轴任务的接纳，发送进度，反馈收敛和一次性终态通知。"""

    def __init__(self, execution, clock=time.monotonic):
        """注入数值执行器和单调时钟，建立任务锁；不建立设备连接。"""
        self.execution, self.clock = execution, clock
        self.task = None
        self.lock = threading.RLock()

    def submit_home(self, target, velocity, timeout):
        """登记限速停靠目标；只用于显式 safe_home，成功后交回反馈保持。

        停靠没有轨迹时间轴，由固件速度上限控制接近；仍等待发送后的新反馈确认到位。
        """
        with self.lock:
            if self.task is not None and not self.task.event.is_set():
                raise RuntimeError('arm resource busy')
            if not self.execution.running:
                raise RuntimeError('home rejected by inactive execution')
            now = self.clock()
            generation = self.execution.begin_scoped_target(target, velocity)
            task = ArmTask(generation, tuple(target), now + timeout, .02, True)
            self.task = task
            return task

    def snapshot(self, task):
        """复制指定任务的结果，旧任务不会读到接替任务的终态。"""
        with self.lock:
            return replace(task.result)

    def submit_prepared(self, curve, generation, *, settle=2., tolerance=.03):
        """在同一任务事务内接纳已检查曲线，末点发送仍需后续新帧到位确认。

        调用前由六轴所有者复核预约上下文；这里建立任务事件与后端完成期限。
        settle 是控制器自身的终点等待，不改变 MoveIt 客户端的执行预算。
        默认不启用 hold_on_success，正常结束继续保持曲线终点命令。
        """
        with self.lock:
            if self.task is not None and not self.task.event.is_set():
                raise RuntimeError('arm resource busy')
            now = self.clock()
            if not self.execution.submit_prepared(curve, generation, now=now):
                raise RuntimeError('prepared trajectory superseded or execution inactive')
            self.task = ArmTask(self.execution.generation, curve.points[-1].positions,
                                now+curve.duration+settle, tolerance)
            return self.task

    def stopping(self, task):
        """将尚未终结的任务置为 stopping，保留事件直到停止确认收尾。"""
        with self.lock:
            if not task.event.is_set():
                task.result = replace(task.result, state='stopping')

    def finish(self, task, state, message):
        """在任务锁内只完成一次终态并唤醒等待者，不重复覆盖已经发布的结果。"""
        with self.lock:
            if not task.event.is_set():
                task.result = replace(task.result, state=state, message=message)
                task.event.set()

    def update(self, positions, observed_at):
        """以六轴实测角和源观测时刻推进当前任务，返回可选的保持交接角。

        成功要求末点发送后产生的新反馈进入终点容差；普通成功保留原终点并返回 None，
        hold_on_success 为真时返回实测角，由调用方接管保持。 期限届满仍未确认到位时，
        以超差终态结束并返回实测角；本方法不直接发送保持命令或失能。
        """
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

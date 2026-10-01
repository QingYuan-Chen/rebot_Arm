"""只保留最新待处理项的串行工作队列（"最新优先、陈旧丢弃"）。

用途：视觉回调的输入帧速率可能高于下游耗时任务（IK 求解 + 碰撞检查）的处理能力。
如果逐条排队，机械臂会按几秒前的旧图像做决策，实时性完全丧失。
因此本队列采取"合并"策略：忙时新来的任务覆盖掉尚未开始的旧任务，只保留最新的一个。

状态机（受同一把锁保护，可跨线程安全调用）：
    idle --submit(item)--> busy（立即返回该 item，调用方需同步执行）
    busy --submit(item)--> busy（覆盖 pending，返回 None，调用方丢弃本次输入）
    busy --complete()--> busy（返回并清空 pending，调用方继续处理新 item）
    busy --complete()（无 pending）--> idle（返回 None）
    idle --complete()--> 抛 RuntimeError（禁止在空闲态报告完成，属于调用方逻辑错误）

线程模型：所有公开方法内部加锁，返回值是值快照；调用方在锁外执行真正的耗时工作。
"""

from __future__ import annotations

from dataclasses import dataclass
from threading import Lock
from typing import Generic, TypeVar


T = TypeVar("T")


@dataclass(frozen=True)
class LatestWork(Generic[T]):
    """出队时交给调用方的一个工作项。

    item: 待处理的数据（如某一帧的候选抓取数组）。
    pending_age_ms: 该数据从"被提交"到"可以开始处理"之间的滞留毫秒数。
        首次提交时恒为 0；被合并后在 complete() 中重新出队时，等于
        提交时刻到本次 complete 时刻的间隔。数值越大说明积压越久，
        下游可据此判断这帧是否已经过时（对应监控指标）。
    """

    item: T
    pending_age_ms: float


@dataclass(frozen=True)
class LatestOnlyWorkQueueStats:
    """队列累计统计快照，用于周期性日志或健康检查。

    received: 累计 submit 次数（含被合并的）。
    started: 累计"交给调用方开始处理"的次数。
    completed: 累计 complete 次数。
    coalesced: 累计被覆盖丢弃的次数；持续增长说明处理能力不足（下游跟不上输入帧率）。
    busy: 当前是否有工作正在处理。
    pending: 当前是否有一个被合并、等待处理的最新项。
    """

    received: int
    started: int
    completed: int
    coalesced: int
    busy: bool
    pending: bool


class LatestOnlyWorkQueue(Generic[T]):
    """Serialize expensive work while retaining only the newest pending item."""

    def __init__(self) -> None:
        self._lock = Lock()
        self._busy = False
        # pending 保存 (数据, 收到时刻)，时刻为调用方时钟（秒，单调递增即可）
        self._pending: tuple[T, float] | None = None
        self._received = 0
        self._started = 0
        self._completed = 0
        self._coalesced = 0

    def submit(self, item: T, *, received_at: float) -> LatestWork[T] | None:
        """提交一个工作项。

        参数:
            item: 待处理数据。
            received_at: 收到该数据的时刻（秒）。仅在被合并后才用于计算 pending_age_ms。
        返回:
            队列空闲时返回 LatestWork（pending_age_ms 恒为 0），
            调用方必须立即处理并在结束后调用 complete()；
            队列忙时返回 None，表示本项已被记入 pending 或被合并丢弃。
        """
        with self._lock:
            self._received += 1
            if not self._busy:
                self._busy = True
                self._started += 1
                return LatestWork(item=item, pending_age_ms=0.0)
            # 已有 pending 时说明上一个被合并项还没被取走，本次会把它挤掉：
            # 计入 coalesced 只用于可观测性，不改变行为。
            if self._pending is not None:
                self._coalesced += 1
            self._pending = (item, float(received_at))
            return None

    def complete(self, *, completed_at: float) -> LatestWork[T] | None:
        """报告当前工作处理完毕，并取出被合并的最新项（如有）。

        参数:
            completed_at: 完成时刻（秒），与 submit 的 received_at 必须使用同一时钟。
        返回:
            有 pending 时返回该 LatestWork，其中 pending_age_ms 为滞留时长（毫秒，
            由 max(0.0, ...) 夹紧以避免时钟回拨产生负数）；无 pending 时返回 None。
        异常:
            RuntimeError: 队列处于空闲态时调用（调用方多调或漏改状态的编程错误）。
        """
        with self._lock:
            if not self._busy:
                raise RuntimeError("cannot complete latest-only work while idle")
            self._completed += 1
            if self._pending is None:
                self._busy = False
                return None
            item, received_at = self._pending
            self._pending = None
            self._started += 1
            # 秒 -> 毫秒：×1000；夹紧到 0 防止时钟跳变导致负年龄
            age_ms = max(0.0, (float(completed_at) - received_at) * 1000.0)
            return LatestWork(item=item, pending_age_ms=age_ms)

    def snapshot(self) -> LatestOnlyWorkQueueStats:
        """返回加锁读取的统计快照，供日志/健康检查使用。"""
        with self._lock:
            return LatestOnlyWorkQueueStats(
                received=self._received,
                started=self._started,
                completed=self._completed,
                coalesced=self._coalesced,
                busy=self._busy,
                pending=self._pending is not None,
            )

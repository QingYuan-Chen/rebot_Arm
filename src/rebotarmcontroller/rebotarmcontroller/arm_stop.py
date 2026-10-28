"""六轴受控停止的周期确认状态机。

职责：记录保持目标、目标代次和首次成功发送时间，按新的反馈时间戳累计静止
窗口。对外接口为 ``begin/sent/update/abort`` 和只读 ``result``；运行时每周期
调用一次，不轮询设备或睡眠。默认位置误差 0.03 rad、速度 0.05 rad/s、
持续静止 0.10 s、总预算 2 s，沿用原停止确认条件。

约束：保持发送失败不能开始确认；反馈必须晚于发送。重复缓存不增加静止时间；
目标代次变化、失能或会话结束时结束旧确认，禁止旧取消覆盖新目标。健康设备
确认超时交回保持，不自动失能。时钟由调用方注入，便于 Python/C++ 共用样本。
"""
from dataclasses import dataclass, replace
import time

import numpy as np
from .command_guard import SafetyDecision, failure_decision


@dataclass(frozen=True)
class StopResult:
    """停止确认的值结果；不包含线程、事件或设备句柄。"""

    state: str = "idle"
    message: str = ""
    generation: int = 0
    target: tuple[float, ...] = ()
    sent_at: float | None = None
    quiet_since: float | None = None
    decision: SafetyDecision | None = None


class ArmStop:
    """唯一设备线程拥有的停止确认；等待由请求运行时负责。"""

    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.result = StopResult()
        self.deadline = 0.

    @property
    def pending(self):
        return self.result.state == "stopping"

    def begin(self, target, generation):
        self.result = StopResult("stopping", generation=generation,
                                 target=tuple(float(v) for v in target))
        self.deadline = self.clock() + 2.

    def sent(self, generation):
        if self.pending and generation == self.result.generation and self.result.sent_at is None:
            self.result = replace(self.result, sent_at=self.clock())

    def abort(self, message):
        if self.pending:
            self.result = replace(self.result, state="aborted", message=message)

    def update(self, positions, velocities, observed_at, *, generation, enabled):
        if not self.pending:
            return self.result
        if not enabled or generation != self.result.generation:
            self.abort("stop superseded by lifecycle or target generation")
            return self.result
        sent = self.result.sent_at
        quiet = (sent is not None and observed_at > sent
                 and np.max(np.abs(np.asarray(positions) - self.result.target)) <= .03
                 and np.max(np.abs(velocities)) <= .05)
        since = (self.result.quiet_since if self.result.quiet_since is not None
                 else observed_at) if quiet else None
        self.result = replace(self.result, quiet_since=since)
        if since is not None and observed_at - since >= .10:
            self.result = replace(self.result, state="confirmed", message="stop/hold confirmed")
        elif self.clock() >= self.deadline:
            decision = failure_decision('fresh stationary hold feedback not confirmed', 'STOP')
            self.result = replace(self.result, state="timeout",
                                  message=decision.reason, decision=decision)
        return self.result


class CycleResult:
    """请求的周期续体；设备线程推进后再完成原请求，等待方不占设备线程。"""

    def __init__(self, poll, *, cancel=None, continuation=None):
        self.poll = poll
        self.cancel = cancel or (lambda reason: None)
        self.continuation = continuation

    def advance(self):
        done, result = self.poll()
        if done and self.continuation is not None:
            result = self.continuation()
        return done, result

"""六轴受控停止的周期确认状态机.

职责与位置
    设备周期中的受控停止状态机, 等待静止反馈时不占住设备线程.

对外接口
    ArmStop.begin/sent/update/abort 维护 StopResult; CycleResult 将轮询与后续交接交给运行时.

安全与并发约束
    先确认本代保持已发送, 再累计其后的新鲜连续证据; 超时不等于已经停稳.
    停止轮询只读缓存且不睡眠; 确认后可在同一设备线程继续交接.
    旧任务和失效代次不能撤销新任务.

文件结构(按定义顺序)
    StopResult: 停止确认的值结果; 不包含线程, 事件或设备句柄.
    ArmStop: 唯一设备线程拥有的停止确认; 等待由请求运行时负责.
    CycleResult: 请求的周期续体; 设备线程推进后再完成原请求, 等待方不占设备线程.
"""

from dataclasses import dataclass, replace
import time

from .stationary_evidence import StationaryEvidence
from .command_guard import SafetyDecision, failure_decision


@dataclass(frozen=True)
class StopResult:
    """停止确认的值结果; 不包含线程, 事件或设备句柄."""

    state: str = "idle"
    message: str = ""
    generation: int = 0
    target: tuple[float, ...] = ()
    sent_at: float | None = None
    quiet_since: float | None = None
    decision: SafetyDecision | None = None


class ArmStop:
    """唯一设备线程拥有的停止确认; 等待由请求运行时负责."""

    def __init__(self, clock=time.monotonic):
        """注入单调时钟并创建静止证据; 构造不发停止命令."""
        self.clock = clock
        self.result = StopResult()
        self.deadline = 0.
        self.evidence = StationaryEvidence()

    @property
    def pending(self):
        """查询本次停止是否仍处于等待发送或静止确认阶段."""
        return self.result.state == "stopping"

    def begin(self, target, generation):
        """固定本代保持目标与两秒预算, 清空上一轮证据; 不直接发送."""
        self.result = StopResult("stopping", generation=generation,
                                 target=tuple(float(v) for v in target))
        self.deadline = self.clock() + 2.
        self.evidence.reset(target)

    def sent(self, generation):
        """仅为匹配代次记录首次成功发送时间, 之后的新帧才可用于确认."""
        if self.pending and generation == self.result.generation and self.result.sent_at is None:
            self.result = replace(self.result, sent_at=self.clock())
            self.evidence.last_stamp = self.result.sent_at

    def abort(self, message):
        """终结尚在等待的停止事务, 保留明确失败原因, 不执行失能."""
        if self.pending:
            self.result = replace(self.result, state="aborted", message=message)

    def update(self, positions, velocities, observed_at, *, generation, enabled, sequences=None):
        """以已验证反馈推进本代停止确认, 返回新的 StopResult.

        positions / velocities 按六轴排列; observed_at 和 clock 使用同一单调秒.
        失能, 代次变化, 断流或源回退终止旧确认; 重复帧不累计静止窗口.
        超时仅记录保持未确认, 具体恢复由上层处理, 不在这里改变设备状态.
        """
        if not self.pending:
            return self.result
        if not enabled or generation != self.result.generation:
            self.abort("stop superseded by lifecycle or target generation")
            return self.result
        if self.clock() >= self.deadline:
            decision = failure_decision('fresh stationary hold feedback not confirmed', 'STOP')
            self.result = replace(self.result, state="timeout", message=decision.reason, decision=decision)
            return self.result
        # 停止请求的接纳时刻不足以证明保持生效; 必须观察本代保持实际送出之后的新反馈.
        sent = self.result.sent_at
        reason = 'waiting_send'
        if sent is not None and observed_at > sent:
            reason = self.evidence.observe(positions, velocities, observed_at, self.clock(), sequences=sequences)
        if reason in ('invalid_or_stale', 'source_regressed', 'source_gap'):
            self.abort('stop feedback continuity lost: ' + reason)
            return self.result
        since = self.evidence.quiet_since
        self.result = replace(self.result, quiet_since=since)
        if self.evidence.confirmed():
            self.result = replace(self.result, state="confirmed", message="stop/hold confirmed")
        elif self.clock() >= self.deadline:
            decision = failure_decision('fresh stationary hold feedback not confirmed', 'STOP')
            self.result = replace(self.result, state="timeout",
                                  message=decision.reason, decision=decision)
        return self.result


class CycleResult:
    """请求的周期续体; 设备线程推进后再完成原请求, 等待方不占设备线程."""

    def __init__(self, poll, *, cancel=None, continuation=None):
        """保存无阻塞轮询, 失效清理和可选后续操作, 供唯一运行时推进."""
        self.poll = poll
        self.cancel = cancel or (lambda reason: None)
        self.continuation = continuation

    def advance(self):
        """推进一拍; 轮询确认完成后才执行后续交接, 返回是否完成及原请求结果."""
        done, result = self.poll()
        if done and self.continuation is not None:
            result = self.continuation()
        return done, result

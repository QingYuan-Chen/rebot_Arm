"""execution trajectory_state components; preserve existing execution contracts."""
from __future__ import annotations
import threading
from enum import Enum, auto


class ActiveTrajectory:
    """线程安全的"单目标准入 + 协作式取消"状态。

    仿真一次只允许执行一条轨迹。`_token` 就是被准入的目标对象（当前实现里是
    目标请求），为 None 表示空闲；所有读取与迁移都在同一把可重入锁下完成，
    使"取消"与"开始下一个目标"不会交错。取消采用协作式：这里只置标志，
    真正的停止与保持由持有仿真锁的命令闸门完成。
    """

    def __init__(self, lock: threading.RLock | None = None) -> None:
        self._lock = lock or threading.RLock()
        self._token: object | None = None
        self._cancel_requested = False

    @property
    def busy(self) -> bool:
        with self._lock:
            return self._token is not None

    @property
    def cancel_requested(self) -> bool:
        with self._lock:
            return self._cancel_requested

    @property
    def token(self) -> object | None:
        with self._lock:
            return self._token

    def try_start(self, token: object) -> bool:
        """尝试占用执行权：空闲时登记 token 并清零取消标志，忙时返回 False。"""
        with self._lock:
            if self._token is not None:
                return False
            self._token = token
            self._cancel_requested = False
            return True

    def stop(self) -> bool:
        """仅置取消标志（不触碰仿真状态）；无活动目标时返回 False。"""
        with self._lock:
            if self._token is None:
                return False
            self._cancel_requested = True
            return True

    def finish(self, token: object) -> None:
        """释放执行权。只有 token 与当前登记一致才清空，避免误释放他人的目标。"""
        with self._lock:
            if self._token is token:
                self._token = None
                self._cancel_requested = False



class GateOutcome(Enum):
    """命令闸门对一次操作给出的裁决结果（决定目标如何终止）。"""

    # 命令已真正下发给仿真，执行循环继续。
    APPLIED = auto()
    # 客户端取消了动作目标：保持当前位置并按 canceled 终止。
    ACTION_CANCEL = auto()
    # 收到停止服务请求：保持当前位置并按 aborted 终止。
    SERVICE_STOP = auto()
    # token 已不匹配（目标已被其他路径终结），本次调用什么都不做。
    INACTIVE = auto()
    # 目标成功收敛，动作按 succeeded 终止。
    SUCCEEDED = auto()



def terminal_disposition(outcome: GateOutcome, action_cancel_requested: bool) -> str:
    """把闸门裁决映射为动作终态字符串。

    只有"闸门判为取消"且客户端确实请求过取消时才算 canceled；停止服务或目标
    被替换等情况一律 aborted，避免把服务停止误报成用户取消。
    """
    if outcome is GateOutcome.ACTION_CANCEL and bool(action_cancel_requested):
        return "canceled"
    return "aborted"



class TrajectoryCommandGate:
    """在持锁状态下原子仲裁"继续下发轨迹"与"停止/保持"两类操作。

    设计要点：所有钩子（apply/hold/succeed 以及取消回调）都在可重入锁内执行，
    因此"取消到达"与"成功收敛"只会有一个赢家；执行循环每轮都必须经过本闸门，
    一旦 token 失效或出现取消请求就立刻停手并保持当前位置，绝不再下发新目标。
    """

    def __init__(self, active: ActiveTrajectory) -> None:
        self._active = active

    def apply_if_active(self, token, cancel_requested, apply, hold) -> bool:
        """下发目标的布尔封装；未下发时返回 False。"""
        return self.apply_with_reason(token, cancel_requested, apply, hold) is GateOutcome.APPLIED

    def apply_with_reason(self, token, action_cancel_requested, apply, hold) -> GateOutcome:
        """在确认目标仍有效且未取消时执行 apply，否则执行 hold 并给出原因。"""
        with self._active._lock:
            action_cancel = bool(action_cancel_requested())
            if self._active._token is not token:
                return GateOutcome.INACTIVE
            if action_cancel:
                self._active._cancel_requested = True
                hold()
                return GateOutcome.ACTION_CANCEL
            # 服务停止（而非动作取消）：已经置过取消标志，同样只保持不再前进。
            if self._active._cancel_requested:
                hold()
                return GateOutcome.SERVICE_STOP
            apply()
            return GateOutcome.APPLIED

    def stop_and_hold(self, hold) -> bool:
        """请求停止：有活动目标时置取消标志并立即保持当前位置。

        返回是否确实停止了某个目标，供取消回调/停止服务决定 ACCEPT 还是 REJECT。
        """
        with self._active._lock:
            stopped = self._active._token is not None
            if stopped:
                self._active._cancel_requested = True
                hold()
            return stopped

    def complete_if_active(self, token, cancel_requested, hold, succeed) -> bool:
        """完成目标的布尔封装；真正成功收敛时返回 True。"""
        return self.complete_with_reason(
            token, cancel_requested, hold, succeed
        ) is GateOutcome.SUCCEEDED

    def complete_with_reason(self, token, action_cancel_requested, hold, succeed) -> GateOutcome:
        """把"取消"与"成功终态迁移"线性化，保证二者互斥。"""
        with self._active._lock:
            # 先执行外部回调再重新读取内部状态：取消回调可能通过可重入的测试
            # 钩子置位，也可能刚好在进入临界区之前完成，因此必须以锁内的
            # 最新标志为准。
            action_cancel = bool(action_cancel_requested())
            if self._active._token is not token:
                return GateOutcome.INACTIVE
            if action_cancel:
                self._active._cancel_requested = True
                hold()
                return GateOutcome.ACTION_CANCEL
            if self._active._cancel_requested:
                hold()
                return GateOutcome.SERVICE_STOP
            # 调用 ROS 终态迁移时不持有仿真锁，避免在锁内触发订阅端回调而死锁。
            succeed()
            self._active._token = None
            self._active._cancel_requested = False
            return GateOutcome.SUCCEEDED



class ExecutionLifecycle:
    """异常路径的兜底清理：保证已准入的 token 一定被释放。

    执行回调抛异常时，若只 abort 而不释放执行权，动作服务器会永久处于 busy
    状态、后续所有目标都被拒绝。这里先尽量把当前位置保持住（失败也不影响
    清理），再 abort，最后无论如何都 finish 掉 token。
    """

    def __init__(self, active: ActiveTrajectory, gate: TrajectoryCommandGate) -> None:
        self._active = active
        self._gate = gate

    def fail(self, token, hold, abort) -> None:
        try:
            self._gate.stop_and_hold(hold)
        except Exception:
            pass
        try:
            abort()
        finally:
            self._active.finish(token)

"""本地命令准入与保护失能决策。

职责：检查透传命令的数值、模式及设备边界，并锁存一次设备故障的保护决策。
对外接口：``passthrough_decision`` 返回不可变的 ``SafetyDecision``；
``CommandGuard.begin_protection`` 决定本故障是否首次触发保护，
``confirm_disabled`` 只在新鲜反馈确认失能后解除锁存。

约束：不访问电机、ROS 或运动规划。保护命令发送成功也不等于设备已失能；
持续故障不重复占用总线，失败后由显式失能请求重试并验证。新鲜反馈恢复本身
不能解除锁存或恢复输出，必须完成失能确认后再显式使能。
"""
from dataclasses import dataclass
from enum import Enum
import math


class SafetyAction(str, Enum):
    """本组件已接管的命令准入和设备故障决策。"""
    ALLOW = 'allow'
    REJECT = 'reject'
    STOP_HOLD = 'stop_hold'
    PROTECTIVE_DISABLE = 'protective_disable'


@dataclass(frozen=True)
class SafetyDecision:
    """可供离线对照的动作、原因及故障分类；不含设备或可变任务对象。"""
    action: SafetyAction
    reason: str = ''
    category: str = ''

    def require_allowed(self):
        if self.action != SafetyAction.ALLOW:
            raise ValueError(self.reason)


def failure_decision(reason, category, *, device_fault=False):
    """任务/计算失败保持，已确认的设备或通信故障才允许保护失能。"""
    action = SafetyAction.PROTECTIVE_DISABLE if device_fault else SafetyAction.STOP_HOLD
    return SafetyDecision(action, str(reason), category)


def passthrough_decision(pos, vel, kp, kd, tau, vlim, bounds,
                         max_velocity, max_effort, mode, *, device_mode=None):
    """对已补齐默认值的单电机透传做最终检查；拒绝时不改变任务或设备模式。"""
    if mode not in (0, 1, 2):
        return SafetyDecision(SafetyAction.REJECT, f'unsupported JointMotorCmd mode: {mode}')
    values = (pos, vel, kp, kd, tau, vlim, *bounds, max_velocity, max_effort)
    if not all(math.isfinite(value) for value in values):
        return SafetyDecision(SafetyAction.REJECT, 'passthrough values must be finite')
    if (bounds[0] > bounds[1] or max_velocity <= 0 or max_effort <= 0
            or not bounds[0] <= pos <= bounds[1]
            or abs(vel) > max_velocity or not 0 < vlim <= max_velocity):
        return SafetyDecision(SafetyAction.REJECT,
                              'passthrough position or velocity outside configured limits')
    if mode == 0 and (not 0 <= kp <= 500 or not 0 <= kd <= 5 or abs(tau) > max_effort):
        return SafetyDecision(SafetyAction.REJECT,
                              'passthrough gains or torque outside protocol limits')
    if device_mode is not None and device_mode != {0: 'mit', 1: 'pos_vel', 2: 'vel'}[mode]:
        return SafetyDecision(SafetyAction.REJECT,
                              'passthrough mode does not match explicit device mode')
    return SafetyDecision(SafetyAction.ALLOW)


class CommandGuard:
    """唯一设备线程拥有的保护锁存；发送异常与反馈恢复均不隐式复位。"""
    def __init__(self):
        self._protection = None

    @property
    def protection(self):
        return self._protection

    def begin_protection(self, reason, category):
        if self._protection is not None:
            return False
        # 先锁存再执行设备动作，防止采集/中性释放异常重入同一保护分支。
        self._protection = failure_decision(reason, category, device_fault=True)
        return True

    def confirm_disabled(self):
        self._protection = None

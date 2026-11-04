"""本地命令准入与保护失能决策.

职责与位置
    不访问设备的命令准入与故障分类层; CommandGuard 只拥有保护决策锁存.

对外接口
    passthrough_decision 校验单电机数值与模式; failure_decision 区分保持与保护失能.

安全与并发约束
    准入失败不能取消原任务; 保护先锁存再尝试设备动作. 反馈恢复不能自动清除保护锁存.

文件结构(按定义顺序)
    SafetyAction: 本组件已接管的命令准入和设备故障决策.
    SafetyDecision: 可供离线对照的动作, 原因及故障分类; 不含设备或可变任务对象.
    failure_decision: 任务/计算失败保持, 已确认的设备或通信故障才允许保护失能.
    passthrough_decision: 对已补齐默认值的单电机透传做最终检查; 拒绝时不改变任务或设备模式.
    CommandGuard: 唯一设备线程拥有的保护锁存; 发送异常与反馈恢复均不隐式复位.
"""

from dataclasses import dataclass
from enum import Enum
import math


class SafetyAction(str, Enum):
    """本组件已接管的命令准入和设备故障决策."""

    ALLOW = 'allow'
    REJECT = 'reject'
    STOP_HOLD = 'stop_hold'
    PROTECTIVE_DISABLE = 'protective_disable'


@dataclass(frozen=True)
class SafetyDecision:
    """可供离线对照的动作, 原因及故障分类; 不含设备或可变任务对象."""

    action: SafetyAction
    reason: str = ''
    category: str = ''

    def require_allowed(self):
        """准入为 ALLOW 才返回, 否则抛出带拒绝原因的异常, 不触发设备操作."""
        if self.action != SafetyAction.ALLOW:
            raise ValueError(self.reason)


def failure_decision(reason, category, *, device_fault=False):
    """任务/计算失败保持, 已确认的设备或通信故障才允许保护失能."""
    action = SafetyAction.PROTECTIVE_DISABLE if device_fault else SafetyAction.STOP_HOLD
    return SafetyDecision(action, str(reason), category)


def passthrough_decision(pos, vel, kp, kd, tau, vlim, bounds,
                         max_velocity, max_effort, mode, *, device_mode=None):
    """对已补齐默认值的单电机透传做最终检查; 拒绝时不改变任务或设备模式."""
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
    """唯一设备线程拥有的保护锁存; 发送异常与反馈恢复均不隐式复位."""

    def __init__(self):
        """创建未锁存的保护决策; 该对象由设备线程串行拥有."""
        self._protection = None

    @property
    def protection(self):
        """读取当前保护决策, None 表示没有尚待确认的锁存."""
        return self._protection

    def begin_protection(self, reason, category):
        """首次故障先锁存再返回 True; 已有锁存返回 False, 避免反复保护写入."""
        if self._protection is not None:
            return False
        # 先锁存再执行设备动作, 防止采集/中性释放异常重入同一保护分支.
        self._protection = failure_decision(reason, category, device_fault=True)
        return True

    def confirm_disabled(self):
        """调用方已用新反馈确认失能后清除锁存; 不能仅凭新鲜反馈恢复调用."""
        self._protection = None

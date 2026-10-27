"""控制器组件之间传递的数值副本，不包含 ROS 或 SDK 对象。"""
from dataclasses import dataclass

@dataclass(frozen=True)
class MotorFeedback:
    pos: float
    vel: float
    torq: float
    status_code: int

    @classmethod
    def copy_from(cls, state):
        return cls(float(state.pos),float(state.vel),float(state.torq),int(state.status_code))

@dataclass(frozen=True)
class VerifiedFeedbackSample:
    """保存验证后的反馈、对应序号和本机单调时钟观测时间。"""
    state: MotorFeedback
    sequence: int
    observed_at: float

@dataclass(frozen=True)
class CachedMotorFeedback:
    values: tuple[float,float,float] | None
    raw_status_code: int | None
    failure_reason: str | None
    sample: VerifiedFeedbackSample | None

    @property
    def status_code(self):
        return 255 if self.failure_reason is not None or self.raw_status_code is None else self.raw_status_code


@dataclass(frozen=True)
class CommandReceipt:
    """记录已发送、失败及未尝试的电机，供调用方处理部分失败。"""
    sent: tuple[str, ...]
    failed: str | None = None
    not_attempted: tuple[str, ...] = ()
    detail: str | None = None

    @property
    def success(self):
        return self.failed is None

@dataclass(frozen=True)
class PositionCommand:
    positions: tuple[float, ...]
    velocity_limits: tuple[float, ...]
    generation: int

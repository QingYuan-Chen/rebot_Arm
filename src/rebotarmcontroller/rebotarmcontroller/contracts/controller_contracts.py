"""控制器组件间使用的不可变值类型。

对外接口包括电机反馈 ``MotorFeedback``、已验证样本
``VerifiedFeedbackSample``、缓存查询结果 ``CachedMotorFeedback``、
设备发送回执 ``CommandReceipt`` 和六轴位置命令 ``PositionCommand``。
设备层通过 ``MotorFeedback.copy_from`` 将 SDK 状态复制为普通数值；
采集与健康检查由硬件管理器负责，这里不主动读取设备或判定反馈新鲜度。

约束：各电机的序号独立，``observed_at`` 使用本机单调时钟；缓存失败时
对外状态码为 255，原始状态码仍保存在 ``raw_status_code``。发送回执区分
已发送、失败和未尝试的电机，调用方不能把部分发送当作整批成功。
"""
from dataclasses import dataclass

@dataclass(frozen=True)
class MotorFeedback:
    """从设备状态复制的位置（rad）、速度（rad/s）、力矩（N·m）和原始状态码。"""
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
    """只读缓存结果：保留原始状态、样本及不可用原因。"""
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
    """六轴位置（rad）、逐轴速度上限（rad/s）及对应目标代次。"""
    positions: tuple[float, ...]
    velocity_limits: tuple[float, ...]
    generation: int

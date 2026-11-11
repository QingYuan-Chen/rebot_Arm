"""控制器组件间使用的不可变值类型.

职责与位置
    组件, 驱动和采集器共享的数值契约. 本模块不持有设备句柄, 不采集或判断反馈时效.
    设备层用 MotorFeedback.copy_from 复制 SDK 状态, 采集和健康检查由反馈组件负责.

对外接口
    MotorFeedback, VerifiedFeedbackSample 和 CachedMotorFeedback 描述反馈及缓存可用性.
    CommandReceipt 区分已发送, 失败和未尝试; PositionCommand 描述六轴待发送参数.

安全与并发约束
    各电机序号独立; observed_at 使用本机单调时钟. 缓存不可用时派生状态码 255,
    raw_status_code 仍保留原始状态. 部分发送不能当作整批成功, 发送成功不能当作到位.

统一约定
    关节位置为 rad, 角速度为 rad/s, 力矩为 N·m, 单调时间为 s.

文件结构(按定义顺序)
    FeedbackUnavailableError: 当前反馈不可用的诊断异常.
    MotorFeedback: 从设备状态复制的位置, 速度, 力矩和原始状态码.
    VerifiedFeedbackSample: 采集时已校验的反馈, 逐电机序号和观测时间.
    CachedMotorFeedback: 保留原始状态, 样本及不可用原因的只读缓存结果.
    CommandReceipt: 一批命令的发送事实和失败详情.
    PositionCommand: 六轴位置, 逐轴速度上限及对应目标代次.
"""

from dataclasses import dataclass


class FeedbackUnavailableError(RuntimeError):
    """反馈当前不可用; 原始原因由采集与健康判定提供."""


@dataclass(frozen=True)
class MotorFeedback:
    """从设备状态复制的位置(rad), 速度(rad/s), 力矩(N·m)和原始状态码."""

    pos: float
    vel: float
    torq: float
    status_code: int

    @classmethod
    def copy_from(cls, state):
        """从 SDK 状态复制四个普通数值; 新鲜度与坐标校验由采集器负责."""
        return cls(float(state.pos),float(state.vel),float(state.torq),int(state.status_code))


@dataclass(frozen=True)
class VerifiedFeedbackSample:
    """保存验证后的反馈, 对应序号和本机单调时钟观测时间."""

    state: MotorFeedback
    sequence: int
    observed_at: float


@dataclass(frozen=True)
class CachedMotorFeedback:
    """只读缓存结果: 保留原始状态, 样本及不可用原因."""

    values: tuple[float,float,float] | None
    raw_status_code: int | None
    failure_reason: str | None
    sample: VerifiedFeedbackSample | None

    @property
    def status_code(self):
        """返回可信原码, 样本缺失或不可用时派生 255; 不改写原始缓存."""
        return 255 if self.failure_reason is not None or self.raw_status_code is None else self.raw_status_code


@dataclass(frozen=True)
class CommandReceipt:
    """记录已发送, 失败及未尝试的电机, 供调用方处理部分失败."""

    sent: tuple[str, ...]
    failed: str | None = None
    not_attempted: tuple[str, ...] = ()
    detail: str | None = None

    @property
    def success(self):
        """查询本次批次是否记录失败电机; 不表示电机已经到达目标."""
        return self.failed is None


@dataclass(frozen=True)
class PositionCommand:
    """六轴位置(rad), 逐轴速度上限(rad/s)及对应目标代次."""

    positions: tuple[float, ...]
    velocity_limits: tuple[float, ...]
    generation: int

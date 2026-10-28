"""反馈、夹爪接纳与执行查询的跨组件值契约。

职责：把可变缓存/请求复制为不可变数值，供周期执行和后续 C++ 对照复用。
接口：``JointSnapshot`` 使用 rad、rad/s、N·m 和单调秒；``GripperTarget``
使用电机角度及力矩；``GripperResult`` 绑定目标代次。六轴输入、命令批次和
回执沿用 ArmTarget、PositionCommand、CommandReceipt，不再建立重复类型。
约束：不含 ROS 消息、设备句柄、线程或可写数组；反馈序号各电机独立。
"""
from dataclasses import dataclass


@dataclass(frozen=True)
class JointSnapshot:
    """同一次缓存锁内复制的六轴反馈；观测时间为本批最旧源时间。"""
    positions: tuple[float, ...]
    velocities: tuple[float, ...]
    torques: tuple[float, ...]
    statuses: tuple[int, ...]
    sequences: tuple[int, ...]
    observed_at: float


@dataclass(frozen=True)
class GripperTarget:
    """准入完成后的夹爪位置目标；超时是相对接纳时间的秒数。"""
    start_angle: float
    goal_angle: float
    effort: float
    timeout: float


@dataclass(frozen=True)
class GripperResult:
    """只属于指定代次的结果；释放中不提前报告成功。"""
    generation: int
    state: str
    message: str | None = None

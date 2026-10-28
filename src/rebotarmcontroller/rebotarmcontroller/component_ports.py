"""控制器组件间的具名能力端口。

职责：在装配时把公开方法投影为冻结的回调集合；bind 不再接收完整组件。
接口：端口按调用职责列出允许的能力，数值返回沿用 controller_values 等值契约。
约束：不保存目标、反馈或设备状态的副本，不提供动态属性转发。回调可能形成
控制交接环，但接收端只能调用端口列出的能力；唯一设备线程和锁顺序仍须遵守。
各反馈端口的 transaction 先于夹爪反馈事务；设备绑定只供硬件内部采集和保护。
"""
from dataclasses import dataclass
from typing import Callable, ContextManager, Sequence
from .controller_contracts import MotorFeedback, VerifiedFeedbackSample, CachedMotorFeedback
from .controller_values import JointSnapshot


@dataclass(frozen=True)
class ArmFeedbackPort:
    """六轴目标和停止确认所需的新鲜值查询。"""
    arm_failure: Callable[..., str | None]
    get_joint_state: Callable[[], tuple]
    joint_snapshot: Callable[[], JointSnapshot]
    verified_sample: Callable[[str], VerifiedFeedbackSample]


@dataclass(frozen=True)
class GripperFeedbackPort:
    """夹爪坐标与目标事务所需的缓存、刷新和健康查询。"""
    cached_motor: Callable[..., CachedMotorFeedback]
    failure_reason: Callable[..., str | None]
    refresh_feedback_if_due: Callable[..., object]
    transaction: Callable[[], ContextManager]
    verified_sample: Callable[[str], VerifiedFeedbackSample]


@dataclass(frozen=True)
class GravityFeedbackPort:
    """重力计算所需的同组新鲜反馈和限位验证。"""
    arm_failure: Callable[..., str | None]
    refresh_feedback_if_due: Callable[..., object]
    transaction: Callable[[], ContextManager]
    validate_sample: Callable[[str, MotorFeedback], object]
    verified_samples: Callable[[Sequence[str]], tuple[VerifiedFeedbackSample, ...]]


@dataclass(frozen=True)
class CycleFeedbackPort:
    """周期采集、健康检查和会话重置。"""
    arm_failure: Callable[..., str | None]
    joint_snapshot: Callable[[], JointSnapshot]
    refresh_active: Callable[[], bool]
    refresh_feedback_if_due: Callable[..., object]
    reset_session: Callable[[], None]


@dataclass(frozen=True)
class GripperAcquisitionPort:
    """采集路径所需的夹爪绑定、复制和诊断；不含目标修改能力。"""
    feedback_binding: Callable[[], tuple[object | None, object | None]]
    feedback_values: Callable[[MotorFeedback], tuple[float, float, float, int]]
    record_feedback: Callable[..., None]
    record_feedback_error: Callable[[str], None]
    cached_feedback: Callable[..., CachedMotorFeedback]
    feedback_transaction: Callable[[], ContextManager]


@dataclass(frozen=True)
class ArmCyclePort:
    """周期对六轴任务和透传租约的推进，不暴露任务对象或锁。"""
    clear_passthrough: Callable[[], None]
    expire_passthrough: Callable[[float], None]
    check_task_generation: Callable[[], None]
    passthrough_active: Callable[[], bool]
    update_task: Callable[[JointSnapshot], None]


@dataclass(frozen=True)
class GripperCyclePort:
    """周期对夹爪租约、执行和会话的推进。"""
    clear_passthrough: Callable[[], None]
    expire_passthrough: Callable[[float], None]
    reset_session: Callable[[], None]
    tick: Callable[[], None]
    transport_error: Callable[[], str | None]


@dataclass(frozen=True)
class ArmProtectionPort:
    """保护只能清透传和终止任务，不接纳运动。"""
    clear_passthrough: Callable[[], None]
    abort_task: Callable[[str], None]


@dataclass(frozen=True)
class GripperProtectionPort:
    """保护清理及设备绑定，不提供目标或执行状态的可写引用。"""
    clear_passthrough: Callable[[], None]
    prepare_protection: Callable[[str], None]
    feedback_binding: Callable[[], tuple[object | None, object | None]]


@dataclass(frozen=True)
class GravityProtectionPort:
    """保护清除补偿执行，不做恢复或模式切换。"""
    clear_execution: Callable[[], None]


@dataclass(frozen=True)
class GravityArmPort:
    """六轴交接需要的补偿状态、停止与只读力矩限位。"""
    stop_gravity_compensation: Callable[[], None]
    gravity_compensation_active: Callable[[], bool]
    effort_limit: Callable[[str], float]


@dataclass(frozen=True)
class ArmGravityPort:
    """重力交接需要的六轴保持能力，停止待确认时拒绝启动补偿。"""
    clear_passthrough: Callable[[], None]
    require_available: Callable[[], None]
    start_position_loop: Callable[..., None]
    stop_control_loop: Callable[[], None]


@dataclass(frozen=True)
class CyclePort:
    """统一周期分发入口，调用方不掌握其他执行器状态。"""
    control_tick: Callable[[object, float, Callable], None]


@dataclass(frozen=True)
class ProtectionPort:
    """设备故障的统一保护入口，由接收端锁存并去重。"""
    protect: Callable[..., None]

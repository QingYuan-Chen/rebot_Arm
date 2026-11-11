"""控制器组件间的具名能力端口.

职责与位置
    装配层按调用职责裁剪组件能力; 每个冻结端口只保存约定的回调.

对外接口
    各 *Port 数据类供 HardwareManager 构造期间接线; 组件通过字段调用所有者的公开窄接口.

安全与并发约束
    端口不复制业务状态, 也不替代运行时仲裁. 冻结字段限制能力面, 不保证回调本身线程安全.

文件结构(按定义顺序)
    ArmGripperPort: 固定夹爪快照及受预约身份约束的独立位置任务接口，不直接发送设备命令。
    ArmFeedbackPort: 六轴目标和停止确认所需的新鲜值查询.
    GripperFeedbackPort: 夹爪坐标与目标事务所需的缓存, 刷新和健康查询.
    GravityFeedbackPort: 重力计算所需的同组新鲜反馈和限位验证.
    CycleFeedbackPort: 周期采集, 健康检查和会话重置.
    GripperAcquisitionPort: 采集路径所需的夹爪绑定, 复制和诊断; 不含目标修改能力.
    ArmCyclePort: 周期对六轴任务和透传租约的推进, 不暴露任务对象或锁.
    GripperCyclePort: 周期对夹爪租约, 执行和会话的推进.
    ArmProtectionPort: 保护只能清透传和终止任务, 不接纳运动.
    GripperProtectionPort: 保护清理及设备绑定, 不提供目标或执行状态的可写引用.
    GravityProtectionPort: 保护清除补偿执行，失能验证后确认故障；不切换模式。
    GravityArmPort: 六轴交接需要的补偿状态, 停止与只读力矩限位.
    ArmGravityPort: 重力交接需要的六轴保持能力, 停止待确认时拒绝启动补偿.
    CyclePort: 统一周期分发入口, 调用方不掌握其他执行器状态.
    ProtectionPort: 设备故障的统一保护入口, 由接收端锁存并去重.
"""

from dataclasses import dataclass
from typing import Callable, ContextManager, Sequence
from .controller_contracts import MotorFeedback, VerifiedFeedbackSample, CachedMotorFeedback
from .controller_values import JointSnapshot


@dataclass(frozen=True)
class ArmGripperPort:
    """读取 positions（两指 m）、raw_position（电机 rad）和 generation。

    快照由夹爪所有者在新鲜、连续静止且空闲时提供；不符合条件时抛错。
    端口不缓存或复制静止证据；协同所有者仅在设备线程通过下列窄接口接管、发送和释放。
    """

    snapshot: Callable[[], dict]
    start_position: Callable = None
    position_state: Callable = None
    cancel_position: Callable = None


@dataclass(frozen=True)
class ArmFeedbackPort:
    """六轴目标和停止确认所需的新鲜值查询."""

    arm_failure: Callable[..., str | None]
    get_joint_state: Callable[[], tuple]
    joint_snapshot: Callable[[], JointSnapshot]
    verified_sample: Callable[[str], VerifiedFeedbackSample]


@dataclass(frozen=True)
class GripperFeedbackPort:
    """夹爪坐标与目标事务所需的缓存, 刷新和健康查询."""

    cached_motor: Callable[..., CachedMotorFeedback]
    failure_reason: Callable[..., str | None]
    refresh_feedback_if_due: Callable[..., object]
    transaction: Callable[[], ContextManager]
    verified_sample: Callable[[str], VerifiedFeedbackSample]


@dataclass(frozen=True)
class GravityFeedbackPort:
    """重力计算所需的同组新鲜反馈和限位验证."""

    arm_failure: Callable[..., str | None]
    refresh_feedback_if_due: Callable[..., object]
    transaction: Callable[[], ContextManager]
    validate_sample: Callable[[str, MotorFeedback], object]
    verified_samples: Callable[[Sequence[str]], tuple[VerifiedFeedbackSample, ...]]


@dataclass(frozen=True)
class CycleFeedbackPort:
    """周期采集, 健康检查和会话重置."""

    arm_failure: Callable[..., str | None]
    joint_snapshot: Callable[[], JointSnapshot]
    refresh_active: Callable[[], bool]
    refresh_feedback_if_due: Callable[..., object]
    reset_session: Callable[[], None]


@dataclass(frozen=True)
class GripperAcquisitionPort:
    """采集路径所需的夹爪绑定, 复制和诊断; 不含目标修改能力."""

    feedback_binding: Callable[[], tuple[object | None, object | None]]
    feedback_values: Callable[[MotorFeedback], tuple[float, float, float, int]]
    record_feedback: Callable[..., None]
    record_feedback_error: Callable[[str], None]
    cached_feedback: Callable[..., CachedMotorFeedback]
    feedback_transaction: Callable[[], ContextManager]


@dataclass(frozen=True)
class ArmCyclePort:
    """周期对六轴任务和透传租约的推进, 不暴露任务对象或锁."""

    reset_session: Callable[[], None]
    clear_passthrough: Callable[[], None]
    expire_passthrough: Callable[[float], None]
    check_task_generation: Callable[[], None]
    check_prepared_context: Callable[[], None]
    passthrough_active: Callable[[], bool]
    update_task: Callable[[JointSnapshot], None]
    coordinated_tick: Callable[[], bool] = lambda: False


@dataclass(frozen=True)
class GripperCyclePort:
    """周期对夹爪租约, 执行和会话的推进."""

    clear_passthrough: Callable[[], None]
    expire_passthrough: Callable[[float], None]
    reset_session: Callable[[], None]
    tick: Callable[[], None]
    transport_error: Callable[[], str | None]


@dataclass(frozen=True)
class ArmProtectionPort:
    """保护只能清透传和终止任务, 不接纳运动."""

    clear_passthrough: Callable[[], None]
    abort_task: Callable[[str], None]


@dataclass(frozen=True)
class GripperProtectionPort:
    """保护清理及设备绑定, 不提供目标或执行状态的可写引用."""

    clear_passthrough: Callable[[], None]
    prepare_protection: Callable[[str], None]
    feedback_binding: Callable[[], tuple[object | None, object | None]]


@dataclass(frozen=True)
class GravityProtectionPort:
    """保护清除执行，验证失能后清除重力故障；端口自身不发送设备命令。"""

    clear_execution: Callable[[], None]
    clear_hold_fault: Callable[[], None]


@dataclass(frozen=True)
class GravityArmPort:
    """六轴交接需要的补偿状态, 停止与只读力矩限位."""

    stop_gravity_compensation: Callable[[], None]
    gravity_compensation_active: Callable[[], bool]
    effort_limit: Callable[[str], float]


@dataclass(frozen=True)
class ArmGravityPort:
    """重力交接需要的六轴保持能力, 停止待确认时拒绝启动补偿."""

    clear_passthrough: Callable[[], None]
    require_available: Callable[[], None]
    start_position_loop: Callable[..., None]
    stop_control_loop: Callable[[], None]


@dataclass(frozen=True)
class CyclePort:
    """统一周期分发入口, 调用方不掌握其他执行器状态."""

    control_tick: Callable[[object, float, Callable], None]


@dataclass(frozen=True)
class ProtectionPort:
    """设备故障的统一保护入口, 由接收端锁存并去重."""

    protect: Callable[..., None]

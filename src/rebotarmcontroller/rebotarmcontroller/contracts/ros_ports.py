"""ROS 适配与关闭协调使用的职责端口.

职责与位置
    ROS 适配边界的静态接口声明, 描述发布, 服务和 Action 各自需要的能力.

对外接口
    StatePort 只查询; CommandPort 在查询基础上声明命令. 实现由装配好的原组件直接提供.

安全与并发约束
    Protocol 不建代理, 不持有状态, 不提供默认安全实现; 设备命令仍经组件入队.

文件结构(按定义顺序)
    RuntimeStatePort: 设备会话的只读元数据, 不暴露驱动, 请求队列或线程操作.
    LifecycleStatePort: 生命周期与执行状态查询, 不提供状态修改能力.
    LifecycleCommandPort: 显式生命周期事务; 排队, 锁和新反馈验证由生命周期组件拥有.
    JointFeedbackPort: 已校验六轴缓存及健康查询; 不提供采集刷新或设备句柄.
    ArmCommandPort: 六轴任务, 保持, 停靠和受仲裁的透传命令.
    GripperStatePort: 夹爪缓存, 执行状态及坐标查询, 不提供发送能力.
    GripperCommandPort: 夹爪目标与取消; 坐标验证, 完成和中性释放由组件处理.
    GravityCommandPort: 重力模式交接与只读结果, 不暴露模型及积分状态.
"""

from typing import Protocol


class RuntimeStatePort(Protocol):
    """设备会话的只读元数据, 不暴露驱动, 请求队列或线程操作."""

    @property
    def arm_resource_reserved(self) -> bool:
        """约定只读准备、轨迹及停止确认的资源占用。"""
        ...

    @property
    def joint_names(self) -> list[str]:
        """约定返回六轴名称, 顺序必须与控制器数值向量一致."""
        ...

    @property
    def mode(self) -> str:
        """约定返回电机协议模式, unknown 表示未确认."""
        ...

    @property
    def control_loop_active(self) -> bool:
        """约定返回统一输出周期是否活动, 不等同使能或反馈健康."""
        ...


class LifecycleStatePort(Protocol):
    """生命周期与执行状态查询, 不提供状态修改能力."""

    @property
    def connected(self) -> bool:
        """约定查询设备会话是否已连接."""
        ...

    @property
    def enabled(self) -> bool:
        """约定查询生命周期使能标志, 须连同 state 解读失能是否已确认."""
        ...

    @property
    def state(self) -> str:
        """约定读取生命周期枚举字符串."""
        ...

    @property
    def execution(self) -> str:
        """约定读取业务执行状态字符串."""
        ...

    @property
    def ready_for_motion(self) -> bool:
        """约定查询本层准入状态; 命令执行时仍须重查反馈和资源."""
        ...


class LifecycleCommandPort(LifecycleStatePort, Protocol):
    """显式生命周期事务; 排队, 锁和新反馈验证由生命周期组件拥有."""

    def connect(self):
        """约定建立默认失能会话; 连接验证和清理可能发送失能."""
        ...

    def enable(self):
        """约定显式使能并建立保持, 失败执行可验证回滚."""
        ...

    def disable(self):
        """约定显式失能并用新帧验证, 失败保留不确定状态."""
        ...

    def shutdown(self):
        """约定仅释放已确认失能的设备, 仍使能时拒绝关闭."""
        ...

    def set_mode(self, mode: str):
        """约定串行切换受支持的电机模式, 返回整组切换是否成功."""
        ...

    def set_zero(self, joint_name: str = ""):
        """约定仅失能时置零; 空名称为六轴, gripper 为夹爪."""
        ...


class JointFeedbackPort(Protocol):
    """已校验六轴缓存及健康查询; 不提供采集刷新或设备句柄."""

    def get_joint_state(self):
        """约定返回可用于控制的六轴位置, 速度和力矩副本, 不可用时失败."""
        ...

    def get_cached_joint_state(self):
        """约定只读缓存数组副本, 不主动刷新总线."""
        ...

    def get_cached_joint_sample(self):
        """约定原子返回数组, 状态, 逐轴序号及最旧观测时间."""
        ...

    def get_feedback_status(self):
        """约定读取逐轴可用状态码与错误列表, 不修改样本源时间."""
        ...


class ArmCommandPort(Protocol):
    """六轴任务、准备交接、保持、停靠和受仲裁的透传命令。

    reserve/admit/cancel 委托设备所有者串行执行；decode 只在调用线程解码数值。
    适配器不能因准备服务成功而跳过 admit，也不能直接写入组件的目标或代次。
    """

    def stop_active_motion(self, task=None):
        """约定停止匹配任务并等待保持确认, 旧任务取消不能覆盖新目标."""
        ...

    def reserve_arm_preparation(self, deadline):
        """约定在设备线程预留空闲静止资源并颁发不可变身份。"""
        ...

    def decode_prepared_arm_trajectory(self, payload):
        """约定在调用线程解码冻结系数，不访问设备。"""
        ...

    def admit_prepared_arm_trajectory(self, token, handoff):
        """约定设备线程原子复核身份后接纳同一份系数。"""
        ...

    def cancel_arm_preparation(self, token):
        """约定撤销本身份并等待停止保持确认。"""
        ...

    def update_trajectory_scene(self, revision):
        """约定交接已观察到的碰撞环境版本。"""
        ...

    def submit_arm_trajectory(self, joint_names, points, times, *, home=False, velocity=None, timeout=2.):
        """约定接纳数值轨迹并返回专属任务, 接纳不表示发送或到位."""
        ...

    def arm_execution_status(self, task):
        """约定返回指定任务的结果快照."""
        ...

    def safe_home(self, target=None, vlim=None, timeout_sec=None):
        """约定执行限速停靠并等待反馈确认, 目标不是现场安全证明."""
        ...

    def ensure_pos_vel_control(self):
        """约定显式使能后确保位置速度控制和统一周期可用."""
        ...

    def send_joint_motor_cmd(self, joint_name: str, cmd, arbitration="reject"):
        """约定按仲裁策略提交单关节透传, 并校验数值, 模式及反馈."""
        ...


class GripperStatePort(Protocol):
    """夹爪缓存, 执行状态及坐标查询, 不提供发送能力."""

    @property
    def has_gripper(self) -> bool:
        """约定查询夹爪设备绑定是否存在."""
        ...

    @property
    def gripper_active(self) -> bool:
        """约定查询位置, 抓取或保持任务是否活动."""
        ...

    @property
    def gripper_mode(self) -> str:
        """约定读取夹爪业务模式, 不是电机协议模式."""
        ...

    @property
    def gripper_command_error(self) -> str | None:
        """约定读取当前命令错误, 无错误为 None."""
        ...

    def get_cached_gripper_sample(self):
        """约定返回同一夹爪缓存的开口, 速度, 力矩, 状态, 序号及源时间."""
        ...

    def get_gripper_state(self):
        """约定读取原始角度, 速度, 力矩和可用状态码, 不主动采集."""
        ...

    def gripper_position_m(self):
        """约定返回标称米制开口; 坐标无效时不得返回可执行距离."""
        ...

    def gripper_execution_status(self, generation):
        """约定查询指定代次终态, 旧代次不能读成接替目标成功."""
        ...


class GripperCommandPort(GripperStatePort, Protocol):
    """夹爪目标与取消; 坐标验证, 完成和中性释放由组件处理."""

    def set_gripper_target(self, position_m: float, max_effort: float = 0.):
        """约定接纳米制开口与电机力矩上限, 返回本目标代次."""
        ...

    def set_gripper_position(self, position_m: float, max_effort: float = 0.):
        """约定同步等待位置任务, 返回是否成功与当前开口 m."""
        ...

    def grasp_gripper(self, **kwargs):
        """约定等待堵转抓取结果, 保持有期限且不声称测得接触力."""
        ...

    def cancel_gripper_position_command(self, *args, **kwargs):
        """约定只取消匹配代次的位置命令, 释放由周期推进."""
        ...

    def send_gripper_motor_cmd(self, cmd, arbitration="reject"):
        """约定按夹爪资源仲裁规则提交低层透传."""
        ...


class GravityCommandPort(Protocol):
    """重力模式交接与只读结果, 不暴露模型及积分状态."""

    def start_gravity_compensation(self):
        """约定显式进入有界重力补偿, 交接期间重新验证反馈."""
        ...

    def stop_gravity_compensation(self):
        """约定退出补偿并接管当前位置保持."""
        ...

    def gravity_compensation_active(self):
        """约定查询补偿业务执行是否活动."""
        ...

    def gravity_compensation_target(self):
        """约定返回补偿目标副本, 未建立时为 None."""
        ...

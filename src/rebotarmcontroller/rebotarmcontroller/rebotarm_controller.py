"""机械臂硬件控制器节点：真机通信与执行安全的唯一入口。

职责与位置
    本模块是硬件包的节点入口，负责装配并持有五类组件：
        HardwareManager     电机 SDK / 串口通道访问、反馈校验与使能状态机（唯一硬件持有者）；
        JointStatePublisher 关节状态、单关节状态、夹爪状态与锁存 arm_status 的发布；
        ArmServices         底层服务（使能/失能、回零、安全停靠、IK 直控、夹爪等）；
        ArmActions          动作服务（位姿运动、关节轨迹跟随、夹爪命令）；
        MotorPassthrough    单关节电机指令透传通道。
    规划、示教、界面与视觉策略都不在本节点内实现，本节点只做"能不能动、怎么安全地动"。

对外接口（<ns> 为 arm_namespace 参数，默认 rebotarm）
    发布 /<ns>/joint_states、/<ns>/joints/<joint>/state、/<ns>/gripper/state、
         /<ns>/arm_status；
    订阅 /<ns>/joints/<joint>/cmd、/<ns>/gripper/cmd（由透传组件注册）；
    服务与动作由服务/动作组件按同一命名空间前缀注册。

安全约束
    1. 硬件上电后处于未使能状态，节点启动时不自动使能；必须收到显式 enable 服务且反馈新鲜，
       电机才会输出力矩（就绪判据见 HardwareManager.ready_for_motion）。
    2. 未使能/未就绪时运动类动作的目标请求一律被拒绝，不会出现"还没使能就收到轨迹"。
    3. 退出时默认只断开连接、不产生运动；只有 shutdown_safe_home=true 且当前状态安全检查
       通过，才会在失能之前额外执行一次条件性安全回零。
    4. 使用多线程执行器：改变硬件状态的慢操作与急停、夹爪、IK 直控分属不同回调组，
       改动时不要破坏这个分组，否则会阻塞操作员的停止请求。
"""

from __future__ import annotations

import math

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from .hardware_manager import HardwareManager
from .motor_passthrough import MotorPassthrough
from .ros_actions import ArmActions
from .ros_publishers import JointStatePublisher
from .ros_services import ArmServices


# 操作员在操作界面记录并确认过的安全停靠姿态（6 个关节角，单位：弧度）。
# 第三个关节为 -1°（-0.017453292519943295 rad）：全零姿态下前臂会折回贴上大臂、
# 连杆间距仅约 1.6 mm，规划器会把该位形判为自碰撞而拒绝一切规划，因此停靠位必须偏离零位。
# 该数值与 MoveIt SRDF 中的 safe_home 命名状态、MuJoCo 关键帧以及测试中的常量为同一姿态。
_WEB_SAFE_HOME_JOINT_POSITIONS = (
    0.0,
    0.0,
    -0.017453292519943295,
    0.0,
    0.0,
    0.0,
)


class reBotArmController(Node):
    """硬件控制器节点：装配硬件管理层并对外暴露话题、服务与动作接口。

    生命周期：构造即连接硬件（失败直接抛出并退出进程）；运行期由多线程执行器并发回调；
    退出时先做可选的收尾动作，再断开硬件并失能。
    """

    def __init__(self) -> None:
        super().__init__("reBotArmController")

        # 两组回调组：slow_group 互斥，串行化会改变硬件状态的慢操作；reentrant_group 允许在
        # 慢操作进行中继续响应急停、夹爪与 IK 直控请求。
        self.reentrant_group = ReentrantCallbackGroup()
        self.slow_group = MutuallyExclusiveCallbackGroup()
        # 关节状态采用传感器数据 QoS（尽力而为、深度小），与 RViz/MoveIt 的订阅习惯一致。
        self.sensor_qos = qos_profile_sensor_data

        # arm_config: 机械臂 SDK 配置文件路径；空串表示使用 SDK 内置默认配置。
        self.declare_parameter("arm_config", "")
        self.declare_parameter("gravity_config", "")
        # gripper_config: 夹爪配置文件路径；空串表示使用默认夹爪配置。
        self.declare_parameter("gripper_config", "")
        # channel: 电机总线通道（串口设备）；空串表示由配置文件决定。
        self.declare_parameter("channel", "")
        # joint_state_rate: 关节状态发布频率（Hz）；越高越实时，同时总线与 CPU 负载越大。
        self.declare_parameter("joint_state_rate", 100.0)
        # hardware_feedback_rate_hz: 反馈刷新频率上限（Hz），硬件层要求落在 [20, 100]。
        self.declare_parameter("hardware_feedback_rate_hz", 50.0)
        # gripper_position_torque_cap_nm: 夹爪位置命令的力矩上限（N·m），硬件层限制在 [0.05, 1.5]。
        self.declare_parameter(
            "gripper_position_torque_cap_nm", 1.0,
            descriptor=None,
        )
        # gripper_position_max_speed_rad_s: 夹爪位置命令的角速度上限（rad/s），硬件层限制在
        # [0.05, 3.0]；调大加快开合但冲击与堵转力矩更大。
        self.declare_parameter("gripper_position_max_speed_rad_s", 0.5)
        # gripper_position_timeout_margin_sec: 夹爪到位超时余量（秒），硬件层限制在 [0.1, 10.0]；
        # 实际超时 = 行程 / 速度 + 该余量。
        self.declare_parameter("gripper_position_timeout_margin_sec", 1.5)
        # gripper_feedback_stale_timeout_sec: 夹爪反馈新鲜度阈值（秒），超时即判反馈过期并拒绝
        # 新的夹爪命令；硬件层限制在 [0.05, 2.0]。
        self.declare_parameter("gripper_feedback_stale_timeout_sec", 0.15)
        # grasp_hold_timeout_sec: 抓取保持的最长时间（秒），到期自动松开，避免电机在 500 Hz 下
        # 持续堵转发热；硬件层限制在 [0.1, 120.0]。
        self.declare_parameter("grasp_hold_timeout_sec", 30.0)
        # arm_namespace: 话题/服务/动作的命名空间前缀，首尾斜杠会被去掉。
        self.declare_parameter("arm_namespace", "rebotarm")
        # cmd_arbitration: 轨迹运行期间收到单关节透传指令时的仲裁策略：
        # "reject" 直接拒绝（默认，安全优先），"preempt" 先停下轨迹再执行透传指令。
        self.declare_parameter("cmd_arbitration", "reject")
        # frame_id / ee_frame_id: 对外位姿的参考坐标系与末端坐标系名称。
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("ee_frame_id", "end_link")
        # shutdown_safe_home: 退出前是否在失能之前额外执行一次条件性安全回零；默认关闭，
        # 避免把"节点退出"和"机械臂必须回零"隐式绑定。
        self.declare_parameter("shutdown_safe_home", False)
        # 操作员记录的安全停靠姿态。这里的显式浮点默认值很重要：空列表会被 ROS 2 推断为
        # BYTE_ARRAY，导致运行中无法再用浮点数组实时更新安全停靠位。仅当该参数被显式重置为
        # 空的浮点数组时，才回退到硬件层（与 SRDF/MuJoCo 共享）的默认安全停靠位。
        self.declare_parameter(
            "safe_home_joint_positions", list(_WEB_SAFE_HOME_JOINT_POSITIONS)
        )

        # 路径类参数的空串统一转成 None，交给硬件层回退到默认配置文件。
        arm_config = self.get_parameter("arm_config").value or None
        gripper_config = self.get_parameter("gripper_config").value or None
        channel = str(self.get_parameter("channel").value or "")
        self.arm_namespace = str(self.get_parameter("arm_namespace").value or "rebotarm").strip("/")
        joint_state_rate = float(self.get_parameter("joint_state_rate").value)
        hardware_feedback_rate_hz = float(
            self.get_parameter("hardware_feedback_rate_hz").value
        )
        gripper_position_torque_cap_nm = float(
            self.get_parameter("gripper_position_torque_cap_nm").value
        )
        gripper_position_max_speed_rad_s = float(
            self.get_parameter("gripper_position_max_speed_rad_s").value
        )
        gripper_position_timeout_margin_sec = float(
            self.get_parameter("gripper_position_timeout_margin_sec").value
        )
        gripper_feedback_stale_timeout_sec = float(
            self.get_parameter("gripper_feedback_stale_timeout_sec").value
        )
        grasp_hold_timeout_sec = float(
            self.get_parameter("grasp_hold_timeout_sec").value
        )
        cmd_arbitration = str(self.get_parameter("cmd_arbitration").value or "reject")
        # 非法仲裁值回退到 "reject"：拒绝透传指令始终是安全的一侧。
        if cmd_arbitration not in ("reject", "preempt"):
            self.get_logger().warn(
                f"unsupported cmd_arbitration={cmd_arbitration!r}; using 'reject'"
            )
            cmd_arbitration = "reject"

        # 先全部置空：构造中途抛异常时，外部可据此判断哪些组件尚未装配。
        self.hardware = None
        self.joint_state_publisher = None
        self.arm_services = None
        self.arm_actions = None
        self.motor_passthrough = None
        self.hardware = HardwareManager(
            arm_cfg=arm_config,
            gravity_config=self.get_parameter("gravity_config").value or None,
            gripper_cfg=gripper_config,
            channel=channel,
            hardware_feedback_rate_hz=hardware_feedback_rate_hz,
            gripper_position_torque_cap_nm=gripper_position_torque_cap_nm,
            gripper_position_max_speed_rad_s=gripper_position_max_speed_rad_s,
            gripper_position_timeout_margin_sec=gripper_position_timeout_margin_sec,
            gripper_feedback_stale_timeout_sec=gripper_feedback_stale_timeout_sec,
            grasp_hold_timeout_sec=grasp_hold_timeout_sec,
        )
        # 连接失败时硬件层已保证电机处于失能状态；这里直接向上抛出，让进程退出，
        # 不允许在"半连接"状态下继续对外提供服务。
        try:
            self.hardware.connect()
        except Exception as exc:
            self.get_logger().error(f"hardware connect failed; disabled before exit: {exc}")
            raise
        self.joint_state_publisher = JointStatePublisher(
            self,
            self.hardware,
            self.arm_namespace,
            joint_state_rate,
        )
        self.arm_services = ArmServices(self, self.hardware, self.arm_namespace)
        self.arm_actions = ArmActions(self, self.hardware, self.arm_namespace)
        self.motor_passthrough = MotorPassthrough(
            self,
            self.hardware,
            self.arm_namespace,
            cmd_arbitration,
        )

        # 启动日志明确打印命名空间、关节名与生命周期状态，并提示必须显式使能。
        self.get_logger().info(
            f"reBotArmController started: namespace=/{self.arm_namespace}, "
            f"joints={self.hardware.joint_names}, "
            f"lifecycle={self.hardware.lifecycle_state}; explicit enable required"
        )

    def publish_arm_status(self) -> None:
        """刷新锁存的 arm_status 话题；状态类服务回调结束后都会调用它。"""
        self.joint_state_publisher.publish_status()

    def shutdown(self) -> None:
        """节点退出前的收尾：按需执行条件性安全回零，然后关闭硬件连接并失能。

        shutdown_safe_home=false（默认）时不做任何运动，直接断开连接。
        """
        if self.hardware is None:
            return
        if bool(self.get_parameter("shutdown_safe_home").value):
            self._conditional_safe_home_before_shutdown()
        self.hardware.shutdown()

    def safe_home_joint_positions(self) -> list[float] | None:
        """安全停靠位目标关节角（弧度）；返回 None 表示使用硬件层的默认停靠位。

        参数为 None 或空列表时都返回 None，因此"显式清空该参数"能回退到内置停靠姿态。
        """
        raw = self.get_parameter("safe_home_joint_positions").value
        if raw is None:
            return None
        values = [float(value) for value in raw]
        return values or None

    def _conditional_safe_home_before_shutdown(self) -> None:
        """退出前的条件性安全回零：确有必要且安全时才执行，任何失败都不阻塞退出。

        先停止当前运动（含位置保持），再通过安全门检查；安全门不过或回零抛异常都只记日志，
        随后照常失能退出——退出流程不能被一个可能已不可信的运动请求拖住。
        """
        # 只有已连接且已使能的机械臂才需要、也才能够执行回零；其余情况直接断开即可。
        if not (self.hardware is not None and self.hardware.connected and self.hardware.enabled):
            return
        # 记录进入时的状态机快照：安全门用它判断"回零是否从一个可控状态起步"。
        initial_state = self.hardware.state_machine
        try:
            self.hardware.stop_active_motion()
        except Exception as exc:
            # 连"停下并保持当前位置"都做不到，说明底层已不可信，放弃回零直接转为失能。
            self.get_logger().error(f"shutdown trajectory stop/hold failed; disabling: {exc}")
            return

        allowed, reason = self._safe_home_shutdown_allowed(initial_state)
        if not allowed:
            self.get_logger().warn(f"shutdown safe_home skipped: {reason}; disabling")
            return

        try:
            self.get_logger().warn("shutdown requested: running conditional safe_home before disable")
            self.hardware.stop_gravity_compensation()
            self.hardware.ensure_pos_vel_control()
            self.hardware.safe_home(self.safe_home_joint_positions())
            self.get_logger().info("shutdown conditional safe_home complete")
        except Exception as exc:
            self.get_logger().error(f"shutdown safe_home failed; disabling anyway: {exc}")

    def _safe_home_shutdown_allowed(self, initial_state: str) -> tuple[bool, str]:
        """退出前安全回零的安全门，返回 (是否允许, 原因文本)。

        以下任一条件命中即不发起运动（原因字符串是给操作员看的诊断信息）：
            * 硬件对象不存在；
            * 进入时处于底层流控或重力补偿状态：此时位置-速度环没有接管硬件，贸然回零会与
              重力补偿抢同一条总线；
            * 夹爪正在执行命令：闭合中的夹爪不应与机械臂运动同时进行；
            * 关节反馈取不到、长度与关节数不符，或位置/速度含非有限值（NaN/Inf），
              说明反馈不可信，不能据此让机械臂运动。
        """
        if self.hardware is None:
            return False, "hardware unavailable"
        # 这两种状态下位置环未接管，属于不安全的起步状态。
        if initial_state in ("LOWLEVEL_STREAMING", "GRAVITY_COMP"):
            return False, f"unsafe controller state {initial_state}"
        if self.hardware.gripper_active:
            return False, f"gripper active in {self.hardware.gripper_mode}"
        try:
            positions, velocities, _effort = self.hardware.get_joint_state()
        except Exception as exc:
            return False, f"joint state unavailable: {exc}"
        # 反馈长度必须与关节数一致，否则无法保证读到的每个关节都有效。
        expected = len(self.hardware.joint_names)
        if len(positions) != expected or len(velocities) != expected:
            return False, "joint state size mismatch"
        # NaN/Inf 会污染后续比较与限位判断，必须在发起运动前拦下。
        if not all(math.isfinite(float(value)) for value in positions):
            return False, "joint positions are not finite"
        if not all(math.isfinite(float(value)) for value in velocities):
            return False, "joint velocities are not finite"
        return True, "ok"


def main(args=None) -> None:
    """节点入口：初始化 ROS、用 4 线程执行器运行节点，并保证收尾顺序。

    收尾顺序为：节点 shutdown（可能含条件性安全回零与失能）-> 执行器关闭 -> 销毁节点 ->
    关闭 rclpy，即"先安全停下，再释放通信资源"。
    """
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=4)
    try:
        node = reBotArmController()
        executor.add_node(node)
        executor.spin()
    finally:
        # 即使构造中途失败（node 为 None），也必须关闭执行器并释放 rclpy。
        if node is not None:
            node.shutdown()
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

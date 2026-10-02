"""把机械臂移动到"视觉就绪位姿"的节点（运动包的运动生成职责）。

用途与位置
    视觉抓取链路要求机械臂先摆到一个固定的观察位姿（相机视角与手眼关系在该位姿下标定），
    之后才启动检测与抓取规划。本节点负责这一步：读取当前关节角，生成一条到
    ``joint_positions`` 的平滑插值轨迹，通过控制器的关节轨迹动作下发执行。

对外接口（命名空间由参数 ``arm_namespace`` 决定，默认 ``rebotarm``）
    订阅 ``/<ns>/joint_states``            取当前关节角，使用 sensor 数据 QoS
                                           （best-effort），与控制器高频反馈发布端一致；
    动作 ``/<ns>/follow_joint_trajectory`` 下发轨迹并等待执行结果；
    服务 ``/<ns>/visual_ready/move``       Trigger 类型，按需触发一次就绪移动。

参数
    ``arm_namespace``            控制器所在的 ROS 命名空间；
    ``auto_move_on_start``       启动后是否自动执行一次就绪移动；
    ``exit_after_startup_move``  启动移动流程结束后是否立即退出进程；
    ``startup_delay_sec``        启动移动前的等待时间（秒），用于等控制器/时钟就绪；
    ``joint_positions``          目标关节角（rad，6 个手臂关节，顺序固定）；
    ``duration_sec``             轨迹总时长（秒），下限 0.2 s；
    ``wait_timeout_sec``         等待关节反馈与等待动作服务的时间上限（秒）；
    ``max_start_delta_rad``      允许的起始偏差上限（rad），<= 0 表示不限制。

两种运行角色
    启动实例：``auto_move_on_start=true`` 且 ``exit_after_startup_move=true``，完成一次移动
              即退出，上层启动组合用"进程退出"事件串联后续节点；
    常驻实例：``auto_move_on_start=false``，只提供 ``visual_ready/move`` 服务，供界面或操作者
              在需要时把机械臂摆回观察位姿。

安全约束
    1. 目标必须是 6 个有限值，否则拒绝下发；
    2. 必须先收到包含 6 个手臂关节的有效反馈，取不到就失败返回，不做"盲发"；
    3. 起始偏差超过 ``max_start_delta_rad`` 时拒绝移动：上电位置离观察位太远时直接插值会
       造成大幅横扫，属于危险动作；
    4. 轨迹端点速度与加速度为零（五次平滑插值），避免起步/停止冲击；
    5. 失败只记录日志并返回失败标志，不自动重试，也不触发其它运动。
"""

from __future__ import annotations

import math
import sys
import time

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# 手臂 6 个关节的固定顺序：轨迹点、反馈解析与日志都依赖这个顺序。
ARM_JOINT_NAMES = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")


def _duration_msg(seconds: float) -> Duration:
    """把秒数转成轨迹点的时间戳（整秒 + 纳秒余数）。

    小数部分直接截断而非四舍五入，因此只适用于非负时间；轨迹点时间必须单调递增，截断不会
    让它倒退。
    """
    sec = int(seconds)
    nanosec = int((float(seconds) - sec) * 1e9)
    return Duration(sec=sec, nanosec=nanosec)


def _smoothstep(ratio: float) -> float:
    """五次平滑插值曲线 ``10r³ - 15r⁴ + 6r⁵``，把行程比例映射成插值系数。

    该多项式在 r=0 与 r=1 处的一阶、二阶导数都为 0，因此轨迹起点与终点的速度、加速度均为
    零，不会出现阶跃冲击。输入先夹到 [0,1]，防止数值误差导致外插。
    """
    ratio = max(0.0, min(1.0, float(ratio)))
    return 10.0 * ratio**3 - 15.0 * ratio**4 + 6.0 * ratio**5


def _finite_positions(values: list[float] | tuple[float, ...]) -> bool:
    """校验是否为 6 个有限关节角。

    NaN/Inf 会让插值与控制器行为不可预期，长度不符则说明反馈里缺少手臂关节，两种情况都必须
    按无效处理。
    """
    return len(values) == len(ARM_JOINT_NAMES) and all(math.isfinite(float(v)) for v in values)


class VisualReadyNode(Node):
    """视觉就绪位姿节点：启动移动 + 常驻服务触发。

    生命周期：构造时只声明参数并建立订阅、动作客户端与服务，不做任何运动；运动只发生在
    :meth:`move_to_visual_ready` 被调用时，调用点有两处——``main`` 中的启动流程，以及
    ``visual_ready/move`` 服务回调。

    回调模型：单线程。``move_to_visual_ready`` 内部用阻塞式等待推进流程（``spin_once`` 轮询
    关节反馈、``spin_until_future_complete`` 等动作结果），因此一次就绪移动会占用调用线程直到
    动作结束；对"一次性动作"的定位可以接受，但服务调用方需要留出足够超时。
    """

    def __init__(self) -> None:
        super().__init__("rebotarm_visual_ready")
        self.declare_parameter("arm_namespace", "rebotarm")
        self.declare_parameter("auto_move_on_start", True)
        self.declare_parameter("exit_after_startup_move", False)
        self.declare_parameter("startup_delay_sec", 0.0)
        # 默认值即当前安装朝向下的观察位姿：相对原始上游姿态（joint1=0，朝 base +X）把 joint1
        # 沿用旧仓库视觉就绪位，joint1=0 使机械臂朝向 base +X 工作区。
        self.declare_parameter(
            "joint_positions",
            [0.0, -0.1, -0.2, 0.2, 0.0, 0.0],
        )
        self.declare_parameter("duration_sec", 4.0)
        self.declare_parameter("wait_timeout_sec", 12.0)
        self.declare_parameter("max_start_delta_rad", 1.0)

        # 允许传入 "rebotarm" 或 "/rebotarm" 两种写法，统一去斜杠后再拼绝对话题/服务名。
        namespace = str(self.get_parameter("arm_namespace").value).strip("/")
        self._joint_state_topic = f"/{namespace}/joint_states"
        self._action_name = f"/{namespace}/follow_joint_trajectory"
        self._latest_joint_state: JointState | None = None

        # 控制器以 sensor 数据 QoS（best-effort）发布关节反馈，订阅侧必须匹配同一 QoS 才能收到。
        self.create_subscription(JointState, self._joint_state_topic, self._on_joint_state, qos_profile_sensor_data)
        self._trajectory_client = ActionClient(self, FollowJointTrajectory, self._action_name)
        self._move_service = self.create_service(
            Trigger,
            f"/{namespace}/visual_ready/move",
            self._handle_move_request,
        )

    def _on_joint_state(self, msg: JointState) -> None:
        # 只保留最新一帧：就绪移动只关心当前状态，历史帧没有用途。
        self._latest_joint_state = msg

    def _handle_move_request(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """``visual_ready/move`` 服务回调：执行一次就绪移动并映射成 Trigger 响应。

        服务是同步的：移动流程走完才返回。注意等待动作结果那一步没有超时（见
        :meth:`move_to_visual_ready`），控制器不返回结果时该服务会一直阻塞，调用方必须设置
        自己的超时。``message`` 文本是固定英文常量，供调用方直接展示。
        """
        response.success = self.move_to_visual_ready()
        response.message = "visual_ready reached" if response.success else "visual_ready move failed"
        return response

    def move_to_visual_ready(self) -> bool:
        """执行一次就绪移动，成功返回 True。

        流程：校验目标关节角 → 等待当前关节反馈 → 校验起始偏差 → 等待动作服务 → 生成轨迹并
        发送 → 等待执行结果。任一步失败都记日志并返回 False（不抛出业务异常，服务回调需要稳定
        的布尔结果）。

        注意：等待动作被接受与等待动作结果这两处都是无超时阻塞；只有等关节反馈和等动作服务
        受 ``wait_timeout_sec`` 约束。

        返回 True 表示控制器报告轨迹执行成功（``error_code`` 为 SUCCESSFUL），不代表末端位姿
        精度——跟踪精度由控制器侧负责。
        """
        target = [float(v) for v in self.get_parameter("joint_positions").value]
        if not _finite_positions(target):
            self.get_logger().error(f"joint_positions must contain 6 finite values, got {target}")
            return False

        current = self._wait_for_current_positions()
        if current is None:
            self.get_logger().error(f"no valid arm joint state received on {self._joint_state_topic}")
            return False

        max_start_delta = float(self.get_parameter("max_start_delta_rad").value)
        # 取所有关节中偏差最大的一项作为判据：只要有一个关节离目标太远就拒绝整体移动。
        worst_delta = max(abs(t - c) for t, c in zip(target, current))
        if max_start_delta > 0.0 and worst_delta > max_start_delta:
            # max_start_delta_rad <= 0 表示显式关闭该保护（仅在明确知道风险时使用）。
            self.get_logger().error(
                "visual_ready startup move refused: "
                f"max joint delta {worst_delta:.3f} rad > limit {max_start_delta:.3f} rad"
            )
            return False

        # 控制器未上线就不要发目标；等待上限与关节反馈共用 wait_timeout_sec，最坏等待可预期。
        if not self._trajectory_client.wait_for_server(timeout_sec=float(self.get_parameter("wait_timeout_sec").value)):
            self.get_logger().error(f"follow_joint_trajectory action unavailable: {self._action_name}")
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = self._build_trajectory(current, target)
        self.get_logger().info(
            "moving to visual_ready: "
            + ", ".join(f"{name}={value:+.3f}" for name, value in zip(ARM_JOINT_NAMES, target))
        )
        send_future = self._trajectory_client.send_goal_async(goal)
        # 显式 spin 推进 future：启动路径下节点还没进入 executor，服务回调路径下也要在这里等结果。
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("visual_ready trajectory rejected")
            return False

        result_future = goal_handle.get_result_async()
        # 同样显式 spin 等待执行结果；此处没有超时，控制器不返回就会一直阻塞。
        rclpy.spin_until_future_complete(self, result_future)
        # 动作返回两层结构：外层是动作封装，内层才是轨迹执行结果。
        result = result_future.result().result
        # 用错误码而不是"没抛异常"判定成功：被取消、超时都会给出非零码。
        success = int(result.error_code) == int(FollowJointTrajectory.Result.SUCCESSFUL)
        if not success:
            self.get_logger().error(
                f"visual_ready trajectory failed: error_code={result.error_code}, message={result.error_string}"
            )
        return success

    def _wait_for_current_positions(self) -> list[float] | None:
        """在 ``wait_timeout_sec`` 内等待一帧包含 6 个手臂关节的有效反馈。

        返回 6 个关节角（rad，顺序为 ``ARM_JOINT_NAMES``）；超时或 rclpy 已关闭时返回 None。
        循环内用 ``spin_once`` 驱动订阅回调，休眠 0.05 s 以控制 CPU 占用。
        """
        deadline = time.monotonic() + float(self.get_parameter("wait_timeout_sec").value)
        while rclpy.ok() and time.monotonic() < deadline:
            msg = self._latest_joint_state
            if msg is not None:
                positions = self._extract_arm_positions(msg)
                if positions is not None:
                    return positions
            rclpy.spin_once(self, timeout_sec=0.05)
        return None

    def _extract_arm_positions(self, msg: JointState) -> list[float] | None:
        """从一帧关节反馈里按名字取出 6 个手臂关节角。

        用 ``msg.name`` 查表而不是依赖消息内的排列顺序（控制器、仿真后端与夹爪关节都可能改变
        顺序）；缺少任一手臂关节或数值非有限时返回 None，由调用方继续等待。
        """
        by_name = {name: index for index, name in enumerate(msg.name)}
        if not all(name in by_name for name in ARM_JOINT_NAMES):
            return None
        positions = [float(msg.position[by_name[name]]) for name in ARM_JOINT_NAMES]
        if not _finite_positions(positions):
            return None
        return positions

    def _build_trajectory(self, current: list[float], target: list[float]) -> JointTrajectory:
        """生成从 ``current`` 到 ``target`` 的关节轨迹（只有位置与时间戳）。

        采样周期固定 0.05 s（20 Hz；默认 4 s 行程即 81 个轨迹点），``duration_sec`` 下限
        0.2 s。每个点的位置按 :func:`_smoothstep` 的比例插值；速度、加速度字段留空，由控制器
        在轨迹点之间插值——端点速度为零这一性质来自插值曲线本身。
        """
        duration = max(float(self.get_parameter("duration_sec").value), 0.2)
        # 至少 2 步：步数为 0 会在下面按 steps 求比例时报除零，1 步则只剩起末两点、过于粗糙。
        steps = max(2, int(duration / 0.05))
        trajectory = JointTrajectory()
        trajectory.joint_names = list(ARM_JOINT_NAMES)
        # range(steps + 1)：含起点（ratio=0）与终点（ratio=1），共 steps + 1 个轨迹点。
        for step in range(steps + 1):
            ratio = step / float(steps)
            blend = _smoothstep(ratio)
            point = JointTrajectoryPoint()
            point.positions = [float(c + (t - c) * blend) for c, t in zip(current, target)]
            # 时间戳随行程比例线性增长，与位置插值使用同一比例。
            point.time_from_start = _duration_msg(duration * ratio)
            trajectory.points.append(point)
        return trajectory

    def wait_before_startup_move(self) -> None:
        """启动移动前的可配置延时，用于等控制器、时钟或底层驱动就绪。

        延时期间持续 ``spin_once``（单次休眠不超过 0.1 s），既让订阅回调得以运行，也能及时响应
        中断；``startup_delay_sec <= 0`` 时立即返回。
        """
        delay = max(float(self.get_parameter("startup_delay_sec").value), 0.0)
        if delay <= 0.0:
            return
        self.get_logger().info(f"visual_ready startup move delayed {delay:.1f}s")
        deadline = time.monotonic() + delay
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(0.1, max(deadline - time.monotonic(), 0.0)))


def main(args: list[str] | None = None) -> None:
    """节点入口：按参数决定"自动移动后退出"与"常驻服务"两种模式。

    ``auto_move_on_start`` 为真时先延时再执行就绪移动；``exit_after_startup_move`` 为真则无论
    移动成功与否都在移动流程结束后退出进程（成功与否只体现在日志中，上层启动组合以进程退出
    事件衔接后续节点）。否则一直 spin，对外提供 ``visual_ready/move`` 服务。
    """
    rclpy.init(args=args)
    node = VisualReadyNode()
    try:
        if bool(node.get_parameter("auto_move_on_start").value):
            node.wait_before_startup_move()
            startup_success = node.move_to_visual_ready()
            if startup_success:
                node.get_logger().info("visual_ready startup move complete")
            if bool(node.get_parameter("exit_after_startup_move").value):
                # The launch layer uses the process exit code as the lifecycle gate.
                # A failed guarded move must not look like a successful startup.
                if not startup_success:
                    sys.exit(1)
                return
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

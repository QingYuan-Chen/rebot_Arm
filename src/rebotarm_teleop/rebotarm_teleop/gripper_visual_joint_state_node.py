"""RViz 夹爪可视化关节状态桥。

职责：真实夹爪只有一路开合自由度，而 URDF 里用左右两个手指关节表示；本节点把
``/<ns>/joint_states``（手臂六轴）与 ``/<ns>/gripper/state``（夹爪开口，m）合成
一路 ``/<ns>/visual_joint_states``（手腕六轴 + left_finger_joint + right_finger_joint），
再交给负责发布机器人姿态的节点驱动 RViz 中的手指 link。

在系统中的位置：RViz 与 MoveIt 的真实关节状态桥。机械臂和夹爪分别发布源时间，
不能用任一路的新帧刷新另一路旧数据，也不以初始化闭合值冒充真实反馈。

QoS 约定：两个订阅都用 BEST_EFFORT + depth 10（随传感器数据流，旧帧可丢）；
发布用 RELIABLE，保证 RViz 侧不丢帧。

安全与健壮性约束：只有包含全部 ``required_arm_joint_names`` 且位置值全部有限的
关节状态才会被转发，避免残缺/含 NaN 的反馈让 RViz 里的机器人模型跳到异常位姿；
夹爪反馈非有限或过期时不发布，不能重新授时旧开口。
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rebotarm_msgs.msg import JointMotorState
from sensor_msgs.msg import JointState

from .gripper_visual_model import DEFAULT_GRIPPER_LIMITS_M, gripper_opening_to_finger_joint_positions


class GripperVisualJointStateNode(Node):
    """为 RViz 和 MoveIt 按各自源时间发布真实 JointState 的节点。

    参数：
    - ``arm_namespace``：命名空间，前后斜杠会被去掉；
    - ``gripper_lower_limit_m`` / ``gripper_upper_limit_m``：夹爪开口的物理范围（m），
      默认取 ``DEFAULT_GRIPPER_LIMITS_M``；写反时内部自动交换；
    - ``left_finger_joint_name`` / ``right_finger_joint_name``：URDF 中两个手指关节名，
      必须与 URDF 完全一致，否则 RViz 无法把位置作用到对应 link；
    - ``required_arm_joint_names``：转发前必须齐全的关节名集合（默认六个手臂关节），
      用于剔除残缺的关节状态。

    线程模型：单线程 executor，两个订阅回调都可能触发发布；两者都只读写
    ``_latest_arm_state`` / ``_latest_gripper_position`` 这两个「最新值」快照。
    """

    def __init__(self) -> None:
        super().__init__("gripper_visual_joint_state_node")
        self.declare_parameter("arm_namespace", "rebotarm")
        self.declare_parameter("gripper_lower_limit_m", DEFAULT_GRIPPER_LIMITS_M[0])
        self.declare_parameter("gripper_upper_limit_m", DEFAULT_GRIPPER_LIMITS_M[1])
        self.declare_parameter("left_finger_joint_name", "left_finger_joint")
        self.declare_parameter("right_finger_joint_name", "right_finger_joint")
        self.declare_parameter(
            "required_arm_joint_names",
            ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
        )

        namespace = str(self.get_parameter("arm_namespace").value).strip("/")
        self._namespace = namespace
        lower = float(self.get_parameter("gripper_lower_limit_m").value)
        upper = float(self.get_parameter("gripper_upper_limit_m").value)
        # 容忍配置写反：交换后再作为可信区间使用。
        if upper < lower:
            lower, upper = upper, lower
        self._gripper_limits = (lower, upper)
        self._left_joint = str(self.get_parameter("left_finger_joint_name").value)
        self._right_joint = str(self.get_parameter("right_finger_joint_name").value)
        self._required_arm_joints = {
            str(name) for name in list(self.get_parameter("required_arm_joint_names").value)
        }
        self._latest_arm_state: JointState | None = None
        # 没有夹爪反馈时不伪造闭合状态。
        self._latest_gripper_position = None
        # 残缺状态只告警一次，避免高频话题刷爆日志。
        self._warned_invalid_arm_state = False

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        visual_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._publisher = self.create_publisher(
            JointState,
            f"/{namespace}/visual_joint_states",
            visual_qos,
        )
        self.create_subscription(
            JointState,
            f"/{namespace}/joint_states",
            self._on_arm_joint_state,
            sensor_qos,
        )
        self.create_subscription(
            JointMotorState,
            f"/{namespace}/gripper/state",
            self._on_gripper_state,
            sensor_qos,
        )
        self.get_logger().info(
            f"publishing RViz visual joint states on /{namespace}/visual_joint_states"
        )

    def _on_arm_joint_state(self, msg: JointState) -> None:
        """缓存有效的手臂关节状态并立即刷新可视化；无效状态丢弃且只告警一次。"""
        if not self._valid_arm_joint_state(msg):
            if not self._warned_invalid_arm_state:
                self.get_logger().warn(
                    "ignored empty or incomplete arm joint state for RViz visual_joint_states"
                )
                self._warned_invalid_arm_state = True
            return
        self._warned_invalid_arm_state = False
        self._latest_arm_state = msg
        self._publish_visual_state(msg.header)

    def _on_gripper_state(self, msg: JointMotorState) -> None:
        """只发布新鲜有限的夹爪开口（m）；无效帧不刷新状态源时间。"""
        stamp = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds-stamp)/1e9
        if not math.isfinite(float(msg.position)) or stamp <= 0 or not -.05 <= age <= .15:
            return
        self._latest_gripper_position = float(msg.position)
        left,right = gripper_opening_to_finger_joint_positions(msg.position,self._gripper_limits)
        state = JointState()
        state.header = msg.header
        state.name = [self._left_joint,self._right_joint]
        state.position = [left,right]
        self._publisher.publish(state)

    def _publish_visual_state(self, header) -> None:
        """只转发本次机械臂源帧；夹爪由自己的回调独立发布，不重新授时。"""
        if self._latest_arm_state is None or not self._valid_arm_joint_state(self._latest_arm_state):
            return
        msg = JointState()
        msg.header = header
        msg.name,msg.position,msg.velocity,msg.effort = self._filtered_state_parts(self._latest_arm_state)
        # 仿真可直接提供完整源帧；只转发该帧真实携带的手指，绝不补旧值。
        source = self._latest_arm_state
        if self._left_joint in source.name and self._right_joint in source.name:
            for name in (self._left_joint,self._right_joint):
                index = source.name.index(name)
                msg.name.append(name)
                msg.position.append(float(source.position[index]))
                if msg.velocity:
                    msg.velocity.append(float(source.velocity[index]))
                if msg.effort:
                    msg.effort.append(float(source.effort[index]))
        self._publisher.publish(msg)

    def _valid_arm_joint_state(self, state: JointState | None) -> bool:
        """判定手臂关节状态是否可用于显示：非空、字段等长、必需关节齐全且数值有限。"""
        if state is None:
            return False
        if not state.name or not state.position:
            return False
        if len(state.position) < len(state.name):
            return False
        names = {str(name) for name in state.name}
        if self._required_arm_joints and not self._required_arm_joints.issubset(names):
            return False
        # 逐值检查有限性（NaN/Inf 会让 RViz 模型消失或跳到异常位姿）。
        return all(math.isfinite(float(value)) for value in state.position[: len(state.name)])

    def _filtered_state_parts(self, state: JointState) -> tuple[list[str], list[float], list[float], list[float]]:
        """拆出 (name, position, velocity, effort) 四个平行列表，并剔除手指关节。

        剔除原因：手臂反馈里若已带手指关节，再追加一次就会出现同名重复项；
        velocity/effort 仅在长度与 name 完全一致时才逐项转发，否则留空不补。
        """
        skip = {self._left_joint, self._right_joint}
        names: list[str] = []
        positions: list[float] = []
        velocities: list[float] = []
        efforts: list[float] = []
        has_velocity = len(state.velocity) == len(state.name)
        has_effort = len(state.effort) == len(state.name)
        for index, name in enumerate(state.name):
            if name in skip:
                continue
            names.append(str(name))
            positions.append(float(state.position[index]))
            if has_velocity:
                velocities.append(float(state.velocity[index]))
            if has_effort:
                efforts.append(float(state.effort[index]))
        return names, positions, velocities, efforts


def main(args=None) -> None:
    """节点入口：初始化后进入 spin，退出时释放节点与 rclpy 上下文。"""
    rclpy.init(args=args)
    node = GripperVisualJointStateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

"""抓取预览位姿发送节点（视觉规划结果 -> 运动执行层的位姿桥）。

职责与位置：
    本节点把视觉链路输出的抓取规划消息转换成一条位姿目标消息（Pose）并发布到
    运动执行层订阅的话题上，用于「先预览/先规划」而不直接执行抓取。
    它只做坐标变换与偏移补偿，不做逆解、不做碰撞检查、不调用任何执行服务，
    因此发布位姿本身不会移动机械臂；真正的规划与执行门控在运动执行层。

外部接口：
    - 订阅：input_topic（默认 /grasp/plan，消息类型 GraspPlan）——视觉规划结果；
    - 发布：output_topic（默认 /rebotarm/interactive_control/pose_target，消息类型 Pose）——
        目标末端位姿，header 不带时间戳（Pose 消息本身无 header），
        查询 TF 时使用「最新可用变换」；
    - 节点名固定为 rebotarm_grasp_preview_sender，可执行文件为 rebotarm_send_grasp_preview。

关键参数（详细语义见 __init__ 中的声明处）：
    pose_mode 选择预抓取位姿还是抓取位姿；target_frame 指定输出坐标系；
    tcp_offset_xyz 做 TCP 到末端连杆的换算；target_base_offset_xyz 做基准坐标系下
    的平移补偿；min_target_z_m 是防撞地面的高度下限；
    publish_count 决定重复发布次数；exit_after_publish 决定发布后是否退出进程。

运行模型：
    单线程 spin，收到规划 -> 变换 -> 发布。默认 exit_after_publish=True，
    即「发一次就退出」，适合脚本式单次预览；上层启动组合把该参数覆盖为 False，
    使节点持续工作，后续每收到一条规划就更新目标位姿。
"""

from __future__ import annotations

from copy import deepcopy

import rclpy
from geometry_msgs.msg import Pose
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

from rebotarm_msgs.msg import GraspPlan

from ..utils.tf_message_adapter import (
    apply_tcp_offset_to_pose,
    transform_from_msg,
    transform_pose_message,
)

# 兼容旧的 Python 导入路径；包内新代码只从 tf_message_adapter 导入。
_transform_from_msg = transform_from_msg


def select_grasp_plan_pose(plan: GraspPlan, pose_mode: str) -> Pose:
    """从抓取规划中按模式取出目标位姿的深拷贝。

    参数：
        plan：视觉规划结果；只有 valid=true 才允许取用，invalid 规划一律拒绝。
        pose_mode：字符串模式，忽略大小写与首尾空白；"pregrasp" 取预抓取位姿，
            "grasp" 取抓取位姿，其余取值直接报错（不做静默兜底，避免发错目标）。
    异常：
        ValueError —— 规划无效（沿用规划自带的 reason）或模式不支持。
    注意：
        返回深拷贝，调用方后续的偏移修改不会污染订阅到的消息对象。
    """
    if not plan.valid:
        reason = plan.reason or "grasp plan is invalid"
        raise ValueError(reason)

    normalized = pose_mode.strip().lower()
    if normalized == "pregrasp":
        return deepcopy(plan.pregrasp_pose)
    if normalized == "grasp":
        return deepcopy(plan.grasp_pose)
    raise ValueError(f"unsupported pose mode: {pose_mode}")


class GraspPreviewSenderNode(Node):
    """抓取预览发送节点：订阅抓取规划，发布一次（或持续更新）目标位姿。

    生命周期：
        __init__ 声明参数、建立 TF 监听与发布/订阅；_on_plan 处理每条规划；
        main 按 exit_after_publish 与 published 标志决定是否继续 spin。
    线程模型：
        全部回调在单线程 executor 中串行执行，不存在并发改写已发布标志的问题。
    """

    def __init__(self) -> None:
        super().__init__("rebotarm_grasp_preview_sender")
        # 输入规划话题：视觉链路最终输出的抓取规划
        self.declare_parameter("input_topic", "/grasp/plan")
        # 输出位姿话题：运动执行层订阅的位姿目标（话题名是对外接口，勿改）
        self.declare_parameter("output_topic", "/rebotarm/interactive_control/pose_target")
        # 取预抓取位姿（"pregrasp"）还是抓取位姿（"grasp"）
        self.declare_parameter("pose_mode", "pregrasp")
        # 输出位姿的目标坐标系；与规划消息 frame_id 不同才查询 TF 做变换
        self.declare_parameter("target_frame", "base_link")
        # 末端坐标系下的 TCP 偏移（m）：节点默认不补偿 (0,0,0)，
        # 现场通过参数文件/启动参数传入实测值 [-0.04, 0.0, 0.0] 后才做换算
        self.declare_parameter("tcp_offset_xyz", [0.0, 0.0, 0.0])
        # 基准坐标系下的整体平移补偿（m），用于修正候选点与期望落点的固定偏差
        self.declare_parameter("target_base_offset_xyz", [0.0, 0.0, 0.0])
        # 目标高度下限（m）：>0 时把 Z 抬到该平面以上，防止目标点落到桌面以下
        self.declare_parameter("min_target_z_m", 0.0)
        # 每条规划的发布次数：重复发布以降低订阅端漏收概率（QoS 深度 10）
        self.declare_parameter("publish_count", 5)
        # 发布成功后是否退出进程：True 为单次预览脚本语义，上层启动时覆盖为 False 以持续更新
        self.declare_parameter("exit_after_publish", True)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.pose_mode = str(self.get_parameter("pose_mode").value)
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.tcp_offset_xyz = self._tuple3("tcp_offset_xyz")
        self.target_base_offset_xyz = self._tuple3("target_base_offset_xyz")
        self.min_target_z_m = float(self.get_parameter("min_target_z_m").value)
        # 至少发布 1 次：参数误配为 0 或负数时不至于静默不发
        self.publish_count = max(1, int(self.get_parameter("publish_count").value))
        self.exit_after_publish = bool(self.get_parameter("exit_after_publish").value)
        self.published = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(Pose, self.output_topic, 10)
        self.subscription = self.create_subscription(
            GraspPlan,
            self.input_topic,
            self._on_plan,
            10,
        )
        self.get_logger().info(
            "grasp preview sender ready: "
            f"input={self.input_topic}, output={self.output_topic}, "
            f"pose_mode={self.pose_mode}, target_frame={self.target_frame}, "
            f"tcp_offset_xyz={self.tcp_offset_xyz}, "
            f"target_base_offset_xyz={self.target_base_offset_xyz}, "
            f"min_target_z_m={self.min_target_z_m:.3f}"
        )

    def _tuple3(self, name: str) -> tuple[float, float, float]:
        """读取长度为 3 的浮点参数数组；长度不符直接抛错。

        这是启动期配置校验：宁可起不来也不要按缺省值继续发布错误位姿。
        """
        values = list(self.get_parameter(name).value)
        if len(values) != 3:
            raise ValueError(f"{name} must contain exactly 3 values")
        return (float(values[0]), float(values[1]), float(values[2]))

    def _on_plan(self, plan: GraspPlan) -> None:
        """规划回调：位姿选择 -> 坐标系变换 -> TCP/基准偏移 -> 高度下限 -> 发布。

        状态语义：
            published=True 且 exit_after_publish=True 时直接忽略后续规划（单次模式已完成）；
            无效规划：警告并跳过；单次模式下把 published 置位以便进程退出，
            持续模式下不置位，等待下一条有效规划；
            TF 尚未就绪：只警告并返回，不置位 published，等 TF 可用后由下一条规划重试。
        """
        if self.published and self.exit_after_publish:
            return

        try:
            pose = select_grasp_plan_pose(plan, self.pose_mode)
        except ValueError as exc:
            self.get_logger().warn(f"skip grasp plan: {exc}")
            if self.exit_after_publish:
                # 单次模式下没有重试机会，置位后退出，避免进程永久挂住。
                self.published = True
            return

        source_frame = str(plan.header.frame_id)
        target_frame = self.target_frame.strip()
        # 仅在两个坐标系都非空且不同的时候才做变换；相同时直接使用原位姿。
        if target_frame and source_frame and source_frame != target_frame:
            try:
                # 取最新可用变换（Time() 为空即 latest），超时 0.2 s：TF 未就绪时快速失败并等下次规划。
                tf_msg = self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.2),
                )
                pose = transform_pose_message(pose, transform_from_msg(tf_msg))
            except Exception as exc:
                self.get_logger().warn(
                    f"skip grasp plan: waiting for TF {target_frame} <- {source_frame}: {exc}"
                )
                return

        pose = apply_tcp_offset_to_pose(pose, self.tcp_offset_xyz)
        # 基准坐标系下的平移补偿（x/y/z 分别叠加，逐项 round 保持 6 位小数）
        pose.position.x = round(float(pose.position.x) + self.target_base_offset_xyz[0], 6)
        pose.position.y = round(float(pose.position.y) + self.target_base_offset_xyz[1], 6)
        pose.position.z = round(float(pose.position.z) + self.target_base_offset_xyz[2], 6)
        # 安全下限：只在显式配置正的下限时生效，绝不把目标压低到配置平面以下
        if self.min_target_z_m > 0.0:
            pose.position.z = max(float(pose.position.z), self.min_target_z_m)
        for _ in range(self.publish_count):
            self.publisher.publish(pose)
        self.published = True
        self.get_logger().info(
            f"sent {self.pose_mode} pose to {self.output_topic}: "
            f"position=({pose.position.x:+.3f},{pose.position.y:+.3f},{pose.position.z:+.3f}), "
            f"orientation=({pose.orientation.x:+.4f},{pose.orientation.y:+.4f},"
            f"{pose.orientation.z:+.4f},{pose.orientation.w:+.4f})"
        )


def main(args=None) -> None:
    """命令行入口：按 0.1 s 步进 spin，直到单次发布完成（exit_after_publish）或进程被关闭。

    KeyboardInterrupt 与 ROS 外部关闭异常都被视为正常退出；finally 中销毁节点并关闭 rclpy。
    """
    rclpy.init(args=args)
    node = GraspPreviewSenderNode()
    try:
        while rclpy.ok() and not (node.exit_after_publish and node.published):
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

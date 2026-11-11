"""抓取 TCP（工具中心点）静态坐标系的广播节点。

职责：把操作员实测的"末端连杆坐标系 -> 抓取 TCP 坐标系"的固定平移
以静态变换的形式发布，供下游抓取链（位姿换算、RViz 显示、手眼链路）
通过坐标变换查询得到真实夹爪闭合中心。

坐标系约定：子坐标系原点相对父坐标系只有平移，旋转为单位四元数
（即两个坐标系轴向平行）。offset 单位为米，沿父坐标系轴向表达。

参数（节点名 rebotarm_grasp_tcp_frame）：
    parent_frame:   父坐标系，默认 end_link（机械臂末端连杆）
    child_frame:    子坐标系，默认 grasp_tcp（抓取 TCP）
    tcp_offset_xyz: 三元素平移，默认 [0.0, 0.0, 0.0]，单位米

安全语义：本节点只发布静态变换，不下发任何运动指令；但它决定了"抓取点在哪"，
配错 offset 会让所有抓取位姿整体偏移，因此真实机械臂上的取值必须来自实测，
不能直接套用仿真模型里的等效偏移。
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster


def build_grasp_tcp_transform(
    *,
    parent_frame: str,
    child_frame: str,
    tcp_offset_xyz: tuple[float, float, float],
) -> TransformStamped:
    """构造一条静态平移变换消息。

    参数:
        parent_frame: 父坐标系名。
        child_frame: 子坐标系名。
        tcp_offset_xyz: 子坐标系原点在父坐标系中的平移 (x, y, z)，单位米。
    返回:
        TransformStamped，旋转为单位四元数 (0, 0, 0, 1)，header.stamp 保持零值
        （静态广播器会忽略时间戳，TF 树中视为永久有效）。
    """
    msg = TransformStamped()
    msg.header.frame_id = parent_frame
    msg.child_frame_id = child_frame
    # 显式 float() 转换：ROS 参数数组可能给出 int 元素，统一为浮点避免类型歧义
    msg.transform.translation.x = float(tcp_offset_xyz[0])
    msg.transform.translation.y = float(tcp_offset_xyz[1])
    msg.transform.translation.z = float(tcp_offset_xyz[2])
    # 单位四元数 = 不做任何旋转，父子坐标系轴向平行
    msg.transform.rotation.x = 0.0
    msg.transform.rotation.y = 0.0
    msg.transform.rotation.z = 0.0
    msg.transform.rotation.w = 1.0
    return msg


class GraspTcpFrameNode(Node):
    """启动时发布一次抓取 TCP 静态变换的常驻节点。

    生命周期：构造期读取参数、构造消息并发送（静态广播器会自动以
    transient_local 语义重复发布，后加入的订阅者也能收到）；
    之后不需要任何周期性工作，main() 仅以 0.1s 超时自旋以响应外部关闭请求。
    """

    def __init__(self) -> None:
        super().__init__("rebotarm_grasp_tcp_frame")
        # 默认值仅用于占位，真实取值由参数文件（如 camera_ubuntu.yaml）覆盖
        self.declare_parameter("parent_frame", "end_link")
        self.declare_parameter("child_frame", "grasp_tcp")
        self.declare_parameter("tcp_offset_xyz", [0.0, 0.0, 0.0])

        self.parent_frame = str(self.get_parameter("parent_frame").value)
        self.child_frame = str(self.get_parameter("child_frame").value)
        self.tcp_offset_xyz = self._tuple3("tcp_offset_xyz")

        self.broadcaster = StaticTransformBroadcaster(self)
        self.transform = build_grasp_tcp_transform(
            parent_frame=self.parent_frame,
            child_frame=self.child_frame,
            tcp_offset_xyz=self.tcp_offset_xyz,
        )
        self.broadcaster.sendTransform(self.transform)
        self.get_logger().info(
            "grasp tcp frame ready: "
            f"{self.parent_frame} -> {self.child_frame}, offset={self.tcp_offset_xyz}"
        )

    def _tuple3(self, name: str) -> tuple[float, float, float]:
        """把三元素数组参数转换成 float 三元组，长度不符时抛 ValueError。"""
        values = list(self.get_parameter(name).value)
        if len(values) != 3:
            raise ValueError(f"{name} must contain exactly 3 values")
        return (float(values[0]), float(values[1]), float(values[2]))


def main(args=None) -> None:
    """入口：初始化、构造节点并以 0.1s 超时自旋，收到中断后安全析构。"""
    rclpy.init(args=args)
    node = GraspTcpFrameNode()
    try:
        while rclpy.ok():
            # 用短超时循环而非 spin()，便于在收到 Ctrl+C 或外部关闭时及时退出
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        # 外部已触发关闭时 rclpy 可能已 shutdown，重复调用会抛异常，故先判断
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

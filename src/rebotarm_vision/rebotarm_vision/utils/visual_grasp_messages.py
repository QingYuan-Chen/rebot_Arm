"""ROS pose conversion for visual grasp adapters."""
from __future__ import annotations
import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from ..visual_grasp_sequence import PoseTarget

def pose_to_target(pose: Pose) -> PoseTarget:
    """把位姿消息转成内部轻量元组表示（位置 3 元组 + 四元数 4 元组，单位均为 m / 无量纲）。"""

    return PoseTarget(
        position=(float(pose.position.x), float(pose.position.y), float(pose.position.z)),
        orientation=(
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        ),
    )


def target_to_pose_stamped(target: PoseTarget, frame_id: str) -> PoseStamped:
    """把内部位姿目标打包成带坐标系的位姿消息。

    `frame_id` 即目标坐标系（本节点默认 `base_link`），下游运动层按该坐标系解释位置。
    时间戳刻意填 0（rclpy 的零时刻），表示"使用最新可用变换"，避免用本地时钟去要求
    运动层做时间对齐；位置单位为 m，姿态为四元数。
    """

    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = rclpy.time.Time().to_msg()
    msg.pose.position.x = float(target.position[0])
    msg.pose.position.y = float(target.position[1])
    msg.pose.position.z = float(target.position[2])
    msg.pose.orientation.x = float(target.orientation[0])
    msg.pose.orientation.y = float(target.orientation[1])
    msg.pose.orientation.z = float(target.orientation[2])
    msg.pose.orientation.w = float(target.orientation[3])
    return msg

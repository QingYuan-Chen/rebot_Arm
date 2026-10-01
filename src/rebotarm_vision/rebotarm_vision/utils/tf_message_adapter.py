"""ROS TF/Pose 消息与纯数学变换之间的适配层。

本模块不创建 ROS 节点，也不查询 TF；调用方提供已经查询到的变换消息。
"""

from __future__ import annotations

from copy import deepcopy

from geometry_msgs.msg import Pose, TransformStamped

from .transform_points import Transform3D, quaternion_to_rotation_matrix, transform_pose_components


def transform_from_msg(tf_msg: TransformStamped) -> Transform3D:
    """将 TF 消息转换为平移单位 m、四元数顺序 xyzw 的内部变换。"""

    translation = tf_msg.transform.translation
    rotation = tf_msg.transform.rotation
    return Transform3D(
        translation=(float(translation.x), float(translation.y), float(translation.z)),
        rotation_xyzw=(float(rotation.x), float(rotation.y), float(rotation.z), float(rotation.w)),
    )


def transform_pose_message(pose: Pose, transform: Transform3D) -> Pose:
    """将 Pose 从源坐标系变换到目标坐标系，返回副本且不修改输入。"""

    transformed = deepcopy(pose)
    position, orientation = transform_pose_components(
        transform,
        (float(pose.position.x), float(pose.position.y), float(pose.position.z)),
        (
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        ),
    )
    transformed.position.x, transformed.position.y, transformed.position.z = position
    (
        transformed.orientation.x,
        transformed.orientation.y,
        transformed.orientation.z,
        transformed.orientation.w,
    ) = orientation
    return transformed


def apply_tcp_offset_to_pose(
    grasp_tcp_pose: Pose,
    tcp_offset_xyz: tuple[float, float, float],
) -> Pose:
    """把 TCP 位姿换算为末端连杆位姿，返回副本且保持原姿态。"""

    target = deepcopy(grasp_tcp_pose)
    rotation = quaternion_to_rotation_matrix(
        (
            float(grasp_tcp_pose.orientation.x),
            float(grasp_tcp_pose.orientation.y),
            float(grasp_tcp_pose.orientation.z),
            float(grasp_tcp_pose.orientation.w),
        )
    )
    ox, oy, oz = tcp_offset_xyz
    dx = rotation[0][0] * ox + rotation[0][1] * oy + rotation[0][2] * oz
    dy = rotation[1][0] * ox + rotation[1][1] * oy + rotation[1][2] * oz
    dz = rotation[2][0] * ox + rotation[2][1] * oy + rotation[2][2] * oz
    target.position.x = round(float(grasp_tcp_pose.position.x) - dx, 6)
    target.position.y = round(float(grasp_tcp_pose.position.y) - dy, 6)
    target.position.z = round(float(grasp_tcp_pose.position.z) - dz, 6)
    return target

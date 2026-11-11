"""位姿表示转换：位姿消息与“位置 + 旋转矩阵 / 欧拉角”之间的互转。

职责与位置
    本模块属于控制器的 ROS 位姿适配：依赖 Pose 消息，但不访问硬件、不建订阅或节点。
    pose_to_xyz_rpy 保留位姿拆分能力；当前服务和 Action 已把位姿规划入口迁到运动层，
    硬件层用它把正运动学结果打包成位姿消息（见 ArmController.current_pose）。

对外接口
    pose_to_xyz_rpy 接收 Pose 返回位置与 RPY；fk_to_pose 接收 FK 数值返回新的 Pose。

安全约束
    仅转换表示，不做目标准入、IK、限位或碰撞检查；输入有限性及旋转合法性由调用方保证。
    六轴组件对本模块的既有依赖保留，目录迁移未把这层 ROS 类型依赖改成共享数值接口。

统一约定(调用方必须遵守, 否则数值会静默错误)
    单位：位置为米（m），姿态角为弧度（rad）；
    四元数顺序：``(x, y, z, w)``，与 ROS 消息字段顺序一致；
    坐标系：输入与输出必须处于同一参考系（通常为基座 base_link），本模块不做任何
    坐标系变换。

文件结构（按定义顺序）
    pose_to_xyz_rpy：把位姿消息拆成 ``(x, y, z, roll, pitch, yaw)`` 六元组。
    fk_to_pose：把正运动学结果打包成位姿消息。
"""

from __future__ import annotations

import numpy as np
from geometry_msgs.msg import Pose
from tf_transformations import euler_from_quaternion, quaternion_from_matrix


def pose_to_xyz_rpy(pose: Pose) -> tuple[float, float, float, float, float, float]:
    """把位姿消息拆成 ``(x, y, z, roll, pitch, yaw)`` 六元组。

    位置单位为米，姿态角单位为弧度。 姿态按 tf_transformations 的默认静态轴序
    ('sxyz')解算，等价于 R = Rz(yaw)·Ry(pitch)·Rx(roll)，与 ROS 常见的 RPY
    约定一致；四元数按 (x, y, z, w) 取出。 调用方应保证四元数有限且归一化，
    本函数不独立校验目标合法性。
    """
    quat = [
        float(pose.orientation.x),
        float(pose.orientation.y),
        float(pose.orientation.z),
        float(pose.orientation.w),
    ]
    roll, pitch, yaw = euler_from_quaternion(quat)
    return (
        float(pose.position.x),
        float(pose.position.y),
        float(pose.position.z),
        float(roll),
        float(pitch),
        float(yaw),
    )


def fk_to_pose(position: np.ndarray, rotation: np.ndarray) -> Pose:
    """把正运动学结果打包成位姿消息。

    position 为 3 维位置向量（米）；rotation 为 3x3 正交旋转矩阵，表示末端在参考
    坐标系下的姿态。 二者必须来自同一坐标系下的同一次正运动学计算，本函数不做坐标
    变换。 这里组 4x4 齐次矩阵只是为了复用 quaternion_from_matrix（该函数要求 4x4
    输入），平移分量单独由 position 写入，所以齐次矩阵中的平移列保持为零。
    """
    # 4x4 齐次矩阵：只填左上 3x3 旋转块，平移保持 0
    mat = np.eye(4)
    mat[:3, :3] = rotation
    quat = quaternion_from_matrix(mat)

    pose = Pose()
    # 位置：米，与 rotation 处于同一参考系
    pose.position.x = float(position[0])
    pose.position.y = float(position[1])
    pose.position.z = float(position[2])
    # 四元数：quaternion_from_matrix 返回顺序为 (x, y, z, w)
    pose.orientation.x = float(quat[0])
    pose.orientation.y = float(quat[1])
    pose.orientation.z = float(quat[2])
    pose.orientation.w = float(quat[3])
    return pose

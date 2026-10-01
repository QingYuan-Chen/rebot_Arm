"""四元数与三维点/位姿变换的纯数学工具（不依赖 ROS，可单独测试）。

用途：把视觉侧算出的三维点或位姿从一个坐标系换算到另一个坐标系，典型场景是把相机
坐标系下测得的参考点（例如 ArUco 标记中心）经手眼标定得到的位姿变换到机械臂基座
坐标系，交给运动侧使用。

统一约定：

- 四元数一律按 ``xyzw`` 顺序存放，语义是「旋转」，调用方需保证其为单位四元数；
- 旋转矩阵为 3x3 行主序嵌套元组，``R[i][j]`` 是第 i 行第 j 列，作用方式为 ``p' = R·p``；
- 长度单位统一为米（与 ROS 一致），角度不出现（只用四元数）；
- ``transform_point`` 的结果四舍五入到小数点后 6 位（1e-6 m = 1 µm），使浮点结果
  可比较、可复现、可安全打印。

本模块除 ``normalize_quaternion`` 的零四元数保护外不做输入校验，也不抛业务异常：
非法数据（NaN、非单位四元数）不会被拦截，属于调用方的责任。
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Transform3D:
    """一个刚体变换：先旋转、再平移。

    ``translation``：平移向量 (x, y, z)，单位 m，表示子坐标系原点在父坐标系中的位置。
    ``rotation_xyzw``：旋转四元数 (x, y, z, w)，把子坐标系中的向量转到父坐标系。
    """

    translation: tuple[float, float, float]
    rotation_xyzw: tuple[float, float, float, float]


def quaternion_to_rotation_matrix(
    rotation_xyzw: tuple[float, float, float, float],
) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    """把单位四元数转成 3x3 旋转矩阵（行主序）。

    要求输入已是单位四元数：本函数**不做归一化**，长度不为 1 的四元数会让结果带上
    缩放，进而使变换后的距离整体失真。上游若来自传感器/参数文件，应先调用
    ``normalize_quaternion``。
    """

    qx, qy, qz, qw = rotation_xyzw
    # 标准展开式 R = I + 2w·[q]× + 2·[q]×²：外层三个元组依次是 R 的第 1/2/3 行，
    # 分别对应变换后坐标的 x/y/z 分量；每行内的三个元素是列。
    return (
        (
            1.0 - 2.0 * qy * qy - 2.0 * qz * qz,
            2.0 * qx * qy - 2.0 * qz * qw,
            2.0 * qx * qz + 2.0 * qy * qw,
        ),
        (
            2.0 * qx * qy + 2.0 * qz * qw,
            1.0 - 2.0 * qx * qx - 2.0 * qz * qz,
            2.0 * qy * qz - 2.0 * qx * qw,
        ),
        (
            2.0 * qx * qz - 2.0 * qy * qw,
            2.0 * qy * qz + 2.0 * qx * qw,
            1.0 - 2.0 * qx * qx - 2.0 * qy * qy,
        ),
    )


def transform_point(
    transform: Transform3D,
    point: tuple[float, float, float],
) -> tuple[float, float, float]:
    """把点从变换的源坐标系换算到目标坐标系：``p' = R·p + t``。

    ``point`` 与返回值均为 (x, y, z)，单位 m。返回值保留 6 位小数（µm 级），
    目的是让结果可比较、可复现。
    """

    rotation = quaternion_to_rotation_matrix(transform.rotation_xyzw)
    tx, ty, tz = transform.translation
    px, py, pz = point
    x = rotation[0][0] * px + rotation[0][1] * py + rotation[0][2] * pz + tx
    y = rotation[1][0] * px + rotation[1][1] * py + rotation[1][2] * pz + ty
    z = rotation[2][0] * px + rotation[2][1] * py + rotation[2][2] * pz + tz
    return (round(x, 6), round(y, 6), round(z, 6))


def normalize_quaternion(
    rotation_xyzw: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    """把四元数缩放到单位长度。

    退化输入（模长 <= 0，即全零四元数）不抛异常，直接返回单位旋转 (0, 0, 0, 1)：
    本模块面向「结果可继续参与运算」的场景，静默回退比中断流程更合适；需要严格
    报错的调用方请使用会抛异常的实现。
    """

    qx, qy, qz, qw = rotation_xyzw
    norm = (qx * qx + qy * qy + qz * qz + qw * qw) ** 0.5
    if norm <= 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return (qx / norm, qy / norm, qz / norm, qw / norm)


def multiply_quaternions(
    left_xyzw: tuple[float, float, float, float],
    right_xyzw: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    """四元数 Hamilton 乘积，表示旋转复合 ``R(left)·R(right)``。

    两个输入先各自归一化，乘积结果再归一化一次以抑制反复相乘累积的数值漂移。
    乘法不可交换：左乘相当于在父坐标系里叠加旋转，右乘相当于绕子坐标系自身的
    局部轴旋转——调用方必须明确需要哪一种。
    """

    lx, ly, lz, lw = normalize_quaternion(left_xyzw)
    rx, ry, rz, rw = normalize_quaternion(right_xyzw)
    return normalize_quaternion(
        (
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        )
    )


def transform_pose_components(
    transform: Transform3D,
    position_xyz: tuple[float, float, float],
    orientation_xyzw: tuple[float, float, float, float],
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    """同时变换位姿的位置与姿态分量。

    位置按 ``p' = R·p + t`` 变换；姿态相对父坐标系做旋转复合（左乘变换的旋转），
    因此返回的四元数描述的是同一个姿态在新坐标系下的表示。返回顺序与输入一致：
    ``(position_xyz, orientation_xyzw)``。
    """

    position = transform_point(transform, position_xyz)
    orientation = multiply_quaternions(transform.rotation_xyzw, orientation_xyzw)
    return (position, orientation)

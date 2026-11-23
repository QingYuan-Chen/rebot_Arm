"""六轴机械臂力控共享基础契约,数值校验与规范包资源定位.

职责与位置
    rebotarm_force_control 的最底层共享基础模块,向重力模型,关节阻抗及硬件适配器
    提供统一关节顺序,向量/标量校验,URDF 位置限位读取与 ROS 包资源定位.

对外接口
    JOINT_NAMES: 固定六轴顺序 ("joint1", ..., "joint6").
    POSITION_LIMITS / read_position_limits: 规范 URDF 六轴位置边界.
    vector / six_vector / finite_scalar / validate_position_limits: 输入维度与有限性校验.
    package_file: 按安装索引优先,无索引时回退源码工作区定位包内资源.

安全与并发约束
    零 ROS 节点,硬件驱动,MuJoCo 或线程依赖;导入仅解析规范 URDF 位置范围,不连接设备.
    显式加载的其他动力学模型须与 POSITION_LIMITS 逐轴一致,不可借局部覆盖扩大限位.

文件结构(按定义顺序)
    JOINT_NAMES → 数值与限位校验 → package_file / read_position_limits → POSITION_LIMITS.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Sequence
import xml.etree.ElementTree as ET

import numpy as np


# 模型与反馈必须保持这六个关节的名称,顺序和坐标定义,不能通过偏移反馈来适配模型.
JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 7))


def vector(value, size: int, label: str) -> np.ndarray:
    """将输入转换为指定长度的一维浮点向量,拒绝 NaN,无穷大或维数不符.

    label 用于指出出错参数;这里只检查形状和有限性,不检查关节限位或力矩范围.
    """
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f"{label} must contain {size} finite values")
    return result


def six_vector(value, label: str) -> np.ndarray:
    """将输入转为长度为 6 的有限浮点向量."""
    return vector(value, 6, label)


def finite_scalar(value, label: str, *, allow_zero: bool = False) -> float:
    """校验布尔以外的有限标量数值."""
    if isinstance(value, bool):
        raise ValueError(f"{label} must be a numeric scalar")
    scalar = float(value)
    if not math.isfinite(scalar) or (scalar < 0.0 if allow_zero else scalar <= 0.0):
        comparator = "nonnegative" if allow_zero else "positive"
        raise ValueError(f"{label} must be finite and {comparator}")
    return scalar


def validate_position_limits(
    position_limits: Sequence[tuple[float, float]],
) -> tuple[tuple[float, float], ...]:
    """核对六轴位置上下界逐轴有限且严格递增."""
    if len(position_limits) != len(JOINT_NAMES):
        raise ValueError("position_limits must contain six (lower, upper) bounds")
    normalized = []
    for index, bounds in enumerate(position_limits):
        if len(bounds) != 2:
            raise ValueError(f"position limits for {JOINT_NAMES[index]} must contain two bounds")
        lower, upper = float(bounds[0]), float(bounds[1])
        if not math.isfinite(lower) or not math.isfinite(upper) or lower >= upper:
            raise ValueError(f"invalid position limits for {JOINT_NAMES[index]}")
        normalized.append((lower, upper))
    return tuple(normalized)


def package_file(package: str, relative: str) -> Path:
    """定位包内资源:ROS 安装索引优先,未加载索引的源码环境才回退源码布局.

    package 为包名,relative 为包内相对路径;选中位置缺少文件时报告实际路径,
    不再静默改用另一处资源.源码回退仅用于没有 ament 索引的离线环境.
    """
    try:
        from ament_index_python.packages import get_package_share_directory
    except ImportError:
        share = None
    else:
        try:
            share = Path(get_package_share_directory(package))
        except LookupError:
            share = None
    if share is not None:
        path = share / relative
        if not path.is_file():
            raise FileNotFoundError(f"missing {path}")
        return path
    # 本模块平铺于包根,parents[2] 为源码工作区的 src/.
    path = Path(__file__).resolve().parents[2] / package / relative
    if not path.is_file():
        raise FileNotFoundError(f"missing {path}")
    return path


def read_position_limits(path: str | Path) -> tuple[tuple[float, float], ...]:
    """按六轴固定顺序读取 URDF 的有限位置范围;缺失或重复关节一律拒绝."""
    joints = ET.parse(path).getroot().findall("joint")
    result = []
    for name in JOINT_NAMES:
        matches = [joint for joint in joints if joint.get("name") == name]
        if len(matches) != 1 or matches[0].get("type") != "revolute":
            raise ValueError(f"{name} must be a unique bounded revolute joint")
        limit = matches[0].find("limit")
        try:
            bounds = (float(limit.get("lower")), float(limit.get("upper")))
        except (AttributeError, TypeError, ValueError) as exc:
            raise ValueError(f"{name} is missing finite position limits") from exc
        if not np.isfinite(bounds).all() or bounds[0] >= bounds[1]:
            raise ValueError(f"{name} has invalid position limits")
        result.append(bounds)
    return tuple(result)


# 所有控制与反馈边界共享规范模型的只读数值;导入仅解析资源,不连接设备.
POSITION_LIMITS = read_position_limits(
    package_file("rebotarm_moveit_config", "config/rebotarm.urdf")
)

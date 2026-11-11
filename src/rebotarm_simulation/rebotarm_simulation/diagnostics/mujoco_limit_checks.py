"""仿真执行器档位与 URDF 关节限位的一致性检查。

职责与位置
    纯离线校验模块：不导入物理引擎、不依赖 ROS 运行时、不接触硬件。它把运动规划
    配置包中 URDF 声明的关节限位，与 canonical ``robot.xml`` 的执行器量程摘要
    逐关节比对，返回全部漂移项。

为什么必须检查
    仿真里执行器的 ``ctrlrange`` 一旦宽于 URDF 的 ``<limit lower/upper>``，轨迹就会在
    仿真中"成功"到达真机根本到不了的角度，仿真结论无法迁移到真实机械臂；反过来过窄
    则会掩盖真实可用工作空间。这里把两侧的一致性做成可断言的结果。

对外接口
    ``motor_profile_position_limit_mismatches``  位置限位漂移列表（空列表即一致）；
    ``load_urdf_joint_limits``                   URDF 的位置/力矩/速度限位，供跨层比较；
    ``load_urdf_position_limits``                只取位置上下限的紧凑视图。

单位约定
    旋转关节角度为弧度（rad），``effort`` 为力矩（N·m），``velocity`` 为角速度（rad/s）；
    滑移关节（夹爪手指）则依次为米（m）、牛顿（N）、米每秒（m/s）。
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable
import xml.etree.ElementTree as ET

from rebotarm_simulation.core.resource_paths import package_resource



@dataclass(frozen=True)
class MotorProfile:
    """Canonical robot.xml 中一个关节的执行器限位摘要。"""

    joint: str
    ctrlrange: str
    forcerange: str
    kp: str = ""
    kv: str | None = None




def load_canonical_motor_profiles(robot_xml: Path | None = None) -> list[MotorProfile]:
    """从 canonical robot.xml 读取执行器限位摘要，不复制一份硬编码档案。"""
    path = robot_xml or package_resource("rebotarm_simulation", "models/rebotarm/robot.xml")
    root = ET.parse(path).getroot()
    joint_ranges = {
        joint.get("name"): joint.get("range")
        for joint in root.findall(".//joint")
        if joint.get("name") and joint.get("range")
    }
    profiles: list[MotorProfile] = []
    for actuator in root.findall(".//actuator/*"):
        joint = actuator.get("joint")
        ctrlrange = actuator.get("ctrlrange")
        forcerange = actuator.get("forcerange")
        position_range = joint_ranges.get(joint)
        if joint and position_range and forcerange and joint.startswith("joint"):
            profiles.append(MotorProfile(joint, position_range, forcerange))
    return profiles


@dataclass(frozen=True)
class PositionLimitMismatch:
    """单个关节在 URDF 与仿真执行器档位之间的位置限位漂移。

    只有在两侧差值超出容差时才生成该记录，用来把"仿真能到、真机到不了"这类
    不一致显式暴露给测试与人工复核。
    """

    joint: str  # 关节名，例如 joint1..joint6
    urdf_lower: float  # URDF 侧位置下限（rad）
    urdf_upper: float  # URDF 侧位置上限（rad）
    mujoco_lower: float  # 仿真执行器 ctrlrange 下限（rad）
    mujoco_upper: float  # 仿真执行器 ctrlrange 上限（rad）


@dataclass(frozen=True)
class UrdfJointLimit:
    """URDF 中一个关节的完整限位信息，供跨层一致性检查复用。"""

    lower: float  # 位置下限（rad 或 m）
    upper: float  # 位置上限（rad 或 m）
    effort: float  # 最大力矩/力（N·m 或 N），同时是仿真 forcerange 的依据
    velocity: float  # 最大速度（rad/s 或 m/s），运行时速度限位必须不超过它


def motor_profile_position_limit_mismatches(
    urdf_path: Path,
    motor_profiles: Iterable[MotorProfile],
    *,
    tolerance: float = 1e-9,
) -> list[PositionLimitMismatch]:
    """逐关节比对仿真执行器档位与 URDF 位置限位，返回全部漂移项。

    参数：
        urdf_path: URDF 路径；未声明位置限位的关节会被跳过。
        motor_profiles: 仿真执行器档位，其 ``ctrlrange`` 为 MuJoCo 风格的空格分隔字符串。
        tolerance: 判定容差（rad）。默认 1e-9 表示要求两侧数值实质相等；调大可用于
            容忍不同导出链路带来的浮点舍入。

    返回：
        漂移记录列表，空列表表示完全一致。只在两侧都存在的关节上比对，仿真独有的
        关节（例如手指滑移关节）不参与，避免误报。
    """
    urdf_limits = load_urdf_position_limits(urdf_path)
    mismatches: list[PositionLimitMismatch] = []
    for profile in motor_profiles:
        if profile.joint not in urdf_limits:
            continue
        urdf_lower, urdf_upper = urdf_limits[profile.joint]
        # 仿真侧量程取自执行器 ctrlrange，而不是关节自身的 range：真正限制仿真的
        # 是执行器量程，二者在档位构建时被分别注入。
        mujoco_lower, mujoco_upper = _parse_ctrlrange(profile.ctrlrange)
        if abs(urdf_lower - mujoco_lower) > tolerance or abs(urdf_upper - mujoco_upper) > tolerance:
            mismatches.append(
                PositionLimitMismatch(
                    joint=profile.joint,
                    urdf_lower=urdf_lower,
                    urdf_upper=urdf_upper,
                    mujoco_lower=mujoco_lower,
                    mujoco_upper=mujoco_upper,
                )
            )
    return mismatches


def load_urdf_position_limits(urdf_path: Path) -> dict[str, tuple[float, float]]:
    """只取位置上下限的紧凑视图：``{关节名: (lower, upper)}``。"""
    return {
        name: (limit.lower, limit.upper)
        for name, limit in load_urdf_joint_limits(urdf_path).items()
    }


def load_urdf_joint_limits(urdf_path: Path) -> dict[str, UrdfJointLimit]:
    """解析 URDF 中所有带完整 ``<limit>`` 的关节。

    任一属性（``lower``/``upper``/``effort``/``velocity``）缺失的关节会被静默跳过，
    以免用 0 冒充"未声明限位"，把缺失当成"限位为零"的假结论。
    """
    root = ET.parse(urdf_path).getroot()
    limits: dict[str, UrdfJointLimit] = {}
    for joint in root.findall("joint"):
        name = joint.get("name")
        limit = joint.find("limit")
        if not name or limit is None:
            continue
        lower = limit.get("lower")
        upper = limit.get("upper")
        effort = limit.get("effort")
        velocity = limit.get("velocity")
        if lower is None or upper is None or effort is None or velocity is None:
            continue
        limits[name] = UrdfJointLimit(
            lower=float(lower),
            upper=float(upper),
            effort=float(effort),
            velocity=float(velocity),
        )
    return limits


def _parse_ctrlrange(value: str) -> tuple[float, float]:
    """把 MuJoCo 风格的 ``"lower upper"`` 字符串解析为两个浮点数。

    这不是自由格式解析：必须恰好两个数，否则抛 ``ValueError``——宁可报错也不要
    拿一个错误的限位继续比对，否则会产生"检查通过"的假象。
    """
    parts = [float(part) for part in str(value).split()]
    if len(parts) != 2:
        raise ValueError(f"expected two ctrlrange values, got: {value}")
    return parts[0], parts[1]

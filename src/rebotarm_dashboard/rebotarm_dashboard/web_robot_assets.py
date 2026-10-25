"""面板专用的机器人资源工具：夹爪行程换算、URDF 限位读取与网格路径安全解析。

本模块把"面板展示与安全钳位所需的模型参数"从 URDF 与运动规划配置中读出来，并以纯函数
形式提供换算，供面板节点在启动时构建关节位置/速度限位表，以及 HTTP 层提供模型文件。

包含四类能力
    1. 夹爪开度换算：把请求位置钳制到限位区间，并换算成左右指棱柱关节的位移；
    2. 资源改写：把 URDF 里的网格包 URI 改写成面板自身的 HTTP 路由；
    3. 取文件安全：网格文件名经过防路径穿越校验后才允许读取；
    4. 限位合并：优先使用模型/配置中读到的限位，缺失时退回调用方给的默认值。

单位约定
    长度一律为米（m），关节角为弧度（rad），速度限位为 rad/s。
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import yaml


# 历史开口接口默认范围（米）：0 对应关节零位，0.09 对应双侧总行程。
# 新软指零位仍有间隙，真实净开口待标定；仅在配置缺失时使用此显示映射。
DEFAULT_GRIPPER_LIMITS_M = (0.0, 0.09)


def clamp_gripper_opening(position_m: float, limits_m: tuple[float, float]) -> float:
    """把请求的夹爪位置钳制到限位内，并转成"以下限为 0 的开度"。

    参数
        position_m：请求的夹爪位置，单位米；可为负或超过行程，都会被钳位。
        limits_m：(下限, 上限) 二元组，单位米；上限小于下限时会自动交换，
            因此调用方把顺序写反也不会得到错误结果。
    返回
        0 .. (上限 - 下限) 之间的开度（米）。返回值以 0 表示闭合位、以行程长度表示全开，
        是"离闭合位多远"的相对量，而不是绝对位置坐标。
    """
    lower, upper = float(limits_m[0]), float(limits_m[1])
    if upper < lower:
        lower, upper = upper, lower
    value = min(max(float(position_m), lower), upper)
    return value - lower


def gripper_opening_to_finger_joint_positions(
    position_m: float,
    limits_m: tuple[float, float] = DEFAULT_GRIPPER_LIMITS_M,
) -> tuple[float, float]:
    """把夹爪开度换算成左右两指棱柱关节的位移（米）。

    夹爪是平行两指结构：两指沿同一轴反向对称运动，所以单指位移是开度的一半，
    左指取正、右指取负（与模型中两个指关节相反的限位方向一致）。返回值按
    (左指, 右指) 顺序给出，可直接写入这两个关节的位置。

    参数 position_m 为请求位置（米）；limits_m 缺省用 DEFAULT_GRIPPER_LIMITS_M，
    实际使用时通常传入面板参数给出的夹爪限位。
    """
    half_opening = 0.5 * clamp_gripper_opening(position_m, limits_m)
    return half_opening, -half_opening


def rewrite_package_mesh_uris(urdf_text: str, *, mesh_route: str = "meshes") -> str:
    """把 URDF 文本里的网格包 URI 前缀改写成面板自己的 HTTP 路由前缀。

    浏览器无法解析网格包协议，因此返回给前端的 URDF 必须把该前缀换成 ``<mesh_route>/``，
    由 HTTP 层的 /robot/meshes/<文件名> 路由提供文件。mesh_route 末尾的斜杠会被去掉，
    避免拼出重复斜杠；除这一处前缀替换外不改动 URDF 的其它内容。
    """
    prefix = "package://rebotarm_moveit_config/meshes/"
    route = mesh_route.rstrip("/")
    return str(urdf_text).replace(prefix, f"{route}/")


def safe_mesh_path(mesh_dir: Path, requested_name: str) -> Path | None:
    """把请求的网格文件名解析为限定目录内的真实文件路径；不合法或不存在时返回 None。

    这是防路径穿越的安全门：含 "/" 或 "\\" 的请求直接拒绝，再取 basename，
    排除空名与 "."/".."，最后要求目标确实存在且是普通文件。
    返回 None 表示调用方应回 404，不要回退到目录列表或其它路径。
    """
    raw_name = str(requested_name)
    if "/" in raw_name or "\\" in raw_name:
        return None
    name = Path(raw_name).name
    if not name or name in {".", ".."}:
        return None
    path = mesh_dir / name
    if not path.exists() or not path.is_file():
        return None
    return path


def load_urdf_joint_limits(urdf_path: Path, joint_names: tuple[str, ...]) -> dict[str, tuple[float, float]]:
    """从 URDF 中读取指定关节的 lower/upper 位置限位。

    单位是弧度（旋转关节）或米（棱柱关节），由关节类型决定。只返回 joint_names 中列出、
    且 <limit> 同时带 lower 与 upper 的关节；缺少 limit 元素或限位属性不全的关节会被跳过，
    由调用方用默认值补齐。上下限写反时会自动交换。
    文件不存在或 XML 非法会直接抛异常——模型缺失属于必须暴露的部署错误，不静默降级。
    """
    root = ET.fromstring(urdf_path.read_text(encoding="utf-8"))
    wanted = set(joint_names)
    limits: dict[str, tuple[float, float]] = {}
    for joint in root.findall("joint"):
        name = str(joint.attrib.get("name", ""))
        if name not in wanted:
            continue
        limit = joint.find("limit")
        if limit is None:
            continue
        if "lower" not in limit.attrib or "upper" not in limit.attrib:
            continue
        lower = float(limit.attrib["lower"])
        upper = float(limit.attrib["upper"])
        if upper < lower:
            lower, upper = upper, lower
        limits[name] = (lower, upper)
    return limits


def merge_joint_limits(
    *,
    joint_names: tuple[str, ...],
    fallback_limits: dict[str, tuple[float, float]],
    preferred_limits: dict[str, tuple[float, float]],
) -> dict[str, tuple[float, float]]:
    """按关节名合并位置限位：优先用 preferred_limits，缺失的关节退回 fallback_limits。

    fallback_limits 必须覆盖 joint_names 里的每一个名字，否则会 KeyError——"每个关节都有
    可用限位"由调用方保证，这里不做静默兜底，避免把配置错误掩盖成默认行为。
    preferred_limits 通常是 URDF 中读到的限位，fallback 则是面板参数给的粗限位。
    """
    merged: dict[str, tuple[float, float]] = {}
    for name in joint_names:
        merged[name] = preferred_limits.get(name, fallback_limits[name])
    return merged


def load_gripper_limits(config: dict[str, Any] | None) -> tuple[float, float]:
    """从配置字典中解析夹爪开度限位（米），兼容多套历史键名。

    依次尝试三组键名：(min_position_m, max_position_m)、(min_opening_m, max_opening_m)、
    (closed_position_m, open_position_m)，命中第一组即返回；上下限写反会自动交换。
    若配置给出的是 gripper 列表，则用首个元素递归解析，但只有结果不同于默认值时才采纳，
    以免把递归兜底得到的默认值误当成显式配置。
    任何缺失、类型不符或键不完整的配置都退回 DEFAULT_GRIPPER_LIMITS_M。
    """
    if not isinstance(config, dict):
        return DEFAULT_GRIPPER_LIMITS_M

    candidates = (
        ("min_position_m", "max_position_m"),
        ("min_opening_m", "max_opening_m"),
        ("closed_position_m", "open_position_m"),
    )
    for lower_key, upper_key in candidates:
        if lower_key in config and upper_key in config:
            lower = float(config[lower_key])
            upper = float(config[upper_key])
            if upper < lower:
                lower, upper = upper, lower
            return lower, upper

    gripper_items = config.get("gripper")
    if isinstance(gripper_items, list) and gripper_items:
        first = gripper_items[0]
        if isinstance(first, dict):
            nested = load_gripper_limits(first)
            if nested != DEFAULT_GRIPPER_LIMITS_M:
                return nested

    return DEFAULT_GRIPPER_LIMITS_M


def load_moveit_velocity_limits(path: Path, joint_names: tuple[str, ...]) -> dict[str, float]:
    """从运动规划配置的关节限位 YAML 中读取每个关节的最大速度（rad/s）。

    只接受同时满足三点的条目：has_velocity_limits 为真、max_velocity 能转成 float、
    且取值大于 0。文件不存在、内容为空、键缺失或类型异常都会跳过该关节——返回的是
    "能确认的部分"，其余关节由 merge_velocity_limits 填默认值。
    注意：YAML 根节点必须是映射，否则随后的取值会抛 AttributeError；当前调用方传入的
    都是本仓自带的受控配置文件。
    """
    if not path.is_file():
        return {}
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    joint_limits = data.get("joint_limits", {})
    limits: dict[str, float] = {}
    for name in joint_names:
        entry = joint_limits.get(name, {})
        if not isinstance(entry, dict):
            continue
        if not bool(entry.get("has_velocity_limits", False)):
            continue
        try:
            value = float(entry["max_velocity"])
        except (KeyError, TypeError, ValueError):
            continue
        if value > 0.0:
            limits[name] = value
    return limits


def merge_velocity_limits(
    *,
    joint_names: tuple[str, ...],
    default_limit: float,
    preferred_limits: dict[str, float],
) -> dict[str, float]:
    """按关节名合并速度限位：优先生效 preferred_limits，缺失时用 default_limit。

    default_limit 来自面板参数中的最大关节速度（rad/s）；返回值一定是每个关节都有
    正上限的完整表，下游可直接按关节名取用。
    """
    return {
        name: float(preferred_limits.get(name, default_limit))
        for name in joint_names
    }

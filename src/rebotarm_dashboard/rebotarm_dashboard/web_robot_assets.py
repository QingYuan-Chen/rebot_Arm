"""面板专用的机器人资源工具：夹爪行程换算、URDF 限位读取与网格路径安全解析。

本模块把"面板展示与安全钳位所需的模型参数"从 URDF 与运动规划配置中读出来，并以纯函数
形式提供换算，供面板节点在启动时构建关节位置/速度限位表，以及 HTTP 层提供模型文件。

包含四类能力
    1. 夹爪开度换算：把请求位置钳制到限位区间，并换算成左右指棱柱关节的位移；
    2. 资源改写：把 URDF 里的网格包 URI 改写成面板自身的 HTTP 路由；
    3. 取文件安全：网格文件名经过防路径穿越校验后才允许读取；
    4. 读取模型中的完整关节限位。

单位约定
    长度一律为米（m），关节角为弧度（rad），速度限位为 rad/s。
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
import math
from pathlib import Path


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

    每个请求的关节都必须存在且具有有效上下限。
    """
    root = ET.fromstring(urdf_path.read_text(encoding="utf-8"))
    wanted = set(joint_names)
    limits: dict[str, tuple[float, float]] = {}
    for joint in root.findall("joint"):
        name = str(joint.attrib.get("name", ""))
        if name not in wanted:
            continue
        limit = joint.find("limit")
        if limit is None or "lower" not in limit.attrib or "upper" not in limit.attrib:
            raise ValueError(f"URDF joint limits missing: {name}")
        lower = float(limit.attrib["lower"])
        upper = float(limit.attrib["upper"])
        if not math.isfinite(lower) or not math.isfinite(upper) or upper < lower:
            raise ValueError(f"invalid URDF joint limits: {name}")
        limits[name] = (lower, upper)
    missing = wanted - limits.keys()
    if missing:
        raise ValueError(f"URDF joint limits missing: {', '.join(sorted(missing))}")
    return limits

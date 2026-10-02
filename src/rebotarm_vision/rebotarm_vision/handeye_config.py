"""手眼标定参数的读取与校验。

职责：把手眼标定 YAML（默认 config/handeye.yaml）解析成不可变的 HandeyeConfig，
供启动文件以静态 TF 的方式把相机坐标系挂到机械臂末端（或反向）上。
本模块只做"读取 + 格式校验"，不做数值求解，也不做坐标变换运算。

YAML 结构约定（键名大小写敏感）：
    handeye:
      parent_frame: <父坐标系名>
      child_frame: <子坐标系名>
      translation: {x: <米>, y: <米>, z: <米>}
      rotation:    {x: <无量纲>, y: <无量纲>, z: <无量纲>, w: <无量纲>}
其中 rotation 是四元数 (x, y, z, w)，需满足模长约为 1；
translation 单位为米，表示子坐标系原点在父坐标系中的平移。

安全约束：标定结果是抓取位姿换算的基准，缺少任一必需字段时直接抛异常终止，
绝不使用默认值静默兜底——用错的外参会让机械臂撞向错误位置。
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


@dataclass(frozen=True)
class HandeyeConfig:
    """一次手眼标定的结果，对应一条父→子静态坐标变换。"""

    parent_frame: str
    child_frame: str
    translation_x: float
    translation_y: float
    translation_z: float
    rotation_x: float
    rotation_y: float
    rotation_z: float
    rotation_w: float

    def as_static_transform_arguments(self) -> list[str]:
        # 使用 ROS 2 tf2_ros 当前推荐的显式参数名，避免旧版位置参数弃用警告。
        # 显式 frame-id/child-frame-id 也降低了平移与四元数顺序误配的风险。
        return [
            "--x", str(self.translation_x),
            "--y", str(self.translation_y),
            "--z", str(self.translation_z),
            "--qx", str(self.rotation_x),
            "--qy", str(self.rotation_y),
            "--qz", str(self.rotation_z),
            "--qw", str(self.rotation_w),
            "--frame-id", self.parent_frame,
            "--child-frame-id", self.child_frame,
        ]


def _required_mapping(data: dict[str, Any], key: str) -> dict[str, Any]:
    # 取出必需的子映射；缺失或类型不是映射时抛异常（不做默认值兜底）。
    value = data.get(key)
    if not isinstance(value, dict):
        raise ValueError(f"handeye config missing mapping: {key}")
    return value


def _required_float(data: dict[str, Any], key: str) -> float:
    # 取出必需的浮点字段；键不存在时抛异常，存在但无法转 float 时由 float() 抛异常。
    if key not in data:
        raise ValueError(f"handeye config missing value: {key}")
    return float(data[key])


def load_handeye_config(path: str | Path) -> HandeyeConfig:
    """读取并校验手眼标定文件。

    参数:
        path: YAML 文件路径（字符串或 Path）。
    返回:
        HandeyeConfig 实例。
    异常:
        OSError: 文件不存在或不可读。
        ValueError: 根节点不是映射、缺少 handeye/translation/rotation、
            rotation 或 translation 缺少 x/y/z(/w)，或 parent_frame/child_frame 为空。
        yaml.YAMLError: YAML 语法错误。
    副作用: 只读文件，无状态写入。
    """
    config_path = Path(path)
    # 空文件会被 safe_load 解析成 None，这里用 or {} 统一成空映射，
    # 让后续 _required_mapping 抛出更明确的"缺少 handeye 段"错误。
    data = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    if not isinstance(data, dict):
        raise ValueError("handeye config root must be a mapping")

    handeye = _required_mapping(data, "handeye")
    translation = _required_mapping(handeye, "translation")
    rotation = _required_mapping(handeye, "rotation")

    # 坐标系名做 strip：YAML 中容易误带尾随空格，而 TF 坐标系名是精确匹配的，
    # 带空格的 frame_id 在运行期只会表现为"查不到变换"，难以排查，因此这里提前规范化。
    parent_frame = str(handeye.get("parent_frame", "")).strip()
    child_frame = str(handeye.get("child_frame", "")).strip()
    if not parent_frame or not child_frame:
        raise ValueError("handeye config requires parent_frame and child_frame")

    return HandeyeConfig(
        parent_frame=parent_frame,
        child_frame=child_frame,
        translation_x=_required_float(translation, "x"),
        translation_y=_required_float(translation, "y"),
        translation_z=_required_float(translation, "z"),
        rotation_x=_required_float(rotation, "x"),
        rotation_y=_required_float(rotation, "y"),
        rotation_z=_required_float(rotation, "z"),
        rotation_w=_required_float(rotation, "w"),
    )

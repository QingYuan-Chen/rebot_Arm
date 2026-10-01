"""抓取候选位姿的坐标系适配器（纯函数，不接触 TF 监听器本身）。

职责：把候选位姿从"来源坐标系"换算到"目标坐标系"。变换查询通过调用方注入的回调
完成，因此本模块不依赖运行中的 TF 系统，可以直接单测。消息转换与位姿运算复用
``tf_message_adapter``，避免同一套变换逻辑出现两份。
"""

from __future__ import annotations

from copy import deepcopy
from typing import Callable

from geometry_msgs.msg import Pose

from .utils.tf_message_adapter import transform_from_msg, transform_pose_message


# 变换查询回调签名：(target_frame, source_frame) -> 带 .transform 字段的变换消息。
# 返回对象只要求具备 transform.translation/rotation 属性，具体类型由调用方决定。
TransformLookup = Callable[[str, str], object]


def transform_candidate_pose_to_target_frame(
    pose: Pose,
    *,
    source_frame: str,
    target_frame: str,
    lookup_transform: TransformLookup,
) -> Pose:
    """把候选位姿从来源坐标系换算到目标坐标系。

    参数：
    - ``pose``：待换算的位姿（位置单位 m，姿态为四元数）。
    - ``source_frame``：``pose`` 当前所在的坐标系名。
    - ``target_frame``：希望得到的坐标系名。
    - ``lookup_transform``：变换查询回调，按 ``(target_frame, source_frame)`` 顺序调用，
      即"求 target 在 source 中的表示"所需的变换。

    返回：换算后的新位姿对象；入参 ``pose`` 不会被修改（返回的是副本）。

    边界条件：``target_frame`` 或 ``source_frame`` 为空、或两者相同时直接返回入参的
    深拷贝——不去查询 TF。这样上游"未标注坐标系"的候选不会被误当成需要变换，
    也避免了自变换查询（某些 TF 实现对相同 frame 的查询会抛异常）。
    """
    if not target_frame or not source_frame or source_frame == target_frame:
        return deepcopy(pose)
    transform_msg = lookup_transform(target_frame, source_frame)
    return transform_pose_message(deepcopy(pose), transform_from_msg(transform_msg))

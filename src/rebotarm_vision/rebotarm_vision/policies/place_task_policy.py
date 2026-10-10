"""抓取成功后的放置阶段构造。

放置功能启用时，在抓取序列尾部追加三个阶段：移动到放置点 → 在放置点张开夹爪
→ 沿 +z 抬起退出。本模块只生成阶段描述，不做规划与执行；阶段名是执行节点的
分派与日志约定，属于对外可见标识，勿改。

位姿单位为米，四元数顺序 (x, y, z, w)，坐标系与抓取序列一致（由上游统一决定）。
"""

from __future__ import annotations

from dataclasses import dataclass

from ..visual_grasp_sequence import PoseTarget, VisualGraspStage


@dataclass(frozen=True)
class PlaceTaskConfig:
    """放置任务参数；``enabled`` 为 False 时不生成任何阶段。"""

    # 总开关，默认 False（保持「只抓不放」的历史行为）。
    enabled: bool = False
    # 放置点位置 (x, y, z)，单位米，基座系。
    place_position_xyz: tuple[float, float, float] = (0.20, -0.20, 0.25)
    # 放置点姿态四元数 (x, y, z, w)；默认单位四元数表示不改变末端朝向。
    place_orientation_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    # 在放置点张开夹爪的目标开口，单位米；应大于被抓物体的宽度以便顺利脱离。
    open_position_m: float = 0.08
    # 张开阶段的最大力矩，单位 N·m；越大越有力，也越容易把轻物体顶飞。
    open_max_effort: float = 0.25
    # 放完后沿 +z 的退出距离，单位米；负值按 0 处理（只张开不抬高）。
    retreat_z_m: float = 0.06


def build_place_stages(config: PlaceTaskConfig) -> list[VisualGraspStage]:
    """按配置生成放置阶段列表。

    未启用时返回空列表，调用方可以无条件拼接而不改变原序列行为。退出位姿复用
    放置点姿态，只把 z 抬高 max(retreat_z_m, 0)。
    """

    if not config.enabled:
        return []
    place = PoseTarget(
        position=tuple(float(value) for value in config.place_position_xyz),
        orientation=tuple(float(value) for value in config.place_orientation_xyzw),
    )
    retreat = PoseTarget(
        position=(
            place.position[0],
            place.position[1],
            place.position[2] + max(0.0, float(config.retreat_z_m)),
        ),
        orientation=place.orientation,
    )
    return [
        VisualGraspStage(name="move_to_place", kind="move", pose=place),
        VisualGraspStage(
            name="open_gripper_at_place",
            kind="gripper",
            gripper_position_m=float(config.open_position_m),
            gripper_max_effort=float(config.open_max_effort),
        ),
        VisualGraspStage(name="place_retreat", kind="move", pose=retreat),
    ]

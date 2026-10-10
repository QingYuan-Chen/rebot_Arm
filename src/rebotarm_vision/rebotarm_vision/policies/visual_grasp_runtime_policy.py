"""纯视觉抓取运行时决策，供 ROS 执行器调用。"""

from __future__ import annotations


def velocity_scaling_for_stage(
    stage_name: str,
    *,
    approach: float,
    retreat: float,
    normal: float,
) -> float:
    """选择阶段速度；不读取 ROS 参数，也不产生运动命令。"""

    if stage_name in {"visual_servo_approach", "approach_grasp"}:
        return float(approach)
    if stage_name == "safe_retreat":
        return float(retreat)
    return float(normal)


def detected_jaw_width(plan, candidate=None) -> float:
    """从计划优先、候选回退地读取夹爪宽度。"""

    plan_width = float(getattr(plan, "jaw_width", 0.0) or 0.0)
    candidate = candidate if candidate is not None else getattr(plan, "candidate", None)
    candidate_width = float(getattr(candidate, "jaw_width", 0.0) or 0.0)
    return plan_width if plan_width > 0.0 else candidate_width


def detected_object_length(plan) -> float:
    """读取候选目标长度；字段缺失时返回 0。"""

    return float(getattr(getattr(plan, "candidate", None), "object_length", 0.0) or 0.0)

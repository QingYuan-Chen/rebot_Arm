"""抓取候选的运动可行性判定流程（把两个外部回调编排成一次判定）。

在系统中的位置：抓取候选 IK 过滤器对每个姿态变体调用本模块，统一"预抓取位姿与
抓取位姿是否都可达且无碰撞，并且运动代价可接受"这一顺序化判定。
可求解性与碰撞检查本身由调用方通过回调注入（运行期是 IK 服务 + 状态有效性服务），
因此本模块保持纯逻辑、可独立单元测试。

回调契约：
    check_target(target, label): 求解并校验单个位姿。
        返回求解结果对象（成功，供后续代价评估使用）或 None（不可行）。
        约定内部已包含 IK 容差判断与碰撞检查，本模块不再重复判断。
    motion_penalty(solution): 用 IK 解评估运动代价。
        返回 (penalty, reason)。penalty 为 None 表示该解被否决（如末轴翻转过大），
        reason 为对应的英文原因文本，会进入日志。

短路顺序（越早越省算力）：预抓取不可行 -> 不再求解抓取位姿；
抓取不可行 -> 不再计算运动代价。只有三者全通过才返回 accepted=True。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

from ..visual_grasp_sequence import PoseTarget


CheckTarget = Callable[[PoseTarget, str], object | None]
MotionPenalty = Callable[[object], tuple[float | None, str]]


@dataclass(frozen=True)
class MotionFeasibilityResult:
    """判定结果。

    accepted: 是否接受该变体。
    reason: 通过时为运动代价的说明文本；未通过时为失败原因
        （"pregrasp infeasible" / "grasp infeasible" / 回调给出的否决原因）。
    motion_penalty: 加权运动代价；未通过时为 None。
    pregrasp_solution: 预抓取位姿的求解结果（若已求得），否则 None。
    grasp_solution: 抓取位姿的求解结果（若已求得），否则 None。
    """

    accepted: bool
    reason: str
    motion_penalty: float | None = None
    pregrasp_solution: object | None = None
    grasp_solution: object | None = None


def evaluate_motion_feasibility(
    *,
    pregrasp: PoseTarget,
    grasp: PoseTarget,
    variant_label: str,
    check_target: CheckTarget,
    motion_penalty: MotionPenalty,
) -> MotionFeasibilityResult:
    """按"预抓取 -> 抓取 -> 运动代价"的顺序判定一个候选姿态变体。

    参数:
        pregrasp: 预抓取位姿（抓取点沿接近轴后退一段距离的位置）。
        grasp: 抓取位姿（夹爪闭合中心对准物体的位置）。
        variant_label: 变体标签（如 yaw/z 偏移组合），用于拼出 check_target 的日志标签。
        check_target: 可求解性 + 碰撞检查回调，见模块说明。
        motion_penalty: 运动代价回调，见模块说明。
    返回:
        MotionFeasibilityResult。失败时尽量带上已求得的部分解，便于调用方记录/复用。
    副作用: 由注入的回调决定（通常会调用 IK 与碰撞检查服务）。
    """
    pregrasp_solution = check_target(pregrasp, f"{variant_label}/pregrasp")
    if pregrasp_solution is None:
        return MotionFeasibilityResult(False, "pregrasp infeasible")

    grasp_solution = check_target(grasp, f"{variant_label}/grasp")
    if grasp_solution is None:
        # 预抓取解已求得，一并返回：调用方可能用它做诊断或轨迹复用
        return MotionFeasibilityResult(False, "grasp infeasible", pregrasp_solution=pregrasp_solution)

    penalty, reason = motion_penalty(grasp_solution)
    if penalty is None:
        # 代价回调否决：例如 joint6 单轴翻转超阈值
        return MotionFeasibilityResult(
            False,
            reason,
            pregrasp_solution=pregrasp_solution,
            grasp_solution=grasp_solution,
        )

    return MotionFeasibilityResult(
        True,
        reason,
        motion_penalty=float(penalty),
        pregrasp_solution=pregrasp_solution,
        grasp_solution=grasp_solution,
    )

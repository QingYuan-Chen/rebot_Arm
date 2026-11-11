"""抓取候选的轻量预筛：置信度与夹爪开口。

在昂贵的逆解/运动可行性检查之前，先用两个几乎零成本的数值条件淘汰明显不可用
的候选，避免把算力浪费在必然失败的位姿上。判定为纯函数，不做坐标变换，也不
访问硬件；长度单位统一为米。

注意：本预筛与门控/工作空间检查是并列的多道关卡，这里只关心候选自身的数值
合理性，不判断可达性。
"""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class CandidatePrecheckConfig:
    """预筛阈值；默认值应与夹爪的实际机械开口范围一致。"""

    # 置信度下限 [0, 1]。0.0 表示不按置信度筛（只筛开口）。
    min_confidence: float = 0.0
    # 夹爪最小开口，单位米；小于它说明物体太薄/夹爪合不拢，直接淘汰。
    # 默认 0.006 接近夹爪的闭合极限。
    min_jaw_width_m: float = 0.006
    # 夹爪最大开口，单位米；大于它说明物体太宽夹不住。默认 0.085。
    max_jaw_width_m: float = 0.085


@dataclass(frozen=True)
class CandidatePrecheckResult:
    """预筛结果；``reason`` 为英文诊断文本，空串表示接受。"""

    accepted: bool
    reason: str = ""


def evaluate_candidate_precheck(
    *,
    confidence: float,
    jaw_width_m: float,
    config: CandidatePrecheckConfig = CandidatePrecheckConfig(),
) -> CandidatePrecheckResult:
    """按「有限性 → 置信度 → 有限性 → 开口区间」的顺序短路判定。

    参数：
        confidence: 候选置信度，期望落在 [0, 1]。
        jaw_width_m: 候选所需的夹爪开口，单位米。
        config: 预筛阈值。

    返回：
        CandidatePrecheckResult。NaN/Inf 一律显式拒绝：NaN 参与大小比较恒为
        假，若不先拦截就会从区间检查里「漏过去」。
    """

    score = float(confidence)
    width = float(jaw_width_m)
    # 先查置信度有限性：NaN 会让下面的下限比较静默失效。
    if not math.isfinite(score):
        return CandidatePrecheckResult(False, "confidence is not finite")
    if score < float(config.min_confidence):
        return CandidatePrecheckResult(
            False,
            f"confidence below minimum ({score:.4f} < {float(config.min_confidence):.4f})",
        )
    # 开口同样先过有限性检查，再判上下限。
    if not math.isfinite(width):
        return CandidatePrecheckResult(False, "jaw_width is not finite")
    if width < float(config.min_jaw_width_m):
        return CandidatePrecheckResult(False, f"jaw_width too small ({width:.3f}m)")
    if width > float(config.max_jaw_width_m):
        return CandidatePrecheckResult(False, f"jaw_width too large ({width:.3f}m)")
    return CandidatePrecheckResult(True)

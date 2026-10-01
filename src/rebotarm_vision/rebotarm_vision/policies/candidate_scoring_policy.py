"""抓取候选的排序打分（可解释的线性罚分）。

候选来自位姿变体生成：原始候选带序号，变体候选带形如 ``..._z<毫米>`` 的标签。
打分规则是「原始序号越小越优，变体偏移与运动代价各自扣分」，三项分解会写进
``reason``，便于事后从日志复查某个候选为什么排在前面。

分数越高越优先；本模块不设通过阈值（是否采纳由上层决定）。
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class CandidateScoringInput:
    """打分输入：候选的静态属性 + 上游算出的运动代价。"""

    # 候选在原始列表中的序号（越小代表原始排序越靠前）。
    original_index: int
    # 变体标签；形如 "xxx_z30" 表示沿 z 方向偏移了 30 毫米。
    variant_label: str
    # 运动代价（如关节行程/可行性罚分），由上层计算，量纲与分数一致。
    motion_penalty: float = 0.0


@dataclass(frozen=True)
class CandidateScoringResult:
    """打分结果；``reason`` 给出三项分解，供日志排查。"""

    score: float
    reason: str


def z_variant_penalty(variant_label: str) -> float:
    """把 z 变体标签里的毫米偏移折算成罚分。

    约定：每偏移 1 毫米罚 0.001 分，因此 30 mm 的变体只比原候选低 0.03 分，
    仍可能凭运动代价优势胜出。标签不含 "_z"、或后缀不是数字时返回 0.0（容错，
    不抛异常）；调用方约定传入字符串标签。
    """

    if "_z" not in variant_label:
        return 0.0
    try:
        # rsplit 取最后一个 "_z" 之后的部分，避免标签前段也含 "_z" 时取错段。
        return float(str(variant_label).rsplit("_z", 1)[1]) * 0.001
    except ValueError:
        return 0.0


def score_candidate(scoring_input: CandidateScoringInput) -> CandidateScoringResult:
    """计算候选总分 = 序号分 - 变体罚分 - 运动罚分。

    序号分取 ``-original_index``：0 号候选得 0 分、序号每加一少 1 分，从而在
    没有其他代价时保持原始（检测/采样）排序不变。
    """

    rank_score = -float(scoring_input.original_index)
    variant_penalty = z_variant_penalty(scoring_input.variant_label)
    motion_penalty = float(scoring_input.motion_penalty)
    score = rank_score - variant_penalty - motion_penalty
    return CandidateScoringResult(
        score=score,
        reason=(
            f"rank_score={rank_score:.2f}, "
            f"variant_penalty={variant_penalty:.3f}, "
            f"motion_penalty={motion_penalty:.3f}"
        ),
    )

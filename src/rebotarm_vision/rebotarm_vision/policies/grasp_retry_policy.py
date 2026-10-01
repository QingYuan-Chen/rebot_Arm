"""抓取重试的候选顺序与次数控制。

一次抓取失败后，执行节点可能需要换一个候选位姿再试。本模块决定「按什么顺序
试、还能试几个」：最优候选优先，其余按原始顺序；已经失败的候选不再重复。状态
（失败集合）由调用方跨次传入，本模块不保存任何状态。
"""

from __future__ import annotations

from dataclasses import dataclass
from collections.abc import Iterable


@dataclass(frozen=True)
class RetryPolicyConfig:
    """重试策略参数；默认等价于「只试最优候选一次」。"""

    # 是否允许换候选重试。默认 False：只尝试最优候选，失败即结束。
    enabled: bool = False
    # 总尝试次数上限（含已经失败的次数），有效值至少为 1。
    # 剩余名额 = max(0, max_attempts - 已失败数)。
    max_attempts: int = 1


def ordered_candidate_indices(
    *,
    candidate_count: int,
    best_index: int,
    failed_indices: Iterable[int],
    config: RetryPolicyConfig,
) -> list[int]:
    """返回本轮应当尝试的候选下标列表（按尝试优先级排序）。

    参数：
        candidate_count: 候选总数；<= 0 时直接返回空列表。
        best_index: 上游给出的最优候选下标；越界（< 0 或 >= count）时回退到 0，
            避免上层数据异常导致整轮放弃。
        failed_indices: 已失败的候选下标集合，会被排除，并计入已用次数。
        config: 重试策略参数。

    返回：
        下标列表。未启用重试时：最优候选若已失败则返回空（不再尝试），否则只
        返回 ``[best_index]``。
    """

    if candidate_count <= 0:
        return []
    failed = {int(index) for index in failed_indices}
    best = int(best_index)
    # 越界的 best_index 一律按 0 处理：宁可按第一个候选尝试，也不空转一轮。
    if best < 0 or best >= candidate_count:
        best = 0

    if not config.enabled:
        return [] if best in failed else [best]

    # 最优候选排在最前，其余保持原始顺序。
    ordered = [best] + [index for index in range(candidate_count) if index != best]
    remaining = [index for index in ordered if index not in failed]
    # 已用次数由失败集合体现，因此剩余名额 = 上限 - 已失败数（下限 0）。
    remaining_attempts = max(0, int(config.max_attempts) - len(failed))
    return remaining[:remaining_attempts]

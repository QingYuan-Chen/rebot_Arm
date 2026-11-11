"""抓取候选可达性过滤策略（纯函数，不接触 ROS 参数与硬件）。

职责：把逆解/碰撞检查得到的"每个候选是否可达"布尔序列，应用回候选数组，
产出一份只含可达候选、且 ``best_index`` 已重新索引的新数组。本模块不做任何
IK 求解或碰撞判断，只负责数组的挑选与下标重映射。
"""

from __future__ import annotations

from copy import deepcopy
from collections.abc import Sequence

from rebotarm_msgs.msg import GraspCandidateArray


def filter_candidate_array_by_reachability(
    candidates: GraspCandidateArray,
    reachable: Sequence[bool],
) -> GraspCandidateArray:
    """按可达性布尔序列筛掉不可达候选，并同步修正最优候选下标。

    参数：
    - ``candidates``：输入候选数组；其 ``header`` 会被原样带到输出（保持坐标系与
      时间戳一致，下游才能判定计划时效），``best_index`` 是原始数组中的下标。
    - ``reachable``：与 ``candidates.candidates`` 逐位对应的可达标记（顺序必须一致）。
      长度不足时按 ``zip`` 语义只处理较短的一侧，多余候选视为不可达而被丢弃。

    返回：新的候选数组；候选对象做深拷贝，因此修改返回值不会影响入参。

    返回值语义（容易被下游误读，务必注意）：
    - 输出 ``best_index`` 指向"筛选后的新数组"；原最优候选若仍可达则保持它最优
      （通过 ``original_to_filtered`` 映射），若被筛掉则回退为 0。
    - 输出为空数组时 ``best_index`` 保持 -1，表示"本帧没有可用候选"，
      下游必须据此判失败而不能当作"第 0 个候选可用"。
    """
    filtered = GraspCandidateArray()
    filtered.header = candidates.header
    filtered.best_index = -1
    filtered.candidates = []
    # 记录"原下标 -> 新下标"的映射，用于把 best_index 从原数组迁移到筛选后的数组
    original_to_filtered: dict[int, int] = {}
    for index, (candidate, ok) in enumerate(zip(candidates.candidates, reachable)):
        if bool(ok):
            original_to_filtered[index] = len(filtered.candidates)
            filtered.candidates.append(deepcopy(candidate))
    if filtered.candidates:
        # 原最优候选被筛掉时回退到 0（若它不可达，0 已是保留下来的最优解）
        original_best = int(getattr(candidates, "best_index", -1))
        filtered.best_index = original_to_filtered.get(original_best, 0)
    return filtered

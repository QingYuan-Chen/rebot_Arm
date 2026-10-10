"""视觉抓取阶段失败后的恢复决策（换下一位候选重试 / 整体中止）。

执行节点按阶段推进抓取序列，任一阶段失败时调用本模块拿到一个决策：是否换下
一个候选重试、是否请求停止当前运动、重试前是否先退回安全位。判定是纯函数，
状态（尝试序号、剩余次数）由调用方传入并维护。

安全语义：只有「尚未夹持物体」的接近类阶段允许自动重试；一旦进入夹持/撤退
阶段，失败后一律中止，绝不在夹着物体的情况下自动继续动作。默认不自动重试。
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class RecoveryConfig:
    """恢复策略参数；默认值是「不自动重试」的最保守组合。"""

    # 是否允许换候选自动重试。默认 False：失败即中止，交由操作者判断。
    auto_retry_enabled: bool = False
    # 重试前是否先退回预抓取安全位（避免贴着物体直接横移）。
    safe_retreat_before_retry: bool = True


@dataclass(frozen=True)
class RecoveryDecision:
    """恢复决策结果；``reason`` 为英文诊断文本，会进日志与服务响应。"""

    # True 表示换下一个候选继续尝试。
    retry: bool
    # True 表示放弃本次视觉抓取。
    abort: bool
    # True 表示需要请求停止当前阶段的运动（软停/取消目标）。
    request_stop: bool
    # True 表示重试前先执行安全后退。
    request_safe_retreat: bool
    reason: str


def recovery_decision_for_stage(
    stage_name: str,
    *,
    attempt_index: int,
    remaining_attempts: int,
    config: RecoveryConfig,
) -> RecoveryDecision:
    """根据失败阶段与剩余次数给出「重试」或「中止」决策。

    参数：
        stage_name: 失败阶段名（英文标识，见序列模块中的阶段定义）。
        attempt_index: 本次尝试的序号，从 0 开始；仅用于生成诊断文本。
        remaining_attempts: 后续还可尝试的候选数量，<= 0 时不允许重试。
        config: 恢复策略参数。

    返回：
        RecoveryDecision。可重试时 ``retry=True`` 且 ``request_stop=True``
        （先停住再退回）；否则 ``abort=True`` 并请求停止运动。
    """

    # 仅这三个阶段可重试：它们都还没闭合夹爪，此时退回不会拖着物体或撞到工件。
    can_retry_stage = stage_name in {
        "move_to_pregrasp",
        "approach_grasp",
        "visual_servo_approach",
    }
    can_retry = bool(config.auto_retry_enabled) and can_retry_stage and int(remaining_attempts) > 0
    if can_retry:
        # 预抓取阶段本身失败时已经在安全位外侧，无需再多退一次。
        request_safe_retreat = bool(config.safe_retreat_before_retry) and stage_name != "move_to_pregrasp"
        return RecoveryDecision(
            retry=True,
            abort=False,
            request_stop=True,
            request_safe_retreat=request_safe_retreat,
            reason=f"{stage_name} failed on attempt {int(attempt_index) + 1}; retrying next candidate",
        )
    return RecoveryDecision(
        retry=False,
        abort=True,
        request_stop=True,
        request_safe_retreat=False,
        reason=f"{stage_name} failed; aborting visual grasp",
    )

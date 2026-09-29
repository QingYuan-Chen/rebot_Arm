"""Pure replay start classification and request gate decisions."""

from __future__ import annotations

from .teach_models import ReplayStartBand, ReplayStartDecision, TeachDryRunDecision


def classify_replay_start(
    *,
    current_positions: tuple[float, ...],
    start_positions: tuple[float, ...],
    direct_threshold: float,
    align_threshold: float,
) -> ReplayStartDecision:
    """按最大关节误差给回放起点分档（本模块的第一道安全门）。

    参数均为 rad：``current_positions`` 当前关节位置，``start_positions`` 记录起点关节位置，
    ``direct_threshold`` 直接回放上限，``align_threshold`` 允许自动对齐的上限（应大于前者）。
    判定用严格小于：``max_error < direct_threshold`` → ``direct``；
    ``max_error < align_threshold`` → ``align``；否则 ``reject``。
    两个关节向量长度不一致时直接 ``reject``（正无穷误差），因为逐关节比较没有意义。
    """
    if len(current_positions) != len(start_positions):
        return ReplayStartDecision(
            band=ReplayStartBand.REJECT,
            max_error=float("inf"),
            per_joint_error=(),
            allow_replay=False,
            allow_auto_align=False,
            message="current and start joint vectors have different lengths",
        )
    errors = tuple(abs(float(a) - float(b)) for a, b in zip(current_positions, start_positions))
    max_error = max(errors, default=0.0)
    if max_error < float(direct_threshold):
        return ReplayStartDecision(
            band=ReplayStartBand.DIRECT,
            max_error=max_error,
            per_joint_error=errors,
            allow_replay=True,
            allow_auto_align=False,
            message="current pose is close enough to replay start",
        )
    if max_error < float(align_threshold):
        return ReplayStartDecision(
            band=ReplayStartBand.ALIGN,
            max_error=max_error,
            per_joint_error=errors,
            allow_replay=True,
            allow_auto_align=True,
            message="small return_to_start alignment required",
        )
    return ReplayStartDecision(
        band=ReplayStartBand.REJECT,
        max_error=max_error,
        per_joint_error=errors,
        allow_replay=False,
        allow_auto_align=False,
        message="start error too large; manually drag the arm near the recording start",
    )


def compute_auto_align_duration(
    max_error_rad: float | None,
    *,
    target_speed_rad_s: float = 0.15,
    min_duration_sec: float = 3.0,
    max_duration_sec: float = 10.0,
) -> float:
    """按"误差 / 目标速度"估算自动对齐所需时长，并夹在给定区间内（返回秒）。

    参数：``max_error_rad`` 起点最大关节误差（rad，None 或非数值按 0 处理）；
    ``target_speed_rad_s`` 对齐时允许的平均关节速度（rad/s，下限 0.01，防止除零）；
    ``min_duration_sec`` / ``max_duration_sec`` 时长下上限（秒）。
    误差为 0 或无法解析时直接返回下限时长；误差很大时时长被 ``max_duration_sec`` 截断，
    即"宁可慢也绝不为了赶时间提速"。
    """
    try:
        error = abs(float(max_error_rad))
    except (TypeError, ValueError):
        error = 0.0
    speed = max(float(target_speed_rad_s), 0.01)
    duration = error / speed if error > 0.0 else float(min_duration_sec)
    return min(max(duration, float(min_duration_sec)), float(max_duration_sec))


def validate_teach_dry_run_request(start_band: str) -> TeachDryRunDecision:
    """dry-run（空跑校验）请求的门控：文件巡检分档合法才接受。

    只有 ``direct`` / ``align`` / ``moveit_align`` 三档被接受（进入 ``dry_run`` 状态）；
    文件缺失、损坏、为空、起点误差过大（``reject``）等一律拒绝并返回 ``blocked``。
    dry-run 不驱动真实硬件，是真实回放的前置条件。分档字符串不区分大小写。
    """
    band = str(start_band or "").strip().lower()
    if band in (
        ReplayStartBand.DIRECT.value,
        ReplayStartBand.ALIGN.value,
        ReplayStartBand.MOVEIT_ALIGN.value,
    ):
        return TeachDryRunDecision(
            accepted=True,
            state="dry_run",
            message=f"dry-run accepted for {band} replay check",
        )
    return TeachDryRunDecision(
        accepted=False,
        state="blocked",
        message=f"dry-run blocked because file check is {band or 'unknown'}",
    )


def validate_teach_replay_execute_request(
    start_band: str,
    *,
    dry_run_passed: bool,
    risk_level: str = "green",
    prepared_risk_level: str | None = None,
    prepared_max_jump_rad: float | None = None,
    max_prepared_jump_rad: float = 0.02,
    retimed_max_acceleration_rad_s2: float | None = None,
    max_replay_acceleration_rad_s2: float = 5.0,
    retimed_max_jerk_rad_s3: float | None = None,
    max_replay_jerk_rad_s3: float = 20.0,
    replay_speed: float = 1.0,
    yellow_max_speed: float = 0.6,
) -> TeachDryRunDecision:
    """真实回放执行请求的门控（本模块最重要的一道安全闸门，条件按固定顺序短路判断）。

    参数：
      ``start_band``                    文件巡检分档，必须是 ``direct`` / ``align`` /
                                        ``moveit_align`` 之一；
      ``dry_run_passed``                是否已经成功空跑过；真实回放要求先 dry-run；
      ``risk_level``                    原始轨迹风险等级（``green``/``yellow``/``red``）；
      ``prepared_risk_level``           预处理后轨迹的风险等级；为 None 时退回用
                                        ``risk_level``（此时错误信息里不会出现"prepared"字样）；
      ``prepared_max_jump_rad``         预处理轨迹的最大单帧跳变（rad），None 表示不检查；
      ``max_prepared_jump_rad``         允许的预处理最大跳变（rad，默认 0.02）；
      ``retimed_max_acceleration_rad_s2`` / ``retimed_max_jerk_rad_s3``
                                        重定时后的实测最大加速度/加加速度（rad/s^2、rad/s^3），
                                        None 表示不检查；
      ``max_replay_acceleration_rad_s2`` / ``max_replay_jerk_rad_s3``
                                        对应的允许上限（默认 5.0、20.0）；
      ``replay_speed``                  请求的回放倍速；
      ``yellow_max_speed``              yellow 等级允许的最大倍速（默认 0.6）。
    判定顺序（先查最基础的分档，再查等级，再查数值，最后查 dry-run）：
      分档非法 → 拒绝；预处理/原始等级为 ``red`` → 拒绝；预处理跳变、重定时加速度、重定时
      加加速度超限 → 拒绝；``yellow`` 且倍速超过 ``yellow_max_speed`` → 拒绝；
      未通过 dry-run → 拒绝；全部通过才返回 ``replaying``。
    注意：所有阈值检查都是"None 即跳过"，因此调用方有责任把真实测量值传进来，否则该层保护
    不会生效。
    """
    band = str(start_band or "").strip().lower()
    raw_risk = str(risk_level or "unknown").strip().lower()
    risk = str(prepared_risk_level or raw_risk).strip().lower()
    if band not in (
        ReplayStartBand.DIRECT.value,
        ReplayStartBand.ALIGN.value,
        ReplayStartBand.MOVEIT_ALIGN.value,
    ):
        return TeachDryRunDecision(
            accepted=False,
            state="blocked",
            message=f"real replay blocked because file check is {band or 'unknown'}",
        )
    if risk == "red":
        # 等级字段存在时优先采信预处理后的等级，错误信息据此区分措辞。
        quality_name = "prepared trajectory quality" if prepared_risk_level else "trajectory quality"
        return TeachDryRunDecision(
            accepted=False,
            state="blocked",
            message=f"real replay blocked because {quality_name} is red",
        )
    if prepared_max_jump_rad is not None and float(prepared_max_jump_rad) > float(max_prepared_jump_rad):
        return TeachDryRunDecision(
            accepted=False,
            state="blocked",
            message=(
                "real replay blocked because prepared max jump "
                f"{float(prepared_max_jump_rad):.4f} rad exceeds {float(max_prepared_jump_rad):.4f} rad"
            ),
        )
    if (
        retimed_max_jerk_rad_s3 is not None
        and float(retimed_max_jerk_rad_s3) > float(max_replay_jerk_rad_s3)
    ):
        return TeachDryRunDecision(
            accepted=False,
            state="blocked",
            message=(
                "real replay blocked because retimed max jerk "
                f"{float(retimed_max_jerk_rad_s3):.4f} rad/s^3 exceeds "
                f"{float(max_replay_jerk_rad_s3):.4f} rad/s^3"
            ),
        )
    if (
        retimed_max_acceleration_rad_s2 is not None
        and float(retimed_max_acceleration_rad_s2) > float(max_replay_acceleration_rad_s2)
    ):
        return TeachDryRunDecision(
            accepted=False,
            state="blocked",
            message=(
                "real replay blocked because retimed max acceleration "
                f"{float(retimed_max_acceleration_rad_s2):.4f} rad/s^2 exceeds "
                f"{float(max_replay_acceleration_rad_s2):.4f} rad/s^2"
            ),
        )
    if risk == "yellow" and float(replay_speed) > float(yellow_max_speed):
        # yellow 轨迹本身带瑕疵：即使用户请求 1.0 倍速也必须降到 yellow_max_speed 以内。
        return TeachDryRunDecision(
            accepted=False,
            state="blocked",
            message=f"yellow replay speed must be <= {float(yellow_max_speed):.2f}",
        )
    if not dry_run_passed:
        return TeachDryRunDecision(
            accepted=False,
            state="blocked",
            message="real replay requires a successful dry-run first",
        )
    return TeachDryRunDecision(
        accepted=True,
        state="replaying",
        message=f"real replay accepted for {band} file check",
    )


def validate_teach_replay_stop_request(has_active_goal: bool) -> TeachDryRunDecision:
    """回放停止（取消）请求的门控：没有活动回放目标时返回 ``idle`` 且不接受。

    有活动目标时返回 ``cancel_requested``，由上层据此去取消动作目标；本函数只做判定，
    不执行取消动作，也不会自行下发停止指令。
    """
    if not has_active_goal:
        return TeachDryRunDecision(
            accepted=False,
            state="idle",
            message="no active teach replay goal",
        )
    return TeachDryRunDecision(
        accepted=True,
        state="cancel_requested",
        message="teach replay cancel requested",
    )


def normalize_teach_replay_settings(
    *,
    replay_speed: float,
    align_duration: float,
    align_steps: int,
    final_hold_sec: float = 1.0,
) -> dict[str, float | int]:
    """把用户请求的回放设置夹到安全范围内（返回可直接使用的设置字典）。

    夹取规则：``replay_speed`` → [0.1, 1.0]（只允许减速，且不能慢到 0.1 倍以下）；
    ``align_duration`` → [1.0, 10.0] 秒；``align_steps`` → [2, 200] 个点。
    ``final_hold_sec`` 是形参但**结果恒为 1.0**：回放结束保持时长被硬编码为 1 秒，
    传入值只用于保持接口兼容，不参与计算。
    """
    return {
        "replay_speed": min(max(float(replay_speed), 0.1), 1.0),
        "align_duration": min(max(float(align_duration), 1.0), 10.0),
        "align_steps": min(max(int(align_steps), 2), 200),
        "final_hold_sec": 1.0,
    }


def estimate_teach_replay(
    *,
    samples: int,
    record_duration_sec: float,
    start_band: str,
    replay_speed: float,
    align_duration: float,
    align_steps: int,
    final_hold_sec: float = 0.0,
) -> dict[str, float | int | bool]:
    """预估一次回放的时长与轨迹点数（用于界面提示，不参与安全判定）。

    参数：``samples`` 记录样本数；``record_duration_sec`` 记录时长（秒）；``start_band``
    分档（只有 ``align`` 档才会计入对齐时长与对齐点）；``replay_speed`` 倍速；
    ``align_duration`` / ``align_steps`` 对齐时长（秒）与对齐点数；``final_hold_sec``
    结束保持时长（秒）。
    计算：回放时长 = 记录时长 / 倍速（倍速下限 0.01 防止除零）；总时长 = 对齐时长 +
    回放时长 + 结束保持时长；总点数 = 样本数 + 对齐点（仅 ``align`` 档）+
    1 个结束保持点（仅在保持时长 > 0 且样本数 > 0 时计入）。
    返回的 ``replay_speed`` / ``align_duration`` / ``align_steps`` / ``final_hold_sec`` 都是
    :func:`normalize_teach_replay_settings` 夹紧之后的实际值；注意 ``final_hold_sec`` 被该
    函数硬编码为 1.0，因此总时长里的保持时长实际恒为 1 秒。
    """
    settings = normalize_teach_replay_settings(
        replay_speed=replay_speed,
        align_duration=align_duration,
        align_steps=align_steps,
        final_hold_sec=final_hold_sec,
    )
    speed = float(settings["replay_speed"])
    use_align = str(start_band or "").lower() == ReplayStartBand.ALIGN.value
    replay_duration = max(0.0, float(record_duration_sec)) / max(speed, 0.01)
    alignment_duration = float(settings["align_duration"]) if use_align else 0.0
    final_hold = float(settings["final_hold_sec"])
    return {
        "use_align": use_align,
        "alignment_duration_sec": alignment_duration,
        "estimated_duration_sec": alignment_duration + replay_duration + final_hold,
        "trajectory_points": max(int(samples), 0)
        + (int(settings["align_steps"]) if use_align else 0)
        + (1 if final_hold > 0.0 and int(samples) > 0 else 0),
        "replay_speed": speed,
        "align_duration": float(settings["align_duration"]),
        "align_steps": int(settings["align_steps"]),
        "final_hold_sec": final_hold,
    }


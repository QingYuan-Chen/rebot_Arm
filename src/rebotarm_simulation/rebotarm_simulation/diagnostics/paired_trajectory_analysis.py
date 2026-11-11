"""成对轨迹对照分析：把仿真与真机执行同一条命令的记录换算成可比指标。

职责与位置
    本模块属于仿真层，是一套只用标准库的纯计算工具（不依赖 ROS、不依赖物理引擎），
    输入是两份纯粹的 Python 数据结构（可来自 JSON），输出是可以直接写进证据报告的
    字典。它不采集数据、不下发指令，只做校验、对齐与统计。

两份输入的结构
    "运行记录"（run）是一个映射，至少含：
    - ``command``：命令字典（见运动层的成对轨迹命令协议），必须带 ``joint_names``
      （固定 joint1..joint6）、``duration_sec``、``points``（首末点的 ``positions``
      即起止角，单位 rad），对照场景还必须有 ``command_sha256``；
    - ``samples``：非空样本列表，每个样本含 ``elapsed_sec``（相对命令起点的秒数）
      以及 6 元的 ``desired_positions``、``observed_positions``、``velocities``、
      ``efforts``。

两类输出
    - :func:`analyze_run`：单次运行的逐轴跟踪质量（RMS/峰值误差、末端误差、峰值与
      窗口峰值速度、调节时间、超调、力矩峰值与均值）；
    - :func:`compare_runs`：仿真与真机两次运行的对齐结果，要求两条命令的内容哈希
      完全一致，否则直接拒绝出报告。

安全与解释边界
    真机 effort 是电机反馈力矩，仿真是施加的广义执行器力，两者名义单位都是 N·m，
    但没有做过同量纲标定，因此只比较趋势相关性，不能当作同一物理量的绝对偏差。
    库名与单位写进输出的 ``effort_semantics`` 字段随报告一起归档，避免结论被误读。
"""

from __future__ import annotations

from bisect import bisect_right
import math
from statistics import fmean
from typing import Mapping, Sequence


# 手臂 6 个旋转关节的规范顺序：命令、真机反馈与仿真样本都按此顺序解释。
ARM_JOINT_NAMES = tuple(f"joint{index}" for index in range(1, 7))


def analyze_run(run: Mapping[str, object]) -> dict[str, object]:
    """分析单次运行，返回逐轴跟踪指标与样本时间覆盖情况。

    先做契约校验（必须有 command 与非空 samples、关节名必须是 joint1..joint6），
    再把样本逐个规范化并按 ``elapsed_sec`` 升序排序——配对与末点取值都依赖时间
    有序，输入顺序不可信。起止角取自命令的首末轨迹点。

    参数 ``run`` 见模块说明。返回字典字段：
    - ``sample_count`` / ``duration_sec`` / ``first_elapsed_sec`` / ``last_elapsed_sec``：
      样本数与时间覆盖，便于先判断数据是否完整（例如末样本是否真的到命令末端）；
    - ``per_joint``：关节名 -> 该轴指标，键的含义与单位：
      ``rms_tracking_error_rad`` 期望角与实测角之差的均方根（rad）；
      ``max_abs_tracking_error_rad`` 跟踪误差绝对值的最大值（rad）；
      ``final_error_rad`` 命令末端目标角减去最后一个实测角（rad，带方向）；
      ``peak_abs_velocity_rad_s`` 样本速度绝对值的峰值（rad/s）；
      ``peak_window_velocity_rad_s`` 0.20 s 滑窗差分峰值速度（rad/s，见
      :func:`_peak_window_velocity`）；
      ``settling_time_sec`` 相对命令末端定义的调节时间（s），未在数据内稳定为 None；
      ``overshoot_rad`` 越冲量（rad，见 :func:`_overshoot`）；
      ``peak_abs_effort`` / ``mean_abs_effort`` 力矩绝对值峰值与均值（真机为反馈
      力矩、仿真为广义执行器力，名义单位 N·m）；
      ``final_observed_position_rad`` 最后一个实测角（rad）。

    异常：结构不合法时抛 :class:`ValueError`，不返回残缺结果。
    """
    command = run.get("command")
    samples = run.get("samples")
    if not isinstance(command, Mapping) or not isinstance(samples, list) or not samples:
        raise ValueError("run must contain command and non-empty samples")
    if tuple(command.get("joint_names", ())) != ARM_JOINT_NAMES:
        raise ValueError("run command uses unexpected joint names")
    normalized = [_normalize_sample(sample) for sample in samples]
    normalized.sort(key=lambda sample: sample["elapsed_sec"])
    duration = float(command["duration_sec"])
    start = tuple(float(value) for value in command["points"][0]["positions"])
    target = tuple(float(value) for value in command["points"][-1]["positions"])

    per_joint: dict[str, object] = {}
    for index, joint in enumerate(ARM_JOINT_NAMES):
        errors = [sample["desired_positions"][index] - sample["observed_positions"][index] for sample in normalized]
        observed = [sample["observed_positions"][index] for sample in normalized]
        velocities = [sample["velocities"][index] for sample in normalized]
        efforts = [sample["efforts"][index] for sample in normalized]
        per_joint[joint] = {
            "rms_tracking_error_rad": _rms(errors),
            "max_abs_tracking_error_rad": max(abs(value) for value in errors),
            "final_error_rad": target[index] - observed[-1],
            "peak_abs_velocity_rad_s": max(abs(value) for value in velocities),
            # 0.20 s 的差分窗口：短于采样噪声相关时间、长于单帧抖动，用于抑制毛刺。
            "peak_window_velocity_rad_s": _peak_window_velocity(normalized, index, 0.20),
            "settling_time_sec": _settling_time(normalized, index, duration),
            "overshoot_rad": _overshoot(observed, start[index], target[index]),
            "peak_abs_effort": max(abs(value) for value in efforts),
            "mean_abs_effort": fmean(abs(value) for value in efforts),
            "final_observed_position_rad": observed[-1],
        }
    return {
        "sample_count": len(normalized),
        "duration_sec": duration,
        "first_elapsed_sec": normalized[0]["elapsed_sec"],
        "last_elapsed_sec": normalized[-1]["elapsed_sec"],
        "per_joint": per_joint,
    }


def compare_runs(sim_run: Mapping[str, object], real_run: Mapping[str, object]) -> dict[str, object]:
    """对齐仿真与真机两次运行，输出逐轴的位置偏差与力矩趋势相关性。

    对齐方式：以真机的采样时刻为基准，把仿真序列按时间线性插值到同一时刻，再求
    "真机实测角 - 仿真实测角"。只有两条命令的 ``command_sha256`` 相同（非空且完全
    一致）才允许对照，否则抛 :class:`ValueError`——哈希一致是"两侧确实执行了同一
    条轨迹"唯一可自动核验的证据。

    参数：
    - ``sim_run`` / ``real_run``：仿真侧与真机侧的运行记录，结构见模块说明。

    返回字典字段：
    - ``command_sha256``：被对照命令的内容哈希；
    - ``effort_semantics``：力矩口径说明（见模块"安全与解释边界"）；
    - ``sim`` / ``real``：两侧各自的 :func:`analyze_run` 结果；
    - ``paired``：关节名 -> ``rms_real_minus_sim_position_rad``（位置差均方根，
      rad）、``max_abs_real_minus_sim_position_rad``（位置差绝对值峰值，rad）、
      ``effort_trend_correlation``（力矩趋势的皮尔逊相关系数，无量纲，-1..1；
      样本不足或任一侧方差为 0 时为 None）。
    """
    sim_command = sim_run.get("command", {})
    real_command = real_run.get("command", {})
    sim_hash = str(sim_command.get("command_sha256", ""))
    real_hash = str(real_command.get("command_sha256", ""))
    if not sim_hash or sim_hash != real_hash:
        raise ValueError("paired runs must use the same command_sha256")
    sim_samples = sorted(
        (_normalize_sample(sample) for sample in sim_run["samples"]),
        key=lambda sample: sample["elapsed_sec"],
    )
    real_samples = sorted(
        (_normalize_sample(sample) for sample in real_run["samples"]),
        key=lambda sample: sample["elapsed_sec"],
    )
    paired: dict[str, object] = {}
    for index, joint in enumerate(ARM_JOINT_NAMES):
        position_deltas = []
        effort_pairs = []
        for real_sample in real_samples:
            sim_position, sim_effort = _interpolate_values(
                sim_samples, real_sample["elapsed_sec"], index
            )
            position_deltas.append(real_sample["observed_positions"][index] - sim_position)
            effort_pairs.append((sim_effort, real_sample["efforts"][index]))
        paired[joint] = {
            "rms_real_minus_sim_position_rad": _rms(position_deltas),
            "max_abs_real_minus_sim_position_rad": max(abs(value) for value in position_deltas),
            "effort_trend_correlation": _correlation(effort_pairs),
        }
    return {
        "command_sha256": sim_hash,
        "effort_semantics": (
            "real effort is Damiao feedback torque; MuJoCo effort is applied generalized "
            "actuator force. Both arm fields are nominally N.m but are not directly "
            "calibrated as identical measurements."
        ),
        "sim": analyze_run(sim_run),
        "real": analyze_run(real_run),
        "paired": paired,
    }


def _normalize_sample(sample: Mapping[str, object]) -> dict[str, object]:
    """把一条样本规范化成时间加 4 个长度 6 的浮点元组，并做合法性校验。

    时间戳必须有限且非负（负时间或 NaN 会破坏排序与插值）；4 个向量必须恰好 6 个
    有限值。不合法时抛 :class:`ValueError`，避免脏数据静默进入统计。
    """
    if not isinstance(sample, Mapping):
        raise ValueError("samples must be mappings")
    elapsed = float(sample["elapsed_sec"])
    if not math.isfinite(elapsed) or elapsed < 0.0:
        raise ValueError("sample elapsed_sec must be finite and non-negative")
    return {
        "elapsed_sec": elapsed,
        "desired_positions": _vector6(sample["desired_positions"], "desired_positions"),
        "observed_positions": _vector6(sample["observed_positions"], "observed_positions"),
        "velocities": _vector6(sample["velocities"], "velocities"),
        "efforts": _vector6(sample["efforts"], "efforts"),
    }


def _peak_window_velocity(samples: Sequence[Mapping[str, object]], index: int, window: float) -> float:
    """用滑窗差分估算位置信号的峰值速度，单位 rad/s。

    对每个窗口终点 ``end``，``start`` 前移的条件是"去掉首点后窗口时长仍不小于
    ``window``"，这样窗口跨度尽量接近但不小于给定宽度（本仓库调用时取 0.20 s）。
    与逐样本的 ``velocities`` 相比，差分能抵消编码器噪声与单帧速度换算带来的毛刺。
    时间跨度非正（重复时间戳）时跳过该窗口以避免除零；始终返回非负值。
    """
    peak = 0.0
    start = 0
    for end in range(1, len(samples)):
        while start + 1 < end and samples[end]["elapsed_sec"] - samples[start + 1]["elapsed_sec"] >= window:
            start += 1
        dt = samples[end]["elapsed_sec"] - samples[start]["elapsed_sec"]
        if dt <= 0.0:
            continue
        delta = samples[end]["observed_positions"][index] - samples[start]["observed_positions"][index]
        peak = max(peak, abs(delta / dt))
    return peak


def _settling_time(samples: Sequence[Mapping[str, object]], index: int, duration: float) -> float | None:
    """判定相对命令末端的调节时间，单位 s；在给定样本内未稳定时返回 None。

    稳定判据（与控制器验收口径一致）：跟踪误差绝对值 ``<= 0.02`` rad、速度绝对值
    ``<= 0.05`` rad/s，且此后 ``0.25`` s 内样本必须持续满足同一对限值。只有时间戳
    已经到达（或超过）命令总时长 ``duration`` 的样本才有资格成为候选，因此返回的是
    ``候选时刻 - duration``，即相对运动结束的额外稳定耗时；找不到候选或数据在
    稳定窗口结束前就中断时返回 None。容差 1e-9 用于容忍浮点时间戳舍入。
    """
    for position, sample in enumerate(samples):
        if sample["elapsed_sec"] + 1e-9 < duration:
            continue
        error = abs(sample["desired_positions"][index] - sample["observed_positions"][index])
        velocity = abs(sample["velocities"][index])
        if error > 0.02 or velocity > 0.05:
            continue
        horizon = sample["elapsed_sec"] + 0.25
        stable = [later for later in samples[position:] if later["elapsed_sec"] <= horizon + 1e-9]
        if stable and stable[-1]["elapsed_sec"] + 1e-9 >= horizon and all(
            abs(later["desired_positions"][index] - later["observed_positions"][index]) <= 0.02
            and abs(later["velocities"][index]) <= 0.05
            for later in stable
        ):
            return sample["elapsed_sec"] - duration
    return None


def _overshoot(observed: Sequence[float], start: float, target: float) -> float:
    """计算越冲量，单位 rad，永远非负。

    方向由 ``target - start`` 决定：正向运动取"最大值超出目标"的部分，负向运动取
    "最小值低于目标"的部分（用相反方向的最大值会得到 0，失去意义）。起止相同
    （零位移）时没有方向可言，退化为取"离目标最远的偏差"，使静止指令下的抖动仍
    能被观测到。
    """
    delta = target - start
    if delta > 0.0:
        return max(0.0, max(observed) - target)
    if delta < 0.0:
        return max(0.0, target - min(observed))
    return max(abs(value - target) for value in observed)


def _interpolate_values(samples: Sequence[Mapping[str, object]], elapsed: float, index: int) -> tuple[float, float]:
    """把有序样本插值到指定时刻，返回 (实测位置 rad, 力矩)。

    超出样本时间范围时按端点保持（不外推，避免用线性外推制造看似合理的假数据）；
    范围内用二分定位区间并做线性插值。要求 ``samples`` 已按时间升序，函数内部不再
    排序。位置与力矩都用同一个插值比例，保证两者时间对齐。
    """
    times = [sample["elapsed_sec"] for sample in samples]
    if elapsed <= times[0]:
        return samples[0]["observed_positions"][index], samples[0]["efforts"][index]
    if elapsed >= times[-1]:
        return samples[-1]["observed_positions"][index], samples[-1]["efforts"][index]
    upper = bisect_right(times, elapsed)
    lower = samples[upper - 1]
    higher = samples[upper]
    ratio = (elapsed - lower["elapsed_sec"]) / (higher["elapsed_sec"] - lower["elapsed_sec"])
    position = lower["observed_positions"][index] + (
        higher["observed_positions"][index] - lower["observed_positions"][index]
    ) * ratio
    effort = lower["efforts"][index] + (
        higher["efforts"][index] - lower["efforts"][index]
    ) * ratio
    return float(position), float(effort)


def _correlation(pairs: Sequence[tuple[float, float]]) -> float | None:
    """手写皮尔逊相关系数（不引入第三方统计库），返回 -1..1 或 None。

    少于 2 对样本、或任一侧方差为 0（信号恒定）时相关性没有定义，返回 None 而不是
    抛异常或返回 0——报告里 None 表示"不可判定"，0 会被误读为"无关"。分母阈值
    1e-15 用于拦住浮点下溢导致的除零。
    """
    if len(pairs) < 2:
        return None
    left = [pair[0] for pair in pairs]
    right = [pair[1] for pair in pairs]
    left_mean = fmean(left)
    right_mean = fmean(right)
    numerator = sum((a - left_mean) * (b - right_mean) for a, b in pairs)
    left_energy = sum((a - left_mean) ** 2 for a in left)
    right_energy = sum((b - right_mean) ** 2 for b in right)
    denominator = math.sqrt(left_energy * right_energy)
    return None if denominator <= 1e-15 else numerator / denominator


def _rms(values: Sequence[float]) -> float:
    """均方根：``sqrt(mean(v^2))``。调用方必须保证序列非空，空序列会除零。"""
    return math.sqrt(sum(value * value for value in values) / float(len(values)))


def _vector6(values: Sequence[float], label: str) -> tuple[float, ...]:
    """把输入转成长度恰好等于手臂关节数的有限浮点元组，否则抛 :class:`ValueError`。

    ``label`` 只用于报错信息，便于定位是哪一个向量不合格（校验失败必须显式失败，
    不能靠切片补齐或忽略多余元素，否则错位的关节数据会静默进入统计）。
    """
    result = tuple(float(value) for value in values)
    if len(result) != len(ARM_JOINT_NAMES) or any(not math.isfinite(value) for value in result):
        raise ValueError(f"{label} must contain six finite values")
    return result

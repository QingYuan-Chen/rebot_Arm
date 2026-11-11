"""示教记录的滤波,质量评估和派生数据装配.

职责:组织平滑/低通,显示重采样和运动层的分级时间参数化,不执行运动或读写设备.
对外接口:prepare_teach_replay_samples,retime_teach_samples 及质量/预览字典转换.
安全约束:原始样本不原地修改;生成导数后必须重新检查实际曲线,质量摘要不能替代
完整拼接后的模型限位与碰撞检查.重建算法归 motion,本模块只做调用和记录格式适配.
文件结构:位置处理 → 重定时适配 → 滤波/重采样 → 准备编排 → 状态与显示预览.
"""

from __future__ import annotations

import math
from dataclasses import replace

from rebotarm_motion.trajectory_time_parameterization import parameterize_teach_samples

from .teach_models import (
    PreparedTeachReplay,
    ReplayStartBand,
    RetimedTeachPoint,
    TeachSample,
    TeachTrajectoryQuality,
)
from .teach_quality import (
    _velocity_limits_for_joints,
    analyze_teach_trajectory,
    teach_trajectory_quality_to_dict,
)


def interpolate_joint_positions(
    *,
    current_positions: tuple[float, ...],
    target_positions: tuple[float, ...],
    steps: int,
) -> list[tuple[float, ...]]:
    """在两组关节位置之间做等参数线性插值,返回含首末点的插值序列(关节角,rad).

    ``steps`` 是**总点数**而非段数,且被夹到至少 2(否则无法同时包含首末点);
    第 i 个点的系数为 ``i / (steps - 1)``,因此首点等于 ``current_positions``,
    末点等于 ``target_positions``.两个向量长度必须一致,否则抛 ``ValueError``
    (这是调用方的编程错误,不是可恢复的运行状态).
    """
    if len(current_positions) != len(target_positions):
        raise ValueError("current and target joint vectors have different lengths")
    count = max(int(steps), 2)
    points: list[tuple[float, ...]] = []
    for index in range(count):
        alpha = float(index) / float(count - 1)
        points.append(
            tuple(
                float(current + (target - current) * alpha)
                for current, target in zip(current_positions, target_positions)
            )
        )
    return points


def build_replay_start_soft_points(
    *,
    current_positions: tuple[float, ...],
    first_positions: tuple[float, ...],
    start_band: str,
    start_hold_sec: float = 0.8,
    soft_start_duration: float = 1.0,
    soft_start_steps: int = 30,
    align_duration: float = 3.0,
    align_steps: int = 30,
    first_hold_sec: float = 0.3,
) -> list[RetimedTeachPoint]:
    """构造回放开始前的"保持 → 对齐 → 再保持"引导段(位置单位 rad,时间单位秒).

    段结构(合成点的 ``source_sample`` 一律为 -1,便于与真实示教帧区分):
      1. 起始保持段:时长 ``start_hold_sec``,停在当前位姿,给控制器和操作者一个缓冲;
      2. 对齐段:按静止到静止五次曲线从当前位姿到记录首帧.分档为 ``align`` 时使用
         ``align_duration`` / ``align_steps``(较慢较长,属于真正的位置对齐);其余分档
         (如 ``direct``)使用 ``soft_start_duration`` / ``soft_start_steps`` 的软启动;
      3. 首帧保持段:时长 ``first_hold_sec``,在记录首帧显式给出零速度,零加速度约束.
    本函数合成完整 q/v/a;整条回放最终使用的等级由构建器根据录制段实际字段决定,
    不能用合成段的导数补造录制段约束,也不能据参考静止声称实际机械臂已停稳.
    时长与点数都被夹到非负/至少 2;时间戳必须严格递增,若某合成点的时间不大于上一点则
    直接丢弃,避免生成零时长或时间回退的点(重定时/执行要求时间严格单调).
    """
    if len(current_positions) != len(first_positions):
        raise ValueError("current and first joint vectors have different lengths")
    elapsed = 0.0
    points: list[RetimedTeachPoint] = []
    zeros = (0.,)*len(current_positions)
    hold = max(float(start_hold_sec), 0.0)
    if hold > 0.0:
        elapsed += hold
        points.append(
            RetimedTeachPoint(
                time_from_start=elapsed,
                positions=tuple(float(v) for v in current_positions),
                source_sample=-1,
                velocities=zeros, accelerations=zeros,
            )
        )
    band = str(start_band or "").strip().lower()
    if band == ReplayStartBand.ALIGN.value:
        # 真正的"回起始点"对齐:用更长的时长和更多的点,避免对齐本身产生速度冲击.
        duration = max(float(align_duration), 0.0)
        steps = int(align_steps)
    else:
        # 起点已经足够接近:只做一段短软启动,把速度从 0 平滑拉起来.
        duration = max(float(soft_start_duration), 0.0)
        steps = int(soft_start_steps)
    align_points = interpolate_joint_positions(
        current_positions=tuple(float(v) for v in current_positions),
        target_positions=tuple(float(v) for v in first_positions),
        steps=steps,
    )
    for index, positions in enumerate(align_points):
        ratio = float(index) / float(max(len(align_points) - 1, 1))
        timestamp = elapsed + duration * ratio
        # 时长为 0 或点数过多时会算出与上一点相同/更早的时间戳,必须跳过.
        if points and timestamp <= points[-1].time_from_start:
            continue
        from rebotarm_motion.trajectory_messages import rest_to_rest_sample
        if duration > 0:
            # 前面的线性列表只确定采样数量;实际 q/v/a 统一取运动层的解析五次曲线.
            positions, velocities, accelerations = rest_to_rest_sample(
                current_positions, first_positions, duration, ratio)
        else:
            velocities = accelerations = zeros
        points.append(
            RetimedTeachPoint(
                time_from_start=timestamp,
                positions=tuple(float(v) for v in positions),
                source_sample=-1,
                velocities=velocities, accelerations=accelerations,
            )
        )
    elapsed += duration
    first_hold = max(float(first_hold_sec), 0.0)
    if first_hold > 0.0:
        elapsed += first_hold
        if not points or elapsed > points[-1].time_from_start:
            points.append(
                RetimedTeachPoint(
                    time_from_start=elapsed,
                    positions=tuple(float(v) for v in first_positions),
                    source_sample=-1,
                    velocities=zeros, accelerations=zeros,
                )
            )
    return points


def _max_position_delta(a: tuple[float, ...], b: tuple[float, ...]) -> float:
    """两组关节位置之间逐关节差值的最大绝对值(rad);长度不一致返回正无穷."""
    if len(a) != len(b):
        return float("inf")
    return max((abs(float(current) - float(last)) for current, last in zip(a, b)), default=0.0)


def _motion_scope(samples: list[TeachSample]) -> tuple[float, float]:
    """统计轨迹运动规模,返回 ``(单关节最大行程, 全轨迹总运动量)``,单位均为 rad.

    单关节最大行程 = 每个关节位置极差(最大减最小)中的最大值;
    总运动量 = 相邻帧之间所有关节增量绝对值之和(反映"走了多远的折线距离").
    样本为空,关节数为 0 或某帧位置长度与首帧不一致时返回 ``(inf, inf)``-长度不一致属于
    结构性异常,返回无穷大可以让上层按"大幅度运动"从严处理,而不是低估风险.
    """
    if not samples:
        return 0.0, 0.0
    joint_count = len(samples[0].positions)
    if joint_count == 0:
        return 0.0, 0.0
    mins = [float("inf") for _ in range(joint_count)]
    maxs = [float("-inf") for _ in range(joint_count)]
    total = 0.0
    previous: TeachSample | None = None
    for sample in samples:
        if len(sample.positions) != joint_count:
            return float("inf"), float("inf")
        for index, position in enumerate(sample.positions):
            value = float(position)
            mins[index] = min(mins[index], value)
            maxs[index] = max(maxs[index], value)
        if previous is not None:
            total += sum(
                abs(float(current) - float(last))
                for current, last in zip(sample.positions, previous.positions)
            )
        previous = sample
    max_span = max((upper - lower for lower, upper in zip(mins, maxs)), default=0.0)
    return max_span, total


def retime_teach_samples(
    samples: list[TeachSample],
    *,
    replay_speed: float,
    max_velocity_rad_s: float,
    max_acceleration_rad_s2: float = 999.0,
    max_jerk_rad_s3: float = 999.0,
    initial_delay_sec: float = 0.2,
    boundary_zero_velocity: bool = True,
) -> list[RetimedTeachPoint]:
    """把录制对象转换成共享数值点,统一调用 motion 六轴重定时."""
    if not samples:
        return []
    result = _parameterize_record(samples, method='auto', replay_speed=replay_speed,
        max_velocity_rad_s=max_velocity_rad_s, max_acceleration_rad_s2=max_acceleration_rad_s2,
        max_jerk_rad_s3=max_jerk_rad_s3, initial_delay_sec=initial_delay_sec,
        boundary_zero_velocity=boundary_zero_velocity)
    return [RetimedTeachPoint(p.time,p.positions,i,p.velocities,p.accelerations)
            for i,p in enumerate(result.points)]


def _parameterize_record(samples, *, max_velocity_rad_s, **settings):
    """唯一记录格式适配;数据等级,曲线生成和重定时结论均由运动层返回."""
    from rebotarm_motion.polynomial_trajectory import TrajectoryPoint
    names = samples[0].joint_names
    if any(s.joint_names != names for s in samples):
        raise ValueError('joint_names mismatch in teach retiming')
    return parameterize_teach_samples(names,
        tuple(TrajectoryPoint(s.stamp,s.positions,s.velocities,s.accelerations) for s in samples),
        velocity_limits=_velocity_limits_for_joints(max_velocity_rad_s,names), **settings)


def lowpass_filter_teach_samples(
    samples: list[TeachSample],
    *,
    sample_rate_hz: float,
    cutoff_hz: float,
    preserve_start_end: bool = True,
) -> list[TeachSample]:
    """对关节位置做零相位一阶低通滤波,抑制示教拖动与编码器噪声(时间戳与速度/力矩原样保留).

    实现是一阶 RC 低通:``alpha = dt / (RC + dt)``,``RC = 1 / (2π·截止频率)``,
    ``dt = 1 / 采样率``.先正向滤波一遍,再反向滤波一遍(filtfilt),使整体相位延迟相互
    抵消-否则滤波后的轨迹会相对原始记录滞后,起点对齐与终点判定都会失真.
    参数:``sample_rate_hz`` 采样率(Hz,下限 1),``cutoff_hz`` 截止频率(Hz,下限 0.01,
    越小越平滑但越容易削掉真实运动),``preserve_start_end`` 是否强制首末点等于原始值
    (默认 True:起点决定回放起点误差,终点决定停位,不允许被滤波改动).
    样本数 ≤ 2 时直接原样返回(无法滤波).位置之外的所有字段都不修改.
    """
    if len(samples) <= 2:
        return list(samples)
    rate = max(float(sample_rate_hz), 1.0)
    cutoff = max(float(cutoff_hz), 0.01)
    dt = 1.0 / rate
    rc = 1.0 / (2.0 * math.pi * cutoff)
    alpha = dt / (rc + dt)
    filtered_positions: list[tuple[float, ...]] = [tuple(float(v) for v in samples[0].positions)]
    for sample in samples[1:]:
        previous = filtered_positions[-1]
        filtered_positions.append(
            tuple(
                float(last) + alpha * (float(current) - float(last))
                for current, last in zip(sample.positions, previous)
            )
        )
    # 反向再滤一遍:与正向结果级联得到零相位响应(代价是幅度被平方衰减).
    backward = [filtered_positions[-1]]
    for positions in reversed(filtered_positions[:-1]):
        previous = backward[-1]
        backward.append(
            tuple(
                float(last) + alpha * (float(current) - float(last))
                for current, last in zip(positions, previous)
            )
        )
    filtered_positions = list(reversed(backward))
    if preserve_start_end:
        # 首末点还原为原始采样:起点/终点位置必须与记录严格一致.
        filtered_positions[0] = samples[0].positions
        filtered_positions[-1] = samples[-1].positions
    result: list[TeachSample] = []
    for sample, positions in zip(samples, filtered_positions):
        result.append(
            replace(sample, positions=tuple(float(v) for v in positions))
        )
    return result


def smooth_teach_samples(
    samples: list[TeachSample],
    *,
    window: int = 5,
    preserve_start_end: bool = True,
) -> list[TeachSample]:
    """对关节位置做滑动均值平滑(窗口不足时自动缩短,边缘不做补零).

    ``window`` 为窗口长度(样本数),会被夹到至少 1 并**向上取整为奇数**,使窗口关于当前
    样本对称,不引入整体相位偏移;窗口半径 ``radius = width // 2``,首尾样本处窗口自动截断
    到有效范围后取平均(因此边缘平滑力度弱于中间,不会像补零那样把边缘拉向 0).
    ``preserve_start_end=True`` 时首末帧原样保留.非位置字段不做任何修改.
    样本数 ≤ 2 时直接原样返回.
    """
    if len(samples) <= 2:
        return list(samples)
    width = max(int(window), 1)
    if width % 2 == 0:
        width += 1
    radius = width // 2
    smoothed: list[TeachSample] = []
    for index, sample in enumerate(samples):
        if preserve_start_end and index in (0, len(samples) - 1):
            smoothed.append(sample)
            continue
        start = max(0, index - radius)
        end = min(len(samples), index + radius + 1)
        segment = samples[start:end]
        positions = tuple(
            sum(float(item.positions[joint_index]) for item in segment) / float(len(segment))
            for joint_index in range(len(sample.positions))
        )
        smoothed.append(
            replace(sample, positions=positions)
        )
    return smoothed


def resample_teach_samples(
    samples: list[TeachSample],
    *,
    rate_hz: float = 50.0,
) -> list[TeachSample]:
    """把示教样本线性插值到等间隔时间网格上(重采样),保证下游可按固定周期执行.

    输出点数由记录时长与目标频率决定(``round(时长 / 周期) + 1``,至少 1 段),网格从首帧
    时间戳开始,按目标周期递增,网格点超出记录末尾时夹到末帧时间戳.
    查找区间时只向前推进 ``source_index``(样本时间单调,单次线性扫描即可),区间内按时间
    比例线性插值;区间退化时 ``alpha`` 取 0.
    输出样本的 ``velocities`` 与 ``efforts`` 被清空(插值后的速度/力矩不再可信,重定时会
    重新计算速度),``joint_names`` / ``motor_status`` / ``arm_state`` 沿用区间左端点.
    首末样本用原始对象整体替换,确保端点位置与状态标记严格等于原始记录.
    ``rate_hz`` 下限 1 Hz;样本数 ≤ 1 或时长非正时原样返回.
    """
    if len(samples) <= 1:
        return list(samples)
    rate = max(float(rate_hz), 1.0)
    period = 1.0 / rate
    start_stamp = float(samples[0].stamp)
    end_stamp = float(samples[-1].stamp)
    duration = max(0.0, end_stamp - start_stamp)
    if duration <= 0.0:
        return list(samples)
    result: list[TeachSample] = []
    source_index = 0
    count = max(int(round(duration / period)), 1)
    for output_index in range(count + 1):
        stamp = start_stamp + min(float(output_index) * period, duration)
        while source_index + 1 < len(samples) and float(samples[source_index + 1].stamp) < stamp:
            source_index += 1
        previous = samples[source_index]
        following = samples[min(source_index + 1, len(samples) - 1)]
        # 左右端点相同(区间退化)时直接取该点,不做除零.
        span = max(float(following.stamp) - float(previous.stamp), 1e-9)
        alpha = 0.0 if following is previous else (stamp - float(previous.stamp)) / span
        alpha = min(max(alpha, 0.0), 1.0)
        positions = tuple(
            float(a) + (float(b) - float(a)) * alpha
            for a, b in zip(previous.positions, following.positions)
        )
        result.append(
            TeachSample(
                stamp=stamp,
                joint_names=previous.joint_names,
                positions=positions,
                velocities=(),
                efforts=(),
                motor_status=dict(previous.motor_status),
                arm_state=previous.arm_state,
            )
        )
    result[0] = samples[0]
    result[-1] = samples[-1]
    return result


def _has_structural_teach_anomaly(quality: TeachTrajectoryQuality) -> bool:
    """判断质量结果里是否含结构性异常(关节名不一致 / 位置长度不一致 / 时间戳非单调).

    这类异常无法靠重定时修复,因此预处理流水线据此跳过重定时;通过匹配英文异常文案实现,
    文案由 :func:`analyze_teach_trajectory` 生成,两处必须同步修改.
    """
    return any(
        "joint_names mismatch" in item
        or "positions length mismatch" in item
        or "timestamp not monotonic" in item
        for item in quality.anomalies
    )


def prepare_teach_replay_samples(
    samples: list[TeachSample],
    *,
    smoothing_enabled: bool = True,
    smoothing_window: int = 7,
    filter_enabled: bool = True,
    filter_cutoff_hz: float = 5.0,
    filter_sample_rate_hz: float = 50.0,
    resample_enabled: bool = True,
    resample_rate_hz: float = 100.0,
    retime_enabled: bool = False,
    replay_speed: float = 1.0,
    max_velocity_rad_s: float = 1.5,
    max_acceleration_rad_s2: float = 5.0,
    max_jerk_rad_s3: float = 20.0,
    time_parameterization_method: str = "auto",
    large_motion_span_rad: float = 0.8,
    large_motion_total_rad: float = 2.5,
    large_motion_max_speed: float = 1.0,
) -> PreparedTeachReplay:
    """生成真实回放要执行的预处理轨迹:原始 → 平滑 → 低通滤波 → 重采样 → 重定时.

    流水线顺序固定,每一步都受开关控制,且每一步之后/之前都会重新做一次质量评估,最终返回
    :class:`PreparedTeachReplay`(含三段质量,各步骤是否生效,实际参数与大幅度运动信息).
    参数含义与单位:
      ``smoothing_enabled`` / ``smoothing_window``   滑动均值平滑开关与窗口长度(样本数,奇数);
      ``filter_enabled`` / ``filter_cutoff_hz``      零相位低通滤波开关与截止频率(Hz);
      ``filter_sample_rate_hz``                      滤波假定的采样率(Hz,需与实际记录采样率相符,
                                                     否则 RC 系数失真);
      ``resample_enabled`` / ``resample_rate_hz``    重采样开关与目标频率(Hz);
      ``retime_enabled``                             是否做时间参数化;只有滤波后质量没有结构性
                                                     异常时才会执行;
      ``replay_speed``                               回放倍速,被夹到 [0.01, 1.0](不允许超过 1 倍);
      ``max_velocity_rad_s`` / ``max_acceleration_rad_s2`` / ``max_jerk_rad_s3``
                                                     速度/加速度/加加速度上限,同时用于质量评估
                                                     与重定时(rad/s,rad/s^2,rad/s^3);
      ``time_parameterization_method``               时间参数化后端(``auto`` 等),实际用哪个由
                                                     运动包的时间参数化模块决定,并把结果记入
                                                     ``time_parameterization_*`` 字段;
      ``large_motion_span_rad`` / ``large_motion_total_rad``
                                                     大幅度运动判据:单关节行程或总运动量任一达到
                                                     阈值即视为大幅度运动(rad);
      ``large_motion_max_speed``                     大幅度运动时的最大倍速(下限 0.01;当前实现
                                                     只回填该值,未用它改写 ``effective_speed``).
    安全语义:真实回放决策必须基于 ``after_quality``(重定时后的质量,等于 ``retimed_quality``);
    ``before_quality`` 只用于展示"处理前有多差".
    """
    from .teach_record_io import arm_replay_samples
    from rebotarm_motion.polynomial_trajectory import TrajectoryPoint
    from rebotarm_motion.trajectory_time_parameterization import teach_input_grade, prepare_teach_reference
    # 必须在滤波及六轴提取之前验证;否则 zip/按名提取可能掩盖非法原记录.
    input_grade = ''
    if samples:
        if len(set(samples[0].joint_names)) != len(samples[0].joint_names):
            raise ValueError('duplicate teach joint_names')
        if any(s.joint_names != samples[0].joint_names for s in samples):
            raise ValueError('joint_names mismatch in teach input')
        if any(len(s.positions) != len(s.joint_names) for s in samples):
            raise ValueError('positions length mismatch in teach input')
        input_grade = teach_input_grade(tuple(TrajectoryPoint(
            s.stamp, s.positions, s.velocities, s.accelerations) for s in samples))
    samples = arm_replay_samples(samples)
    max_joint_span, total_joint_motion = _motion_scope(samples)
    requested_speed = min(max(float(replay_speed), 0.01), 1.0)
    large_motion = (max_joint_span >= float(large_motion_span_rad)
                    or total_joint_motion >= float(large_motion_total_rad))
    effective_speed = requested_speed
    raw_quality = analyze_teach_trajectory(
        samples,
        max_velocity_rad_s=max_velocity_rad_s,
        max_acceleration_rad_s2=max_acceleration_rad_s2,
        max_jerk_rad_s3=max_jerk_rad_s3,
    )
    prepared = list(samples)
    smoothing_applied = False
    filter_applied = False
    resample_applied = False
    retime_applied = False
    if smoothing_enabled and prepared:
        prepared = smooth_teach_samples(prepared, window=smoothing_window)
        smoothing_applied = len(prepared) > 0
    if filter_enabled and len(prepared) > 2:
        # 只有超过 2 个样本才滤波:少于此数无明显噪声可滤,且 filtfilt 需要前后向都有数据.
        prepared = lowpass_filter_teach_samples(
            prepared,
            sample_rate_hz=filter_sample_rate_hz,
            cutoff_hz=filter_cutoff_hz,
        )
        filter_applied = len(prepared) > 0
    filtered_quality = analyze_teach_trajectory(
        prepared,
        max_velocity_rad_s=max_velocity_rad_s,
        max_acceleration_rad_s2=max_acceleration_rad_s2,
        max_jerk_rad_s3=max_jerk_rad_s3,
    )
    # 控制节点保留滤波后的源位置和相对时间;q/v 不生成 a,关闭处理时保留中间 v.
    path_samples = list(prepared)
    position_processed = smoothing_applied or filter_applied
    if path_samples:
        reference = prepare_teach_reference(tuple(TrajectoryPoint(
            s.stamp-samples[0].stamp, s.positions, s.velocities, s.accelerations) for s in path_samples),
            derive_velocity=position_processed)
        path_samples = [replace(s, velocities=p.velocities, accelerations=p.accelerations)
                        for s,p in zip(path_samples,reference)]
    velocity_source = ('derived_filtered_q_t' if position_processed else 'input') if input_grade != 'q' else 'absent'
    acceleration_source = ('derived_filtered_q_t' if position_processed else
        'derived_legacy_v4' if any(s.format_version == 4 for s in samples) else 'input') if input_grade == 'q/v/a' else 'absent'
    # 显示重采样不参与后续控制节点生成,避免线性细分或清空字段改变输入等级.
    six_axis = bool(samples) and samples[0].joint_names == tuple(f'joint{i}' for i in range(1,7))
    if resample_enabled and len(prepared) > 1 and not six_axis:
        prepared = resample_teach_samples(prepared, rate_hz=resample_rate_hz)
        # 以"样本数是否变化"判定重采样是否真的生效(等间隔记录可能点数不变).
        resample_applied = len(prepared) != len(samples)
    retimed_points: list[RetimedTeachPoint] = []
    if retime_enabled and path_samples and not _has_structural_teach_anomaly(filtered_quality):
        # 数据格式适配后直接交运动层;执行参数化只有一套分级算法.
        time_parameterization = _parameterize_record(
            path_samples,
            method=time_parameterization_method,
            replay_speed=effective_speed,
            max_velocity_rad_s=max_velocity_rad_s,
            max_acceleration_rad_s2=max_acceleration_rad_s2,
            max_jerk_rad_s3=max_jerk_rad_s3,
            initial_delay_sec=0.0,
            boundary_zero_velocity=True,
        )
        retimed_points = [RetimedTeachPoint(p.time,p.positions,i,p.velocities,p.accelerations)
                          for i,p in enumerate(time_parameterization.points)]
        retime_applied = len(retimed_points) > 0
    else:
        time_parameterization = None
    # 把重定点包装成样本以便复用统一的质量评估(efforts/motor_status 无意义故清空).
    retimed_samples = [
        TeachSample(
            stamp=point.time_from_start,
            joint_names=prepared[0].joint_names if prepared else (),
            positions=point.positions,
            velocities=point.velocities,
            accelerations=point.accelerations,
            efforts=(),
            motor_status={},
            arm_state="RETIMED",
        )
        for point in retimed_points
    ]
    curve_analysis = {}
    if retimed_samples and six_axis:
        from rebotarm_motion.polynomial_trajectory import TrajectoryPoint, build_curve
        curve = build_curve(retimed_samples[0].joint_names, tuple(TrajectoryPoint(
            p.time_from_start,p.positions,p.velocities,p.accelerations) for p in retimed_points))
        if curve.duration > 300.:
            raise ValueError('retimed duration exceeds execution budget')
        # 跳变/差分质量仍按原有固定采样频率评估;控制节点间距不是周期命令跳变.
        # 从实际参考曲线采样,计算 a 仅用于分析,不能回填为新的执行约束.
        from rebotarm_motion.trajectory_time_parameterization import teach_curve_analysis
        curve_analysis = teach_curve_analysis(curve)
        count = math.ceil(curve.duration*max(float(resample_rate_hz),1.))
        retimed_samples = [replace(retimed_samples[0],stamp=t,positions=q,
            velocities=v if input_grade != 'q' else (),accelerations=a if input_grade == 'q/v/a' else ())
            for t in ((curve.duration*i/count for i in range(count+1)) if count else (0.,))
            for q,v,a,_ in (curve.sample(t),)]
        resample_applied = bool(resample_enabled) and len(retimed_samples) != len(samples)
    # 没有重定点时退回评估预处理样本本身,保证 after_quality 始终反映"将要执行的东西".
    retimed_quality = analyze_teach_trajectory(
        retimed_samples if retimed_samples else prepared,
        max_velocity_rad_s=max_velocity_rad_s,
        max_acceleration_rad_s2=max_acceleration_rad_s2,
        max_jerk_rad_s3=max_jerk_rad_s3,
        check_dynamics=not bool(curve_analysis),
    )
    if curve_analysis:
        # 固定频率位置跳变门仍生效;执行动态判定只使用段内解析峰值.
        peaks = curve_analysis['segment_peaks']
        retimed_quality = replace(retimed_quality, max_velocity_rad_s=peaks['velocity'],
            max_acceleration_rad_s2=peaks['acceleration'],max_jerk_rad_s3=peaks['jerk'],
            velocity_limit_rad_s=max(_velocity_limits_for_joints(max_velocity_rad_s,samples[0].joint_names)),
            acceleration_limit_rad_s2=max_acceleration_rad_s2,jerk_limit_rad_s3=max_jerk_rad_s3)
    return PreparedTeachReplay(
        samples=path_samples if six_axis else prepared,
        raw_quality=raw_quality,
        filtered_quality=filtered_quality,
        retimed_quality=retimed_quality,
        smoothing_applied=smoothing_applied,
        filter_applied=filter_applied,
        resample_applied=resample_applied,
        retime_applied=retime_applied,
        # 以下参数一律回填"夹紧之后的实际值",与真正参与计算的值保持一致.
        smoothing_window=max(int(smoothing_window), 1),
        filter_cutoff_hz=max(float(filter_cutoff_hz), 0.01),
        filter_sample_rate_hz=max(float(filter_sample_rate_hz), 1.0),
        resample_rate_hz=max(float(resample_rate_hz), 1.0),
        retimed_points=retimed_points,
        input_grade=input_grade,reference_grade=input_grade,velocity_source=velocity_source,
        acceleration_source=acceleration_source,curve_analysis=curve_analysis,
        large_motion=large_motion,
        max_joint_span_rad=max_joint_span,
        total_joint_motion_rad=total_joint_motion,
        requested_replay_speed=requested_speed,
        effective_replay_speed=effective_speed,
        large_motion_max_speed=max(float(large_motion_max_speed), 0.01),
        time_parameterization_requested_method=(
            time_parameterization.requested_method if time_parameterization is not None else str(time_parameterization_method or "auto")
        ),
        time_parameterization_used_method=(
            time_parameterization.used_method if time_parameterization is not None else "none"
        ),
        time_parameterization_message=(
            time_parameterization.message if time_parameterization is not None else "retime disabled"
        ),
    )


def prepared_teach_replay_to_dict(prepared: PreparedTeachReplay) -> dict:
    """把预处理结果转成 JSON 可序列化的状态 payload(键名为对外接口,不可改动).

    只输出摘要与质量数据,不输出全部轨迹点(点数很大);``prepared_samples`` /
    ``retimed_points`` 是数量而非数组.``before_quality`` 与 ``after_quality`` 是门控与实际
    判定的两个关键字段,``raw/filtered/retimed_quality`` 用于排查是哪一步导致了等级变化.
    """
    return {
        "input_grade": prepared.input_grade,
        "reference_grade": prepared.reference_grade,
        "velocity_source": prepared.velocity_source,
        "acceleration_source": prepared.acceleration_source,
        "curve_analysis": prepared.curve_analysis,
        "smoothing_applied": prepared.smoothing_applied,
        "filter_applied": prepared.filter_applied,
        "resample_applied": prepared.resample_applied,
        "retime_applied": prepared.retime_applied,
        "smoothing_window": prepared.smoothing_window,
        "filter_cutoff_hz": prepared.filter_cutoff_hz,
        "filter_sample_rate_hz": prepared.filter_sample_rate_hz,
        "resample_rate_hz": prepared.resample_rate_hz,
        "prepared_samples": len(prepared.samples),
        "retimed_points": len(prepared.retimed_points),
        "time_parameterization": {
            "requested_method": prepared.time_parameterization_requested_method,
            "used_method": prepared.time_parameterization_used_method,
            "message": prepared.time_parameterization_message,
        },
        "before_quality": teach_trajectory_quality_to_dict(prepared.before_quality),
        "after_quality": teach_trajectory_quality_to_dict(prepared.after_quality),
        "raw_quality": teach_trajectory_quality_to_dict(prepared.raw_quality),
        "filtered_quality": teach_trajectory_quality_to_dict(prepared.filtered_quality),
        "retimed_quality": teach_trajectory_quality_to_dict(prepared.retimed_quality),
        "large_motion": {
            "enabled": prepared.large_motion,
            "max_joint_span_rad": prepared.max_joint_span_rad,
            "total_joint_motion_rad": prepared.total_joint_motion_rad,
            "requested_speed": prepared.requested_replay_speed,
            "effective_speed": prepared.effective_replay_speed,
            "large_motion_max_speed": prepared.large_motion_max_speed,
        },
    }


def teach_trajectory_preview_to_dict(
    samples: list[TeachSample],
    *,
    max_points: int = 500,
) -> dict:
    """把(可能很长的)示教轨迹压缩成给界面画曲线用的预览 payload.

    点数超过 ``max_points``(下限 1)时按 ``round(i * (n-1) / (limit-1))`` 均匀抽取,
    并用集合去重后排序,因此首末点一定包含,下标严格递增;实际使用的抽样步长以
    ``downsample_step`` 回填,便于界面说明"这是抽样后的曲线".
    每个点的时间 ``t`` 是相对首帧的秒数(负数被夹到 0),位置按关节名成字典输出(rad).
    ``sample`` 保留原始下标,便于把质量事件里的 ``sample`` 直接映射到曲线位置.
    样本为空时返回空结构(但仍带回一份 red 质量结果).
    """
    source_count = len(samples)
    if (samples and samples[0].joint_names == tuple(f'joint{i}' for i in range(1,7))
            and all(s.arm_state == 'PREPARED_REPLAY' for s in samples)):
        from rebotarm_motion.polynomial_trajectory import TrajectoryPoint, build_curve
        curve = build_curve(samples[0].joint_names,tuple(TrajectoryPoint(
            s.stamp-samples[0].stamp,s.positions,s.velocities,s.accelerations) for s in samples))
        # 页面抽样的是执行曲线,不能将源节点间折线当作三次参考路径.
        count = min(max(int(max_points),2)-1,math.ceil(curve.duration*150.))
        samples = [replace(samples[0],stamp=t,positions=q,
            velocities=v if curve.degree>=3 else (),accelerations=a if curve.degree==5 else ())
            for t in ((curve.duration*i/count for i in range(count+1)) if count else (0.,))
            for q,v,a,_ in (curve.sample(t),)]
    quality = analyze_teach_trajectory(samples)
    if not samples:
        return {
            "joint_names": [],
            "raw_samples": 0,
            "returned_samples": 0,
            "downsample_step": 1,
            "duration_sec": 0.0,
            "quality": teach_trajectory_quality_to_dict(quality),
            "points": [],
            "events": [],
        }
    limit = max(int(max_points), 1)
    # 向上取整的采样步长,仅用于展示"原始点 → 抽样点"的稀疏程度.
    step = max(1, (len(samples) + limit - 1) // limit)
    first_stamp = float(samples[0].stamp)
    if len(samples) <= limit:
        selected_indices = list(range(len(samples)))
    elif limit == 1:
        selected_indices = [0]
    else:
        selected_indices = sorted(
            {
                round(index * (len(samples) - 1) / (limit - 1))
                for index in range(limit)
            }
        )
    points = []
    for index in selected_indices:
        sample = samples[index]
        points.append(
            {
                "sample": index,
                "t": max(0.0, float(sample.stamp) - first_stamp),
                "positions": {
                    name: float(position)
                    for name, position in zip(sample.joint_names, sample.positions)
                },
                "arm_state": sample.arm_state,
            }
        )
    return {
        "joint_names": list(samples[0].joint_names),
        "raw_samples": source_count,
        "returned_samples": len(points),
        "downsample_step": step,
        "duration_sec": max(0.0, float(samples[-1].stamp) - first_stamp),
        "quality": teach_trajectory_quality_to_dict(quality),
        "points": points,
        "events": [
            {
                "sample": event.sample,
                "joint_name": event.joint_name,
                "level": event.level,
                "message": event.message,
                "delta_rad": event.delta_rad,
                "velocity_rad_s": event.velocity_rad_s,
            }
            for event in quality.events
        ],
    }

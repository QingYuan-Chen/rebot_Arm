"""Teach trajectory quality analysis and record inspection."""

from __future__ import annotations

from json import JSONDecodeError
from pathlib import Path

from .teach_models import TeachRecordInfo, TeachSample, TeachTrajectoryEvent, TeachTrajectoryQuality
from .teach_record_io import load_teach_samples
from .teach_replay_gates import classify_replay_start


def _velocity_limits_for_joints(
    max_velocity_rad_s,
    joint_names: tuple[str, ...],
) -> tuple[float, ...]:
    """把速度上限参数展开成与 ``joint_names`` 等长的逐关节上限（rad/s）。

    支持三种写法：字典（键为关节名，``"*"`` 为缺省值，默认 2.0 rad/s）、与关节数等长的
    列表/元组、以及单值（所有关节同限）。每个上限都被夹到至少 0.01 rad/s，防止 0 或负数
    导致后续按速度推算时间时除零；列表长度不等于关节数时抛 ``ValueError``。
    """
    if isinstance(max_velocity_rad_s, dict):
        fallback = float(max_velocity_rad_s.get("*", 2.0))
        return tuple(max(float(max_velocity_rad_s.get(name, fallback)), 0.01) for name in joint_names)
    if isinstance(max_velocity_rad_s, (list, tuple)):
        if len(max_velocity_rad_s) != len(joint_names):
            raise ValueError("max_velocity_rad_s length must match joint_names")
        return tuple(max(float(value), 0.01) for value in max_velocity_rad_s)
    return tuple(max(float(max_velocity_rad_s), 0.01) for _ in joint_names)


def _velocity_limit_summary(max_velocity_rad_s) -> float:
    """把速度上限参数压成一个标量用于回填质量结果：多关节时取最大值，单位 rad/s。"""
    if isinstance(max_velocity_rad_s, dict):
        values = [float(value) for value in max_velocity_rad_s.values()]
        return max(values, default=0.0)
    if isinstance(max_velocity_rad_s, (list, tuple)):
        return max((float(value) for value in max_velocity_rad_s), default=0.0)
    return float(max_velocity_rad_s)


def analyze_teach_trajectory(
    samples: list[TeachSample],
    *,
    green_jump_rad: float = 0.03,
    yellow_jump_rad: float = 0.05,
    max_velocity_rad_s: float = 2.0,
    max_acceleration_rad_s2: float = 999.0,
    max_jerk_rad_s3: float = 999.0,
) -> TeachTrajectoryQuality:
    """逐帧差分评估示教轨迹质量，给出 green/yellow/red 分档与异常清单。

    判定规则（阈值单位：rad、rad/s、rad/s^2、rad/s^3）：
      - 单帧关节增量 > ``yellow_jump_rad`` 记 ``red`` 事件（并按 red 处理，禁止真实回放）；
      - 增量在 ``green_jump_rad`` 与 ``yellow_jump_rad`` 之间记 ``yellow`` 事件；
      - 速度 / 加速度 / 加加速度超过各自上限各记一条 ``yellow`` 异常；
      - 关节名不一致、位置向量长度不一致、时间戳非单调（``dt <= 0``）记结构性异常并直接
        判 ``red``：这类记录连差分都不可信，不能靠重定时"抢救"。
    风险等级只升不降（已经 red 不会被后续 yellow 覆盖）。速度/加速度/加加速度用相邻帧差分，
    因此对采集噪声敏感，这也是后续要做滤波与重采样的原因。
    ``samples`` 为空时返回 red 且 ``replay_policy`` 为 "record contains no samples"。
    """
    anomalies: list[str] = []
    events: list[TeachTrajectoryEvent] = []
    risk_level = "green"
    max_jump = 0.0
    max_velocity = 0.0
    max_acceleration = 0.0
    max_jerk = 0.0
    worst_joint = ""
    worst_sample = -1
    if not samples:
        return TeachTrajectoryQuality(
            risk_level="red",
            replay_policy="record contains no samples",
            allow_real_replay=False,
            requires_safe_retiming=False,
            max_jump_rad=0.0,
            max_velocity_rad_s=0.0,
            max_acceleration_rad_s2=0.0,
            max_jerk_rad_s3=0.0,
            worst_joint="",
            worst_sample=-1,
            anomalies=("empty record",),
            events=(),
            green_jump_rad=float(green_jump_rad),
            yellow_jump_rad=float(yellow_jump_rad),
            velocity_limit_rad_s=_velocity_limit_summary(max_velocity_rad_s),
            acceleration_limit_rad_s2=float(max_acceleration_rad_s2),
            jerk_limit_rad_s3=float(max_jerk_rad_s3),
        )
    expected_joints = samples[0].joint_names
    expected_len = len(expected_joints)
    velocity_limits = _velocity_limits_for_joints(max_velocity_rad_s, expected_joints)
    previous = samples[0]
    # 首帧没有前一帧，差分初值取零：首帧速度按 0 处理，避免伪造出一个"跳跃起步"。
    previous_velocities = tuple(0.0 for _ in range(expected_len))
    previous_accelerations = tuple(0.0 for _ in range(expected_len))
    if len(previous.positions) != expected_len:
        anomalies.append("positions length mismatch at sample 0")
        risk_level = "red"
    for index, sample in enumerate(samples[1:], start=1):
        if sample.joint_names != expected_joints:
            anomalies.append(f"joint_names mismatch at sample {index}")
            risk_level = "red"
        if len(sample.positions) != expected_len:
            anomalies.append(f"positions length mismatch at sample {index}")
            risk_level = "red"
        dt = float(sample.stamp) - float(previous.stamp)
        if dt <= 0.0:
            # 时间戳不前进：差分方向无意义，整条记录判 red，且下面不再使用该 dt。
            anomalies.append(f"timestamp not monotonic at sample {index}")
            risk_level = "red"
        current_velocities = tuple(0.0 for _ in range(expected_len))
        current_accelerations = tuple(0.0 for _ in range(expected_len))
        if dt > 0.0 and len(sample.positions) == expected_len and len(previous.positions) == expected_len:
            # 一阶差分得速度 (rad/s)，再对速度一阶差分得加速度 (rad/s^2)。
            current_velocities = tuple(
                (float(current) - float(last)) / dt
                for current, last in zip(sample.positions, previous.positions)
            )
            current_accelerations = tuple(
                (float(current) - float(last)) / dt
                for current, last in zip(current_velocities, previous_velocities)
            )
        for joint_index, (joint_name, current, last) in enumerate(zip(expected_joints, sample.positions, previous.positions)):
            delta = abs(float(current) - float(last))
            velocity = abs(current_velocities[joint_index]) if dt > 0.0 else None
            # 只记录"更差"的极值，用于回填 worst_joint / worst_sample（最大单帧跳变点）。
            if delta > max_jump:
                max_jump = delta
                worst_joint = joint_name
                worst_sample = index
            if velocity is not None and velocity > max_velocity:
                max_velocity = velocity
            acceleration = None
            jerk = None
            if velocity is not None and dt > 0.0:
                acceleration = abs(float(current_accelerations[joint_index]))
                if acceleration > max_acceleration:
                    max_acceleration = acceleration
                # 加加速度 = 相邻两帧加速度之差除以 dt (rad/s^3)。
                jerk = abs(float(current_accelerations[joint_index]) - float(previous_accelerations[joint_index])) / dt
                if jerk > max_jerk:
                    max_jerk = jerk
            level = ""
            message = ""
            if delta > float(yellow_jump_rad):
                level = "red"
                message = f"{joint_name} jump {delta:.4f} rad at sample {index}"
                risk_level = "red"
                anomalies.append(message)
            elif delta > float(green_jump_rad):
                level = "yellow"
                message = f"{joint_name} jump {delta:.4f} rad at sample {index}"
                if risk_level != "red":
                    risk_level = "yellow"
            velocity_limit = velocity_limits[joint_index]
            if velocity is not None and velocity > velocity_limit:
                # 同一帧已有更严重的事件（跳变）时保留原 message/level，只追加异常行。
                velocity_message = f"{joint_name} velocity {velocity:.4f} rad/s at sample {index}"
                if not message:
                    message = velocity_message
                    level = "yellow"
                if risk_level != "red":
                    risk_level = "yellow"
                anomalies.append(velocity_message)
            if acceleration is not None and acceleration > float(max_acceleration_rad_s2):
                acceleration_message = f"{joint_name} acceleration {acceleration:.4f} rad/s^2 at sample {index}"
                if not message:
                    message = acceleration_message
                    level = "yellow"
                if risk_level != "red":
                    risk_level = "yellow"
                anomalies.append(acceleration_message)
            if jerk is not None and jerk > float(max_jerk_rad_s3):
                jerk_message = f"{joint_name} jerk {jerk:.4f} rad/s^3 at sample {index}"
                if not message:
                    message = jerk_message
                    level = "yellow"
                if risk_level != "red":
                    risk_level = "yellow"
                anomalies.append(jerk_message)
            if level:
                events.append(
                    TeachTrajectoryEvent(
                        sample=index,
                        joint_name=joint_name,
                        level=level,
                        message=message,
                        delta_rad=delta,
                        velocity_rad_s=velocity,
                        acceleration_rad_s2=acceleration,
                        jerk_rad_s3=jerk,
                    )
                )
        if dt > 0.0:
            # 只有 dt 有效时才滚动差分状态，避免非法 dt 污染后续帧的加速度/jerk。
            previous_velocities = current_velocities
            previous_accelerations = current_accelerations
        previous = sample
    if risk_level == "green":
        replay_policy = "normal replay allowed"
    elif risk_level == "yellow":
        replay_policy = "safe retiming required before real replay"
    else:
        replay_policy = "real replay blocked; record a cleaner teach trajectory"
    return TeachTrajectoryQuality(
        risk_level=risk_level,
        replay_policy=replay_policy,
        allow_real_replay=risk_level != "red",
        requires_safe_retiming=risk_level == "yellow",
        max_jump_rad=max_jump,
        max_velocity_rad_s=max_velocity,
        max_acceleration_rad_s2=max_acceleration,
        max_jerk_rad_s3=max_jerk,
        worst_joint=worst_joint,
        worst_sample=worst_sample,
        anomalies=tuple(anomalies),
        events=tuple(events),
        green_jump_rad=float(green_jump_rad),
        yellow_jump_rad=float(yellow_jump_rad),
        velocity_limit_rad_s=max(velocity_limits, default=0.0),
        acceleration_limit_rad_s2=float(max_acceleration_rad_s2),
        jerk_limit_rad_s3=float(max_jerk_rad_s3),
    )


def teach_trajectory_quality_to_dict(quality: TeachTrajectoryQuality) -> dict:
    """把质量评估转成可直接 JSON 序列化的字典（字段名即对外 payload 键，不可改动）。"""
    return {
        "risk_level": quality.risk_level,
        "replay_policy": quality.replay_policy,
        "allow_real_replay": quality.allow_real_replay,
        "requires_safe_retiming": quality.requires_safe_retiming,
        "max_jump_rad": quality.max_jump_rad,
        "max_velocity_rad_s": quality.max_velocity_rad_s,
        "max_acceleration_rad_s2": quality.max_acceleration_rad_s2,
        "worst_joint": quality.worst_joint,
        "worst_sample": quality.worst_sample,
        "anomalies": list(quality.anomalies),
        "events": [
            {
                "sample": event.sample,
                "joint_name": event.joint_name,
                "level": event.level,
                "message": event.message,
                "delta_rad": event.delta_rad,
                "velocity_rad_s": event.velocity_rad_s,
                "acceleration_rad_s2": event.acceleration_rad_s2,
                "jerk_rad_s3": event.jerk_rad_s3,
            }
            for event in quality.events
        ],
        "green_jump_rad": quality.green_jump_rad,
        "yellow_jump_rad": quality.yellow_jump_rad,
        "velocity_limit_rad_s": quality.velocity_limit_rad_s,
        "acceleration_limit_rad_s2": quality.acceleration_limit_rad_s2,
        "max_jerk_rad_s3": quality.max_jerk_rad_s3,
        "jerk_limit_rad_s3": quality.jerk_limit_rad_s3,
    }


def detect_teach_record_anomalies(
    samples: list[TeachSample],
    *,
    max_jump_rad: float = 0.75,
    max_velocity_rad_s: float = 2.0,
) -> tuple[str, ...]:
    """文件巡检用的轻量异常扫描：只看跳变、速度、关节名与时间戳单调性。

    与 :func:`analyze_teach_trajectory` 的区别是阈值更宽松（默认跳变 0.75 rad、速度
    2.0 rad/s）且只返回文本清单，用于"文件是否值得拿来用"的粗筛；质量分档以
    :func:`analyze_teach_trajectory` 的结果为准。
    """
    anomalies: list[str] = []
    if not samples:
        return ()
    expected_joints = samples[0].joint_names
    previous = samples[0]
    for index, sample in enumerate(samples[1:], start=1):
        if sample.joint_names != expected_joints:
            anomalies.append(f"joint_names mismatch at sample {index}")
        dt = float(sample.stamp) - float(previous.stamp)
        if dt <= 0.0:
            anomalies.append(f"timestamp not monotonic at sample {index}")
        for joint_name, current, last in zip(expected_joints, sample.positions, previous.positions):
            delta = abs(float(current) - float(last))
            if delta > float(max_jump_rad):
                anomalies.append(f"{joint_name} jump {delta:.4f} rad at sample {index}")
            if dt > 0.0:
                velocity = delta / dt
                if velocity > float(max_velocity_rad_s):
                    anomalies.append(f"{joint_name} velocity {velocity:.4f} rad/s at sample {index}")
        previous = sample
    return tuple(anomalies)


def inspect_teach_record(
    path: str | Path,
    *,
    current_positions: dict[str, float] | None = None,
    direct_threshold: float = 0.01,
    align_threshold: float = 0.25,
) -> TeachRecordInfo:
    """巡检一条示教记录文件：能否读取、轨迹质量如何、与当前位姿的起点误差多大。

    ``path``               记录文件（JSONL）；
    ``current_positions``  当前关节位置（rad，键为关节名）；为 None 时不做起点比较，
                           结果中 ``start_band`` 为 ``unknown``；
    ``direct_threshold`` / ``align_threshold`` 起点分档阈值（rad），透传给
                           :func:`classify_replay_start`。
    一切异常都转成结构化的"坏结果"而不是抛异常：
      文件不存在 → ``start_band="missing"``；解析失败 → ``"invalid"``（异常文本进 message
      与 anomalies）；没有样本 → ``"empty"``；缺少记录里的某些关节 → ``"unknown"`` 并在
      message 里列出缺失关节名。
    ``anomalies`` 是轻量巡检结果与质量评估异常的合并去重（``dict.fromkeys`` 保序去重）。
    """
    record_path = Path(path)
    if not record_path.exists():
        return TeachRecordInfo(
            path=str(record_path),
            exists=False,
            samples=0,
            duration_sec=0.0,
            joint_names=(),
            start_positions=(),
            end_positions=(),
            start_band="missing",
            max_error=None,
            worst_joint="",
            per_joint_error={},
            anomalies=(),
            message="record file does not exist",
        )
    try:
        samples = load_teach_samples(record_path)
    except (OSError, UnicodeError, JSONDecodeError, KeyError, TypeError, ValueError) as exc:
        # 记录文件损坏时只报告，不猜测内容，也不让异常穿到 ROS 回调里。
        return TeachRecordInfo(
            path=str(record_path),
            exists=True,
            samples=0,
            duration_sec=0.0,
            joint_names=(),
            start_positions=(),
            end_positions=(),
            start_band="invalid",
            max_error=None,
            worst_joint="",
            per_joint_error={},
            anomalies=(f"invalid jsonl: {exc}",),
            message=f"failed to read record: {exc}",
        )
    if not samples:
        return TeachRecordInfo(
            path=str(record_path),
            exists=True,
            samples=0,
            duration_sec=0.0,
            joint_names=(),
            start_positions=(),
            end_positions=(),
            start_band="empty",
            max_error=None,
            worst_joint="",
            per_joint_error={},
            anomalies=("empty record",),
            message="record contains no samples",
        )
    first = samples[0]
    last = samples[-1]
    duration_sec = max(0.0, float(last.stamp) - float(first.stamp))
    per_joint_error: dict[str, float] = {}
    max_error: float | None = None
    worst_joint = ""
    start_band = "unknown"
    message = "current joint state unavailable"
    if current_positions is not None:
        # 记录里的关节必须都能在当前状态里找到，否则起点比较不可信 → unknown。
        missing = [name for name in first.joint_names if name not in current_positions]
        if missing:
            message = f"current joint state missing: {', '.join(missing)}"
        else:
            current = tuple(float(current_positions[name]) for name in first.joint_names)
            decision = classify_replay_start(
                current_positions=current,
                start_positions=first.positions,
                direct_threshold=direct_threshold,
                align_threshold=align_threshold,
            )
            per_joint_error = {
                name: float(error)
                for name, error in zip(first.joint_names, decision.per_joint_error)
            }
            max_error = float(decision.max_error)
            if per_joint_error:
                worst_joint = max(per_joint_error, key=per_joint_error.get)
            start_band = str(decision.band.value)
            message = decision.message
    quality = analyze_teach_trajectory(samples)
    return TeachRecordInfo(
        path=str(record_path),
        exists=True,
        samples=len(samples),
        duration_sec=duration_sec,
        joint_names=first.joint_names,
        start_positions=first.positions,
        end_positions=last.positions,
        start_band=start_band,
        max_error=max_error,
        worst_joint=worst_joint,
        per_joint_error=per_joint_error,
        anomalies=tuple(dict.fromkeys((*detect_teach_record_anomalies(samples), *quality.anomalies))),
        message=message,
        quality=quality,
    )


def teach_record_info_to_dict(info: TeachRecordInfo) -> dict:
    """把巡检结果转成 JSON 可序列化的字典（键名为对外 payload 接口，不可改动）。

    ``start_positions`` / ``end_positions`` 在这里从与关节名对齐的元组转成"关节名 → 位置"的
    字典（rad）；``quality`` 只在有质量结果时出现（文件缺失/损坏时没有该键）。
    """
    payload = {
        "path": info.path,
        "exists": info.exists,
        "samples": info.samples,
        "duration_sec": info.duration_sec,
        "joint_names": list(info.joint_names),
        "start_positions": {
            name: float(position)
            for name, position in zip(info.joint_names, info.start_positions)
        },
        "end_positions": {
            name: float(position)
            for name, position in zip(info.joint_names, info.end_positions)
        },
        "start_band": info.start_band,
        "max_error": info.max_error,
        "worst_joint": info.worst_joint,
        "per_joint_error": dict(info.per_joint_error),
        "anomalies": list(info.anomalies),
        "message": info.message,
    }
    if info.quality is not None:
        payload["quality"] = teach_trajectory_quality_to_dict(info.quality)
    return payload



def list_teach_record_files(directory: str | Path) -> list[dict]:
    """列出目录下的原始示教记录文件（``*.jsonl``），按文件名排序返回。

    文件名以 ``.prepared.jsonl`` 结尾的是 :func:`write_prepared_teach_record` 生成的派生
    文件，会被跳过，避免把预处理结果当成可再次回放的原始示教记录。
    返回项包含路径、文件名、大小（字节）、修改时间（Unix 秒）、样本数、时长（秒）、
    起点分档与异常清单；每条记录都会完整巡检一次，因此目录很大时本函数开销与文件大小成正比。
    目录不存在时返回空列表。
    """
    base = Path(directory)
    if not base.exists():
        return []
    records: list[dict] = []
    for path in sorted(base.glob("*.jsonl")):
        if path.name.endswith(".prepared.jsonl"):
            continue
        stat = path.stat()
        info = teach_record_info_to_dict(inspect_teach_record(path))
        records.append(
            {
                "path": str(path),
                "name": path.name,
                "size_bytes": int(stat.st_size),
                "modified_time": float(stat.st_mtime),
                "samples": int(info["samples"]),
                "duration_sec": float(info["duration_sec"]),
                "start_band": str(info["start_band"]),
                "anomalies": list(info["anomalies"]),
            }
        )
    return records

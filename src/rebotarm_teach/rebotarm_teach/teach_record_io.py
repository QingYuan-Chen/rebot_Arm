"""示教 JSONL 读写与六轴记录适配。

职责：保存录制/派生数据，不做导数生成、质量评估或执行准入。
对外接口：encode/decode_teach_sample、load_teach_samples、write_prepared_teach_record。
安全约束：原文件及旧派生文件均不覆盖；v5 保留输入等级与导数来源，缺 a 保持为空。
文件结构：六轴按名提取 → 编解码 → 加载与派生路径 → 派生写出及列表。
"""

from __future__ import annotations

import json
from dataclasses import replace
from pathlib import Path

from .teach_models import PreparedTeachReplay, TeachSample


def arm_replay_samples(samples):
    """只在派生副本按名提取六轴，原文件及夹爪元数据保持可查看。"""
    names = tuple(f'joint{i}' for i in range(1,7))
    result = []
    for sample in samples:
        if set(sample.joint_names).issubset(names) and len(sample.joint_names) < 6:
            # 数值预处理保留旧的局部维度分析；正式 FJT 仍只接纳完整六轴。
            result.append(sample)
            continue
        if len(set(sample.joint_names)) != len(sample.joint_names) or not set(names).issubset(sample.joint_names):
            raise ValueError('record must contain all six arm joints')
        order = [sample.joint_names.index(n) for n in names]
        result.append(replace(sample,joint_names=names,
            positions=tuple(sample.positions[i] for i in order),
            velocities=tuple(sample.velocities[i] for i in order) if sample.velocities else (),
            accelerations=tuple(sample.accelerations[i] for i in order) if sample.accelerations else (),
            efforts=tuple(sample.efforts[i] for i in order) if sample.efforts else ()))
    return result


def is_quit_key(key: str | None, *, quit_key: str = "q") -> bool:
    """判断键盘输入是否等同于“退出示教采集”。

    忽略大小写与首尾空白；``key`` 为 None（例如非阻塞读取没读到按键）时返回 False。
    只做纯字符串比较，不读取键盘。
    """
    if key is None:
        return False
    return str(key).strip().lower() == str(quit_key).strip().lower()


def encode_teach_sample(sample: TeachSample) -> str:
    """把一个示教样本编码成单行 JSON（JSONL 的一行）。

    分隔符使用最紧凑形式 ``(",", ":")``：采样率高、记录文件大，这里省掉空格。
    元组被转成列表以便 JSON 序列化；键名是记录文件格式的一部分，不可改动。
    """
    return json.dumps(
        {
            "stamp": sample.stamp,
            "joint_names": list(sample.joint_names),
            "positions": list(sample.positions),
            "velocities": list(sample.velocities),
            "accelerations": list(sample.accelerations),
            "efforts": list(sample.efforts),
            "motor_status": sample.motor_status,
            "arm_state": sample.arm_state,
            "format_version": sample.format_version,
            "gripper_width_m": sample.gripper_width_m,
            "gripper_source_stamp": sample.gripper_source_stamp,
            "gripper_valid": sample.gripper_valid,
            "input_grade": sample.input_grade,
            "reference_grade": sample.reference_grade,
            "velocity_source": sample.velocity_source,
            "acceleration_source": sample.acceleration_source,
        },
        separators=(",", ":"),
    )


def decode_teach_sample(payload: str) -> TeachSample:
    """把一行 JSON 解析回示教样本（与 :func:`encode_teach_sample` 互为逆操作）。

    ``velocities`` / ``efforts`` / ``motor_status`` / ``arm_state`` 都是可选字段，缺失时
    分别退化为空元组、空元组、空字典和空串，以兼容早期版本写出的记录文件。
    数值字段强制转 float / int，字符串字段强制转 str；JSON 本身非法时异常会上抛，
    由调用方（:func:`inspect_teach_record`）决定如何降级处理。
    """
    data = json.loads(payload)
    return TeachSample(
        stamp=float(data["stamp"]),
        joint_names=tuple(str(v) for v in data["joint_names"]),
        positions=tuple(float(v) for v in data["positions"]),
        velocities=tuple(float(v) for v in data.get("velocities", [])),
        accelerations=tuple(float(v) for v in data.get("accelerations", [])),
        efforts=tuple(float(v) for v in data.get("efforts", [])),
        motor_status={str(k): int(v) for k, v in data.get("motor_status", {}).items()},
        arm_state=str(data.get("arm_state", "")),
        format_version=int(data.get('format_version',1)),
        gripper_width_m=data.get('gripper_width_m'),
        gripper_source_stamp=data.get('gripper_source_stamp'),
        gripper_valid=data.get('gripper_valid',False),
        input_grade=str(data.get('input_grade','')),
        reference_grade=str(data.get('reference_grade','')),
        velocity_source=str(data.get('velocity_source','')),
        acceleration_source=str(data.get('acceleration_source',
            'derived_legacy_v4' if data.get('format_version') == 4 else '')),
    )


def load_teach_samples(path: str | Path) -> list[TeachSample]:
    """按行读取 JSONL 示教记录，返回样本列表（文件内的顺序即时间顺序）。

    以 ``utf-8-sig`` 打开：兼容带 BOM 的文件（采集侧可能由不同工具写入）。
    空行被跳过；单行解析失败会直接抛出异常，不会静默丢弃那一帧——记录缺帧会改变轨迹
    时序，必须让上层察觉。
    """
    samples: list[TeachSample] = []
    with Path(path).open("r", encoding="utf-8-sig") as handle:
        for line in handle:
            line = line.strip()
            if line:
                samples.append(decode_teach_sample(line))
    return samples


def prepared_record_path(raw_path: str | Path) -> Path:
    """由原始记录路径推导“预处理记录”的落盘路径。

    规则：在扩展名前插入 ``.prepared``（``a.jsonl`` → ``a.prepared.jsonl``）；
    没有扩展名时直接追加 ``.prepared.jsonl``。这样预处理结果与原始记录同目录、同名前缀，
    列举原始记录时可以按后缀把它们排除掉。
    """
    path = Path(raw_path)
    if path.suffix:
        return path.with_name(f"{path.stem}.prepared.v5{path.suffix}")
    return path.with_name(f"{path.name}.prepared.v5.jsonl")


def write_prepared_teach_record(
    raw_path: str | Path,
    prepared: PreparedTeachReplay,
    *,
    output_path: str | Path | None = None,
) -> Path:
    """把示教段预处理结果写成 v5 JSONL，返回派生路径；不包含本次 MoveIt 对齐段。

    写出的内容分两种情况：
      1. 有重定点（``prepared.retimed_points`` 非空）时，以重定点为准：``stamp`` 写
         ``time_from_start``（相对示教段起点的时间，秒），写入实际生成的速度及加速度，
         力矩/电机状态清空，``arm_state`` 标记为 ``PREPARED_REPLAY``；
      2. 没有重定点时，保留预处理样本的相对时序，位置及已有导数直接保存，
         ``arm_state`` 同样标记为 ``PREPARED_REPLAY``。
    ``output_path`` 为空时落到 :func:`prepared_record_path` 推导出的默认路径；父目录会自动
    创建。注意：这里写的是**派生数据**，原始记录文件不会被改写；``arm_state`` 标记用于让
    下游一眼看出该文件不是手工示教采集的原始记录。
    """
    target = Path(output_path) if output_path is not None else prepared_record_path(raw_path)
    if target.resolve() == Path(raw_path).resolve():
        raise ValueError('derived teach record must not overwrite raw recording')
    target.parent.mkdir(parents=True, exist_ok=True)
    samples: list[TeachSample]
    if prepared.retimed_points:
        joint_names = prepared.samples[0].joint_names if prepared.samples else ()
        samples = [
            TeachSample(
                stamp=float(point.time_from_start),
                joint_names=joint_names,
                positions=point.positions,
                velocities=point.velocities,
                accelerations=point.accelerations,
                efforts=(),
                motor_status={},
                arm_state="PREPARED_REPLAY",
                format_version=5,
                input_grade=prepared.input_grade,reference_grade=prepared.reference_grade,
                velocity_source=prepared.velocity_source,acceleration_source=prepared.acceleration_source,
            )
            for point in prepared.retimed_points
        ]
    else:
        samples = [
            TeachSample(
                # 无重定时也保留已有相对时序，不把显示频率当成控制节点时间。
                stamp=float(sample.stamp)-float(prepared.samples[0].stamp),
                joint_names=sample.joint_names,
                positions=sample.positions,
                velocities=sample.velocities,
                accelerations=sample.accelerations,
                efforts=sample.efforts,
                motor_status=sample.motor_status,
                arm_state="PREPARED_REPLAY",
                format_version=5,
                input_grade=prepared.input_grade,reference_grade=prepared.reference_grade,
                velocity_source=prepared.velocity_source,acceleration_source=prepared.acceleration_source,
            )
            for sample in prepared.samples
        ]
    # 检查/执行可多次派生。独占创建避免覆盖已有证据；显式输出路径冲突直接报错。
    base, index = target, 0
    while True:
        try:
            with target.open("x", encoding="utf-8") as handle:
                for sample in samples:
                    handle.write(encode_teach_sample(sample) + "\n")
            break
        except FileExistsError:
            if output_path is not None:
                raise
            index += 1
            target = base.with_name(f'{base.stem}.{index}{base.suffix}')
    return target

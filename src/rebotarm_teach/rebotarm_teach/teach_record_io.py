"""Teach JSONL file storage and small input adapters.

This module owns record serialization and filesystem access; it does not
perform quality analysis or replay gating.
"""

from __future__ import annotations

import json
from pathlib import Path

from .teach_models import PreparedTeachReplay, TeachSample


def is_quit_key(key: str | None, *, quit_key: str = "q") -> bool:
    """判断键盘输入是否等同于"退出示教采集"。

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
            "efforts": list(sample.efforts),
            "motor_status": sample.motor_status,
            "arm_state": sample.arm_state,
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
        efforts=tuple(float(v) for v in data.get("efforts", [])),
        motor_status={str(k): int(v) for k, v in data.get("motor_status", {}).items()},
        arm_state=str(data.get("arm_state", "")),
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
    """由原始记录路径推导"预处理记录"的落盘路径。

    规则：在扩展名前插入 ``.prepared``（``a.jsonl`` → ``a.prepared.jsonl``）；
    没有扩展名时直接追加 ``.prepared.jsonl``。这样预处理结果与原始记录同目录、同名前缀，
    列举原始记录时可以按后缀把它们排除掉。
    """
    path = Path(raw_path)
    if path.suffix:
        return path.with_name(f"{path.stem}.prepared{path.suffix}")
    return path.with_name(f"{path.name}.prepared.jsonl")


def write_prepared_teach_record(
    raw_path: str | Path,
    prepared: PreparedTeachReplay,
    *,
    output_path: str | Path | None = None,
) -> Path:
    """把预处理结果写成 JSONL 文件（真实回放实际执行的那条轨迹），返回落盘路径。

    写出的内容分两种情况：
      1. 有重定点（``prepared.retimed_points`` 非空）时，以重定点为准：``stamp`` 写
         ``time_from_start``（相对回放起点的时间，秒），速度写重定时算出的速度，
         力矩/电机状态清空，``arm_state`` 标记为 ``PREPARED_REPLAY``；
      2. 没有重定点时，按重采样频率把预处理样本重新打时间戳（第 i 点 = i / 采样率），
         位置取预处理样本，``arm_state`` 同样标记为 ``PREPARED_REPLAY``。
    ``output_path`` 为空时落到 :func:`prepared_record_path` 推导出的默认路径；父目录会自动
    创建。注意：这里写的是**派生数据**，原始记录文件不会被改写；``arm_state`` 标记用于让
    下游一眼看出该文件不是手工示教采集的原始记录。
    """
    target = Path(output_path) if output_path is not None else prepared_record_path(raw_path)
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
                efforts=(),
                motor_status={},
                arm_state="PREPARED_REPLAY",
            )
            for point in prepared.retimed_points
        ]
    else:
        samples = [
            TeachSample(
                # 没有重定时信息时只能按目标采样率等间隔重建时间轴；采样率下限 1 Hz，
                # 避免错误的 0 值导致除零。
                stamp=float(index) / max(float(prepared.resample_rate_hz), 1.0),
                joint_names=sample.joint_names,
                positions=sample.positions,
                velocities=sample.velocities,
                efforts=sample.efforts,
                motor_status=sample.motor_status,
                arm_state="PREPARED_REPLAY",
            )
            for index, sample in enumerate(prepared.samples)
        ]
    with target.open("w", encoding="utf-8") as handle:
        for sample in samples:
            handle.write(encode_teach_sample(sample) + "\n")
    return target

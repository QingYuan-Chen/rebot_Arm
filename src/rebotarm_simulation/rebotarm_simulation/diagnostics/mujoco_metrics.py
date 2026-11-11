"""仿真轨迹跟踪指标记录器：逐关节采样偏差并落盘为 CSV 与摘要 JSON。

模块职责：在仿真执行一条关节轨迹的过程中，按固定抽样间隔记录“目标位置 / 实际位置 / 偏差 /
速度 / 执行器出力”，并在轨迹结束时生成统计摘要，供离线验收、回归对比与报告引用。

产物（写在 ``output_dir`` 下，目录不存在时自动创建）：

- ``trajectory_metrics.csv``：逐行一条“某采样时刻某关节”的记录，列名为英文键，便于脚本解析；
- ``summary.json``：整条轨迹的汇总（成功标志、停止原因、最大/均方根偏差、最大速度、最大出力等）。

本模块不依赖 ROS，也不接触真实硬件，只做纯数据记录与统计，因此可在无 ROS 环境下测试。
统计口径：偏差与速度、出力在求最大值/均方根时都取绝对值；平均值与均方根只在“实际写入的行”
上计算，因此 ``sample_stride > 1`` 时摘要反映的是抽样后的样本，而不是全部控制周期。
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import csv
import json
import math


@dataclass(frozen=True)
class MetricRow:
    """一条已固化的采样记录（对应 CSV 的一行）。

    字段含义与单位：
        elapsed: 相对轨迹起点的仿真时刻（秒）；
        joint: 关节名（与 ``TrajectoryMetricsRecorder.joint_names`` 中的名字一致）；
        target_position: 该时刻的目标关节角（rad）；
        actual_position: 该时刻的实际关节角（rad）；
        position_error: 目标减实际（rad），保留符号，便于区分超前/滞后；
        velocity: 实际关节角速度（rad/s）；
        actuator_force: 执行器出力（关节力矩，N·m；夹爪滑轨为力 N），统计时取绝对值。
    """

    elapsed: float
    joint: str
    target_position: float
    actual_position: float
    position_error: float
    velocity: float
    actuator_force: float


class TrajectoryMetricsRecorder:
    """轨迹跟踪指标的累积器：先多次 ``record()``，最后一次 ``finish()`` 落盘。

    使用约定：
        - ``record()`` 每个控制周期调用一次，入参为同一时刻、按 ``joint_names`` 顺序排列的向量；
        - ``sample_stride`` 控制抽样并存比例（1 表示每个周期都存，N 表示每 N 个周期存 1 个）；
        - ``finish()`` 只能作为收尾调用，负责创建目录、写 CSV 与摘要 JSON 并返回摘要字典。

    线程模型：本类不做加锁，必须在同一个执行线程中顺序调用（仿真步进线程）。
    """

    def __init__(self, output_dir: Path, *, joint_names: list[str], sample_stride: int = 1) -> None:
        """初始化记录器。

        参数：
            output_dir: 结果输出目录，可尚不存在（``finish()`` 时创建）；
            joint_names: 关节名列表，决定向量下标到关节名的映射与 CSV 中的 ``joint`` 值；
            sample_stride: 抽样间隔，至少为 1；小于 1 会被夹到 1，避免除零或负数取模。
        """
        self.output_dir = Path(output_dir)
        self.joint_names = [str(name) for name in joint_names]
        # 夹到最小 1：stride<=0 会让“每 N 个周期存一行”的取模逻辑失去意义。
        self.sample_stride = max(1, int(sample_stride))
        self.rows: list[MetricRow] = []
        self.sample_count = 0

    def record(
        self,
        *,
        elapsed: float,
        targets: list[float],
        actual: list[float],
        velocities: list[float],
        actuator_forces: list[float],
    ) -> None:
        """记录一个控制周期的数据。

        参数（三个向量都按 ``joint_names`` 顺序排列，长度应与关节数一致）：
            elapsed: 该周期的仿真时刻（秒）；
            targets: 目标关节角（rad）；
            actual: 实际关节角（rad）；
            velocities: 实际关节角速度（rad/s）；短于关节数时缺失项按 0 处理；
            actuator_forces: 执行器出力（N·m 或 N）；短于关节数时缺失项按 0 处理。

        说明：``sample_count`` 每次调用都自增（用于统计总控制周期数），但只有满足抽样间隔的
        周期才会真正写入 ``rows``；即使不写入，目标值与实际值也会被读取一次，从而对越界下标
        早失败而不是悄悄跳过一个关节。
        """
        self.sample_count += 1
        # 第 1 个周期（sample_count==1）一定落盘，保证轨迹起点被记录。
        should_store = (self.sample_count - 1) % self.sample_stride == 0
        for index, joint in enumerate(self.joint_names):
            target = float(targets[index])
            position = float(actual[index])
            velocity = float(velocities[index]) if index < len(velocities) else 0.0
            force = float(actuator_forces[index]) if index < len(actuator_forces) else 0.0
            if should_store:
                self.rows.append(
                    MetricRow(
                        elapsed=float(elapsed),
                        joint=joint,
                        target_position=target,
                        actual_position=position,
                        position_error=target - position,
                        velocity=velocity,
                        actuator_force=force,
                    )
                )

    def finish(
        self,
        *,
        success: bool,
        stop_reason: str,
        violated_joint: str | None = None,
        tolerance: float | None = None,
    ) -> dict[str, object]:
        """收尾：创建输出目录、写 CSV 与摘要 JSON，并返回摘要字典。

        参数：
            success: 该条轨迹是否达成验收（由上层的安全/收敛判定给出，本模块只如实记录）；
            stop_reason: 停止原因字符串（如正常完成、超时、安全停止等），原样写入摘要；
            violated_joint: 若因某个关节越限/超差而停止，记录该关节名；否则为 None；
            tolerance: 触发该次判定所用的容差（rad），便于事后复核判定尺度。

        输出目录已存在时直接复用；同名文件会被覆盖。
        """
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self._write_csv(self.output_dir / "trajectory_metrics.csv")
        summary = self._summary(
            success=success,
            stop_reason=stop_reason,
            violated_joint=violated_joint,
            tolerance=tolerance,
        )
        # ensure_ascii=False 保留中文等非 ASCII 文本的可读性；文件末尾补换行，便于 diff 与追加。
        (self.output_dir / "summary.json").write_text(
            json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        return summary

    def _write_csv(self, path: Path) -> None:
        """把全部采样行写成带表头的 CSV。

        ``newline=""`` 交给 csv 模块自行处理换行，避免在 Windows 上出现空行；
        列名固定为英文键，供下游脚本按列名解析。
        """
        with path.open("w", encoding="utf-8", newline="") as file:
            writer = csv.DictWriter(
                file,
                fieldnames=[
                    "elapsed",
                    "joint",
                    "target_position",
                    "actual_position",
                    "position_error",
                    "velocity",
                    "actuator_force",
                ],
            )
            writer.writeheader()
            for row in self.rows:
                writer.writerow(row.__dict__)

    def _summary(
        self,
        *,
        success: bool,
        stop_reason: str,
        violated_joint: str | None = None,
        tolerance: float | None = None,
    ) -> dict[str, object]:
        """汇总统计量并组装摘要字典（不落盘，纯计算）。

        统计口径：
            - ``errors``/``velocities``/``forces`` 都是取绝对值后的列表，因此最大值衡量的是
              “偏差/速度/出力的幅值上限”，与方向无关；
            - ``rms_error`` 为偏差均方根（rad），比最大值更能反映整体跟随质量；
            - ``written_sample_count`` 用 ``行数 // 关节数`` 反推实际落盘的周期数；
            - ``violated_joint`` 与 ``tolerance`` 只在调用方提供时才写入摘要键，
              避免在无越限场景下出现无意义的空字段。
        """
        errors = [abs(row.position_error) for row in self.rows]
        velocities = [abs(row.velocity) for row in self.rows]
        forces = [abs(row.actuator_force) for row in self.rows]
        rms_error = 0.0
        if errors:
            rms_error = math.sqrt(sum(error * error for error in errors) / float(len(errors)))
        summary: dict[str, object] = {
            "success": bool(success),
            "stop_reason": str(stop_reason),
            # sample_count 是控制周期总数；written_sample_count 是真正落盘的周期数（两者可不等）。
            "sample_count": self.sample_count,
            # max(1, ...) 防止关节名为空时除零；空记录时各极值回退为 0.0。
            "written_sample_count": len(self.rows) // max(1, len(self.joint_names)),
            "joint_count": len(self.joint_names),
            "max_abs_error": max(errors) if errors else 0.0,
            "rms_error": rms_error,
            "max_abs_velocity": max(velocities) if velocities else 0.0,
            "max_abs_actuator_force": max(forces) if forces else 0.0,
        }
        if violated_joint is not None:
            summary["violated_joint"] = str(violated_joint)
        if tolerance is not None:
            summary["tolerance"] = float(tolerance)
        return summary

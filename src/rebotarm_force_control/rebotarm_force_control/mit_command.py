"""MIT 协议增益上限,有界 MIT 参数裁剪与控制周期计时器.

职责与位置
    为重力补偿和关节阻抗提供统一的 MIT 输出限幅与控制循环计时判据,
    消除两侧重复实现或跨算法模块反向导入.

对外接口
    PROTOCOL_KP_MAX / PROTOCOL_KD_MAX: 达妙 DM 系列 MIT 帧增益硬件范围.
    bounded_mit_command: 合并 PD 与前馈力矩,受总力矩幅值与变化率约束后缩放 Kp/Kd.
    evaluate_mit_torque: 按当前 feedback 计算并限幅 MIT 内环总力矩.
    LoopTiming: 记录控制周期,单轮执行耗时与连续超时统计.

安全与并发约束
    MIT 的比例-微分项由电机内环计算,总力矩约束必须计入当前反馈与历史实际输出;
    约束仅基于当前采样时刻的估算,不能替代真机实测验证.本模块不调度线程或休眠.

文件结构(按定义顺序)
    协议上限常量 → bounded_mit_command → LoopTiming.
"""

from __future__ import annotations

import numpy as np

from .contracts import vector


# 达妙 DM 系列 MIT 帧的硬件增益编码范围,见手册 "Kp range: [0,500], Kd range: [0,5]".
PROTOCOL_KP_MAX = 500.0
PROTOCOL_KD_MAX = 5.0


def bounded_mit_command(
    target,
    q,
    qd,
    kp,
    kd,
    feedforward,
    limits,
    rates,
    previous,
    dt,
):
    """计算满足力矩幅值和变化率约束的 MIT 参数,不向电机发送指令.

    输入
        target/q/qd: 目标关节角,反馈关节角和角速度;目标角速度固定为零.
        kp/kd: 每轴 PD 增益;feedforward: 前馈力矩,可包含重力补偿或积分修正.
        limits/rates: 每轴力矩幅值上限,变化率上限.
        previous/dt: 上一周期的估算总力矩,本周期实测时间间隔.
        除 dt 为标量外,其余输入均为六轴向量.

    返回
        (缩放后的 kp, 缩放后的 kd, 前馈力矩 tau_ff, 本周期估算总力矩).
        前三项用于构造 MIT 指令,最后一项由调用方确认发送成功后保存为下一周期的 previous.

    力矩饱和时按必要程度降低 PD 增益,保证用于抵消 PD 的前馈力矩仍可在限幅内表示.
    约束基于当前 q/qd 计算;两次新鲜反馈之间的实际力矩仍需通过真机验证.
    """
    # 先统一形状,避免 NumPy 广播把错误的输入维数变成看似合法的计算结果.
    target, q, qd, kp, kd, feedforward, limits, rates, previous = [
        vector(v, 6, label)
        for v, label in zip(
            (target, q, qd, kp, kd, feedforward, limits, rates, previous),
            (
                "target",
                "position",
                "velocity",
                "kp",
                "kd",
                "feedforward",
                "torque limits",
                "torque rates",
                "previous torque",
            ),
        )
    ]
    if not np.isfinite(dt) or dt <= 0 or (limits <= 0).any() or (rates <= 0).any():
        raise ValueError("dt, torque limits and rates must be finite and positive")
    if (kp < 0).any() or (kd < 0).any() or (np.abs(previous) > limits + 1e-9).any():
        raise ValueError("invalid gains or previous torque outside limit")
    # 1. 估算电机侧 PD 力矩:目标速度为零,所以阻尼项为 -kd * qd.
    pd = vector(kp * (target - q) - kd * qd, 6, "PD torque")
    # 2. 先限制 PD + 前馈的总幅值,再限制相对上一周期的变化量(rates * dt).
    desired = np.clip(pd + feedforward, -limits, limits)
    total = np.clip(desired, previous - rates * dt, previous + rates * dt)
    # 3. 为前馈中的 PD 抵消项预留余量:scale 在 [0, 1] 内,同时缩放 kp 和 kd.
    #    使 |缩放后 PD| 不超过 limits - |total|,避免 tau_ff = total - PD 超限.
    scale = np.minimum(
        1.0,
        np.maximum(0.0, limits - np.abs(total)) / np.maximum(np.abs(pd), 1e-12),
    )
    # 当前反馈下,缩放后的 PD 与重新计算的前馈之和仍等于已限幅的 total.
    return kp * scale, kd * scale, total - pd * scale, total


def evaluate_mit_torque(
    target,
    q,
    qd,
    kp,
    kd,
    feedforward,
    limits,
) -> tuple[np.ndarray, np.ndarray]:
    """按 MIT 内环律计算并限幅当前时刻力矩: ``raw = kp*(target-q) - kd*qd + tau_ff``.

    返回 ``(raw_torque, bounded_torque)``,供高频内环在每个物理/控制子步实时求值.
    """
    target_v, q_v, qd_v, kp_v, kd_v, ff_v, limits_v = [
        vector(v, 6, label)
        for v, label in zip(
            (target, q, qd, kp, kd, feedforward, limits),
            (
                "target",
                "position",
                "velocity",
                "kp",
                "kd",
                "feedforward",
                "torque limits",
            ),
        )
    ]
    if (limits_v <= 0).any():
        raise ValueError("torque limits must be finite and positive")
    if (kp_v < 0).any() or (kd_v < 0).any():
        raise ValueError("mit gains must be nonnegative")
    try:
        with np.errstate(over="raise", invalid="raise"):
            raw = kp_v * (target_v - q_v) - kd_v * qd_v + ff_v
    except FloatingPointError as exc:
        raise ValueError("mit torque calculation overflowed") from exc
    return raw, np.clip(raw, -limits_v, limits_v)


class LoopTiming:
    """控制循环计时器:记录周期间隔,单轮执行耗时及连续超时次数.

    调用方在每轮开始/结束时传入单调时钟时间;本类不休眠,不调度线程,
    也不直接触发硬件保护,只通过 faulted 暴露连续超时判据.
    """

    def __init__(self, rate_hz: float) -> None:
        """校验控制频率,初始化周期预算和耗时统计;不调度或创建线程."""
        if not np.isfinite(rate_hz) or rate_hz <= 0:
            raise ValueError("control rate must be finite and positive")
        self.period = 1.0 / rate_hz
        self.last_start = None
        self.start = None
        self.consecutive_misses = 0
        self.cycles = self.misses = 0
        self.max_period = self.max_execution = 0.0
        self.last_dt = self.period

    def begin(self, now: float) -> float:
        """记录本轮开始时间,并返回与上一轮开始时间的间隔 dt(秒)."""
        # 第一轮没有历史时间,使用配置周期;后续使用实测间隔供积分和变化率限制使用.
        dt = self.period if self.last_start is None else now - self.last_start
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError("control clock must advance monotonically")
        self.last_start = self.start = now
        self.last_dt = dt
        self.max_period = max(self.max_period, dt)
        return dt

    def finish(self, now: float) -> None:
        """记录本轮耗时与超时统计;正常一轮会清零连续超时计数."""
        execution = now - self.start
        # 执行耗时超过配置周期,或两轮开始间隔超过周期的 1.5 倍,均计为超时.
        missed = execution > self.period or self.last_dt > 1.5 * self.period
        self.cycles += 1
        self.misses += int(missed)
        self.consecutive_misses = self.consecutive_misses + 1 if missed else 0
        self.max_execution = max(self.max_execution, execution)

    @property
    def faulted(self) -> bool:
        """连续三轮超时即报告异常;具体保持或保护由适配器与保护组件协调."""
        return self.consecutive_misses >= 3

    def snapshot(self) -> dict:
        """返回供诊断使用的统计快照,其中周期与执行耗时的单位均为秒."""
        return dict(
            configured_rate_hz=1.0 / self.period,
            cycles=self.cycles,
            missed_cycles=self.misses,
            consecutive_misses=self.consecutive_misses,
            max_period_sec=self.max_period,
            max_execution_sec=self.max_execution,
        )

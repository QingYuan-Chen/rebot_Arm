"""职责:供 MuJoCo 各前端共用的机械臂双频位置/速度控制数学库与夹爪标定数据契约.

对外接口:
    MotorSpec / ArmControlParameters / GripperControlParameters / MotorControlParameters
        仿真控制参数不可变数据类,由 control_config 模块从 YAML 与权威 URDF 加载组装.
    PosVelController
        机械臂双频位置/速度控制器:100 Hz 外环串级 PI(compute_step/compute)更新积分与
        限速平滑后的 outer_torque,500 Hz 内环速度阻尼(evaluate_inner_torque)按实时速度
        输出受 effort_limit 裁剪的电机力矩,并提供 seed_torque/sync_outer_after_handoff
        供复位与力控交接同步内部状态.

安全约束:
    本模块不访问仿真运行时或真实硬件,不含 ROS/线程/时钟依赖;
    velocity_damping_nm_s_per_rad 必须为六轴有限非负数,输出力矩始终受 URDF effort_limit 裁剪;
    位置控制高频阻尼完全自足于本控制器内部,不依赖 MJCF 机械阻尼补偿,也不作用于 MIT/重力补偿模式.

文件结构:
    参数数据类 → 双频位置/速度控制器 PosVelController → 向量形状与有限性校验辅助函数.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class MotorSpec:
    """单个电机型号的力矩与减速比规格(取自标定文件 motor_specs)."""

    name: str                 # 电机型号名,与标定文件 motor_specs 的键一致
    rated_torque_nm: float    # 额定力矩(N·m,减速器输出/关节侧)
    peak_torque_nm: float     # 峰值力矩(N·m,减速器输出/关节侧),短时可承受
    reduction_ratio: float    # 减速比(无量纲),电机侧→输出侧的力矩/转角倍率


@dataclass(frozen=True)
class ArmControlParameters:
    """机械臂 6 个关节的控制与标定参数.

    所有按关节展开的字段都是长度为 6 的元组,顺序固定为 joint1..joint6,
    与 ``joint_names`` 一一对应;缺少任何一项都会导致后续向量运算错位.
    """

    joint_names: tuple[str, ...]              # 关节名顺序(joint1..joint6),决定所有元组的索引含义
    motor_models: tuple[str, ...]             # 每个关节所用电机型号,键指向 MotorSpec
    pos_kp: tuple[float, ...]                 # 位置环比例增益(固件原始整定值,1/s 量纲)
    pos_ki: tuple[float, ...]                 # 位置环积分增益(固件原始整定值)
    vel_kp: tuple[float, ...]                 # 速度环比例增益(固件原始整定值)
    vel_ki: tuple[float, ...]                 # 速度环积分增益(固件原始整定值)
    velocity_limit: tuple[float, ...]         # 位置环输出的目标速度上限(rad/s,即固件 vlim)
    effort_limit: tuple[float, ...]           # 关节力矩上限(N·m),取自 URDF 的 limit effort
    rated_torque: tuple[float, ...]           # 各关节电机额定力矩(N·m),用于重力补偿前馈
    firmware_to_torque_scale: tuple[float, ...]  # 固件速度环输出→物理力矩的比例系数(随电机型号不同)
    velocity_damping_nm_s_per_rad: tuple[float, ...]  # 位置控制环内部 500 Hz 高频速度阻尼(N·m·s/rad)
    torque_rate_limit_nm_s: tuple[float, ...]  # 外环力矩变化率限幅(N·m/s),越大越接近阶跃,越易激发振荡
    torque_lowpass_alpha: tuple[float, ...]    # 外环力矩一阶低通系数,取值 (0, 1],越小越平滑但相位滞后越大
    gravity_compensation_scale: float          # 重力补偿缩放系数,1.0 为全量补偿,0.0 为关闭
    position_integral_limit: float             # 位置环积分限幅(rad·s),抗积分饱和用
    velocity_integral_limit: float             # 速度环积分限幅(rad),抗积分饱和用

    def __post_init__(self) -> None:
        if len(self.velocity_damping_nm_s_per_rad) != 6:
            raise ValueError(
                f"velocity_damping_nm_s_per_rad must have length 6, got {len(self.velocity_damping_nm_s_per_rad)}"
            )
        for index, value in enumerate(self.velocity_damping_nm_s_per_rad):
            numeric = float(value)
            if not math.isfinite(numeric) or numeric < 0.0:
                raise ValueError(
                    f"velocity_damping_nm_s_per_rad[{index}] must be finite and non-negative, got {value}"
                )


@dataclass(frozen=True)
class GripperControlParameters:
    """夹爪控制与标定参数(标定文件 gripper 段)."""

    motor_model: str                  # 夹爪电机型号,键指向 MotorSpec
    motor_radians_per_opening_m: float  # 开口位移→电机转角换算(rad/m),负号表示旋转方向与开口增大的方向相反
    transmission_efficiency: float      # 传动效率(0~1),用于由电机力矩估算指尖力
    motor_torque_limit_nm: float        # 电机侧力矩上限(N·m),先于指尖力上限生效
    finger_force_limit_n: float         # 指尖力上限(N),保守估计值,同时是 MuJoCo 手指执行器的 force range
    sim_force_kp_n_per_m: float         # 仿真指尖力伺服位置刚度(N/m),偏大则接触更硬
    sim_force_kd_n_s_per_m: float       # 仿真指尖力伺服速度阻尼(N·s/m),抑制接触振荡
    sim_force_deadband_m: float         # 力伺服位置死区(m),误差小于该值不施加接触力,避免抖动
    sim_velocity_deadband_m_s: float    # 力伺服速度死区(m/s),速度小于该值按静止处理
    displacement_min_m: float           # 夹爪开口位移下限(m),完全闭合
    displacement_max_m: float           # 夹爪开口位移上限(m),完全张开


@dataclass(frozen=True)
class MotorControlParameters:
    """一次加载得到的完整控制参数集合,供仿真运行时按名取用."""

    control_rate_hz: float                # 控制环频率(Hz),同时决定积分步长 dt = 1/rate
    motor_specs: dict[str, MotorSpec]     # 电机型号 → 规格
    arm: ArmControlParameters             # 机械臂参数
    gripper: GripperControlParameters     # 夹爪参数


class PosVelController:
    """双频串级位置/速度控制器(100 Hz 外环 PI + 500 Hz 内环速度阻尼),带条件式抗积分饱和.

    状态跨周期保持:位置环积分,速度环积分,上周期目标速度,100 Hz 外环基准力矩(outer_torque)
    与叠加 500 Hz 速度阻尼后的最新电机输出力矩(applied_torque).
    异类控制模式切换时清理记忆;position/hold 交接须保留同一控制器的支撑力矩.
    reset 会清除积分和输出历史,上层不能在普通目标更新或 position→hold 时无条件调用.
    本类不做任何限速/安全判断(限幅只依据标定上限),上层仍需自行保证目标合法.
    """

    def __init__(self, parameters: ArmControlParameters):
        self.parameters = parameters
        self.position_integral = np.zeros(6, dtype=float)
        self.velocity_integral = np.zeros(6, dtype=float)
        self.target_velocity = np.zeros(6, dtype=float)
        self.outer_torque = np.zeros(6, dtype=float)
        self.applied_torque = np.zeros(6, dtype=float)

    def reset(self) -> None:
        """清空全部积分项与记忆量(目标速度,外环基准力矩与已施加力矩归零)."""
        self.position_integral.fill(0.0)
        self.velocity_integral.fill(0.0)
        self.target_velocity.fill(0.0)
        self.outer_torque.fill(0.0)
        self.applied_torque.fill(0.0)

    def seed_torque(self, torque, velocity=None) -> None:
        """用已知力矩播种外环基准与已施加力矩;提供 velocity 时反算 outer_torque 以消除切入跳变."""
        seeded = _vector6(torque, "torque")
        self.applied_torque[:] = seeded
        if velocity is None:
            self.outer_torque[:] = seeded
            return
        vel = _vector6(velocity, "velocity")
        damping = np.asarray(self.parameters.velocity_damping_nm_s_per_rad, dtype=float)
        self.outer_torque[:] = seeded + damping * vel

    def sync_outer_after_handoff(self, applied_motor_torque, velocity) -> None:
        """力控退出 handoff 经外部限速裁剪后,按实际电机侧力矩与实时速度反写 outer_torque 基线."""
        self.seed_torque(applied_motor_torque, velocity=velocity)

    def evaluate_inner_torque(self, velocity) -> tuple[np.ndarray, np.ndarray]:
        """500 Hz 物理步内环:基于锁存的 outer_torque 与实时关节速度计算电机力矩.

        返回 ``(requested_torque, bounded_torque)`` 两份拷贝,分别表示裁剪前请求力矩与
        受 URDF ``effort_limit`` 限幅后的实际电机侧力矩,并刷新 ``self.applied_torque``.
        """
        vel = _vector6(velocity, "velocity")
        damping = np.asarray(self.parameters.velocity_damping_nm_s_per_rad, dtype=float)
        effort = np.asarray(self.parameters.effort_limit, dtype=float)
        requested = self.outer_torque - damping * vel
        bounded = np.clip(requested, -effort, effort)
        self.applied_torque[:] = bounded
        return requested.copy(), bounded.copy()

    def compute_step(
        self, target, position, velocity, dt: float, feedforward=None
    ) -> tuple[np.ndarray, np.ndarray]:
        """推进一个 100 Hz 外环控制周期并立即执行内环速度阻尼,返回 ``(requested, bounded)``.

        参数:
            target: 目标关节角(rad,长度 6).
            position: 当前关节角(rad,长度 6).
            velocity: 当前关节角速度(rad/s,长度 6).
            dt: 外环控制周期(s),必须为正,否则抛 ``ValueError``.
            feedforward: 前馈力矩(N·m,长度 6),典型为重力补偿;``None`` 视为零.
        """
        if dt <= 0.0:
            raise ValueError("dt must be positive")
        target = _vector6(target, "target")
        position = _vector6(position, "position")
        velocity = _vector6(velocity, "velocity")
        feedforward_torque = (
            np.zeros(6, dtype=float)
            if feedforward is None
            else _vector6(feedforward, "feedforward")
        )

        # 位置环:先用"积分 + 本次误差增量"得到候选积分,再整体限幅
        pos_kp = np.asarray(self.parameters.pos_kp)
        pos_ki = np.asarray(self.parameters.pos_ki)
        velocity_limit = np.asarray(self.parameters.velocity_limit)
        position_error = target - position
        position_candidate = np.clip(
            self.position_integral + position_error * dt,
            -self.parameters.position_integral_limit,
            self.parameters.position_integral_limit,
        )
        raw_target_velocity = pos_kp * position_error + pos_ki * position_candidate
        self.target_velocity = np.clip(raw_target_velocity, -velocity_limit, velocity_limit)
        # 条件式抗积分饱和:仅当未被限幅,或误差方向与未限幅输出相反(正在退出饱和)时才接受积分
        position_accept = (raw_target_velocity == self.target_velocity) | (
            np.sign(position_error) != np.sign(raw_target_velocity)
        )
        self.position_integral = np.where(
            position_accept, position_candidate, self.position_integral
        )

        # 速度环:输入为"目标速度 - 实际速度",输出经固件换算系数变为物理力矩
        velocity_error = self.target_velocity - velocity
        velocity_candidate = np.clip(
            self.velocity_integral + velocity_error * dt,
            -self.parameters.velocity_integral_limit,
            self.parameters.velocity_integral_limit,
        )
        raw_feedback_torque = (
            (
                np.asarray(self.parameters.vel_kp) * velocity_error
                + np.asarray(self.parameters.vel_ki) * velocity_candidate
            )
            * np.asarray(self.parameters.firmware_to_torque_scale)
        )
        raw_torque = raw_feedback_torque + feedforward_torque
        effort = np.asarray(self.parameters.effort_limit)
        clipped_torque = np.clip(raw_torque, -effort, effort)
        # 同样采用条件式抗积分饱和:饱和时若误差仍在把输出推向饱和方向,则冻结积分
        velocity_accept = (raw_torque == clipped_torque) | (
            np.sign(velocity_error) != np.sign(raw_feedback_torque)
        )
        self.velocity_integral = np.where(
            velocity_accept, velocity_candidate, self.velocity_integral
        )
        # 外环力矩变化率限幅(N·m/s × dt = 本周期允许的最大增量),抑制外环指令阶跃
        max_delta = np.asarray(self.parameters.torque_rate_limit_nm_s) * float(dt)
        rate_limited = self.outer_torque + np.clip(
            clipped_torque - self.outer_torque,
            -max_delta,
            max_delta,
        )
        # 外环一阶低通(指数平滑):alpha 越大跟随越快,越小越平滑
        alpha = np.asarray(self.parameters.torque_lowpass_alpha)
        smoothed = self.outer_torque + alpha * (rate_limited - self.outer_torque)
        self.outer_torque = np.clip(smoothed, -effort, effort)
        return self.evaluate_inner_torque(velocity)

    def compute(self, target, position, velocity, dt: float, feedforward=None) -> np.ndarray:
        """推进一个外环控制周期并叠加内环速度阻尼,返回限幅后的关节力矩指令(N·m,长度 6)."""
        _, bounded = self.compute_step(
            target, position, velocity, dt, feedforward=feedforward
        )
        return bounded


def _vector6(value, name: str) -> np.ndarray:
    """把输入转为 float64 的 (6,) 向量;形状不符或含非有限值时抛 ``ValueError``."""
    result = np.asarray(value, dtype=float)
    if result.shape != (6,):
        raise ValueError(f"{name} must have shape (6,), got {result.shape}")
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain only finite values")
    return result

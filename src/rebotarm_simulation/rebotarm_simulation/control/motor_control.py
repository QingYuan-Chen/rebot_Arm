"""供 MuJoCo 各前端共用的纯电机控制数学库。

本模块不访问仿真运行时，也不访问真实硬件，只负责把「固件速度环输出」换算为物理力矩：

- 机械臂：串级位置/速度 PI 控制器（位置环输出目标速度，速度环输出力矩），
  带条件式抗积分饱和、力矩变化率限幅与一阶低通平滑；
- 夹爪：MIT 模式下的位置-速度 PD 控制，并按传动比折算指尖力。

固件整定增益（pos_kp/pos_ki/vel_kp/vel_ki 等）以原始形式保留自标定文件；
而把固件输出换算成物理力矩的比例系数等属于「仿真标定」的参数，单独放在
配置目录下的 motor_control_calibration.yaml 中。两者分离是为了保证硬件侧
重新整定不会悄悄改变已经冻结的仿真标定结果。

对外主要接口：参数加载由 control_config 模块负责；
``PosVelController`` 与 ``GripperMitController`` 负责逐控制周期计算控制量。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np



@dataclass(frozen=True)
class MotorSpec:
    """单个电机型号的力矩与减速比规格（取自标定文件 motor_specs）。"""

    name: str                 # 电机型号名，与标定文件 motor_specs 的键一致
    rated_torque_nm: float    # 额定力矩（N·m，减速器输出/关节侧）
    peak_torque_nm: float     # 峰值力矩（N·m，减速器输出/关节侧），短时可承受
    reduction_ratio: float    # 减速比（无量纲），电机侧→输出侧的力矩/转角倍率


@dataclass(frozen=True)
class ArmControlParameters:
    """机械臂 6 个关节的控制与标定参数。

    所有按关节展开的字段都是长度为 6 的元组，顺序固定为 joint1..joint6，
    与 ``joint_names`` 一一对应；缺少任何一项都会导致后续向量运算错位。
    """

    joint_names: tuple[str, ...]              # 关节名顺序（joint1..joint6），决定所有元组的索引含义
    motor_models: tuple[str, ...]             # 每个关节所用电机型号，键指向 MotorSpec
    pos_kp: tuple[float, ...]                 # 位置环比例增益（固件原始整定值，1/s 量纲）
    pos_ki: tuple[float, ...]                 # 位置环积分增益（固件原始整定值）
    vel_kp: tuple[float, ...]                 # 速度环比例增益（固件原始整定值）
    vel_ki: tuple[float, ...]                 # 速度环积分增益（固件原始整定值）
    velocity_limit: tuple[float, ...]         # 位置环输出的目标速度上限（rad/s，即固件 vlim）
    effort_limit: tuple[float, ...]           # 关节力矩上限（N·m），取自 URDF 的 limit effort
    rated_torque: tuple[float, ...]           # 各关节电机额定力矩（N·m），用于重力补偿前馈
    firmware_to_torque_scale: tuple[float, ...]  # 固件速度环输出→物理力矩的比例系数（随电机型号不同）
    torque_rate_limit_nm_s: tuple[float, ...]  # 力矩变化率限幅（N·m/s），越大越接近阶跃、越易激发振荡
    torque_lowpass_alpha: tuple[float, ...]    # 力矩一阶低通系数，取值 (0, 1]，越小越平滑但相位滞后越大
    gravity_compensation_scale: float          # 重力补偿缩放系数，1.0 为全量补偿，0.0 为关闭
    position_integral_limit: float             # 位置环积分限幅（rad·s），抗积分饱和用
    velocity_integral_limit: float             # 速度环积分限幅（rad），抗积分饱和用


@dataclass(frozen=True)
class GripperControlParameters:
    """夹爪控制与标定参数（标定文件 gripper 段）。"""

    firmware_default_kp: float        # 固件侧夹爪默认位置增益（仅作参考，仿真不直接使用）
    firmware_default_kd: float        # 固件侧夹爪默认速度增益（仅作参考，仿真不直接使用）
    motor_model: str                  # 夹爪电机型号，键指向 MotorSpec
    move_kp: float                    # move 模式位置增益：正常开合，跟踪目标开口
    move_kd: float                    # move 模式速度增益：抑制超调
    closing_kp: float                 # closing 模式位置增益：标定为 0，纯阻尼式力控闭合
    closing_kd: float                 # closing 模式速度增益：控制闭合速度与接触力
    hold_kp: float                    # hold 模式位置增益：保持夹持位置
    hold_kd: float                    # hold 模式速度增益：保持夹持阻尼
    motor_radians_per_opening_m: float  # 开口位移→电机转角换算（rad/m），负号表示旋转方向与开口增大的方向相反
    transmission_efficiency: float      # 传动效率（0~1），用于由电机力矩估算指尖力
    motor_torque_limit_nm: float        # 电机侧力矩上限（N·m），先于指尖力上限生效
    finger_force_limit_n: float         # 指尖力上限（N），保守估计值，同时是 MuJoCo 手指执行器的 force range
    sim_force_kp_n_per_m: float         # 仿真指尖力伺服位置刚度（N/m），偏大则接触更硬
    sim_force_kd_n_s_per_m: float       # 仿真指尖力伺服速度阻尼（N·s/m），抑制接触振荡
    sim_force_deadband_m: float         # 力伺服位置死区（m），误差小于该值不施加接触力，避免抖动
    sim_velocity_deadband_m_s: float    # 力伺服速度死区（m/s），速度小于该值按静止处理
    displacement_min_m: float           # 夹爪开口位移下限（m），完全闭合
    displacement_max_m: float           # 夹爪开口位移上限（m），完全张开


@dataclass(frozen=True)
class MotorControlParameters:
    """一次加载得到的完整控制参数集合，供仿真运行时按名取用。"""

    control_rate_hz: float                # 控制环频率（Hz），同时决定积分步长 dt = 1/rate
    motor_specs: dict[str, MotorSpec]     # 电机型号 → 规格
    arm: ArmControlParameters             # 机械臂参数
    gripper: GripperControlParameters     # 夹爪参数


@dataclass(frozen=True)
class GripperCommand:
    """单个控制周期夹爪求解结果，供仿真侧换算成手指执行器控制量。"""

    mode: str                     # 实际使用的控制模式：move / closing / hold
    target_displacement_m: float  # 限幅后的目标开口位移（m）
    motor_target_rad: float       # 换算到电机侧的目标转角（rad）
    kp: float                     # 本周期实际使用的比例增益
    kd: float                     # 本周期实际使用的微分增益
    motor_torque_nm: float        # 限幅后的电机力矩指令（N·m）
    finger_force_n: float         # 由电机力矩与传动效率折算的指尖力（N）


class PosVelController:
    """串级位置/速度 PI 控制器（按标定频率运行，当前为 100 Hz），带条件式抗积分饱和。

    状态跨周期保持：位置环积分、速度环积分、上周期目标速度与实际施加力矩。
    仿真切换控制模式或恢复检查点后必须调用 :meth:`reset`，否则残留积分会产生突变力矩。
    本类不做任何限速/安全判断（限幅只依据标定上限），上层仍需自行保证目标合法。
    """

    def __init__(self, parameters: ArmControlParameters):
        self.parameters = parameters
        self.position_integral = np.zeros(6, dtype=float)
        self.velocity_integral = np.zeros(6, dtype=float)
        self.target_velocity = np.zeros(6, dtype=float)
        self.applied_torque = np.zeros(6, dtype=float)

    def reset(self) -> None:
        """清空全部积分项与记忆量（目标速度、已施加力矩归零）。"""
        self.position_integral.fill(0.0)
        self.velocity_integral.fill(0.0)
        self.target_velocity.fill(0.0)
        self.applied_torque.fill(0.0)

    def compute(self, target, position, velocity, dt: float, feedforward=None) -> np.ndarray:
        """推进一个控制周期，返回限幅后的关节力矩指令（N·m，长度 6）。

        参数：
            target: 目标关节角（rad，长度 6）。
            position: 当前关节角（rad，长度 6）。
            velocity: 当前关节角速度（rad/s，长度 6）。
            dt: 控制周期（s），必须为正，否则抛 ``ValueError``。
            feedforward: 前馈力矩（N·m，长度 6），典型为重力补偿；``None`` 视为零。

        输入会经 :func:`_vector6` 校验形状；输出是内部状态的一份拷贝，
        调用方修改返回值不会影响控制器状态。
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

        # 位置环：先用「积分 + 本次误差增量」得到候选积分，再整体限幅
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
        # 条件式抗积分饱和：仅当未被限幅，或误差方向与未限幅输出相反（正在退出饱和）时才接受积分
        position_accept = (raw_target_velocity == self.target_velocity) | (
            np.sign(position_error) != np.sign(raw_target_velocity)
        )
        self.position_integral = np.where(
            position_accept, position_candidate, self.position_integral
        )

        # 速度环：输入为「目标速度 - 实际速度」，输出经固件换算系数变为物理力矩
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
        # 同样采用条件式抗积分饱和：饱和时若误差仍在把输出推向饱和方向，则冻结积分
        velocity_accept = (raw_torque == clipped_torque) | (
            np.sign(velocity_error) != np.sign(raw_feedback_torque)
        )
        self.velocity_integral = np.where(
            velocity_accept, velocity_candidate, self.velocity_integral
        )
        # 力矩变化率限幅（N·m/s × dt = 本周期允许的最大增量），抑制力矩阶跃
        max_delta = np.asarray(self.parameters.torque_rate_limit_nm_s) * float(dt)
        rate_limited = self.applied_torque + np.clip(
            clipped_torque - self.applied_torque,
            -max_delta,
            max_delta,
        )
        # 一阶低通（指数平滑）：alpha 越大跟随越快，越小越平滑
        alpha = np.asarray(self.parameters.torque_lowpass_alpha)
        smoothed = self.applied_torque + alpha * (rate_limited - self.applied_torque)
        self.applied_torque = np.clip(smoothed, -effort, effort)
        return self.applied_torque.copy()


class GripperMitController:
    """夹爪 MIT 模式 PD 控制器：以开口位移为输入，输出电机力矩与折算指尖力。

    无内部积分状态，因此无需 reset；三种模式共用同一套换算，只是增益不同，
    ``closing`` 模式 kp 标定为 0，实际是纯阻尼闭合，靠接触被动产生夹持力。
    """

    def __init__(self, parameters: GripperControlParameters):
        self.parameters = parameters

    def compute(
        self,
        target: float,
        position: float,
        velocity: float,
        mode: str = "move",
        feedforward_torque: float = 0.0,
    ) -> GripperCommand:
        """求解一个控制周期的夹爪指令。

        参数（长度单位均为米）：
            target: 目标开口位移（会先被限幅到 displacement_min_m..displacement_max_m）。
            position: 当前开口位移（同样限幅）。
            velocity: 当前开口速度（m/s），正方向为开口增大。
            mode: ``move`` / ``closing`` / ``hold`` 之一，其他值抛 ``ValueError``。
            feedforward_torque: 电机侧前馈力矩（N·m），叠加在 PD 输出上。

        返回 :class:`GripperCommand`；其中电机力矩已按 motor_torque_limit_nm 限幅，
        指尖力由「电机力矩 × 换算比 × 传动效率」折算，是理想上界而非实测值。
        """
        p = self.parameters
        gains = {
            "move": (p.move_kp, p.move_kd),
            "closing": (p.closing_kp, p.closing_kd),
            "hold": (p.hold_kp, p.hold_kd),
        }
        if mode not in gains:
            raise ValueError(f"unsupported gripper mode: {mode}")
        kp, kd = gains[mode]
        target_m = float(np.clip(target, p.displacement_min_m, p.displacement_max_m))
        position_m = float(np.clip(position, p.displacement_min_m, p.displacement_max_m))
        # 开口位移 → 电机转角：负换算比表示电机正转对应开口减小
        ratio = float(p.motor_radians_per_opening_m)
        motor_target = target_m * ratio
        motor_position = position_m * ratio
        motor_velocity = float(velocity) * ratio
        raw_torque = kp * (motor_target - motor_position) - kd * motor_velocity
        raw_torque += float(feedforward_torque)
        motor_torque = float(
            np.clip(raw_torque, -p.motor_torque_limit_nm, p.motor_torque_limit_nm)
        )
        return GripperCommand(
            mode=mode,
            target_displacement_m=target_m,
            motor_target_rad=motor_target,
            kp=kp,
            kd=kd,
            motor_torque_nm=motor_torque,
            finger_force_n=motor_torque * ratio * p.transmission_efficiency,
        )


def _vector6(value, name: str) -> np.ndarray:
    """把输入转为 float64 的 (6,) 向量；形状不符时抛 ``ValueError``（错误信息带字段名便于定位）。"""
    result = np.asarray(value, dtype=float)
    if result.shape != (6,):
        raise ValueError(f"{name} must have shape (6,), got {result.shape}")
    return result

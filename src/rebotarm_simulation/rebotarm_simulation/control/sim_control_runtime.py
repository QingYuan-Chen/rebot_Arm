"""Own control targets, controller memory and cadence; borrow physics data per call.

Never loads, steps, resets or closes a MuJoCo instance.
"""
import math
import numpy as np
from typing import Mapping, Sequence
from rebotarm_simulation.core.model_contract import ARM_JOINT_NAMES, JOINT_NAMES
from rebotarm_simulation.core.validation import finite_vector as _finite_vector
from rebotarm_simulation.control.motor_control import PosVelController, GripperMitController
from rebotarm_simulation.control.sim_gripper import gripper_joint_positions_for_width

CONTROL_MODES = ("gravity_comp", "hold", "pos_vel")

class SimControlRuntime:
    def __init__(self, parameters, timestep, joint_ids, actuator_ids):
        self._motor_parameters = parameters
        self._arm_controller = PosVelController(parameters.arm)
        self._gripper_controller = GripperMitController(parameters.gripper)
        self._control_steps_per_update = max(1, int(round((1.0 / parameters.control_rate_hz) / timestep)))
        self._control_phase = 0
        self._position_targets = np.zeros(len(JOINT_NAMES), dtype=float)
        self._control_mode = "pos_vel"
        self._joint_ids = joint_ids
        self._actuator_ids = actuator_ids

    @property
    def control_targets(self):
        return tuple(float(v) for v in self._position_targets)

    @property
    def control_mode(self):
        return self._control_mode

    def before_step(self, model, data):
        if self._control_phase == 0:
            self.apply_motor_control(model, data)

    def after_step(self):
        self._control_phase = (self._control_phase + 1) % self._control_steps_per_update

    def reset_memory(self):
        self._arm_controller.reset()
        self._control_phase = 0

    def set_reset_target(self, index, value):
        self._position_targets[index] = value

    def capture(self):
        return dict(control_targets=self.control_targets,
            position_integral=tuple(self._arm_controller.position_integral),
            velocity_integral=tuple(self._arm_controller.velocity_integral),
            applied_torque=tuple(self._arm_controller.applied_torque),
            control_phase=self._control_phase, control_mode=self._control_mode)

    def restore(self, state):
        self._position_targets[:] = state.control_targets
        self._arm_controller.position_integral[:] = state.position_integral
        self._arm_controller.velocity_integral[:] = state.velocity_integral
        self._arm_controller.applied_torque[:] = state.applied_torque
        self._control_phase = int(state.control_phase) % self._control_steps_per_update
        self._control_mode = state.control_mode

    def set_control_mode(self, model, data, mode: str) -> str:
        """切换控制模式并立即按新模式重算一次控制量，返回生效后的模式。

        模式必须是 CONTROL_MODES 之一（"gravity_comp" / "hold" / "pos_vel"）。
        切到 "hold" 会先把手臂目标同步为当前关节角（原地保持，不会突然蹿向旧
        目标）；切到 "gravity_comp" 或 "hold" 会复位控制器积分与已施加力矩，
        避免把上一个模式的积分/滤波状态带进新模式。
        """
        mode = str(mode)
        if mode not in CONTROL_MODES:
            raise ValueError(f"control mode must be one of {CONTROL_MODES}")
        if mode == "hold":
            self._sync_arm_targets_to_current_position(model, data)
        if mode in ("gravity_comp", "hold"):
            self._arm_controller.reset()
        self._control_mode = mode
        self.apply_motor_control(model, data)
        return self._control_mode


    def set_joint_position_targets(
        self, model, targets: Mapping[str, float] | Sequence[float]
    ) -> tuple[float, ...]:
        """设置手臂关节位置目标，返回实际生效（已按限位裁剪）的 6 个角度。

        支持两种入参：{关节名: 角度} 的映射（只更新列出的关节，其余保持原目标；
        关节名不合法直接报错），或 6 个角度的序列（整体替换）。角度单位 rad，
        必须是有限值。写入后控制模式强制切回 "pos_vel"，确保目标被闭环跟踪。
        """
        current = list(self.control_targets[: len(ARM_JOINT_NAMES)])
        if isinstance(targets, Mapping):
            unknown = set(targets) - set(ARM_JOINT_NAMES)
            if unknown:
                raise ValueError(f"Unknown arm joint names: {sorted(unknown)}")
            updates = {name: float(value) for name, value in targets.items()}
            if not all(math.isfinite(value) for value in updates.values()):
                raise ValueError("Joint targets must be finite")
            for name, value in updates.items():
                current[ARM_JOINT_NAMES.index(name)] = value
        else:
            current = list(_finite_vector(targets, len(ARM_JOINT_NAMES), "joint targets"))

        reached = []
        for index, joint_id in enumerate(self._joint_ids[:6]):
            # 目标值裁剪到模型关节限位，避免把不可达目标交给力矩控制器。
            lower, upper = (float(value) for value in model.jnt_range[joint_id])
            value = min(max(current[index], lower), upper)
            self._position_targets[index] = value
            reached.append(value)
        self._control_mode = "pos_vel"
        return tuple(reached)


    def set_gripper_width(self, width: float) -> float:
        """设置夹爪张开宽度目标，返回实际可达宽度（m）。

        入参为两指间净开口（m）；内部按 [0, 0.09] m 裁剪后拆成左右手指关节位置
        （±宽度/2）写入目标向量。返回的是裁剪后的宽度，便于调用方判断是否被限幅。
        """
        value = float(width)
        if not math.isfinite(value):
            raise ValueError("Gripper width must be finite")
        left, right, reached = gripper_joint_positions_for_width(value)
        self._position_targets[-2] = left
        self._position_targets[-1] = right
        return reached


    def apply_motor_control(self, model, data) -> None:
        """按当前控制模式重算 8 个执行器的控制量并写入 MuJoCo 的 ctrl。

        每个固件控制周期调用一次（由 step() 的相位计数触发）。手臂力矩写入 6 个
        扭矩执行器，夹爪以等大反向的力写入两个手指力执行器。
        """
        qpos_addresses = [int(model.jnt_qposadr[joint_id]) for joint_id in self._joint_ids[:6]]
        qvel_addresses = [int(model.jnt_dofadr[joint_id]) for joint_id in self._joint_ids[:6]]
        position = np.asarray([data.qpos[address] for address in qpos_addresses], dtype=float)
        velocity = np.asarray([data.qvel[address] for address in qvel_addresses], dtype=float)
        # 重力/科氏偏置力矩（MuJoCo 的 qfrc_bias），按标定比例缩放后作为前馈。
        gravity = self._gravity_compensation_torque(model, data, qvel_addresses)
        if self._control_mode == "gravity_comp":
            # 只输出重力补偿：机械臂近似自由漂浮，用于拖动演示。
            arm_torque = gravity
            self._arm_controller.applied_torque[:] = arm_torque
        elif self._control_mode == "hold":
            # hold 模式的固定 PD 增益：前 3 个承重关节较强，腕部 3 轴较弱。
            # 仅用于仿真内稳定保持，不代表真机调参。
            kp = np.asarray((12.0, 12.0, 12.0, 8.0, 8.0, 4.0), dtype=float)
            kd = np.asarray((1.2, 1.2, 1.2, 0.8, 0.8, 0.4), dtype=float)
            # 合力矩必须限制在 URDF 力矩上限内，与真机一致。
            effort = np.asarray(self._motor_parameters.arm.effort_limit, dtype=float)
            arm_torque = np.clip(
                gravity + kp * (self._position_targets[:6] - position) - kd * velocity,
                -effort,
                effort,
            )
            self._arm_controller.applied_torque[:] = arm_torque
        else:
            # 默认 "pos_vel"：复现真机固件的串级位置/速度 PI，dt 取固件控制周期。
            arm_torque = self._arm_controller.compute(
                target=self._position_targets[:6],
                position=position,
                velocity=velocity,
                dt=1.0 / self._motor_parameters.control_rate_hz,
                feedforward=gravity,
            )
        for actuator_id, torque in zip(self._actuator_ids[:6], arm_torque):
            data.ctrl[actuator_id] = torque

        # 夹爪按“开合量”控制：左右手指位置之差即净开口，速度同理取差值，
        # 这样单个标量目标就能驱动对称的两个滑动关节。
        left_qpos = float(data.qpos[int(model.jnt_qposadr[self._joint_ids[-2]])])
        right_qpos = float(data.qpos[int(model.jnt_qposadr[self._joint_ids[-1]])])
        left_qvel = float(data.qvel[int(model.jnt_dofadr[self._joint_ids[-2]])])
        right_qvel = float(data.qvel[int(model.jnt_dofadr[self._joint_ids[-1]])])
        command = self._gripper_controller.compute(
            target=float(self._position_targets[-2] - self._position_targets[-1]),
            position=left_qpos - right_qpos,
            velocity=left_qvel - right_qvel,
            mode="move",
        )
        # 真机 MIT 指令算出的手指力只用于确定目标/反馈量，仿真里改用线性空间 PD
        # 直接生成手指推力，以保持滑动关节数值稳定。
        finger_force = self._stable_gripper_force(
            target_width=command.target_displacement_m,
            current_width=left_qpos - right_qpos,
            current_velocity=left_qvel - right_qvel,
        )
        # 左右手指等大反向：闭合力在夹爪内部自平衡，不会给末端带来净推力。
        left_force = self._clamp_actuator_control(model, self._actuator_ids[-2], finger_force)
        right_force = self._clamp_actuator_control(model, self._actuator_ids[-1], -finger_force)
        data.ctrl[self._actuator_ids[-2]] = left_force
        data.ctrl[self._actuator_ids[-1]] = right_force


    def _stable_gripper_force(
        self,
        *,
        target_width: float,
        current_width: float,
        current_velocity: float,
    ) -> float:
        """用带死区的线性 PD 计算仿真手指推力（N），供两个手指等大反向下发。

        死区的作用：宽度误差小于 sim_force_deadband_m、速度小于
        sim_velocity_deadband_m_s 时输出 0，避免在目标附近高频抖动耗散。
        """
        gripper = self._motor_parameters.gripper
        error = float(target_width) - float(current_width)
        velocity = float(current_velocity)
        if (
            abs(error) < gripper.sim_force_deadband_m
            and abs(velocity) < gripper.sim_velocity_deadband_m_s
        ):
            return 0.0
        # MuJoCo 的力执行器直接作用在滑动手指关节上，单位是 N。
        # 真机 MIT 指令仍定义电机侧力矩上限，而这里的线性空间 PD
        # 负责让仿真中的移动副保持稳定。
        return (
            gripper.sim_force_kp_n_per_m * error
            - gripper.sim_force_kd_n_s_per_m * velocity
        )


    def _clamp_actuator_control(self, model, actuator_id: int, value: float) -> float:
        """把控制量裁剪到该执行器在 MJCF 中声明的 ctrlrange（若声明为受限）。

        返回裁剪后的值；执行器未设置 ctrllimited 时原样返回，交由 MuJoCo 自身处理。
        """
        if int(model.actuator_ctrllimited[actuator_id]):
            lower, upper = (float(v) for v in model.actuator_ctrlrange[actuator_id])
            return min(max(float(value), lower), upper)
        return float(value)


    def seed_arm_torque_from_gravity(self, model, data) -> None:
        """把控制器“上一拍力矩”预置为当前重力补偿值，避免复位瞬间的力矩阶跃。"""
        qvel_addresses = [int(model.jnt_dofadr[joint_id]) for joint_id in self._joint_ids[:6]]
        self._arm_controller.applied_torque[:] = self._gravity_compensation_torque(model, data, qvel_addresses)


    def _sync_arm_targets_to_current_position(self, model, data) -> None:
        """把手臂目标角同步为当前实测角，用于模式切换时的“原地保持”。"""
        for index, joint_id in enumerate(self._joint_ids[:6]):
            self._position_targets[index] = float(data.qpos[int(model.jnt_qposadr[joint_id])])


    def _gravity_compensation_torque(self, model, data, qvel_addresses: Sequence[int]) -> np.ndarray:
        """取 6 个手臂关节的重力/科氏偏置力矩，并按标定系数缩放。

        数据源是 MuJoCo 的 qfrc_bias（单位 N·m，符号与关节方向一致）。
        gravity_compensation_scale 为 0 时直接返回零向量，便于做纯前馈关闭的对照实验。
        """
        scale = float(self._motor_parameters.arm.gravity_compensation_scale)
        if scale == 0.0:
            return np.zeros(len(ARM_JOINT_NAMES), dtype=float)
        return np.asarray([data.qfrc_bias[address] for address in qvel_addresses], dtype=float) * scale

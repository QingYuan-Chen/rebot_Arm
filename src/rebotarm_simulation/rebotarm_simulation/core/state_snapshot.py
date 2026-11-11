"""校验并复制物理与控制器检查点;调用方负责仿真锁和原生资源生命周期.

save_state 导出不可变数值,restore_state 在修改活动状态前检查模型身份与全部字段.
模型指纹,保存,恢复依次定义于本模块,所有操作借用调用方传入的 model/data.
"""
from dataclasses import replace
import hashlib
import numpy as np
from rebotarm_simulation.core.model_contract import ARM_JOINT_NAMES, JOINT_NAMES
from rebotarm_simulation.core.mujoco_types import SavedSimulationState
from rebotarm_simulation.control.sim_control_runtime import CONTROL_MODES

class StateSnapshot:
    def __init__(self, mj, model):
        self._state_spec = (
            int(mj.mjtState.mjSTATE_INTEGRATION)
            | int(mj.mjtState.mjSTATE_USER)
            | int(mj.mjtState.mjSTATE_CTRL)
        )
        # 模型维度指纹的一部分:nq/nv/na/nu,body/joint/geom 数量与状态向量长度.
        # 快照恢复时逐项比对,防止把 A 模型的状态灌进 B 模型.
        self._model_dimensions = tuple(int(value) for value in (
            model.nq, model.nv, model.na, model.nu,
            model.nbody, model.njnt, model.ngeom,
            mj.mj_stateSize(model, self._state_spec),
        ))
        self._model_fingerprint = self._fingerprint_model(model)

    def _fingerprint_model(self, model) -> str:
        """对模型结构做 SHA-256 指纹,用于校验状态快照与模型是否匹配.

        参与哈希的量:维度元组,名称表,关节类型/地址/限位,执行器传动目标,
        geom 所属 body,以及物理步长.只要 MJCF 结构或积分步长改变,指纹即变化,
        restore_state 会因此拒绝恢复旧快照.
        """
        digest = hashlib.sha256()
        digest.update(repr(self._model_dimensions).encode("ascii"))
        for values in (
            model.names,
            model.jnt_type,
            model.jnt_qposadr,
            model.jnt_dofadr,
            model.jnt_range,
            model.actuator_trnid,
            model.geom_bodyid,
        ):
            digest.update(np.asarray(values).tobytes())
        digest.update(np.asarray([model.opt.timestep], dtype=np.float64).tobytes())
        return digest.hexdigest()


    def save_state(self, mj, model, data, control) -> SavedSimulationState:
        """保存完整检查点:物理状态 + 控制器内部状态 + 目标与控制模式.

        物理状态按 _state_spec 导出(积分/用户/控制量),并附带模型实例 id,结构
        指纹与维度,供 restore_state 做兼容性校验.控制器侧记录目标角,位置/速度
        积分,已施加力矩,控制相位与模式,保证恢复后控制行为连续.返回值为不可变
        快照,可安全长期持有.
        """
        state = np.empty(self._model_dimensions[-1], dtype=float)
        mj.mj_getState(model, data, state, self._state_spec)
        return SavedSimulationState(
            model_identity=id(model),
            model_fingerprint=self._model_fingerprint,
            model_dimensions=self._model_dimensions,
            state_spec=self._state_spec,
            state=tuple(float(value) for value in state),
            **control.capture(),
        )


    def restore_state(self, mj, model, data, control, state: SavedSimulationState) -> None:
        """从 save_state() 的快照恢复物理与控制器状态.

        只接受同一模型实例产生的快照:模型实例 id,结构指纹,维度,状态向量长度,
        控制器各向量长度与控制模式必须全部匹配;不匹配时抛 ValueError,防止把
        不兼容状态灌进当前模型(例如 MJCF 已重新生成).控制相位按控制周期取模,
        保持控制节拍与当前模型步长一致.
        """
        if not isinstance(state, SavedSimulationState):
            raise TypeError("state must be returned by save_state()")
        # 重建冻结值对象以重新校验所有数值,校验完成后才修改活动仿真.
        state = replace(state)
        compatible = (
            state.model_identity == id(model)
            and state.model_fingerprint == self._model_fingerprint
            and state.model_dimensions == self._model_dimensions
            and state.state_spec == self._state_spec
            and len(state.state) == self._model_dimensions[-1]
            and len(state.control_targets) == len(JOINT_NAMES)
            and len(state.position_integral) == len(ARM_JOINT_NAMES)
            and len(state.velocity_integral) == len(ARM_JOINT_NAMES)
            and len(state.applied_torque) == len(ARM_JOINT_NAMES)
            and len(state.raw_torque_command) == len(ARM_JOINT_NAMES)
            and len(state.raw_torque_requested) == len(ARM_JOINT_NAMES)
            and state.control_mode in CONTROL_MODES
        )
        if not compatible:
            raise ValueError("saved state must belong to the same MuJoCo model instance")
        if state.impedance_command is not None:
            control.validate_impedance_target(model, state.impedance_command)
        if state.impedance_command is not None or state.impedance_handoff:
            # 先解码到临时数据以验证历史时间和实际输出,坏快照保持实时物理状态原样.
            candidate = mj.MjData(model)
            mj.mj_setState(model, candidate, np.asarray(state.state, dtype=float), self._state_spec)
            if state.last_control_time > float(candidate.time):
                raise ValueError("saved control time exceeds simulation time")
            torque = np.asarray(candidate.ctrl)[list(control._actuator_ids[:6])]
            if np.any(abs(torque) > control.impedance_effort_limits(model)):
                raise ValueError("saved impedance torque exceeds effort limits")
            if state.impedance_deadline is not None and state.impedance_deadline <= float(candidate.time):
                raise ValueError("saved impedance command has expired")
            if state.impedance_command is not None:
                control.compute_impedance_output(model, candidate, state.impedance_command,
                    torque, 1.0 / control._motor_parameters.control_rate_hz)
        mj.mj_setState(
            model, data, np.asarray(state.state, dtype=float), self._state_spec
        )
        control.restore(state, model, data)
        # mj_forward 更新派生运动学时可能舍入求解器热启动值(MuJoCo 3.3 尤其明显).
        # 恢复原积分种子,使下一物理步与连续运行时采用相同的初始求解状态.
        warmstart = data.qacc_warmstart.copy()
        mj.mj_forward(model, data)
        data.qacc_warmstart[:] = warmstart

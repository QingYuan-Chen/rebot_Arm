"""Validate and transfer physical and controller state without owning the runtime."""
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
        # 模型维度指纹的一部分：nq/nv/na/nu、body/joint/geom 数量与状态向量长度。
        # 快照恢复时逐项比对，防止把 A 模型的状态灌进 B 模型。
        self._model_dimensions = tuple(int(value) for value in (
            model.nq, model.nv, model.na, model.nu,
            model.nbody, model.njnt, model.ngeom,
            mj.mj_stateSize(model, self._state_spec),
        ))
        self._model_fingerprint = self._fingerprint_model(model)

    def _fingerprint_model(self, model) -> str:
        """对模型结构做 SHA-256 指纹，用于校验状态快照与模型是否匹配。

        参与哈希的量：维度元组、名称表、关节类型/地址/限位、执行器传动目标、
        geom 所属 body，以及物理步长。只要 MJCF 结构或积分步长改变，指纹即变化，
        restore_state 会因此拒绝恢复旧快照。
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
        """保存完整检查点：物理状态 + 控制器内部状态 + 目标与控制模式。

        物理状态按 _state_spec 导出（积分/用户/控制量），并附带模型实例 id、结构
        指纹与维度，供 restore_state 做兼容性校验。控制器侧记录目标角、位置/速度
        积分、已施加力矩、控制相位与模式，保证恢复后控制行为连续。返回值为不可变
        快照，可安全长期持有。
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
        """从 save_state() 的快照恢复物理与控制器状态。

        只接受同一模型实例产生的快照：模型实例 id、结构指纹、维度、状态向量长度、
        控制器各向量长度与控制模式必须全部匹配；不匹配时抛 ValueError，防止把
        不兼容状态灌进当前模型（例如 MJCF 已重新生成）。控制相位按控制周期取模，
        保持控制节拍与当前模型步长一致。
        """
        if not isinstance(state, SavedSimulationState):
            raise TypeError("state must be returned by save_state()")
        # Normalize and validate every numeric field before touching live state.
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
            and state.control_mode in CONTROL_MODES
        )
        if not compatible:
            raise ValueError("saved state must belong to the same MuJoCo model instance")
        mj.mj_setState(
            model, data, np.asarray(state.state, dtype=float), self._state_spec
        )
        control.restore(state)
        mj.mj_forward(model, data)

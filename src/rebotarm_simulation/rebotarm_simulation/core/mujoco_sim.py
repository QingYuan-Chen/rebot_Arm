from __future__ import annotations

import hashlib
import importlib
import math
import os
from pathlib import Path
import sys
from types import MappingProxyType
from typing import Mapping, Sequence

import numpy as np

from rebotarm_simulation.control.control_config import load_motor_control_parameters
from rebotarm_simulation.core.mujoco_types import (
    ContactInfo,
    ControlStatus,
    RandomizedScene,
    SavedSimulationState,
    SimulationState,
)
from rebotarm_simulation.sim2real.randomization import RandomizationSample
from rebotarm_simulation.control.sim_gripper import gripper_joint_positions_for_width
from rebotarm_simulation.core.model_contract import actuator_name_for_joint
from rebotarm_simulation.core.resource_paths import model_resource
from rebotarm_simulation.control.sim_control_runtime import SimControlRuntime
from rebotarm_simulation.core.state_snapshot import StateSnapshot
from rebotarm_simulation.core.observations import read_state, read_contacts


ARM_JOINT_NAMES = tuple(f"joint{index}" for index in range(1, 7))
FINGER_JOINT_NAMES = ("left_finger_joint", "right_finger_joint")
JOINT_NAMES = ARM_JOINT_NAMES + FINGER_JOINT_NAMES
CONTROL_MODES = ("position", "hold", "gravity_comp", "raw_torque")
_CONTROL_MODE_ALIASES = {"pos_vel": "position"}


def _finite_vector(values: Sequence[float], length: int, label: str) -> tuple[float, ...]:
    if isinstance(values, (str, bytes)):
        raise TypeError(f"{label} must be a numeric sequence")
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError) as exc:
        raise TypeError(f"{label} must be a numeric sequence") from exc
    if len(result) != length:
        raise ValueError(f"{label} must contain exactly {length} values")
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{label} values must be finite")
    return result


def _ordered_bounds(values: Sequence[float], label: str) -> tuple[float, float]:
    lower, upper = float(values[0]), float(values[1])
    if lower > upper:
        raise ValueError(f"{label} lower bound must be <= upper bound")
    return lower, upper


def _bounds2(values: Sequence[Sequence[float]], label: str) -> tuple[tuple[float, float], tuple[float, float]]:
    bounds = tuple(_finite_vector(item, 2, label) for item in values)
    if len(bounds) != 2:
        raise ValueError(f"{label} must contain exactly 2 bounds")
    return (_ordered_bounds(bounds[0], label), _ordered_bounds(bounds[1], label))


def _bounds3(
    values: Sequence[Sequence[float]], label: str
) -> tuple[tuple[float, float], tuple[float, float], tuple[float, float]]:
    bounds = tuple(_finite_vector(item, 2, label) for item in values)
    if len(bounds) != 3:
        raise ValueError(f"{label} must contain exactly 3 bounds")
    return (
        _ordered_bounds(bounds[0], label),
        _ordered_bounds(bounds[1], label),
        _ordered_bounds(bounds[2], label),
    )


class RebotArmMujoco:
    joint_names = JOINT_NAMES

    def __init__(self, model_path: str | os.PathLike[str] | None = None, *, collisionless: bool = False, motor_parameters: MotorControlParameters | None = None) -> None:
        """加载 MuJoCo 模型并完成控制器与初始状态同步。

        model_path 为 None 时按 model_resource() 的优先级搜索默认场景，
        找不到则抛 FileNotFoundError。构造末尾调用 reset()：把控制目标对齐到
        初始 qpos、清零控制量，并预置重力补偿力矩，避免第一步出现下坠冲击。
        """
        # 延迟导入 MuJoCo：缺失时给出可执行的安装提示，而不是裸 ImportError。
        try:
            self._mj = importlib.import_module("mujoco")
        except (ImportError, ModuleNotFoundError) as exc:
            raise RuntimeError(
                "MuJoCo is required. Install src/rebotarm_simulation/requirements-mujoco.txt "
                "in the active Python environment."
            ) from exc
        self.model_path = str(model_resource(explicit=model_path))
        # MjModel 是不可变的模型定义；MjData 保存逐次步进的运行时状态。
        self._model = self._mj.MjModel.from_xml_path(self.model_path)
        if collisionless:
            # Reach-only opt-in: visual meshes and inertial properties remain;
            # costly self-mesh contact is omitted from this contact-free task.
            self._model.geom_contype[:] = 0
            self._model.geom_conaffinity[:] = 0
        self._data = self._mj.MjData(self._model)
        self._closed = False
        # 复现用随机数发生器；reset(seed=...) 会按给定种子重建它。
        self._rng = np.random.default_rng()
        self._randomization_baseline = {
            "body_mass": np.asarray(self._model.body_mass).copy(),
            "dof_damping": np.asarray(self._model.dof_damping).copy(),
            "geom_friction": np.asarray(self._model.geom_friction).copy(),
        }
        self._randomization_sample = None

        # 控制参数 = 仿真标定文件（冻结的固件参考增益）+ URDF 力矩上限，
        # 真机调参不得反向改变这里的仿真标定值。
        motor_parameters = motor_parameters if motor_parameters is not None else load_motor_control_parameters()
        # 名称 → MuJoCo 内部 id 的映射；缺名会立即报错，避免运行期静默错位。
        self._joint_ids = tuple(self._name_id(self._mj.mjtObj.mjOBJ_JOINT, name) for name in JOINT_NAMES)
        self._actuator_ids = tuple(
            self._name_id(self._mj.mjtObj.mjOBJ_ACTUATOR, actuator_name_for_joint(name))
            for name in JOINT_NAMES
        )
        self._control = SimControlRuntime(motor_parameters, float(self._model.opt.timestep), self._joint_ids, self._actuator_ids)
        # 末端执行器参考点（site）用于输出 TCP 位置与姿态。
        self._ee_site_id = self._name_id(self._mj.mjtObj.mjOBJ_SITE, "ee_site")
        # 场景中带自由关节的物体（被操作物），用于读写物体位姿。
        self._free_bodies = self._find_free_bodies()
        # 状态检查点范围显式声明：即使在 mjSTATE_INTEGRATION 已包含 CTRL/USER 的
        # MuJoCo 版本上也保持一致，覆盖控制量、已施加力、mocap/userdata/equality
        # 状态、插件状态以及求解器积分/热启动状态。
        self._snapshots = StateSnapshot(self._mj, self._model)
        self.reset()


    @property
    def timestep(self) -> float:
        self._ensure_open()
        return float(self._model.opt.timestep)


    def borrow_viewer_handles(self):
        """Borrow mutable native model/data exclusively for a passive Viewer.

        This does not transfer ownership or acquire a lock. The caller must
        serialize physics access and viewer.sync() using its existing simulation
        access gate. Native handles may be retained for the Viewer lifetime;
        stop rendering and finish native Viewer shutdown before sim.close().
        Do not use this interface for business logic or release the model/data.
        Already borrowed handles cannot be revoked by Python after close().
        """
        self._ensure_open()
        return self._model, self._data


    def _unsafe_viewer_handles(self):
        return self.borrow_viewer_handles()

    @property
    def control_targets(self) -> tuple[float, ...]:
        self._ensure_open()
        return tuple(float(value) for value in self._control._position_targets)


    @property
    def control_mode(self) -> str:
        self._ensure_open()
        return self._control._control_mode


    @property
    def randomization_sample(self) -> RandomizationSample | None:
        self._ensure_open()
        return self._randomization_sample


    @property
    def arm_joint_limits(self) -> tuple[tuple[float, float], ...]:
        """Return the six arm joint ranges declared by the loaded MJCF."""
        self._ensure_open()
        return tuple(
            tuple(float(value) for value in self._model.jnt_range[joint_id])
            for joint_id in self._joint_ids[:6]
        )


    @property
    def arm_actuator_force_limits(self) -> tuple[float, ...]:
        """Return symmetric arm actuator limits used for safety validation."""
        self._ensure_open()
        limits = []
        for actuator_id in self._actuator_ids[:6]:
            if int(self._model.actuator_ctrllimited[actuator_id]):
                lower, upper = self._model.actuator_ctrlrange[actuator_id]
                limits.append(max(abs(float(lower)), abs(float(upper))))
            else:
                limits.append(math.inf)
        return tuple(limits)


    def randomization_session(self, sample: RandomizationSample):
        from rebotarm_simulation.sim2real.randomization import RandomizationSession

        return RandomizationSession(self, sample)


    def apply_randomization(self, sample: RandomizationSample) -> None:
        self._ensure_open()
        if not isinstance(sample, RandomizationSample):
            raise TypeError("sample must be a RandomizationSample")
        self.restore_randomization()
        self._model.body_mass[:] = self._randomization_baseline["body_mass"] * sample.mass_scale
        self._model.dof_damping[:] = self._randomization_baseline["dof_damping"] * sample.damping_scale
        self._model.geom_friction[:] = self._randomization_baseline["geom_friction"] * sample.friction_scale
        self._control._randomization_torque_scale = float(sample.torque_scale)
        self._randomization_sample = sample
        self._mj.mj_forward(self._model, self._data)


    def restore_randomization(self) -> None:
        self._ensure_open()
        self._model.body_mass[:] = self._randomization_baseline["body_mass"]
        self._model.dof_damping[:] = self._randomization_baseline["dof_damping"]
        self._model.geom_friction[:] = self._randomization_baseline["geom_friction"]
        self._control._randomization_torque_scale = 1.0
        self._randomization_sample = None
        self._mj.mj_forward(self._model, self._data)


    def set_mode(self, mode: str) -> str:
        self._ensure_open()
        return self._control.set_mode(self._model, self._data, mode)

    def set_control_mode(self, mode: str) -> str:
        self._ensure_open()
        return self._control.set_control_mode(self._model, self._data, mode)

    def _name_id(self, object_type, name: str) -> int:
        identifier = int(self._mj.mj_name2id(self._model, object_type, name))
        if identifier < 0:
            raise ValueError(f"MuJoCo model is missing required {name!r}")
        return identifier


    def _ensure_open(self) -> None:
        if self._closed:
            raise RuntimeError("MuJoCo simulation is closed")


    def _find_free_bodies(self) -> dict[str, tuple[int, int]]:
        result: dict[str, tuple[int, int]] = {}
        free_type = int(self._mj.mjtJoint.mjJNT_FREE)
        for body_id in range(1, int(self._model.nbody)):
            joint_start = int(self._model.body_jntadr[body_id])
            joint_count = int(self._model.body_jntnum[body_id])
            for joint_id in range(joint_start, joint_start + joint_count):
                if int(self._model.jnt_type[joint_id]) == free_type:
                    name = self._mj.mj_id2name(self._model, self._mj.mjtObj.mjOBJ_BODY, body_id)
                    result[str(name)] = (body_id, int(self._model.jnt_qposadr[joint_id]))
        return result


    def reset(self, seed: int | None = None) -> SimulationState:
        self._ensure_open()
        self._rng = np.random.default_rng(seed)
        home_key = self._mj.mj_name2id(self._model, self._mj.mjtObj.mjOBJ_KEY, "home")
        if home_key >= 0:
            self._mj.mj_resetDataKeyframe(self._model, self._data, home_key)
        else:
            self._mj.mj_resetData(self._model, self._data)
        return self._finish_reset()


    def reset_home(self, seed: int | None = None) -> SimulationState:
        return self.reset(seed=seed)


    def reset_joint_positions(self, positions: Sequence[float]) -> SimulationState:
        """Reset the arm exactly while rejecting limits instead of clamping."""
        self._ensure_open()
        values = _finite_vector(positions, len(ARM_JOINT_NAMES), "joint positions")
        for index, (joint_id, value) in enumerate(zip(self._joint_ids[:6], values)):
            # 先做限位校验（而不是裁剪）：回放起点来自实机，越限说明数据或模型不匹配，
            # 此时静默裁剪会掩盖问题，因此直接报错。
            lower, upper = (float(bound) for bound in self._model.jnt_range[joint_id])
            if value < lower or value > upper:
                raise ValueError(
                    f"{ARM_JOINT_NAMES[index]} position {value} outside [{lower}, {upper}]"
                )
        # Reject the whole reset before changing any joint state.
        for joint_id, value in zip(self._joint_ids[:6], values):
            self._data.qpos[int(self._model.jnt_qposadr[joint_id])] = value
        self._data.qvel[:] = 0.0
        gripper_targets = self.control_targets[-2:]
        self._finish_reset()
        self._control._position_targets[-2:] = gripper_targets
        self._control._apply_motor_control(self._model, self._data)
        self._mj.mj_forward(self._model, self._data)
        return self.get_state()


    def _finish_reset(self) -> SimulationState:
        for index, joint_id in enumerate(self._joint_ids):
            qpos_address = int(self._model.jnt_qposadr[joint_id])
            self._control._position_targets[index] = self._data.qpos[qpos_address]
        self._data.ctrl[:] = 0.0
        self._control._arm_controller.reset()
        self._control._control_mode = "hold"
        self._control._raw_torque_command.fill(0.0)
        self._control._raw_torque_requested.fill(0.0)
        self._control._raw_torque_deadline = None
        self._control._requested_arm_torque.fill(0.0)
        self._control._applied_arm_torque.fill(0.0)
        self._control._arm_torque_saturated.fill(False)
        self._control._gripper_max_force_n = None
        self._control._gripper_control_force.fill(0.0)
        self._control._control_phase = 0
        self._mj.mj_forward(self._model, self._data)
        self._control._seed_arm_torque_from_gravity(self._model, self._data)
        self._control._apply_motor_control(self._model, self._data)
        self._mj.mj_forward(self._model, self._data)
        return self.get_state()


    def command_joint_positions(
        self, targets: Mapping[str, float] | Sequence[float]
    ) -> tuple[float, ...]:
        self._ensure_open()
        return self._control.command_joint_positions(self._model, self._data, targets)

    def set_joint_position_targets(
        self, targets: Mapping[str, float] | Sequence[float]
    ) -> tuple[float, ...]:
        self._ensure_open()
        return self._control.set_joint_position_targets(self._model, self._data, targets)

    def command_joint_torques(
        self, torques: Sequence[float], timeout_s: float = 0.1
    ) -> tuple[float, ...]:
        self._ensure_open()
        return self._control.command_joint_torques(self._model, self._data, torques, timeout_s)

    def command_gripper_width(self, width_m: float, max_force_n: float | None = None) -> float:
        self._ensure_open()
        return self._control.command_gripper_width(self._model, self._data, width_m, max_force_n)

    def set_gripper_width(self, width: float) -> float:
        self._ensure_open()
        return self._control.set_gripper_width(self._model, self._data, width)

    def mirror_joint_state(
        self,
        positions: Sequence[float],
        velocities: Sequence[float] | None = None,
        *,
        gripper_width: float | None = None,
    ) -> SimulationState:
        """Kinematically synchronize arm state while preserving free objects."""
        self._ensure_open()
        position_values = _finite_vector(positions, len(ARM_JOINT_NAMES), "joint positions")
        velocity_values = (
            (0.0,) * len(ARM_JOINT_NAMES)
            if velocities is None
            else _finite_vector(velocities, len(ARM_JOINT_NAMES), "joint velocities")
        )
        for index, (joint_id, position, velocity) in enumerate(
            zip(self._joint_ids[:6], position_values, velocity_values)
        ):
            lower, upper = (float(value) for value in self._model.jnt_range[joint_id])
            if position < lower - 1e-6 or position > upper + 1e-6:
                raise ValueError(
                    f"joint position {ARM_JOINT_NAMES[index]}={position} is outside [{lower}, {upper}]"
                )
            qpos_address = int(self._model.jnt_qposadr[joint_id])
            qvel_address = int(self._model.jnt_dofadr[joint_id])
            self._data.qpos[qpos_address] = min(max(position, lower), upper)
            self._data.qvel[qvel_address] = velocity
            self._control._position_targets[index] = self._data.qpos[qpos_address]
        if gripper_width is not None:
            left, right, _reached = gripper_joint_positions_for_width(gripper_width)
            for index, value in zip((-2, -1), (left, right)):
                joint_id = self._joint_ids[index]
                self._data.qpos[int(self._model.jnt_qposadr[joint_id])] = value
                self._data.qvel[int(self._model.jnt_dofadr[joint_id])] = 0.0
                self._control._position_targets[index] = value
        self._control._arm_controller.reset()
        self._control._control_phase = 0
        self._mj.mj_forward(self._model, self._data)
        self._control._seed_arm_torque_from_gravity(self._model, self._data)
        self._control._apply_motor_control(self._model, self._data)
        self._mj.mj_forward(self._model, self._data)
        return self.get_state()


    def step(self, n_steps: int = 1) -> SimulationState:
        """推进 n_steps 个物理步并返回步进后的状态。

        n_steps 必须是正整数（bool 会被拒）。控制量按固件控制周期（默认 100 Hz）
        保持：仅当控制相位回到 0 时才重算一次，其余物理步复用同一控制量。
        """
        self._ensure_open()
        if isinstance(n_steps, bool) or not isinstance(n_steps, int):
            raise TypeError("n_steps must be a positive integer")
        if n_steps <= 0:
            raise ValueError("n_steps must be a positive integer")
        for _ in range(n_steps):
            self._control.before_step(self._model, self._data)
            self._mj.mj_step(self._model, self._data)
            self._control.after_step(self._model, self._data)
        # mj_step 在位置阶段之后才积分 qpos；这里刷新一次派生运动学，
        # 使返回的位姿对应步进结束时的 qpos，而不是最后一步开始时的状态。
        self._mj.mj_forward(self._model, self._data)
        return self.get_state()


    def get_state(self) -> SimulationState:
        """汇总当前仿真状态为不可变快照，供上层发布或记录。

        返回的 joint_positions/joint_velocities 覆盖全部 8 个关节（手臂 rad、rad/s，
        手指 m、m/s），actuator_forces 为各执行器实际出力；末端位姿取自 ee_site；
        object_poses 为每个自由物体被标准化成 (x,y,z,qx,qy,qz,qw) 的位姿。
        """
        self._ensure_open()
        return read_state(self._mj, self._model, self._data, self._joint_ids, self._actuator_ids, self._ee_site_id, self._free_bodies)


    def get_control_status(self) -> ControlStatus:
        self._ensure_open()
        return self._control.get_control_status(self.get_state())

    def get_contacts(self) -> tuple[ContactInfo, ...]:
        """返回当前所有接触点信息（世界坐标位置与合力大小，单位 N）。

        对每个接触调用 mj_contactForce 取得 6 维 wrench（前 3 项力、后 3 项力矩），
        这里只报告力部分的模长。body/geom 名缺失时分别回退为 "world" 与 "geom<id>"。
        """
        self._ensure_open()
        return read_contacts(self._mj, self._model, self._data)


    def save_state(self) -> SavedSimulationState:
        self._ensure_open()
        return self._snapshots.save_state(self._mj, self._model, self._data, self._control)


    def restore_state(self, state: SavedSimulationState) -> SimulationState:
        self._ensure_open()
        self._snapshots.restore_state(self._mj, self._model, self._data, self._control, state)
        return self.get_state()


    def set_object_pose(
        self,
        body_name: str,
        position: Sequence[float],
        orientation: Sequence[float],
        *,
        zero_velocity: bool = True,
    ) -> tuple[float, ...]:
        self._ensure_open()
        if body_name not in self._free_bodies:
            raise ValueError(f"Body {body_name!r} is not a free object")
        position_values = _finite_vector(position, 3, "position")
        quaternion = np.asarray(_finite_vector(orientation, 4, "orientation"), dtype=float)
        norm = float(np.linalg.norm(quaternion))
        if norm <= 1e-12:
            raise ValueError("orientation quaternion must have non-zero norm")
        quaternion /= norm
        body_id, address = self._free_bodies[body_name]
        orientation_xyzw = tuple(float(value) for value in quaternion)
        internal_pose_wxyz = (
            *position_values,
            orientation_xyzw[3],
            *orientation_xyzw[:3],
        )
        self._data.qpos[address : address + 7] = internal_pose_wxyz
        if zero_velocity:
            joint_id = int(self._model.body_jntadr[body_id])
            dof_address = int(self._model.jnt_dofadr[joint_id])
            self._data.qvel[dof_address : dof_address + 6] = 0.0
        self._mj.mj_forward(self._model, self._data)
        return (*position_values, *orientation_xyzw)


    def randomize_scene(
        self,
        seed: int | None = None,
        *,
        cube_xy_bounds: Sequence[Sequence[float]] = ((0.22, 0.38), (-0.14, 0.14)),
        cube_z: float = 0.04,
        reach_target_bounds: Sequence[Sequence[float]] = (
            (0.18, 0.45),
            (-0.22, 0.22),
            (0.08, 0.35),
        ),
    ) -> RandomizedScene:
        self._ensure_open()
        rng = np.random.default_rng(seed) if seed is not None else self._rng
        cube_x_bounds, cube_y_bounds = _bounds2(cube_xy_bounds, "cube_xy_bounds")
        target_x_bounds, target_y_bounds, target_z_bounds = _bounds3(
            reach_target_bounds, "reach_target_bounds"
        )
        cube_position = (
            float(rng.uniform(*cube_x_bounds)),
            float(rng.uniform(*cube_y_bounds)),
            float(cube_z),
        )
        target = (
            float(rng.uniform(*target_x_bounds)),
            float(rng.uniform(*target_y_bounds)),
            float(rng.uniform(*target_z_bounds)),
        )
        cube_pose = self.set_object_pose(
            "test_cube",
            cube_position,
            (0.0, 0.0, 0.0, 1.0),
        )
        return RandomizedScene(
            cube_pose=cube_pose,
            reach_target_position=target,
            seed=seed,
        )


    def randomize_bottle_pose(self, seed: int | None = None) -> tuple[float, ...]:
        """Place the canonical bottle reproducibly within the tabletop workspace."""
        self._ensure_open()
        if "bottle" not in self._free_bodies:
            raise ValueError("the loaded scene has no free bottle")
        rng = np.random.default_rng(seed) if seed is not None else self._rng
        position = (float(rng.uniform(0.22, 0.38)), float(rng.uniform(-0.14, 0.14)), 0.0)
        return self.set_object_pose("bottle", position, (0.0, 0.0, 0.0, 1.0))


    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._data = None
        self._model = None


    def __enter__(self) -> "RebotArmMujoco":
        self._ensure_open()
        return self


    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()


    def has_body(self, name: str) -> bool:
        self._ensure_open()
        return self._mj.mj_name2id(self._model, self._mj.mjtObj.mjOBJ_BODY, name) >= 0

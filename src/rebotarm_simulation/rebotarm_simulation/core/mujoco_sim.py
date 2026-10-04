"""MuJoCo 仿真后端：机械臂与夹爪的物理步进、控制与状态快照。

本模块是仿真包的核心：用 MuJoCo 加载 MJCF 场景（默认 models/rebotarm/scene.xml），
把六轴机械臂与平行两指夹爪建模为扭矩/力驱动器，并在仿真内复现真实固件的控制律。

系统位置：
- 上层仿真 ROS 节点、无头健康检查、离线查看器与命令行工具都通过本模块导出的
  RebotArmMujoco 类访问物理引擎；
- 本模块只读写仿真状态，不接触任何真实电机通道，仿真启动不会开启硬件通道。

控制模型（三种模式，见 CONTROL_MODES）：
- "pos_vel"：默认模式，用与真实固件参考值一致的串级位置/速度 PI 产生关节力矩；
- "hold"：内置固定增益 PD + 重力补偿，用于把关节稳定保持在当前位置目标；
- "gravity_comp"：只输出重力补偿力矩，用于自由漂浮/拖动演示。

安全约束：
- set_joint_position_targets 会把手臂目标角裁剪到模型关节限位 [lower, upper]（弧度）；
  reset_joint_positions 走另一条路径：越限直接抛错，不做静默裁剪；
- 每个执行器的控制量受 MJCF ctrlrange 限制，超限自动裁剪；
- 对外报告的夹爪张开宽度限制在 [0, 0.09] m。

单位与坐标约定：
- 关节角 rad、角速度 rad/s、力矩 N·m、手指推力 N、长度 m；
- 位姿四元数对外统一为 (x, y, z, w)，而 MuJoCo 自由关节内部为 (w, x, y, z)，
  两种顺序的转换见 get_state() 与 set_object_pose()。
"""

from __future__ import annotations

import importlib
import os
from typing import Mapping, Sequence

import numpy as np

from rebotarm_simulation.control.motor_control import MotorControlParameters
from rebotarm_simulation.control.control_config import load_motor_control_parameters
from rebotarm_simulation.core.mujoco_types import ContactInfo, SavedSimulationState, SimulationState
from rebotarm_simulation.core.model_contract import ARM_JOINT_NAMES, JOINT_NAMES, actuator_name_for_joint
from rebotarm_simulation.core.resource_paths import model_resource
from rebotarm_simulation.core.validation import finite_vector as _finite_vector
from rebotarm_simulation.control.sim_control_runtime import CONTROL_MODES, SimControlRuntime
from rebotarm_simulation.core.state_snapshot import StateSnapshot
from rebotarm_simulation.core.observations import read_state, read_contacts


class RebotArmMujoco:
    """MuJoCo 中的 reBotArm 模型封装：物理步进、控制与状态快照。

    生命周期：构造时加载 MJCF、建立关节/执行器/site 的名称索引，并做一次初始
    状态同步；使用期间由调用方保证串行访问（上层 ROS 节点把对仿真对象的调用
    排队到同一线程）；close() 之后任何访问都会抛 RuntimeError。

    单位约定：关节角 rad、角速度 rad/s、力矩 N·m、手指推力 N、长度 m。
    """

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
        # 物理积分步长，单位秒（当前场景为 0.002，即 500 Hz）。
        self._ensure_open()
        return float(self._model.opt.timestep)

    def has_body(self, name: str) -> bool:
        """Check model membership without exposing mutable MuJoCo handles."""
        self._ensure_open()
        return self._mj.mj_name2id(self._model, self._mj.mjtObj.mjOBJ_BODY, name) >= 0

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

    @property
    def control_targets(self) -> tuple[float, ...]:
        # 只读镜像当前 8 维控制目标（手臂 rad，手指 m）。
        self._ensure_open()
        return self._control.control_targets

    @property
    def control_mode(self) -> str:
        self._ensure_open()
        return self._control.control_mode

    def set_control_mode(self, mode: str) -> str:
        self._ensure_open()
        return self._control.set_control_mode(self._model, self._data, mode)

    def _name_id(self, object_type, name: str) -> int:
        # 名称查询失败时 MuJoCo 返回负 id；这里直接升级为异常，尽早暴露模型缺件。
        identifier = int(self._mj.mj_name2id(self._model, object_type, name))
        if identifier < 0:
            raise ValueError(f"MuJoCo model is missing required {name!r}")
        return identifier

    def _ensure_open(self) -> None:
        # 关闭后所有公开入口的统一闸门，防止对已释放句柄做操作。
        if self._closed:
            raise RuntimeError("MuJoCo simulation is closed")

    def _find_free_bodies(self) -> dict[str, tuple[int, int]]:
        """收集所有带自由关节（6 自由度浮动）的物体，返回 {body 名: (body_id, qpos 起始下标)}。

        自由关节的 qpos 布局固定为 7 个分量：位置 (x,y,z) 在前，随后是内部顺序
        为 (w,x,y,z) 的四元数；本函数记录其起始下标，供位姿读写定位使用。
        """
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
        """把物理状态复位到模型默认值（非关键帧），并返回复位后的状态。

        seed 用于重建内部随机数发生器，便于实验复现；None 表示不固定种子。
        """
        self._ensure_open()
        self._rng = np.random.default_rng(seed)
        self._mj.mj_resetData(self._model, self._data)
        return self._finish_reset()

    def reset_home(self, seed: int | None = None) -> SimulationState:
        """复位到场景关键帧 "home"；若场景未定义该关键帧则退化为默认复位。

        home 关键帧是全部演示与抓取任务的统一起始位姿，复位后同样会重新同步
        控制目标与重力前馈。
        """
        self._ensure_open()
        self._rng = np.random.default_rng(seed)
        home_key = self._mj.mj_name2id(
            self._model, self._mj.mjtObj.mjOBJ_KEY, "home"
        )
        if home_key >= 0:
            self._mj.mj_resetDataKeyframe(self._model, self._data, home_key)
        else:
            self._mj.mj_resetData(self._model, self._data)
        return self._finish_reset()

    def reset_joint_positions(self, positions: Sequence[float]) -> SimulationState:
        """把六个手臂关节复位到给定的实测起始角，用于可复现的仿真到实机回放。

        入参为 6 个关节角（rad），顺序同 ARM_JOINT_NAMES；越出模型关节限位会抛
        ValueError。复位同时清零关节速度与控制量，并把控制目标对齐到该位姿。
        本方法只改仿真状态，不会向任何物理控制器下发指令。
        """
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
        for index, (joint_id, value) in enumerate(zip(self._joint_ids[:6], values)):
            self._data.qpos[int(self._model.jnt_qposadr[joint_id])] = value
            self._data.qvel[int(self._model.jnt_dofadr[joint_id])] = 0.0
            self._control.set_reset_target(index, value)
        self._data.ctrl[:] = 0.0
        self._control.reset_memory()
        self._mj.mj_forward(self._model, self._data)
        self._control.seed_arm_torque_from_gravity(self._model, self._data)
        self._control.apply_motor_control(self._model, self._data)
        self._mj.mj_forward(self._model, self._data)
        return self.get_state()

    def _finish_reset(self) -> SimulationState:
        """两类复位（默认复位/关键帧复位）共用的收尾：同步目标、清空控制状态。

        顺序很关键：先把 8 个控制目标对齐到复位后的 qpos（否则控制器会把关节拉回
        复位前的目标），再清零 ctrl 与控制器积分，前向运动学后用重力补偿预热
        已施加力矩，最后重算一次控制量，保证复位瞬间不会产生力矩跳变。
        """
        for index, joint_id in enumerate(self._joint_ids):
            qpos_address = int(self._model.jnt_qposadr[joint_id])
            self._control.set_reset_target(index, self._data.qpos[qpos_address])
        self._data.ctrl[:] = 0.0
        self._control.reset_memory()
        self._mj.mj_forward(self._model, self._data)
        self._control.seed_arm_torque_from_gravity(self._model, self._data)
        self._control.apply_motor_control(self._model, self._data)
        self._mj.mj_forward(self._model, self._data)
        return self.get_state()

    def set_joint_position_targets(
        self, targets: Mapping[str, float] | Sequence[float]
    ) -> tuple[float, ...]:
        self._ensure_open()
        return self._control.set_joint_position_targets(self._model, targets)

    def set_gripper_width(self, width: float) -> float:
        self._ensure_open()
        return self._control.set_gripper_width(width)

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
            self._control.after_step()
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
        """摆放带自由关节的物体（例如被操作物），返回写入的 (x,y,z,qx,qy,qz,qw)。

        body_name 必须是 _find_free_bodies 识别到的自由物体，否则报错；position 为
        3 维世界坐标（m），orientation 为 (x,y,z,w) 四元数（内部归一化，零范数报错）。
        zero_velocity 默认为 True：同时清零该物体的 6 维速度，避免摆放后带着残余
        动量飞出场景。
        """
        self._ensure_open()
        if body_name not in self._free_bodies:
            raise ValueError(f"Body {body_name!r} is not a free object")
        position_values = _finite_vector(position, 3, "position")
        quaternion = np.asarray(_finite_vector(orientation, 4, "orientation"), dtype=float)
        # 归一化四元数，避免非单位四元数导致姿态缩放或数值退化。
        norm = float(np.linalg.norm(quaternion))
        if norm <= 1e-12:
            raise ValueError("orientation quaternion must have non-zero norm")
        quaternion /= norm
        body_id, address = self._free_bodies[body_name]
        orientation_xyzw = tuple(float(value) for value in quaternion)
        # 写回 qpos 时需换回 MuJoCo 的内部顺序 (x,y,z, qw,qx,qy,qz)。
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

    def randomize_bottle_pose(self, seed: int | None = None) -> tuple[float, ...]:
        """Place the canonical bottle reproducibly within the tabletop workspace."""
        self._ensure_open()
        if "bottle" not in self._free_bodies:
            raise ValueError("the loaded scene has no free bottle")
        rng = np.random.default_rng(seed) if seed is not None else self._rng
        position = (float(rng.uniform(0.22, 0.38)), float(rng.uniform(-0.14, 0.14)), 0.0)
        return self.set_object_pose("bottle", position, (0.0, 0.0, 0.0, 1.0))

    def close(self) -> None:
        """释放模型与数据句柄（可重复调用）；此后任何访问都会报“已关闭”。"""
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

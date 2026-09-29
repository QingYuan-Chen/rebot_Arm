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

import hashlib
import importlib
import math
import os
from pathlib import Path
import sys
from types import MappingProxyType
from typing import Mapping, Sequence

import numpy as np

from .motor_control import GripperMitController, PosVelController, load_motor_control_parameters
from .mujoco_types import ContactInfo, SavedSimulationState, SimulationState
from .sim_gripper import gripper_joint_positions_for_width
from .urdf_to_mjcf import actuator_name_for_joint


# 六轴机械臂关节名；数组下标顺序即控制器与状态向量的顺序。
ARM_JOINT_NAMES = tuple(f"joint{index}" for index in range(1, 7))
# 平行夹爪的两个滑动指关节：left 取值 [0, 0.045] m，right 取值 [-0.045, 0] m；
# 张开宽度定义为 left - right（见 get_state 与 set_gripper_width）。
FINGER_JOINT_NAMES = ("left_finger_joint", "right_finger_joint")
# 对外统一的 8 关节顺序：前 6 个手臂关节（rad），后 2 个手指（m）。
JOINT_NAMES = ARM_JOINT_NAMES + FINGER_JOINT_NAMES
# 允许的控制模式；取值属于对外接口，保持英文原样不做翻译。
CONTROL_MODES = ("gravity_comp", "hold", "pos_vel")


def _default_scene_path() -> Path:
    """按优先级搜索默认 MJCF 场景文件并返回其路径。

    搜索顺序：源码包目录下的 models/rebotarm/scene.xml → 当前 Python 前缀的
    share 目录 → AMENT_PREFIX_PATH 中每个前缀的 share 目录。这样源码运行与
    安装后运行都能命中同一场景。全部落空时抛 FileNotFoundError，并在消息中
    列出已搜索过的候选路径，便于定位环境问题。
    """
    package_root = Path(__file__).resolve().parents[1]
    relative = Path("models/rebotarm/scene.xml")
    # 候选路径按优先级排列：源码包目录 → 当前 Python 前缀 → 各 ament 前缀。
    candidates = [package_root / relative, Path(sys.prefix) / "share/rebotarm_simulation" / relative]
    for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        if prefix:
            candidates.append(Path(prefix) / "share/rebotarm_simulation" / relative)
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    searched = ", ".join(str(candidate) for candidate in candidates)
    raise FileNotFoundError(f"Could not locate reBotArm MuJoCo scene.xml; searched: {searched}")


def _finite_vector(values: Sequence[float], length: int, label: str) -> tuple[float, ...]:
    """把输入校验为指定长度、全部有限的浮点向量。

    label 仅用于构造错误消息。所有对外数值接口（关节角、位姿、四元数）都经此
    把关：非数值、长度不符、含 NaN/Inf 一律拒绝，避免坏值进入物理积分。
    """
    # 显式拦截 str/bytes：它们本身可迭代，否则会被逐字符转换而得到隐蔽的错误值。
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


class RebotArmMujoco:
    """MuJoCo 中的 reBotArm 模型封装：物理步进、控制与状态快照。

    生命周期：构造时加载 MJCF、建立关节/执行器/site 的名称索引，并做一次初始
    状态同步；使用期间由调用方保证串行访问（上层 ROS 节点把对仿真对象的调用
    排队到同一线程）；close() 之后任何访问都会抛 RuntimeError。

    单位约定：关节角 rad、角速度 rad/s、力矩 N·m、手指推力 N、长度 m。
    """

    joint_names = JOINT_NAMES

    def __init__(self, model_path: str | os.PathLike[str] | None = None, *, collisionless: bool = False) -> None:
        """加载 MuJoCo 模型并完成控制器与初始状态同步。

        model_path 为 None 时按 _default_scene_path() 的优先级搜索默认场景，
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
        self.model_path = str(Path(model_path) if model_path is not None else _default_scene_path())
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
        motor_parameters = load_motor_control_parameters()
        self._motor_parameters = motor_parameters
        self._arm_controller = PosVelController(motor_parameters.arm)
        self._gripper_controller = GripperMitController(motor_parameters.gripper)
        # 固件控制周期（默认 100 Hz）对应多少个物理步：例如 timestep=2 ms 时为 5 步。
        # 控制量只在周期起点重算一次，其余步保持恒定（零阶保持），与真机一致。
        self._control_steps_per_update = max(
            1, int(round((1.0 / motor_parameters.control_rate_hz) / float(self._model.opt.timestep)))
        )
        # 当前控制周期内已走过的物理步数：0 表示需要重新计算控制量。
        self._control_phase = 0
        # 8 维目标向量：前 6 项为手臂关节角(rad)，后 2 项为左/右手指位置(m)。
        self._position_targets = np.zeros(len(JOINT_NAMES), dtype=float)
        # 默认使用与真机固件行为一致的串级位置/速度模式。
        self._control_mode = "pos_vel"

        # 名称 → MuJoCo 内部 id 的映射；缺名会立即报错，避免运行期静默错位。
        self._joint_ids = tuple(self._name_id(self._mj.mjtObj.mjOBJ_JOINT, name) for name in JOINT_NAMES)
        self._actuator_ids = tuple(
            self._name_id(self._mj.mjtObj.mjOBJ_ACTUATOR, actuator_name_for_joint(name))
            for name in JOINT_NAMES
        )
        # 末端执行器参考点（site）用于输出 TCP 位置与姿态。
        self._ee_site_id = self._name_id(self._mj.mjtObj.mjOBJ_SITE, "ee_site")
        # 场景中带自由关节的物体（被操作物），用于读写物体位姿。
        self._free_bodies = self._find_free_bodies()
        # 状态检查点范围显式声明：即使在 mjSTATE_INTEGRATION 已包含 CTRL/USER 的
        # MuJoCo 版本上也保持一致，覆盖控制量、已施加力、mocap/userdata/equality
        # 状态、插件状态以及求解器积分/热启动状态。
        self._state_spec = (
            int(self._mj.mjtState.mjSTATE_INTEGRATION)
            | int(self._mj.mjtState.mjSTATE_USER)
            | int(self._mj.mjtState.mjSTATE_CTRL)
        )
        # 模型维度指纹的一部分：nq/nv/na/nu、body/joint/geom 数量与状态向量长度。
        # 快照恢复时逐项比对，防止把 A 模型的状态灌进 B 模型。
        self._model_dimensions = tuple(int(value) for value in (
            self._model.nq, self._model.nv, self._model.na, self._model.nu,
            self._model.nbody, self._model.njnt, self._model.ngeom,
            self._mj.mj_stateSize(self._model, self._state_spec),
        ))
        self._model_fingerprint = self._fingerprint_model()
        self.reset()

    @property
    def timestep(self) -> float:
        # 物理积分步长，单位秒（当前场景为 0.002，即 500 Hz）。
        self._ensure_open()
        return float(self._model.opt.timestep)

    def _unsafe_viewer_handles(self):
        """返回底层模型/数据句柄，仅供本包的查看器与相机适配层使用。

        名称中的 unsafe 是使用契约：返回的原生对象只在本仿真对象存活期间有效，
        调用方不得长期持有，也不得在仿真归属线程之外改动物理状态。普通业务
        代码应改用本类的高层方法。
        """
        self._ensure_open()
        return self._model, self._data

    @property
    def control_targets(self) -> tuple[float, ...]:
        # 只读镜像当前 8 维控制目标（手臂 rad，手指 m）。
        self._ensure_open()
        return tuple(float(value) for value in self._position_targets)

    @property
    def control_mode(self) -> str:
        self._ensure_open()
        return self._control_mode

    def set_control_mode(self, mode: str) -> str:
        """切换控制模式并立即按新模式重算一次控制量，返回生效后的模式。

        模式必须是 CONTROL_MODES 之一（"gravity_comp" / "hold" / "pos_vel"）。
        切到 "hold" 会先把手臂目标同步为当前关节角（原地保持，不会突然蹿向旧
        目标）；切到 "gravity_comp" 或 "hold" 会复位控制器积分与已施加力矩，
        避免把上一个模式的积分/滤波状态带进新模式。
        """
        self._ensure_open()
        mode = str(mode)
        if mode not in CONTROL_MODES:
            raise ValueError(f"control mode must be one of {CONTROL_MODES}")
        if mode == "hold":
            self._sync_arm_targets_to_current_position()
        if mode in ("gravity_comp", "hold"):
            self._arm_controller.reset()
        self._control_mode = mode
        self._apply_motor_control()
        return self._control_mode

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

    def _fingerprint_model(self) -> str:
        """对模型结构做 SHA-256 指纹，用于校验状态快照与模型是否匹配。

        参与哈希的量：维度元组、名称表、关节类型/地址/限位、执行器传动目标、
        geom 所属 body，以及物理步长。只要 MJCF 结构或积分步长改变，指纹即变化，
        restore_state 会因此拒绝恢复旧快照。
        """
        digest = hashlib.sha256()
        digest.update(repr(self._model_dimensions).encode("ascii"))
        for values in (
            self._model.names,
            self._model.jnt_type,
            self._model.jnt_qposadr,
            self._model.jnt_dofadr,
            self._model.jnt_range,
            self._model.actuator_trnid,
            self._model.geom_bodyid,
        ):
            digest.update(np.asarray(values).tobytes())
        digest.update(np.asarray([self._model.opt.timestep], dtype=np.float64).tobytes())
        return digest.hexdigest()

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
        for joint_id, value in zip(self._joint_ids[:6], values):
            self._data.qpos[int(self._model.jnt_qposadr[joint_id])] = value
            self._data.qvel[int(self._model.jnt_dofadr[joint_id])] = 0.0
            self._position_targets[index] = value
        self._data.ctrl[:] = 0.0
        self._arm_controller.reset()
        self._control_phase = 0
        self._mj.mj_forward(self._model, self._data)
        self._seed_arm_torque_from_gravity()
        self._apply_motor_control()
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
            self._position_targets[index] = self._data.qpos[qpos_address]
        self._data.ctrl[:] = 0.0
        self._arm_controller.reset()
        self._control_phase = 0
        self._mj.mj_forward(self._model, self._data)
        self._seed_arm_torque_from_gravity()
        self._apply_motor_control()
        self._mj.mj_forward(self._model, self._data)
        return self.get_state()

    def set_joint_position_targets(
        self, targets: Mapping[str, float] | Sequence[float]
    ) -> tuple[float, ...]:
        """设置手臂关节位置目标，返回实际生效（已按限位裁剪）的 6 个角度。

        支持两种入参：{关节名: 角度} 的映射（只更新列出的关节，其余保持原目标；
        关节名不合法直接报错），或 6 个角度的序列（整体替换）。角度单位 rad，
        必须是有限值。写入后控制模式强制切回 "pos_vel"，确保目标被闭环跟踪。
        """
        self._ensure_open()
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
            lower, upper = (float(value) for value in self._model.jnt_range[joint_id])
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
        self._ensure_open()
        value = float(width)
        if not math.isfinite(value):
            raise ValueError("Gripper width must be finite")
        left, right, reached = gripper_joint_positions_for_width(value)
        self._position_targets[-2] = left
        self._position_targets[-1] = right
        return reached

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
            if self._control_phase == 0:
                self._apply_motor_control()
            self._mj.mj_step(self._model, self._data)
            self._control_phase = (self._control_phase + 1) % self._control_steps_per_update
        # mj_step 在位置阶段之后才积分 qpos；这里刷新一次派生运动学，
        # 使返回的位姿对应步进结束时的 qpos，而不是最后一步开始时的状态。
        self._mj.mj_forward(self._model, self._data)
        return self.get_state()

    def _apply_motor_control(self) -> None:
        """按当前控制模式重算 8 个执行器的控制量并写入 MuJoCo 的 ctrl。

        每个固件控制周期调用一次（由 step() 的相位计数触发）。手臂力矩写入 6 个
        扭矩执行器，夹爪以等大反向的力写入两个手指力执行器。
        """
        qpos_addresses = [int(self._model.jnt_qposadr[joint_id]) for joint_id in self._joint_ids[:6]]
        qvel_addresses = [int(self._model.jnt_dofadr[joint_id]) for joint_id in self._joint_ids[:6]]
        position = np.asarray([self._data.qpos[address] for address in qpos_addresses], dtype=float)
        velocity = np.asarray([self._data.qvel[address] for address in qvel_addresses], dtype=float)
        # 重力/科氏偏置力矩（MuJoCo 的 qfrc_bias），按标定比例缩放后作为前馈。
        gravity = self._gravity_compensation_torque(qvel_addresses)
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
            self._data.ctrl[actuator_id] = torque

        # 夹爪按“开合量”控制：左右手指位置之差即净开口，速度同理取差值，
        # 这样单个标量目标就能驱动对称的两个滑动关节。
        left_qpos = float(self._data.qpos[int(self._model.jnt_qposadr[self._joint_ids[-2]])])
        right_qpos = float(self._data.qpos[int(self._model.jnt_qposadr[self._joint_ids[-1]])])
        left_qvel = float(self._data.qvel[int(self._model.jnt_dofadr[self._joint_ids[-2]])])
        right_qvel = float(self._data.qvel[int(self._model.jnt_dofadr[self._joint_ids[-1]])])
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
        left_force = self._clamp_actuator_control(self._actuator_ids[-2], finger_force)
        right_force = self._clamp_actuator_control(self._actuator_ids[-1], -finger_force)
        self._data.ctrl[self._actuator_ids[-2]] = left_force
        self._data.ctrl[self._actuator_ids[-1]] = right_force

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

    def _clamp_actuator_control(self, actuator_id: int, value: float) -> float:
        """把控制量裁剪到该执行器在 MJCF 中声明的 ctrlrange（若声明为受限）。

        返回裁剪后的值；执行器未设置 ctrllimited 时原样返回，交由 MuJoCo 自身处理。
        """
        if int(self._model.actuator_ctrllimited[actuator_id]):
            lower, upper = (float(v) for v in self._model.actuator_ctrlrange[actuator_id])
            return min(max(float(value), lower), upper)
        return float(value)

    def _seed_arm_torque_from_gravity(self) -> None:
        """把控制器“上一拍力矩”预置为当前重力补偿值，避免复位瞬间的力矩阶跃。"""
        qvel_addresses = [int(self._model.jnt_dofadr[joint_id]) for joint_id in self._joint_ids[:6]]
        self._arm_controller.applied_torque[:] = self._gravity_compensation_torque(qvel_addresses)

    def _sync_arm_targets_to_current_position(self) -> None:
        """把手臂目标角同步为当前实测角，用于模式切换时的“原地保持”。"""
        for index, joint_id in enumerate(self._joint_ids[:6]):
            self._position_targets[index] = float(self._data.qpos[int(self._model.jnt_qposadr[joint_id])])

    def _gravity_compensation_torque(self, qvel_addresses: Sequence[int]) -> np.ndarray:
        """取 6 个手臂关节的重力/科氏偏置力矩，并按标定系数缩放。

        数据源是 MuJoCo 的 qfrc_bias（单位 N·m，符号与关节方向一致）。
        gravity_compensation_scale 为 0 时直接返回零向量，便于做纯前馈关闭的对照实验。
        """
        scale = float(self._motor_parameters.arm.gravity_compensation_scale)
        if scale == 0.0:
            return np.zeros(len(ARM_JOINT_NAMES), dtype=float)
        return np.asarray([self._data.qfrc_bias[address] for address in qvel_addresses], dtype=float) * scale

    def get_state(self) -> SimulationState:
        """汇总当前仿真状态为不可变快照，供上层发布或记录。

        返回的 joint_positions/joint_velocities 覆盖全部 8 个关节（手臂 rad、rad/s，
        手指 m、m/s），actuator_forces 为各执行器实际出力；末端位姿取自 ee_site；
        object_poses 为每个自由物体被标准化成 (x,y,z,qx,qy,qz,qw) 的位姿。
        """
        self._ensure_open()
        positions = []
        velocities = []
        for joint_id in self._joint_ids:
            positions.append(float(self._data.qpos[int(self._model.jnt_qposadr[joint_id])]))
            velocities.append(float(self._data.qvel[int(self._model.jnt_dofadr[joint_id])]))
        # 自由关节 qpos 内部顺序为 (pos_x,pos_y,pos_z, qw,qx,qy,qz)；
        # 这里重排为对外的 (x,y,z, qx,qy,qz,qw)，即把 w 分量从第 4 位挪到末位。
        object_poses = {
            name: (
                *(float(value) for value in self._data.qpos[address : address + 3]),
                *(float(value) for value in self._data.qpos[address + 4 : address + 7]),
                float(self._data.qpos[address + 3]),
            )
            for name, (_, address) in self._free_bodies.items()
        }
        ee_quaternion_wxyz = np.empty(4, dtype=float)
        self._mj.mju_mat2Quat(
            ee_quaternion_wxyz, self._data.site_xmat[self._ee_site_id]
        )
        return SimulationState(
            joint_names=JOINT_NAMES,
            joint_positions=tuple(positions),
            joint_velocities=tuple(velocities),
            actuator_forces=tuple(float(self._data.actuator_force[index]) for index in self._actuator_ids),
            end_effector_position=tuple(float(value) for value in self._data.site_xpos[self._ee_site_id]),
            # mju_mat2Quat 输出 (w,x,y,z)，对外统一改排为 (x,y,z,w)。
            end_effector_orientation=(
                *(float(value) for value in ee_quaternion_wxyz[1:]),
                float(ee_quaternion_wxyz[0]),
            ),
            # 张开宽度 = 左指位置 - 右指位置，并按夹爪机械行程 [0, 0.09] m 夹紧，
            # 以吸收数值抖动与接触穿透带来的瞬时越界。
            gripper_width=max(0.0, min(0.09, positions[-2] - positions[-1])),
            object_poses=MappingProxyType(object_poses),
            simulation_time=float(self._data.time),
        )

    def get_contacts(self) -> tuple[ContactInfo, ...]:
        """返回当前所有接触点信息（世界坐标位置与合力大小，单位 N）。

        对每个接触调用 mj_contactForce 取得 6 维 wrench（前 3 项力、后 3 项力矩），
        这里只报告力部分的模长。body/geom 名缺失时分别回退为 "world" 与 "geom<id>"。
        """
        self._ensure_open()
        contacts = []
        force = np.zeros(6, dtype=float)
        for index in range(int(self._data.ncon)):
            contact = self._data.contact[index]
            geom1, geom2 = int(contact.geom1), int(contact.geom2)
            body1, body2 = int(self._model.geom_bodyid[geom1]), int(self._model.geom_bodyid[geom2])
            force.fill(0.0)
            self._mj.mj_contactForce(self._model, self._data, index, force)
            contacts.append(ContactInfo(
                body1=str(self._mj.mj_id2name(self._model, self._mj.mjtObj.mjOBJ_BODY, body1) or "world"),
                body2=str(self._mj.mj_id2name(self._model, self._mj.mjtObj.mjOBJ_BODY, body2) or "world"),
                geom1=str(self._mj.mj_id2name(self._model, self._mj.mjtObj.mjOBJ_GEOM, geom1) or f"geom{geom1}"),
                geom2=str(self._mj.mj_id2name(self._model, self._mj.mjtObj.mjOBJ_GEOM, geom2) or f"geom{geom2}"),
                position=tuple(float(value) for value in contact.pos),
                force=float(np.linalg.norm(force[:3])),
                penetration_depth=max(0.0, -float(contact.dist)),
                normal=tuple(float(value) for value in contact.frame[:3]),
            ))
        return tuple(contacts)

    def save_state(self) -> SavedSimulationState:
        """保存完整检查点：物理状态 + 控制器内部状态 + 目标与控制模式。

        物理状态按 _state_spec 导出（积分/用户/控制量），并附带模型实例 id、结构
        指纹与维度，供 restore_state 做兼容性校验。控制器侧记录目标角、位置/速度
        积分、已施加力矩、控制相位与模式，保证恢复后控制行为连续。返回值为不可变
        快照，可安全长期持有。
        """
        self._ensure_open()
        state = np.empty(self._model_dimensions[-1], dtype=float)
        self._mj.mj_getState(self._model, self._data, state, self._state_spec)
        return SavedSimulationState(
            model_identity=id(self._model),
            model_fingerprint=self._model_fingerprint,
            model_dimensions=self._model_dimensions,
            state_spec=self._state_spec,
            state=tuple(float(value) for value in state),
            control_targets=tuple(float(value) for value in self._position_targets),
            position_integral=tuple(float(value) for value in self._arm_controller.position_integral),
            velocity_integral=tuple(float(value) for value in self._arm_controller.velocity_integral),
            applied_torque=tuple(float(value) for value in self._arm_controller.applied_torque),
            control_phase=self._control_phase,
            control_mode=self._control_mode,
        )

    def restore_state(self, state: SavedSimulationState) -> SimulationState:
        """从 save_state() 的快照恢复物理与控制器状态，返回恢复后的状态。

        只接受同一模型实例产生的快照：模型实例 id、结构指纹、维度、状态向量长度、
        控制器各向量长度与控制模式必须全部匹配；不匹配时抛 ValueError，防止把
        不兼容状态灌进当前模型（例如 MJCF 已重新生成）。控制相位按控制周期取模，
        保持控制节拍与当前模型步长一致。
        """
        self._ensure_open()
        if not isinstance(state, SavedSimulationState):
            raise TypeError("state must be returned by save_state()")
        compatible = (
            state.model_identity == id(self._model)
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
        self._mj.mj_setState(
            self._model, self._data, np.asarray(state.state, dtype=float), self._state_spec
        )
        self._position_targets[:] = np.asarray(state.control_targets, dtype=float)
        self._arm_controller.position_integral[:] = np.asarray(state.position_integral, dtype=float)
        self._arm_controller.velocity_integral[:] = np.asarray(state.velocity_integral, dtype=float)
        self._arm_controller.applied_torque[:] = np.asarray(state.applied_torque, dtype=float)
        self._control_phase = int(state.control_phase) % self._control_steps_per_update
        self._control_mode = state.control_mode
        self._mj.mj_forward(self._model, self._data)
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

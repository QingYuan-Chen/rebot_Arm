"""仿真对外数据记录：不可变的物理状态、接触信息与断点存档。

模块职责：定义仿真核心与上层适配层之间传递的数据结构。这些记录只描述“事实”，不含任何控制
策略，也不依赖 ROS 或 MuJoCo 运行时，因此既能在仿真步进线程里构造，也能在测试里直接构造比对。

统一约定（很容易读错，务必注意）：

- 所有记录都是 frozen 数据类，构造完成后不可再变；``__post_init__`` 负责把所有数值字段规整为
  有限浮点数、把序列固化为元组、把字典包成只读视图，避免调用方事后修改快照导致“历史数据漂移”；
- 任何字段出现 NaN/inf、长度不符或非法类型都会抛 ``ValueError``，不做静默截断或补零；
- 四元数一律使用 ``(x, y, z, w)`` 顺序（与 MuJoCo 原生的 ``(w, x, y, z)`` 不同，转换发生在
  仿真核心读取状态时），位置与姿态默认在 MuJoCo 世界坐标系下，长度单位为米、角度单位为弧度。
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from types import MappingProxyType
from typing import Mapping


# 三维位置/向量：(x, y, z)，单位米。
Vector3 = tuple[float, float, float]
# 四元数姿态：(x, y, z, w)，注意 w 在末位，与 MuJoCo 原生顺序相反。
Quaternion = tuple[float, float, float, float]
# 六自由度位姿：(x, y, z, qx, qy, qz, qw)，位置单位米、姿态为上述 xyzw 四元数。
Pose = tuple[float, float, float, float, float, float, float]


def _float_tuple(values, *, length: int | None = None, label: str) -> tuple[float, ...]:
    """把任意数值序列规整为有限浮点元组，并做长度检查。

    参数：
        values: 待规整的序列；元素无法转成 float 时抛 ``ValueError``；
        length: 期望长度；``None`` 表示不检查（用于长度随关节数变化的向量）；
        label: 出错信息中使用的字段名，便于定位是哪个字段非法。

    异常：
        ValueError: 含非数值元素、长度不等于 ``length``、或存在 NaN/inf。
    """
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must contain numeric values") from exc
    if length is not None and len(result) != length:
        raise ValueError(f"{label} must contain exactly {length} values")
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{label} values must be finite")
    return result


@dataclass(frozen=True)
class SimulationState:
    """某一仿真时刻的完整可观测状态快照（只读）。

    字段含义与单位：

        joint_names: 状态向量各分量对应的关节名，顺序与下面三个向量一致。
            仿真核心给出的顺序是六个手臂关节（joint1..joint6，rad）后接两个手指滑轨关节（m）；
        joint_positions: 关节位置；手臂为弧度（rad），手指滑轨为米（m）；
        joint_velocities: 关节速度；手臂为 rad/s，手指滑轨为 m/s；
        actuator_forces: 各执行器当前出力；手臂为关节力矩（N·m），手指为滑轨力（N）；
        end_effector_position: 末端参考点在 MuJoCo 世界系下的位置（m）；
        end_effector_orientation: 末端参考点的姿态四元数（xyzw，无单位，应为单位四元数）；
        gripper_width: 夹爪开口宽度（m），定义为左指位置减右指位置并夹到 [0, 0.09]，
            即“机械开口”而不是目标值；被物体撑住时会小于指令宽度；
        object_poses: 场景中自由物体的位姿表，键为物体名，值为 ``Pose``（米 + xyzw 四元数）；
        simulation_time: 仿真时钟（s），单调递增，是全模块统一的时间基准。
    """

    joint_names: tuple[str, ...]
    joint_positions: tuple[float, ...]
    joint_velocities: tuple[float, ...]
    actuator_forces: tuple[float, ...]
    end_effector_position: Vector3
    end_effector_orientation: Quaternion
    gripper_width: float
    object_poses: Mapping[str, Pose]
    simulation_time: float

    def __post_init__(self) -> None:
        """规整并校验全部字段，把快照变成真正不可变的对象。

        使用 ``object.__setattr__`` 是因为 frozen 数据类在 ``__post_init__`` 中禁止普通赋值；
        这里做的是“就地规整”，不改变字段语义。
        """
        names = tuple(str(name) for name in self.joint_names)
        if not names or any(not name for name in names):
            raise ValueError("joint names must be non-empty")
        object.__setattr__(self, "joint_names", names)
        # 三个向量长度必须一致（都与 joint_names 对应），否则状态无法按下标解释。
        for field_name in (
            "joint_positions",
            "joint_velocities",
            "actuator_forces",
        ):
            object.__setattr__(
                self, field_name, _float_tuple(getattr(self, field_name), label=field_name)
            )
        object.__setattr__(self, "end_effector_position", _float_tuple(
            self.end_effector_position, length=3, label="end_effector_position"
        ))
        object.__setattr__(self, "end_effector_orientation", _float_tuple(
            self.end_effector_orientation, length=4, label="end_effector_orientation"
        ))
        # 开口宽度与仿真时刻这里只要求有限；开口的物理范围由仿真核心在读取状态时夹取。
        width = float(self.gripper_width)
        time = float(self.simulation_time)
        if not math.isfinite(width) or not math.isfinite(time):
            raise ValueError("gripper_width and simulation_time must be finite")
        object.__setattr__(self, "gripper_width", width)
        object.__setattr__(self, "simulation_time", time)
        # 物体位姿表重新逐项规整后包成只读视图，既保证数值合法，也防止调用方修改快照内容。
        immutable_poses = {
            str(name): _float_tuple(pose, length=7, label=f"object pose {name!r}")
            for name, pose in self.object_poses.items()
        }
        object.__setattr__(self, "object_poses", MappingProxyType(immutable_poses))


@dataclass(frozen=True)
class ContactInfo:
    """一次接触事件的描述（只读），用于把物理接触反馈给上层做抓取判定与安全监视。

    字段含义：

        body1 / body2: 参与接触的两个刚体名；接触的一方是世界（静态几何）时名为 ``world``；
        geom1 / geom2: 参与接触的两个几何体名，用于区分是哪个碰撞体真的碰上了；
        position: 接触点在 MuJoCo 世界坐标系下的位置（m）；
        force: 接触力大小（N），取接触力前三个分量（法向与切向）的模长，因此恒为非负；
            它是瞬时量，不区分法向/切向，也不做时间平均。
    """

    body1: str
    body2: str
    geom1: str
    geom2: str
    position: Vector3
    force: float
    penetration_depth: float = 0.0
    normal: Vector3 = (0.0, 0.0, 0.0)

    def __post_init__(self) -> None:
        """校验名称非空、接触点长度为 3 且有限、接触力有限且非负。"""
        names = (self.body1, self.body2, self.geom1, self.geom2)
        if any(not isinstance(name, str) or not name for name in names):
            raise ValueError("contact names must be non-empty strings")
        object.__setattr__(self, "position", _float_tuple(
            self.position, length=3, label="contact position"
        ))
        force = float(self.force)
        if not math.isfinite(force):
            raise ValueError("contact force must be finite")
        if force < 0.0:
            raise ValueError("contact force must be non-negative")
        object.__setattr__(self, "force", force)
        penetration = float(self.penetration_depth)
        if not math.isfinite(penetration) or penetration < 0.0:
            raise ValueError("contact penetration_depth must be finite and non-negative")
        object.__setattr__(self, "penetration_depth", penetration)
        object.__setattr__(self, "normal", _float_tuple(
            self.normal, length=3, label="contact normal"
        ))


@dataclass(frozen=True)
class SavedSimulationState:
    """仿真断点存档（只读）：足以把物理世界与控制器状态恢复到存档时刻。

    存档内容分两部分：MuJoCo 的积分状态（``state``，按 ``state_spec`` 掩码导出，含关节位置速度、
    执行器控制量、求解器热启动等），以及本包控制器自身的内部状态（目标、积分项、已施加力矩、
    控制相位与模式）。两者必须成套恢复，否则“位置目标与实际状态”会错位。

    字段含义：

        model_identity: 源模型对象标识（``id(model)``），用于识别存档是否来自同一个模型实例；
        model_fingerprint: 模型内容指纹；即使换了新实例，指纹一致也说明模型几何/参数未变；
        model_dimensions: 模型维度快照（nq/nv/na/nu/nbody/njnt/ngeom 与状态长度），
            维度不一致时状态数组无法对应，必须拒绝恢复；
        state_spec: 导出状态时使用的 MuJoCo 状态掩码，恢复时必须与当前实例一致；
        state: 按 ``state_spec`` 导出的积分状态数组（长度等于 ``model_dimensions`` 末项）；
        control_targets: 各关节（含手指）的位置目标，手臂 rad、手指 m；
        position_integral: 手臂控制器的位置误差积分项（抗静差），长度等于六个手臂关节；
        velocity_integral: 手臂控制器的速度误差积分项，长度等于六个手臂关节；
        applied_torque: 控制器上一步施加的关节力矩（N·m），用于限制力矩变化率/限幅衔接；
        control_phase: 控制状态机相位（整数枚举），恢复后从同一相位继续；
        control_mode: 控制模式字符串，取值属于对外接口（``gravity_comp``/``hold``/``pos_vel``）。
    """

    model_identity: int
    model_fingerprint: str
    model_dimensions: tuple[int, ...]
    state_spec: int
    state: tuple[float, ...]
    control_targets: tuple[float, ...] = ()
    position_integral: tuple[float, ...] = ()
    velocity_integral: tuple[float, ...] = ()
    applied_torque: tuple[float, ...] = ()
    control_phase: int = 0
    control_mode: str = "pos_vel"

    def __post_init__(self) -> None:
        """规整全部字段：维度转 int、数值向量转有限浮点元组，便于跨进程比较与序列化。"""
        object.__setattr__(self, "model_dimensions", tuple(int(value) for value in self.model_dimensions))
        object.__setattr__(self, "state", _float_tuple(self.state, label="saved state"))
        object.__setattr__(self, "control_targets", _float_tuple(self.control_targets, label="control_targets"))
        object.__setattr__(self, "position_integral", _float_tuple(self.position_integral, label="position_integral"))
        object.__setattr__(self, "velocity_integral", _float_tuple(self.velocity_integral, label="velocity_integral"))
        object.__setattr__(self, "applied_torque", _float_tuple(self.applied_torque, label="applied_torque"))
        object.__setattr__(self, "control_phase", int(self.control_phase))
        object.__setattr__(self, "control_mode", str(self.control_mode))

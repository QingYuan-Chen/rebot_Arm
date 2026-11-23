"""重力参数配置与 Pinocchio 六轴降维动力学模型加载.

职责与位置
    持有重力补偿配置的加载校验与基于 Pinocchio 的六轴广义重力/末端雅可比计算;
    延迟导入 pinocchio,避免未用到动力学模型的使用方引入外部求解器开销.

对外接口
    GravitySettings: 保存,校验并从 YAML 加载补偿与负载参数.
    GravityModel / load_gravity_model: 校验并降维规范 URDF,计算 g(q) 与末端雅可比.
    apply_gravity_compensation_tau_scale: 复制并按已校核比例缩放六轴重力前馈.

安全与并发约束
    GravityModel 内部持有可变 Data 缓存,调用方须由唯一周期串行调用或使用独立实例.
    URDF 六轴位置限位必须与 contracts.POSITION_LIMITS 逐轴一致,不允许悄然放宽.

文件结构(按定义顺序)
    GravitySettings → GravityModel → load_gravity_model → apply_gravity_compensation_tau_scale.
"""

from __future__ import annotations

from dataclasses import dataclass, fields
import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from .contracts import JOINT_NAMES, POSITION_LIMITS, vector
from .mit_command import PROTOCOL_KD_MAX, PROTOCOL_KP_MAX


@dataclass(frozen=True)
class GravitySettings:
    """不可重新赋值的补偿配置;创建时校验,YAML 未提供的字段使用下列默认值.

    积分由 GravityExecutionState.step 更新并限幅,本类只提供参数.
    总力矩上限来自 URDF 的 effort 字段,不在这里另设一套默认值.
    """

    # 六轴共用的 PD 增益:kp 为刚度(N·m/rad),kd 为阻尼(N·m·s/rad).
    kp: float = 7.0
    kd: float = 0.8
    # 积分按实际 dt 累积:增益单位 N·m/(rad·s),积分输出限幅单位 N·m.
    integral_gain: float = 1.0
    integral_limit_nm: float = 0.5
    # 每轴总力矩变化率上限(N·m/s),顺序与 JOINT_NAMES 一致.
    torque_rate_limits_nm_s: tuple = (108.0, 108.0, 108.0, 28.0, 28.0, 28.0)
    # 重力在 base_link 中的向量(m/s²);默认基座水平安装,重力沿 -Z 方向.
    gravity_xyz_m_s2: tuple = (0.0, 0.0, -9.81)
    # URDF 已建模质量之外的额外负载:质量为 kg,质心为 end_link 中的坐标(m).
    payload_mass_kg: float = 0.0
    payload_com_xyz_m: tuple = (0.0, 0.0, 0.0)

    def __post_init__(self) -> None:
        """校验参数数值,MIT 协议增益范围,以及重力和负载的基本物理约定."""
        scalars = (
            self.kp,
            self.kd,
            self.integral_gain,
            self.integral_limit_nm,
            self.payload_mass_kg,
        )
        if not np.isfinite(scalars).all() or min(scalars) < 0 or self.kp <= 0:
            raise ValueError("gravity settings must be finite and nonnegative; kp > 0")
        if self.kp > PROTOCOL_KP_MAX or self.kd > PROTOCOL_KD_MAX:
            raise ValueError(
                f"MIT gains exceed protocol range: kp <= {int(PROTOCOL_KP_MAX)}, "
                f"kd <= {int(PROTOCOL_KD_MAX)}"
            )
        if (vector(self.torque_rate_limits_nm_s, 6, "torque rate") <= 0).any():
            raise ValueError("torque rate limits must be positive")
        # 校验地球重力的模长,允许按实际安装方向配置向量,而不强制沿 -Z.
        gravity = vector(self.gravity_xyz_m_s2, 3, "gravity")
        if not 9.0 < np.linalg.norm(gravity) < 10.0:
            raise ValueError("gravity vector must describe Earth gravity in base_link")
        vector(self.payload_com_xyz_m, 3, "payload COM")

    @classmethod
    def from_yaml(cls, path: str | Path) -> GravitySettings:
        """读取 YAML 顶层字段;拒绝未知键,避免参数拼写错误后静默使用默认值."""
        values = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        if not isinstance(values, dict):
            raise ValueError("gravity settings must be a mapping")
        unknown = set(values) - {f.name for f in fields(cls)}
        if unknown:
            raise ValueError(f"unknown gravity settings: {sorted(unknown)}")
        return cls(**values)


class GravityModel:
    """持有六轴动力学模型及计算缓存,提供重力力矩和末端雅可比.

    由 load_gravity_model() 完成模型校验后构造;path/sha256 用于追溯模型来源,
    effort 保存校验后的 URDF 力矩上限.计算会更新实例内的 data 缓存,调用方应串行使用.
    """

    def __init__(self, pin, model, path: Path, settings: GravitySettings, effort: np.ndarray) -> None:
        """保存已校验模型,独立 Data,来源哈希和参数,供唯一周期串行计算."""
        self.pin, self.model = pin, model
        self.data = model.createData()
        self.frame_id = model.getFrameId("end_link")
        self.effort = effort
        self.path = str(path.resolve())
        self.sha256 = hashlib.sha256(path.read_bytes()).hexdigest()
        self.settings = settings

    def gravity(self, q) -> np.ndarray:
        """按当前六轴关节角计算抵消重力所需的关节力矩 g(q),单位 N·m."""
        q = vector(q, 6, "position")
        # Pinocchio 返回值可能引用内部缓存,复制后交给调用方,避免后续计算覆盖旧结果.
        return self.pin.computeGeneralizedGravity(self.model, self.data, q).copy()

    def jacobian(self, q) -> np.ndarray:
        """返回 end_link 的 6×6 雅可比;前三行为线速度,后三行为角速度.

        LOCAL_WORLD_ALIGNED: 参考点位于末端原点,坐标轴方向与世界坐标系对齐.
        与关节角速度相乘可得到末端空间速度,供分析或拖动判据使用.
        """
        # 先更新关节雅可比和末端位姿,再提取指定参考系下的末端雅可比.
        self.pin.computeJointJacobians(self.model, self.data, q)
        self.pin.updateFramePlacements(self.model, self.data)
        return self.pin.getFrameJacobian(
            self.model, self.data, self.frame_id, self.pin.LOCAL_WORLD_ALIGNED
        ).copy()


def load_gravity_model(path: str | Path, settings: GravitySettings) -> GravityModel:
    """从指定 URDF 构建并校验重力模型,返回 GravityModel.

    按顺序检查六轴关节及限位,力矩上限,降维后的自由度和末端帧,
    再设置重力方向并叠加额外负载.模型不匹配时直接报错,不自动修正坐标或限位.
    """
    # 延迟导入:仅使用配置校验,计时或指令限幅工具时,不需要加载 Pinocchio.
    import pinocchio as pin

    path = Path(path)
    # 1. 直接读取 URDF 标签,核对机械臂关节类型,位置范围和 effort(N·m).
    root = ET.parse(path).getroot()
    joints = {j.get("name"): j for j in root.findall("joint")}
    effort = []
    for name, bounds in zip(JOINT_NAMES, POSITION_LIMITS):
        joint = joints.get(name)
        if joint is None or joint.get("type") != "revolute":
            raise ValueError(f"{name} must be a bounded revolute joint")
        limit = joint.find("limit")
        actual = (float(limit.get("lower")), float(limit.get("upper")))
        if not np.allclose(actual, bounds, atol=1e-9, rtol=0):
            raise ValueError(f"{name} model limit {actual} differs from hardware {bounds}")
        effort.append(float(limit.get("effort")))
    effort = vector(effort, 6, "effort")
    if (effort <= 0).any():
        raise ValueError("model effort must be positive")
    if (effort > np.array([27.0, 27.0, 27.0, 7.0, 7.0, 7.0])).any():
        raise ValueError("model effort exceeds the reviewed arm torque ceilings")
    # 2. 加载完整模型,将夹爪关节锁定在 neutral(中性配置),构建六轴降维模型.
    #    只允许锁定左右手指,不得静默移除机械臂的任何关节自由度.
    full = pin.buildModelFromUrdf(str(path))
    extras = list(full.names)[1:]
    lock_names = [n for n in extras if n not in JOINT_NAMES]
    if set(lock_names) - {"left_finger_joint", "right_finger_joint"}:
        raise ValueError(f"unexpected model joints: {lock_names}")
    model = (
        pin.buildReducedModel(
            full, [full.getJointId(n) for n in lock_names], pin.neutral(full)
        )
        if lock_names
        else full
    )
    # 3. nq/nv 分别为位置,速度维数;二者及关节顺序都必须与六轴反馈对应.
    if model.nq != 6 or model.nv != 6 or tuple(model.names)[1:] != JOINT_NAMES:
        raise ValueError("model must preserve joint1..joint6 order and coordinates")
    if not model.existFrame("end_link"):
        raise ValueError("model is missing end_link")
    # 4. 配置基座坐标系下的重力方向,适配实际安装朝向.
    model.gravity.linear = vector(settings.gravity_xyz_m_s2, 3, "gravity")
    if settings.payload_mass_kg:
        # 5. 将额外负载建模为质心处的点质量,不重复计算 URDF 已包含的夹爪质量.
        frame = model.frames[model.getFrameId("end_link")]
        payload = pin.Inertia(
            settings.payload_mass_kg,
            vector(settings.payload_com_xyz_m, 3, "payload COM"),
            np.zeros((3, 3)),
        )
        # 负载质心在 end_link 中给出,先变换到所属关节坐标系,再叠加到该关节惯量.
        model.inertias[frame.parentJoint] += frame.placement.act(payload)
    return GravityModel(pin, model, path, settings, effort)


# 保留当前已校核的单位补偿增益;迁移不改变各轴前馈.
_GC_TAU_SCALE = np.ones(6, dtype=np.float64)


def apply_gravity_compensation_tau_scale(tau: np.ndarray) -> np.ndarray:
    """对六轴广义重力力矩施加补偿增益,返回独立副本;维数不符明确失败."""
    scaled = np.array(tau, dtype=np.float64, copy=True)
    if scaled.shape != _GC_TAU_SCALE.shape:
        raise ValueError("gravity torque must contain 6 values")
    scaled *= _GC_TAU_SCALE
    return scaled

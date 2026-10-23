"""重力补偿计算工具：加载动力学模型、计算重力力矩，并约束 MIT 控制输出。

职责与位置
    本模块属于 rebotarmcontroller 的计算层，可用于离线测试，不连接电机或启动 ROS 节点。
    HardwareManager 使用本模块加载模型与配置，在硬件控制循环中计算重力补偿输出。
    反馈读取、拖动跟随、位置误差积分、模式切换和电机指令发送由 HardwareManager 负责。

文件结构（按下文定义顺序）
    vector                统一检查向量维数与数值有效性；
    GravitySettings       保存并校验重力补偿参数，支持从 YAML 加载；
    bounded_mit_command   合并 PD 与前馈力矩，限制总力矩及其变化率；
    LoopTiming            记录控制周期和执行耗时，判断是否连续超时；
    GravityModel          封装 Pinocchio 模型、重力力矩与末端雅可比计算；
    load_gravity_model    从 URDF 构建六轴模型，校验关节约定并加入额外负载；
    package_file          定位安装目录或源码工作区中的模型、配置资源。

主要计算关系
    当前关节角 q → GravityModel.gravity(q) → 重力前馈力矩；
    目标角、反馈 q/qd、前馈力矩 → bounded_mit_command() → 待发送的 MIT 参数。
    MIT 指令的 PD（比例-微分）力矩由电机侧计算，因此总力矩限幅也必须计入该项。

统一约定
    六轴向量均按 joint1..joint6 排列；关节角为 rad，角速度为 rad/s，力矩为 N·m，
    时间为 s。重力向量在基座 base_link 中表达，额外负载质心在 end_link 中表达。
    这里约束的是基于当前反馈估算的输出；实际硬件效果仍需单独验证。
"""

from __future__ import annotations

from dataclasses import dataclass, fields
import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml

# 模型必须保留这六个关节的名称、顺序和坐标定义，不能通过偏移反馈来适配模型。
JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 7))
# 模型加载时逐轴核对的硬件位置范围（rad），不是在此对输入关节角做裁剪。
POSITION_LIMITS = ((-2.8, 2.8), (-3.14, .02), (-3.14, .02),
                   (-1.87, 1.57), (-1.57, 1.57), (-3.14, 3.14))


def vector(value, size, label):
    """将输入转换为指定长度的一维浮点向量，拒绝 NaN、无穷大或维数不符。

    label 用于指出出错参数；这里只检查形状和有限性，不检查关节限位或力矩范围。
    """
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f"{label} must contain {size} finite values")
    return result


@dataclass(frozen=True)
class GravitySettings:
    """不可重新赋值的补偿配置；创建时校验，YAML 未提供的字段使用下列默认值。

    积分的更新和限幅由 HardwareManager 执行，本类只提供参数。
    总力矩上限来自 URDF 的 effort 字段，不在这里另设一套默认值。
    """

    # 六轴共用的 PD 增益：kp 为刚度（N·m/rad），kd 为阻尼（N·m·s/rad）。
    kp: float = 7.
    kd: float = .8
    # 积分按实际 dt 累积：增益单位 N·m/(rad·s)，积分输出限幅单位 N·m。
    integral_gain: float = 1.
    integral_limit_nm: float = .5
    # 每轴总力矩变化率上限（N·m/s），顺序与 JOINT_NAMES 一致。
    torque_rate_limits_nm_s: tuple = (108., 108., 108., 28., 28., 28.)
    # 重力在 base_link 中的向量（m/s²）；默认基座水平安装，重力沿 -Z 方向。
    gravity_xyz_m_s2: tuple = (0., 0., -9.81)
    # URDF 已建模质量之外的额外负载：质量为 kg，质心为 end_link 中的坐标（m）。
    payload_mass_kg: float = 0.
    payload_com_xyz_m: tuple = (0., 0., 0.)

    def __post_init__(self):
        """校验参数数值、MIT 协议增益范围，以及重力和负载的基本物理约定。"""
        scalars = (self.kp, self.kd, self.integral_gain, self.integral_limit_nm,
                   self.payload_mass_kg)
        if not np.isfinite(scalars).all() or min(scalars) < 0 or self.kp <= 0:
            raise ValueError("gravity settings must be finite and nonnegative; kp > 0")
        if self.kp > 500 or self.kd > 5:
            raise ValueError('MIT gains exceed protocol range: kp <= 500, kd <= 5')
        if (vector(self.torque_rate_limits_nm_s, 6, 'torque rate') <= 0).any():
            raise ValueError('torque rate limits must be positive')
        # 校验地球重力的模长，允许按实际安装方向配置向量，而不强制沿 -Z。
        gravity = vector(self.gravity_xyz_m_s2, 3, 'gravity')
        if not 9. < np.linalg.norm(gravity) < 10.:
            raise ValueError('gravity vector must describe Earth gravity in base_link')
        vector(self.payload_com_xyz_m, 3, 'payload COM')

    @classmethod
    def from_yaml(cls, path):
        """读取 YAML 顶层字段；拒绝未知键，避免参数拼写错误后静默使用默认值。"""
        values = yaml.safe_load(Path(path).read_text())
        if not isinstance(values, dict):
            raise ValueError('gravity settings must be a mapping')
        unknown = set(values) - {f.name for f in fields(cls)}
        if unknown:
            raise ValueError(f'unknown gravity settings: {sorted(unknown)}')
        return cls(**values)


def bounded_mit_command(target, q, qd, kp, kd, feedforward, limits, rates,
                        previous, dt):
    """计算满足力矩幅值和变化率约束的 MIT 参数，不向电机发送指令。

    输入
        target/q/qd：目标关节角、反馈关节角和角速度；目标角速度固定为零。
        kp/kd：每轴 PD 增益；feedforward：前馈力矩，可包含重力补偿和积分修正。
        limits/rates：每轴力矩幅值上限、变化率上限。
        previous/dt：上一周期的估算总力矩、本周期实测时间间隔。
        除 dt 为标量外，其余输入均为六轴向量。

    返回
        (缩放后的 kp, 缩放后的 kd, 前馈力矩 tau_ff, 本周期估算总力矩)。
        前三项用于构造 MIT 指令，最后一项由调用方保存为下一周期的 previous。

    力矩饱和时按必要程度降低 PD 增益，保证用于抵消 PD 的前馈力矩仍可在限幅内表示。
    约束基于当前 q/qd 计算；两次新鲜反馈之间的实际力矩仍需通过真机验证。
    """
    # 先统一形状，避免 NumPy 广播把错误的输入维数变成看似合法的计算结果。
    target, q, qd, kp, kd, feedforward, limits, rates, previous = [
        vector(v, 6, label) for v, label in zip(
            (target, q, qd, kp, kd, feedforward, limits, rates, previous),
            ('target', 'position', 'velocity', 'kp', 'kd', 'feedforward',
             'torque limits', 'torque rates', 'previous torque'))]
    if not np.isfinite(dt) or dt <= 0 or (limits <= 0).any() or (rates <= 0).any():
        raise ValueError('dt, torque limits and rates must be finite and positive')
    if (kp < 0).any() or (kd < 0).any() or (np.abs(previous) > limits + 1e-9).any():
        raise ValueError('invalid gains or previous torque outside limit')
    # 1. 估算电机侧 PD 力矩：目标速度为零，所以阻尼项为 -kd * qd。
    pd = vector(kp * (target - q) - kd * qd, 6, 'PD torque')
    # 2. 先限制 PD + 前馈的总幅值，再限制相对上一周期的变化量（rates * dt）。
    desired = np.clip(pd + feedforward, -limits, limits)
    total = np.clip(desired, previous - rates * dt, previous + rates * dt)
    # 3. 为前馈中的 PD 抵消项预留余量：scale 在 [0, 1] 内，同时缩放 kp 和 kd。
    #    使 |缩放后 PD| 不超过 limits - |total|，避免 tau_ff = total - PD 超限。
    scale = np.minimum(1., np.maximum(0., limits - np.abs(total)) /
                       np.maximum(np.abs(pd), 1e-12))
    # 当前反馈下，缩放后的 PD 与重新计算的前馈之和仍等于已限幅的 total。
    return kp * scale, kd * scale, total - pd * scale, total


class LoopTiming:
    """控制循环计时器：记录周期间隔、单轮执行耗时及连续超时次数。

    调用方在每轮开始/结束时传入单调时钟时间；本类不休眠、不调度线程，
    也不直接触发硬件保护，只通过 faulted 暴露连续超时判据。
    """

    def __init__(self, rate_hz):
        if not np.isfinite(rate_hz) or rate_hz <= 0:
            raise ValueError('control rate must be finite and positive')
        self.period = 1. / rate_hz
        self.last_start = None
        self.start = None
        self.consecutive_misses = 0
        self.cycles = self.misses = 0
        self.max_period = self.max_execution = 0.
        self.last_dt = self.period

    def begin(self, now):
        """记录本轮开始时间，并返回与上一轮开始时间的间隔 dt（秒）。"""
        # 第一轮没有历史时间，使用配置周期；后续使用实测间隔供积分和变化率限制使用。
        dt = self.period if self.last_start is None else now - self.last_start
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError('control clock must advance monotonically')
        self.last_start = self.start = now
        self.last_dt = dt
        self.max_period = max(self.max_period, dt)
        return dt

    def finish(self, now):
        """记录本轮耗时与超时统计；正常一轮会清零连续超时计数。"""
        execution = now - self.start
        # 执行耗时超过配置周期，或两轮开始间隔超过周期的 1.5 倍，均计为超时。
        missed = execution > self.period or self.last_dt > 1.5 * self.period
        self.cycles += 1
        self.misses += int(missed)
        self.consecutive_misses = self.consecutive_misses + 1 if missed else 0
        self.max_execution = max(self.max_execution, execution)

    @property
    def faulted(self):
        """连续三轮超时即报告异常；具体保持或保护动作由 HardwareManager 决定。"""
        return self.consecutive_misses >= 3

    def snapshot(self):
        """返回供诊断使用的统计快照，其中周期与执行耗时的单位均为秒。"""
        return dict(configured_rate_hz=1. / self.period, cycles=self.cycles,
                    missed_cycles=self.misses, consecutive_misses=self.consecutive_misses,
                    max_period_sec=self.max_period, max_execution_sec=self.max_execution)


class GravityModel:
    """持有六轴动力学模型及计算缓存，提供重力力矩和末端雅可比。

    由 load_gravity_model() 完成模型校验后构造；path/sha256 用于追溯模型来源，
    effort 保存校验后的 URDF 力矩上限。计算会更新实例内的 data 缓存，调用方应串行使用。
    """

    def __init__(self, pin, model, path, settings, effort):
        self.pin, self.model = pin, model
        self.data = model.createData()
        self.frame_id = model.getFrameId('end_link')
        self.effort = effort
        self.path = str(path.resolve())
        self.sha256 = hashlib.sha256(path.read_bytes()).hexdigest()
        self.settings = settings

    def gravity(self, q):
        """按当前六轴关节角计算抵消重力所需的关节力矩 g(q)，单位 N·m。"""
        q = vector(q, 6, 'position')
        # Pinocchio 返回值可能引用内部缓存，复制后交给调用方，避免后续计算覆盖旧结果。
        return self.pin.computeGeneralizedGravity(self.model, self.data, q).copy()

    def jacobian(self, q):
        """返回 end_link 的 6×6 雅可比；前三行为线速度，后三行为角速度。

        LOCAL_WORLD_ALIGNED：参考点位于末端原点，坐标轴方向与世界坐标系对齐。
        与关节角速度相乘可得到末端空间速度，供分析或拖动判据使用。
        """
        # 先更新关节雅可比和末端位姿，再提取指定参考系下的末端雅可比。
        self.pin.computeJointJacobians(self.model, self.data, q)
        self.pin.updateFramePlacements(self.model, self.data)
        return self.pin.getFrameJacobian(self.model, self.data, self.frame_id,
                                        self.pin.LOCAL_WORLD_ALIGNED).copy()


def load_gravity_model(path, settings):
    """从指定 URDF 构建并校验重力模型，返回 GravityModel。

    按顺序检查六轴关节及限位、力矩上限、降维后的自由度和末端帧，
    再设置重力方向并叠加额外负载。模型不匹配时直接报错，不自动修正坐标或限位。
    """
    # 延迟导入：仅使用配置校验、计时或指令限幅工具时，不需要加载 Pinocchio。
    import pinocchio as pin
    path = Path(path)
    # 1. 直接读取 URDF 标签，核对机械臂关节类型、位置范围和 effort（N·m）。
    root = ET.parse(path).getroot()
    joints = {j.get('name'): j for j in root.findall('joint')}
    effort = []
    for name, bounds in zip(JOINT_NAMES, POSITION_LIMITS):
        joint = joints.get(name)
        if joint is None or joint.get('type') != 'revolute':
            raise ValueError(f'{name} must be a bounded revolute joint')
        limit = joint.find('limit')
        actual = (float(limit.get('lower')), float(limit.get('upper')))
        if not np.allclose(actual, bounds, atol=1e-9, rtol=0):
            raise ValueError(f'{name} model limit {actual} differs from hardware {bounds}')
        effort.append(float(limit.get('effort')))
    effort = vector(effort, 6, 'effort')
    if (effort <= 0).any():
        raise ValueError('model effort must be positive')
    if (effort > np.array([27., 27., 27., 7., 7., 7.])).any():
        raise ValueError('model effort exceeds the reviewed arm torque ceilings')
    # 2. 加载完整模型，将夹爪关节锁定在 neutral（中性配置），构建六轴降维模型。
    #    只允许锁定左右手指，不得静默移除机械臂的任何关节自由度。
    full = pin.buildModelFromUrdf(str(path))
    extras = list(full.names)[1:]
    lock_names = [n for n in extras if n not in JOINT_NAMES]
    if set(lock_names) - {'left_finger_joint', 'right_finger_joint'}:
        raise ValueError(f'unexpected model joints: {lock_names}')
    model = pin.buildReducedModel(full, [full.getJointId(n) for n in lock_names],
                                  pin.neutral(full)) if lock_names else full
    # 3. nq/nv 分别为位置、速度维数；二者及关节顺序都必须与六轴反馈对应。
    if model.nq != 6 or model.nv != 6 or tuple(model.names)[1:] != JOINT_NAMES:
        raise ValueError('model must preserve joint1..joint6 order and coordinates')
    if not model.existFrame('end_link'):
        raise ValueError('model is missing end_link')
    # 4. 配置基座坐标系下的重力方向，适配实际安装朝向。
    model.gravity.linear = vector(settings.gravity_xyz_m_s2, 3, 'gravity')
    if settings.payload_mass_kg:
        # 5. 将额外负载建模为质心处的点质量，不重复计算 URDF 已包含的夹爪质量。
        frame = model.frames[model.getFrameId('end_link')]
        payload = pin.Inertia(settings.payload_mass_kg,
                              vector(settings.payload_com_xyz_m, 3, 'payload COM'),
                              np.zeros((3, 3)))
        # 负载质心在 end_link 中给出，先变换到所属关节坐标系，再叠加到该关节惯量。
        model.inertias[frame.parentJoint] += frame.placement.act(payload)
    return GravityModel(pin, model, path, settings, effort)


def package_file(package, relative):
    """定位包内资源：优先查 ROS 安装索引，再查当前源码工作区。

    package 为包名，relative 为包内相对路径；两处都不存在时抛出 FileNotFoundError。
    回退路径按本仓库 src/<package>/ 的布局解析，便于未加载 ROS 环境时进行离线测试。
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory(package)) / relative
        if path.is_file():
            return path
    except (ImportError, LookupError):
        pass
    # __file__ 位于 src/rebotarmcontroller/rebotarmcontroller/，parents[2] 即 src/。
    path = Path(__file__).resolve().parents[2] / package / relative
    if not path.is_file():
        raise FileNotFoundError(f'missing {package}/{relative}')
    return path

"""正式准备服务与离线分析共用的轨迹规范化、检查及数值交接。

文件职责
    在运动层把点约束变成可核验曲线，检查段内限值和完整碰撞采样覆盖。
    本文件不创建 ROS 节点；正式服务通过注入碰撞检查器复用同一准备流程。
对外接口
    PreparationContext 描述会话、目标代次和保持快照；normalize_boundaries
    规范化首末约束；point_to_point 生成静止到静止约束；analyze_trajectory
    返回 (PreparedTrajectory 或 None, 报告)。load_reference_limits 读取现有限值。
安全约束
    新鲜反馈下将起点对齐已发送保持命令，不以实际位置与保持值的误差单独拒绝。
    对齐后的准确曲线仍须完整限值及碰撞检查；不得修改中间导数或原有时间。
    超限、未知或碰撞检查覆盖不足均不能生成候选；不自动拉长时长、裁剪或降级。
    候选不是执行许可：正式服务另外绑定场景/夹爪，后端在所有者事务内复核后接纳。
    observed_at/now 使用同源单调秒；反馈采集和资源预约由执行后端负责。
文件结构
    限值读取 → 准备上下文 → 边界规范化与点对点约束 → 极值/参考分析 → 冻结候选
    → 完整准备流程。报告保留离线分析字段，不能将数值对照当作物理跟踪证据。
"""

from dataclasses import asdict, dataclass, replace
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from numpy.polynomial import polynomial as poly

from .polynomial_trajectory import (
    CALC_ATOL, JOINT_NAMES, PolynomialTrajectory, TrajectoryPoint, build_curve,
    curve_from_payload, ordered_points, vector6,
)
from .trajectory_runtime_limits import load_joint_runtime_limits
from rebotarm_trajectory.polynomial import canonical_names, finite_vector


MAX_AGE = .15


def load_reference_limits(config_directory, joint_names=JOINT_NAMES):
    """直接读取规范 URDF 和现有动态 YAML; 不加载物理或电机库."""
    directory = Path(config_directory)
    root = ET.parse(directory / 'rebotarm.urdf').getroot()
    joints = {joint.get('name'): joint.find('limit') for joint in root.findall('joint')}
    lower = tuple(float(joints[name].get('lower')) for name in joint_names)
    upper = tuple(float(joints[name].get('upper')) for name in joint_names)
    return lower, upper, load_joint_runtime_limits(directory / 'joint_limits.yaml', joint_names)


@dataclass(frozen=True)
class PreparationContext:
    """冻结的六轴保持快照；hold 是已发送命令，feedback 是带观测时刻的实际值。"""

    session: int
    generation: int
    hold: tuple
    feedback: tuple
    observed_at: float
    stationary: bool
    enabled: bool
    busy: bool = False

    def __post_init__(self):
        for name in ('session', 'generation'):
            if type(getattr(self, name)) is not int or getattr(self, name) < 0:
                raise ValueError('nonnegative integer context identity required')
        for name in ('hold', 'feedback'):
            object.__setattr__(self, name, vector6(getattr(self, name)))
        for name in ('stationary', 'enabled', 'busy'):
            if type(getattr(self, name)) is not bool:
                raise ValueError('boolean context evidence required')
        if not math.isfinite(self.observed_at):
            raise ValueError('finite feedback observation time required')

    def require_ready(self, now):
        """检查反馈新鲜度和资源状态；不要求机械臂连续静止窗口。"""
        if not math.isfinite(now) or not -.005 <= now - self.observed_at <= MAX_AGE:
            raise ValueError('fresh feedback required')
        if not self.enabled or self.busy:
            raise ValueError('enabled idle hold required')


def normalize_boundaries(joint_names, points, context, *, now):
    """校验前明确调整起止边界; 其后不得再改点或时间."""
    context.require_ready(now)
    points = list(ordered_points(joint_names, points))
    names = canonical_names(joint_names)
    hold = context.hold
    if names != JOINT_NAMES:
        raise ValueError('execution preparation requires six arm joints')
    first = points[0]
    zeros = (0.,) * len(names)
    if first.time == 0:
        if len(points) == 1 and first.positions != hold:
            raise ValueError('zero-duration position change forbidden')
        # 六轴从已发送保持命令连续起步，残差不设单独门限；随后检查整条规范化曲线。
        points[0] = replace(first, positions=hold)
    else:
        # 首点晚于零时刻时显式补起点；导数字段是否存在仍沿用输入的兼容等级。
        points.insert(0, TrajectoryPoint(0., hold,
            zeros if first.velocities else (), zeros if first.accelerations else ()))
    # 仅将已提供的首末导数归零以连接保持状态，中间点及末点位置均保留。
    for i in {0, len(points) - 1}:
        point = points[i]
        points[i] = replace(point, velocities=zeros if point.velocities else (),
                            accelerations=zeros if point.accelerations else ())
    return tuple(points)


def point_to_point(start, target, duration=.4):
    """完整静止到静止五次约束; 不重定时, 不生成密集位置点."""
    if not math.isfinite(duration) or duration <= 0:
        raise ValueError('positive finite point-to-point duration required')
    start = finite_vector(start)
    target = finite_vector(target, len(start))
    zeros = (0.,)*len(start)
    return (TrajectoryPoint(0., start, zeros, zeros),
            TrajectoryPoint(duration, target, zeros, zeros))


def minimum_point_to_point_duration(start, target, limits):
    """用解析峰值给出最低时长提示; 不自动改变请求."""
    start, target = vector6(start), vector6(target)
    required = 0.
    for name, a, b in zip(JOINT_NAMES, start, target):
        distance, limit = abs(b - a), limits[name]
        # 静止到静止五次曲线的 v/a/jerk 峰值分别按 1/T、1/T²、1/T³ 缩放。
        for peak, bound, power in ((1.875 * distance, limit.max_velocity, 1),
                (10 / math.sqrt(3) * distance, limit.max_acceleration, 2),
                (60 * distance, limit.max_jerk, 3)):
            if bound is not None and distance:
                if bound <= 0:
                    raise ValueError('motion forbidden by zero dynamic limit')
                required = max(required, (peak / bound) ** (1 / power))
    return required


def check_curve(curve, lower, upper, limits):
    """以解析导数的实根检查全部段内极值; 浮点求解失败由调用方记为未知."""
    names = curve.joint_names
    lower, upper = finite_vector(lower, len(names)), finite_vector(upper, len(names))
    if any(a > b for a, b in zip(lower, upper)):
        raise ValueError('invalid position bounds')
    if set(limits) != set(names):
        raise ValueError('all named dynamic limits required')
    if any(getattr(limits[name], 'max_' + kind) is None
           for name in names for kind in ('velocity', 'acceleration', 'jerk')
           if kind != 'jerk' or name in JOINT_NAMES):
        raise ValueError('complete velocity acceleration jerk limits required')
    issues, peaks = [], {}
    for j, name in enumerate(names):
        joint_peaks = {}
        for order, kind in enumerate(('position', 'velocity', 'acceleration', 'jerk')):
            values = [(0., curve.points[0].positions[j] if order == 0 else 0.)]
            for segment in curve.segments:
                values.extend((segment.start + segment.duration * u,
                    float(poly.polyval(u, segment.axis(j, order))))
                    for u in segment.candidates(j, order))
            if not all(math.isfinite(v) for _, v in values):
                raise ValueError('unknown curve extrema: nonfinite value')
            stamp, peak = max(values, key=lambda item: abs(item[1]))
            joint_peaks[kind] = dict(absolute=abs(peak), time=stamp)
            for stamp, value in values:
                if order == 0:
                    violated = value < lower[j] - CALC_ATOL or value > upper[j] + CALC_ATOL
                    bound = (lower[j], upper[j])
                else:
                    bound = getattr(limits[name], 'max_' + kind)
                    violated = bound is not None and abs(value) > bound + CALC_ATOL
                if violated:
                    issues.append(dict(joint=name, kind=kind, time=stamp, value=value, limit=bound))
        peaks[name] = joint_peaks
    # 低阶曲线的段内导数可能合格，但节点跳变不能据此宣称满足更高阶动态限值。
    return dict(state='rejected' if issues else 'pass', issues=issues, peaks=peaks,
                continuity=curve.continuity(),
                derivative_limits_cover_node_jumps=curve.degree == 5)


def compare_reference(curve, source, *, period=.002, max_samples=100_000):
    """对照原有到点更新与连续参考; 时间栅格仅用于分析, 不充当限位或碰撞证明."""
    if not math.isfinite(period) or period <= 0:
        raise ValueError('positive finite analysis period required')
    count = max(1, math.ceil(curve.duration / period))
    if count + 1 > max_samples:
        raise ValueError('analysis time grid exceeds sample budget')
    source = tuple(source)
    index, old = 0, curve.points[0].positions
    staircase, linear, increments = 0., 0., 0.
    previous = curve.sample(0.)[0]
    for i in range(count + 1):
        stamp = curve.duration * i / count
        while index < len(source) and source[index].time <= stamp:
            old = source[index].positions
            index += 1
        reference = curve.sample(stamp)[0]
        staircase = max(staircase, max(abs(a - b) for a, b in zip(reference, old)))
        increments = max(increments, max(abs(a - b) for a, b in zip(reference, previous)))
        previous = reference
        right = min(len(curve.points) - 1, max(1, next((j for j, p in enumerate(curve.points)
                    if p.time >= stamp), len(curve.points) - 1)))
        a, b = curve.points[max(0, right - 1)], curve.points[right]
        ratio = (stamp - a.time) / (b.time - a.time) if b.time > a.time else 0.
        linear = max(linear, max(abs(q - (x + (y - x) * ratio))
                     for q, x, y in zip(reference, a.positions, b.positions)))
    gaps = [b.time - a.time for a, b in zip(source, source[1:])]
    return dict(sample_count=count + 1, max_period=curve.duration / count,
                source_min_gap=min(gaps, default=None), source_max_gap=max(gaps, default=None),
                source_max_joint_increment=max((abs(b - a) for first, last in
                    zip(source, source[1:]) for a, b in zip(first.positions, last.positions)), default=0.),
                max_staircase_deviation=staircase, max_linear_path_deviation=linear,
                max_reference_increment=increments, physical_tracking_evidence=False)


@dataclass(frozen=True)
class PreparedTrajectory:
    """通过数值与注入碰撞检查的不可变候选；后端原子接纳之前不能用于运动。"""

    curve: PolynomialTrajectory
    context: PreparationContext

    def require_current(self, current, *, now):
        """复核六轴上下文；正式接纳还须由后端检查夹爪、模式、场景和资源预约。"""
        current.require_ready(now)
        if ((current.session, current.generation, current.hold)
                != (self.context.session, self.context.generation, self.context.hold)):
            raise ValueError('prepared trajectory superseded')

    def to_payload(self):
        """导出精确系数和冻结上下文；正式服务在此载荷外追加场景与夹爪绑定。"""
        return dict(curve=self.curve.to_payload(), context=asdict(self.context))

    @classmethod
    def from_payload(cls, payload):
        """供运动层离线往返验证；执行端不应调用会重新核对生成规则的运动层实现。"""
        return cls(curve_from_payload(payload['curve']), PreparationContext(**payload['context']))


def analyze_trajectory(joint_names, points, *, lower, upper, limits, context=None,
                       now=0., collision_check=None, include_reference_comparison=True,
                       preserve_points=False):
    """返回候选与可审计报告; 只有完整覆盖的碰撞 pass 才产生候选.

    collision_check(names, positions) 由调用者注入, 应绑定其记录的规划场景与完整
    夹爪状态. 无检查器、异常、超时或样本数不足都不能生成候选.
    include_reference_comparison 仅控制离线阶梯对照统计，不影响极值或碰撞检查。
    """
    source = ordered_points(joint_names, points)
    names = canonical_names(joint_names)
    # 去除首点修正幅度门限后，仍明确拒绝越界输入，不能由起点替换掩盖非法位置。
    if context and any(q < lo-CALC_ATOL or q > hi+CALC_ATOL
            for q,lo,hi in zip(source[0].positions, lower, upper)):
        raise ValueError('first input position outside limits')
    if preserve_points:
        if context is None:
            raise ValueError('bound execution requires a hold context')
        context.require_ready(now)
        if source[0].time != 0 or source[0].positions != context.hold:
            raise ValueError('planned initial hold changed; re-plan')
        if any(v for p in (source[0], source[-1]) for v in (*p.velocities, *p.accelerations)):
            raise ValueError('planned boundary derivatives must already match hold')
        points = source
    else:
        points = normalize_boundaries(names, source, context, now=now) if context else source
    if points[0].time != 0:
        raise ValueError('explicit initial state at time zero required')
    report = dict(source_points=len(source), normalized_points=len(points),
                  normalized=points != source,
                  # 保留历史报告格式；正式服务的执行许可来自后端复核，不来自此标签。
                  scope='offline_reference_only')
    try:
        curve = build_curve(names, points)
        report.update(duration=curve.duration, degree=curve.degree,
                      normalized_trajectory=curve.to_payload())
        # 阶梯误差对照仅供离线分析；正式准备不生成长时域网格，仍全量检查极值和碰撞。
        if include_reference_comparison:
            report['comparison'] = compare_reference(curve, source)
        report['limits'] = check_curve(curve, lower, upper, limits)
        times = curve.collision_times(tuple(.00018 if n == 'left_finger_joint' else .02 for n in names))
    except (ValueError, OverflowError, np.linalg.LinAlgError) as exc:
        report.update(state='unknown', reason=str(exc), collision=dict(state='unknown'))
        return None, report
    if report['limits']['state'] != 'pass':
        report.update(state='rejected', collision=dict(state='not_checked'))
        return None, report
    collision = dict(state='unknown', message='no collision checker provided')
    if collision_check is not None:
        try:
            collision = dict(collision_check(names, [curve.sample(t)[0] for t in times]))
            # 检查器必须证明整个批次都检查过，不能以一部分样本成功代表整条曲线。
            if (collision.get('state') == 'pass' and
                    (collision.get('checked_samples') != len(times)
                     or collision.get('requested_samples') != len(times))):
                collision = dict(state='unknown', message='incomplete collision coverage')
        except Exception as exc:
            collision = dict(state='unknown', message=str(exc))
    report['collision'] = dict(collision, sampling_count=len(times), max_joint_step=.02,
                               continuous_collision_proof=False)
    report['state'] = ('pass' if context is not None and collision.get('state') == 'pass'
                       else 'rejected' if collision.get('state') == 'collision' else 'unknown')
    if context is None:
        report['reason'] = 'no hold context; curve analysis only'
    return (PreparedTrajectory(curve, context) if report['state'] == 'pass' else None), report

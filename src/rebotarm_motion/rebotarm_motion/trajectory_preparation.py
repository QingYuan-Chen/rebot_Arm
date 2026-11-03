"""P005 离线准备与数值交接契约, 尚未接入生产 Action.

生成、限位和碰撞检查位于运动层. 返回值只用于离线实验; 本模块不发命令、
不取得硬件资源. 正式接入还需将当前代次复核与任务接纳放在同一设备事务内.
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


# 与生产静止证据的位置跨度一致, 仅作为已批准的起点修正额度, 不扩大旧准入门限.
START_SPAN = 3 * 25. / 65535
MAX_AGE = .15


def load_reference_limits(config_directory):
    """直接读取规范 URDF 和现有动态 YAML; 不加载物理或电机库."""
    directory = Path(config_directory)
    root = ET.parse(directory / 'rebotarm.urdf').getroot()
    joints = {joint.get('name'): joint.find('limit') for joint in root.findall('joint')}
    lower = tuple(float(joints[name].get('lower')) for name in JOINT_NAMES)
    upper = tuple(float(joints[name].get('upper')) for name in JOINT_NAMES)
    return lower, upper, load_joint_runtime_limits(directory / 'joint_limits.yaml', JOINT_NAMES)


@dataclass(frozen=True)
class PreparationContext:
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
        """静止必须来自既有证据, 不能从单份反馈自行推断."""
        if not math.isfinite(now) or not -.005 <= now - self.observed_at <= MAX_AGE:
            raise ValueError('fresh feedback required')
        if not self.enabled or not self.stationary or self.busy:
            raise ValueError('enabled stationary idle hold required')
        if max(abs(a - b) for a, b in zip(self.hold, self.feedback)) > START_SPAN:
            raise ValueError('feedback outside hold span')


def normalize_boundaries(joint_names, points, context, *, now):
    """校验前明确调整起止边界; 其后不得再改点或时间."""
    context.require_ready(now)
    points = list(ordered_points(joint_names, points))
    first = points[0]
    zeros = (0.,) * 6
    if first.time == 0:
        if len(points) == 1 and first.positions != context.hold:
            raise ValueError('zero-duration position change forbidden')
        if max(abs(a - b) for a, b in zip(first.positions, context.hold)) > START_SPAN:
            raise ValueError('first point outside hold span')
        points[0] = replace(first, positions=context.hold)
    else:
        points.insert(0, TrajectoryPoint(0., context.hold,
            zeros if first.velocities else (), zeros if first.accelerations else ()))
    for i in {0, len(points) - 1}:
        point = points[i]
        points[i] = replace(point, velocities=zeros if point.velocities else (),
                            accelerations=zeros if point.accelerations else ())
    return tuple(points)


def point_to_point(start, target, duration=.4):
    """完整静止到静止五次约束; 不重定时, 不生成密集位置点."""
    if not math.isfinite(duration) or duration <= 0:
        raise ValueError('positive finite point-to-point duration required')
    return (TrajectoryPoint(0., start, (0.,) * 6, (0.,) * 6),
            TrajectoryPoint(duration, target, (0.,) * 6, (0.,) * 6))


def minimum_point_to_point_duration(start, target, limits):
    """用解析峰值给出最低时长提示; 不自动改变请求."""
    start, target = vector6(start), vector6(target)
    required = 0.
    for name, a, b in zip(JOINT_NAMES, start, target):
        distance, limit = abs(b - a), limits[name]
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
    lower, upper = vector6(lower), vector6(upper)
    if any(a > b for a, b in zip(lower, upper)):
        raise ValueError('invalid position bounds')
    if set(limits) != set(JOINT_NAMES):
        raise ValueError('all six dynamic limits required')
    if any(getattr(limits[name], 'max_' + kind) is None
           for name in JOINT_NAMES for kind in ('velocity', 'acceleration', 'jerk')):
        raise ValueError('complete velocity acceleration jerk limits required')
    issues, peaks = [], {}
    for j, name in enumerate(JOINT_NAMES):
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
    """通过全部离线检查的不可变候选; 不是硬件执行许可."""

    curve: PolynomialTrajectory
    context: PreparationContext

    def require_current(self, current, *, now):
        """允许新的新鲜反馈, 但不允许校验期间更换保持命令、会话或代次."""
        current.require_ready(now)
        if ((current.session, current.generation, current.hold)
                != (self.context.session, self.context.generation, self.context.hold)):
            raise ValueError('prepared trajectory superseded')

    def to_payload(self):
        return dict(curve=self.curve.to_payload(), context=asdict(self.context))

    @classmethod
    def from_payload(cls, payload):
        return cls(curve_from_payload(payload['curve']), PreparationContext(**payload['context']))


def analyze_trajectory(joint_names, points, *, lower, upper, limits, context=None,
                       now=0., collision_check=None):
    """返回候选与可审计报告; 只有完整覆盖的碰撞 pass 才产生候选.

    collision_check(names, positions) 由调用者注入, 应绑定其记录的规划场景与完整
    夹爪状态. 无检查器、异常、超时或样本数不足都不能生成候选.
    """
    source = ordered_points(joint_names, points)
    points = normalize_boundaries(JOINT_NAMES, source, context, now=now) if context else source
    if points[0].time != 0:
        raise ValueError('explicit initial state at time zero required')
    report = dict(source_points=len(source), normalized_points=len(points),
                  normalized=points != source,
                  scope='offline_reference_only')
    try:
        curve = build_curve(JOINT_NAMES, points)
        report.update(duration=curve.duration, degree=curve.degree,
                      normalized_trajectory=curve.to_payload())
        report['comparison'] = compare_reference(curve, source)
        report['limits'] = check_curve(curve, lower, upper, limits)
        times = curve.collision_times()
    except (ValueError, OverflowError, np.linalg.LinAlgError) as exc:
        report.update(state='unknown', reason=str(exc), collision=dict(state='unknown'))
        return None, report
    if report['limits']['state'] != 'pass':
        report.update(state='rejected', collision=dict(state='not_checked'))
        return None, report
    collision = dict(state='unknown', message='no collision checker provided')
    if collision_check is not None:
        try:
            collision = dict(collision_check(JOINT_NAMES, [curve.sample(t)[0] for t in times]))
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
        report['reason'] = 'no stationary hold context; curve analysis only'
    return (PreparedTrajectory(curve, context) if report['state'] == 'pass' else None), report

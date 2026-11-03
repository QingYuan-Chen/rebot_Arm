"""六轴分段多项式的数据契约和数值求值内核。

文件职责
    保存不可变轨迹点和系数，校验数值结构，并按外部提供的经过时间求 q/v/a/jerk。
    motion 生成、检查曲线；控制器和仿真复用本文件执行同一组系数。

对外接口
    TrajectoryPoint、PolynomialSegment、PolynomialTrajectory 表示点、段和整条轨迹；
    ordered_points 统一关节顺序；curve_from_payload 解码已准备的曲线，不重新拟合。
    系数按归一化时间 u=(t-start)/duration 的升幂排列，六行依次对应 joint1..joint6。
    位置单位 rad，时间单位 s，返回的导数分别为 rad/s、rad/s²、rad/s³。

安全约束
    本库不读取时钟，不持有执行状态，不访问 ROS、设备或物理引擎。
    数值合法不代表限位、碰撞或执行上下文有效；这些检查由上层各自完成。
    缺失导数用空元组表示，不能补零后声称更高连续性；五次曲线也不承诺 jerk 连续。

文件结构
    常量及向量校验 → 轨迹点与排序 → 分段求值 → 整轨采样/编码 → 系数解码。
"""

from bisect import bisect_right
from dataclasses import dataclass, field
import math

JOINT_NAMES = tuple(f'joint{i}' for i in range(1, 7))
CALC_ATOL = 1e-9


def vector6(values):
    """复制六个有限数值; 返回不可变元组."""
    result = tuple(float(v) for v in values)
    if len(result) != 6 or not all(math.isfinite(v) for v in result):
        raise ValueError('six finite joint values required')
    return result


@dataclass(frozen=True)
class TrajectoryPoint:
    """一个带相对时刻的六轴约束点；输入序列复制为元组，避免调用者事后修改。"""

    time: float
    positions: tuple
    velocities: tuple = ()
    accelerations: tuple = ()

    def __post_init__(self):
        stamp = float(self.time)
        if not math.isfinite(stamp) or stamp < 0:
            raise ValueError('finite nonnegative trajectory time required')
        object.__setattr__(self, 'time', stamp)
        object.__setattr__(self, 'positions', vector6(self.positions))
        for name in ('velocities', 'accelerations'):
            values = tuple(getattr(self, name))
            object.__setattr__(self, name, vector6(values) if values else ())


def ordered_points(joint_names, points):
    """按规范顺序复制所有位置及导数; 混合缺失不能默默降级."""
    names = tuple(joint_names)
    if len(names) != 6 or set(names) != set(JOINT_NAMES):
        raise ValueError('exactly joint1..joint6 required')
    source = tuple(points)
    if not source:
        raise ValueError('trajectory points required')
    order = tuple(names.index(n) for n in JOINT_NAMES)
    result = tuple(TrajectoryPoint(p.time, tuple(p.positions[i] for i in order),
        tuple(p.velocities[i] for i in order) if p.velocities else (),
        tuple(p.accelerations[i] for i in order) if p.accelerations else ()) for p in source)
    if any(b.time <= a.time for a, b in zip(result, result[1:])):
        raise ValueError('trajectory times must strictly increase')
    # 全轨迹必须采用同一约束等级；部分节点缺导数时不能静默降级或自行补零。
    for field in ('velocities', 'accelerations'):
        present = [bool(getattr(p, field)) for p in result]
        if any(present) and not all(present):
            raise ValueError(f'mixed missing {field}')
    if result[0].accelerations and not result[0].velocities:
        raise ValueError('accelerations require velocities')
    return result


@dataclass(frozen=True)
class PolynomialSegment:
    """六轴共用一个时间区间；每行 2/4/6 个升幂系数分别表示线性/三次/五次。"""

    start: float
    duration: float
    coefficients: tuple

    def __post_init__(self):
        start, duration = float(self.start), float(self.duration)
        coefficients = tuple(tuple(float(v) for v in row) for row in self.coefficients)
        if (not math.isfinite(start) or start < 0 or not math.isfinite(duration) or duration <= 0
                or len(coefficients) != 6 or any(len(row) not in (2, 4, 6) for row in coefficients)
                or len({len(row) for row in coefficients}) != 1
                or not all(math.isfinite(v) for row in coefficients for v in row)):
            raise ValueError('invalid finite polynomial segment')
        object.__setattr__(self, 'start', start)
        object.__setattr__(self, 'duration', duration)
        object.__setattr__(self, 'coefficients', coefficients)

    def axis(self, joint, order=0):
        """返回实际秒制导数系数；求根和极值判定由运动层负责。"""
        c = tuple(self.coefficients[joint])
        for _ in range(order):
            c = tuple(i * value for i, value in enumerate(c) if i) or (0.,)
        # 对 u 求导后还需除以 T**order，才能得到相对于实际秒的导数。
        try:
            values = tuple(value / self.duration ** order for value in c)
        except (OverflowError, ZeroDivisionError) as exc:
            raise ValueError('unknown polynomial derivatives: numerical overflow') from exc
        if not all(math.isfinite(value) for value in values):
            raise ValueError('unknown polynomial derivatives: numerical overflow')
        return values

    def at(self, u, order=0):
        """在归一化时刻 u 用 Horner 法求六轴值；区间选择和钳位由整轨采样负责。"""
        values = []
        for joint in range(6):
            value = 0.
            for coefficient in reversed(self.axis(joint, order)):
                value = coefficient + u * value
            values.append(value)
        return vector6(values)


@dataclass(frozen=True)
class PolynomialTrajectory:
    """点约束和分段系数的冻结容器；构造检查结构，交换数据还须经过解码校验。"""

    points: tuple
    segments: tuple
    degree: int
    times: tuple = field(init=False, repr=False)
    positions: tuple = field(init=False, repr=False)

    def __post_init__(self):
        points = ordered_points(JOINT_NAMES, self.points)
        segments = tuple(self.segments)
        if any(not isinstance(segment, PolynomialSegment) for segment in segments):
            raise ValueError('immutable polynomial segments required')
        degree = 5 if points[0].accelerations else 3 if points[0].velocities else 1
        if points[0].time != 0. or self.degree != degree or len(segments) != len(points)-1:
            raise ValueError('invalid polynomial trajectory structure')
        for first, last, segment in zip(points, points[1:], segments):
            if (segment.start != first.time or segment.duration != last.time-first.time
                    or len(segment.coefficients[0]) != degree+1):
                raise ValueError('invalid polynomial segment boundaries')
        object.__setattr__(self, 'points', points)
        object.__setattr__(self, 'segments', segments)
        object.__setattr__(self, 'times', tuple(p.time for p in points))
        object.__setattr__(self, 'positions', tuple(p.positions for p in points))

    @property
    def duration(self):
        return self.points[-1].time

    def sample(self, elapsed):
        """按实际经过时间求 q/v/a/j; 区间外保持端点, 不外推."""
        elapsed = float(elapsed)
        if not math.isfinite(elapsed):
            raise ValueError('finite sample time required')
        if elapsed < 0 or elapsed > self.duration or not self.segments:
            point = self.points[0] if elapsed <= 0 else self.points[-1]
            return (point.positions, (0.,) * 6, (0.,) * 6, (0.,) * 6)
        # 内部节点取右段，终点取最后一段；C0/C1 的高阶导数跳变不会被平滑隐藏。
        index = min(len(self.segments) - 1,
                    bisect_right(self.times, elapsed) - 1)
        segment = self.segments[index]
        u = min(1., max(0., (elapsed - segment.start) / segment.duration))
        return tuple(segment.at(u, k) for k in range(4))

    def to_payload(self):
        """编码版本化数值载荷；执行端采样携带的系数，避免重新拟合造成检查与执行不一致。"""
        return {'version': 1, 'joint_names': list(JOINT_NAMES), 'degree': self.degree,
                'points': [dict(time_from_start=p.time, positions=p.positions,
                    velocities=p.velocities, accelerations=p.accelerations) for p in self.points],
                'coefficients': [s.coefficients for s in self.segments]}


def curve_from_payload(payload):
    """校验版本、形状、有限性及端点约束并载入系数；不替代上层限位和碰撞检查。"""
    if payload['version'] != 1 or tuple(payload['joint_names']) != JOINT_NAMES:
        raise ValueError('unsupported curve version or joint order')
    points = ordered_points(JOINT_NAMES, tuple(TrajectoryPoint(
        p['time_from_start'], p['positions'], p.get('velocities', ()),
        p.get('accelerations', ())) for p in payload['points']))
    degree = 5 if points[0].accelerations else 3 if points[0].velocities else 1
    if points[0].time != 0 or payload['degree'] != degree:
        raise ValueError('curve metadata mismatch')
    rows = payload['coefficients']
    if len(rows) != len(points) - 1:
        raise ValueError('curve segment count mismatch')
    if len(points) == 1 and any((*points[0].velocities, *points[0].accelerations)):
        raise ValueError('zero-duration hold requires zero derivatives')
    segments = []
    for first, last, coefficients in zip(points, points[1:], rows):
        c = tuple(tuple(float(value) for value in row) for row in coefficients)
        if (len(c) != 6 or any(len(row) != degree + 1 for row in c)
                or not all(math.isfinite(value) for row in c for value in row)):
            raise ValueError('curve coefficient shape or finite values invalid')
        duration = last.time - first.time
        if not math.isfinite(duration) or duration > 1e100:
            raise ValueError('unknown polynomial duration: numerical range')
        segment = PolynomialSegment(first.time, duration, c)
        for order in range(4):
            segment.at(0., order)
            segment.at(1., order)
        # 只核对源轨迹实际声明的导数；解码不能改变兼容等级或修复不一致的载荷。
        for order, field in enumerate(('positions', 'velocities', 'accelerations')):
            if order and not getattr(first, field):
                continue
            for u, point in ((0., first), (1., last)):
                if max(abs(a - b) for a, b in zip(segment.at(u, order),
                                                  getattr(point, field))) > CALC_ATOL:
                    raise ValueError('curve coefficient endpoints mismatch')
        segments.append(segment)
    return PolynomialTrajectory(points, tuple(segments), degree)

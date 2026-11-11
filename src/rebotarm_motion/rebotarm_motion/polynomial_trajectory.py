"""运动层的分级曲线生成、解析极值和碰撞采样安排。

文件职责
    根据完整点约束生成线性、三次 Hermite 或五次 Hermite 系数，并为限值和碰撞检查
    提供候选时刻。数值求值继承共享轨迹内核，不在运动层维护第二套采样算法。
对外接口
    build_curve 构造曲线；PolynomialSegment.candidates 给出端点与段内极值候选；
    PolynomialTrajectory 提供连续性报告及 collision_times；curve_from_payload
    用于运动层往返核对生成规则，执行后端使用共享内核的直接解码接口。
安全约束
    q/qv/qva 分别保留 C0/C1/C2 等级；不补造缺失导数、不重定时、不裁剪超限值。
    系数以 u=(t-t0)/T 升幂排列，导数按实际秒缩放。求根数值不可判定时明确失败。
    碰撞时刻控制相邻样本的关节跨度，但离散采样不是连续碰撞证明；超预算直接拒绝。
文件结构
    实根求解 → 极值候选 → 连续性与碰撞采样 → 系数生成 → 交换载荷一致性检查。
"""

import math

import numpy as np
from numpy.polynomial import polynomial as poly


from rebotarm_trajectory.polynomial import (
    CALC_ATOL, JOINT_NAMES, TrajectoryPoint, ordered_points, vector6, canonical_names,
    PolynomialSegment as NumericSegment, PolynomialTrajectory as NumericTrajectory,
)


def real_roots(coefficients):
    """求 [0,1] 内导数根；非有限、求解失败及不可用残差均拒绝判定。"""
    c = np.trim_zeros(np.asarray(coefficients, dtype=float), 'b')
    if not np.isfinite(c).all():
        raise ValueError('unknown polynomial roots: nonfinite coefficients')
    if len(c) <= 1:
        return ()
    c = c / np.max(np.abs(c))
    try:
        roots = poly.polyroots(c)
    except (ValueError, np.linalg.LinAlgError) as exc:
        raise ValueError('unknown polynomial roots: solver failed') from exc
    if not np.isfinite(roots).all():
        raise ValueError('unknown polynomial roots: nonfinite roots')
    result = []
    for root in roots:
        if abs(root.imag) <= 1e-7 and -1e-9 <= root.real <= 1 + 1e-9:
            u = min(1., max(0., float(root.real)))
            if abs(poly.polyval(u, c)) > 1e-7:
                raise ValueError('unknown polynomial roots: unstable residual')
            result.append(u)
    return tuple(sorted(set(result)))


class PolynomialSegment(NumericSegment):
    """运动层追加解析极值候选；数值求值直接复用共享内核。"""

    def candidates(self, joint, order=0):
        """当前 order 阶量的极值在端点或其下一阶导数零点，返回归一化时间 u。"""
        return (0., *real_roots(poly.polyder(self.axis(joint, order))), 1.)


class PolynomialTrajectory(NumericTrajectory):
    """运动层拥有连续性报告和碰撞采样安排。"""

    def continuity(self):
        """报告首尾保持及内部节点左右导数差；低阶兼容不伪称 C2。"""
        gaps = [0.] * 3
        if self.segments:
            # 直接比较解析左右极限，不用靠近节点的差分近似。
            for k in range(3):
                pairs = [(a.at(1., k), b.at(0., k))
                         for a, b in zip(self.segments, self.segments[1:])]
                if k:
                    pairs += [((0.,) * len(self.joint_names), self.segments[0].at(0., k)),
                              (self.segments[-1].at(1., k), (0.,) * len(self.joint_names))]
                gaps[k] = max((abs(a - b) for left, right in pairs
                               for a, b in zip(left, right)), default=0.)
        return {'guaranteed_order': {1: 0, 3: 1, 5: 2}[self.degree],
                'max_boundary_gaps': dict(zip(('position', 'velocity', 'acceleration'), gaps)),
                'c2_with_hold': self.degree == 5 and max(gaps) <= CALC_ATOL}

    def collision_times(self, max_joint_step=.02, max_samples=100_000):
        """用段内最大速度给空间步长上界，同时包含位置极值；不按端点距离漏掉回转。"""
        steps = ((float(max_joint_step),) * len(self.joint_names)
                 if isinstance(max_joint_step, (int, float)) else tuple(max_joint_step))
        if len(steps) != len(self.joint_names) or any(not math.isfinite(v) or v <= 0 for v in steps):
            raise ValueError('positive finite per-joint collision steps required')
        times = {0., self.duration}
        for segment in self.segments:
            speed = max(abs(poly.polyval(u, segment.axis(j, 1)))/steps[j] for j in range(len(steps))
                        for u in segment.candidates(j, 1))
            # 使用段内速度峰值而非首末距离；即使关节先去后回，也不会漏掉中间行程。
            count = max(1, math.ceil(speed * segment.duration))
            if count + len(times) > max_samples:
                raise ValueError('collision coverage exceeds sample budget')
            times.update(segment.start + segment.duration * i / count for i in range(count + 1))
            times.update(segment.start + segment.duration * u for j in range(len(steps))
                         for u in segment.candidates(j))
        if len(times) > max_samples:
            raise ValueError('collision coverage exceeds sample budget')
        return tuple(sorted(times))



def build_curve(joint_names, points):
    """从全部端点约束构造 1/3/5 次多项式；不改变时间或中间点。"""
    names = canonical_names(joint_names)
    points = ordered_points(joint_names, points)
    if points[0].time != 0:
        raise ValueError('explicit initial state at time zero required')
    if len(points) == 1 and any(v != 0. for v in
            (*points[0].velocities, *points[0].accelerations)):
        raise ValueError('zero-duration hold requires zero derivatives')
    degree = 5 if points[0].accelerations else 3 if points[0].velocities else 1
    segments = []
    for first, last in zip(points, points[1:]):
        duration = last.time - first.time
        if not math.isfinite(duration) or duration > 1e100:
            raise ValueError('unknown polynomial duration: numerical range')
        coefficients = []
        for j in range(len(names)):
            q, delta = first.positions[j], last.positions[j] - first.positions[j]
            if degree == 1:
                c = (q, delta)
            elif degree == 3:
                # 输入 v/a 是秒制约束，先乘 T/T² 转成对 u 的约束，再解端点方程。
                v0, v1 = first.velocities[j] * duration, last.velocities[j] * duration
                c = (q, v0, 3 * delta - 2 * v0 - v1, -2 * delta + v0 + v1)
            else:
                v0, v1 = first.velocities[j] * duration, last.velocities[j] * duration
                a0, a1 = first.accelerations[j] * duration**2, last.accelerations[j] * duration**2
                d, v, a = delta - v0 - a0 / 2, v1 - v0 - a0, a1 - a0
                c = (q, v0, a0 / 2, 10 * d - 4 * v + a / 2,
                     -15 * d + 7 * v - a, 6 * d - 3 * v + a / 2)
            coefficients.append(c)
        segment = PolynomialSegment(first.time, duration, tuple(coefficients))
        for order in range(4):
            segment.at(0., order)
            segment.at(1., order)
        # 生成后用同一求值器核对端点，拒绝浮点范围或残差已使约束失真的系数。
        for order, field in enumerate(('positions', 'velocities', 'accelerations')):
            if order and not getattr(first, field):
                continue
            for u, point in ((0., first), (1., last)):
                if not np.allclose(segment.at(u, order), getattr(point, field),
                                   rtol=0, atol=CALC_ATOL):
                    raise ValueError('unknown polynomial endpoints: numerical residual')
        segments.append(segment)
    return PolynomialTrajectory(points, tuple(segments), degree, names)


def curve_from_payload(payload):
    """校验交换数据与端点契约；系数必须与当前版本的生成规则精确一致。"""
    if payload['version'] not in (1, 2):
        raise ValueError('unsupported curve version')
    points = tuple(TrajectoryPoint(p['time_from_start'], p['positions'],
        p.get('velocities', ()), p.get('accelerations', ())) for p in payload['points'])
    curve = build_curve(payload['joint_names'], points)
    if payload['degree'] != curve.degree or len(payload['coefficients']) != len(curve.segments):
        raise ValueError('curve metadata mismatch')
    received = []
    for segment, coefficients in zip(curve.segments, payload['coefficients']):
        c = np.asarray(coefficients, dtype=float)
        if c.shape != (len(curve.joint_names), curve.degree + 1) or not np.array_equal(c, segment.coefficients):
            raise ValueError('curve coefficient mismatch')
        received.append(PolynomialSegment(segment.start, segment.duration,
                                          tuple(tuple(float(v) for v in row) for row in c)))
    return PolynomialTrajectory(curve.points, tuple(received), curve.degree, curve.joint_names)

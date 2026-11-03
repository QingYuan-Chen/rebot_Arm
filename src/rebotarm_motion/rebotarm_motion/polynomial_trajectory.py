"""离线轨迹参考: 完整点约束生成分段系数, 采样和极值均使用同一组系数.

只依赖数值库, 不创建 ROS 节点或设备. 系数按归一化段时间 u=(t-t0)/T
升幂排列, 位置 rad, 时间 s. 准备服务生成并检查, 执行端使用共享内核采样.
"""

import math

import numpy as np
from numpy.polynomial import polynomial as poly


from rebotarm_trajectory.polynomial import (
    CALC_ATOL, JOINT_NAMES, TrajectoryPoint, ordered_points, vector6,
    PolynomialSegment as NumericSegment, PolynomialTrajectory as NumericTrajectory,
)


def real_roots(coefficients):
    """求 [0,1] 内导数根; 非有限、求解失败及不可用残差均拒绝判定."""
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
        return (0., *real_roots(poly.polyder(self.axis(joint, order))), 1.)


class PolynomialTrajectory(NumericTrajectory):
    """运动层拥有连续性报告和碰撞采样安排。"""

    def continuity(self):
        """报告首尾保持及内部节点左右导数差; 低阶兼容不伪称 C2."""
        gaps = [0.] * 3
        if self.segments:
            # 直接比较解析左右极限, 不用靠近节点的差分近似.
            for k in range(3):
                pairs = [(a.at(1., k), b.at(0., k))
                         for a, b in zip(self.segments, self.segments[1:])]
                if k:
                    pairs += [((0.,) * 6, self.segments[0].at(0., k)),
                              (self.segments[-1].at(1., k), (0.,) * 6)]
                gaps[k] = max((abs(a - b) for left, right in pairs
                               for a, b in zip(left, right)), default=0.)
        return {'guaranteed_order': {1: 0, 3: 1, 5: 2}[self.degree],
                'max_boundary_gaps': dict(zip(('position', 'velocity', 'acceleration'), gaps)),
                'c2_with_hold': self.degree == 5 and max(gaps) <= CALC_ATOL}

    def collision_times(self, max_joint_step=.02, max_samples=100_000):
        """用段内最大速度给空间步长上界, 同时包含位置极值; 不按端点距离漏掉回转."""
        if not math.isfinite(max_joint_step) or max_joint_step <= 0:
            raise ValueError('positive finite collision joint step required')
        times = {0., self.duration}
        for segment in self.segments:
            speed = max(abs(poly.polyval(u, segment.axis(j, 1))) for j in range(6)
                        for u in segment.candidates(j, 1))
            count = max(1, math.ceil(speed * segment.duration / max_joint_step))
            if count + len(times) > max_samples:
                raise ValueError('collision coverage exceeds sample budget')
            times.update(segment.start + segment.duration * i / count for i in range(count + 1))
            times.update(segment.start + segment.duration * u for j in range(6)
                         for u in segment.candidates(j))
        if len(times) > max_samples:
            raise ValueError('collision coverage exceeds sample budget')
        return tuple(sorted(times))



def build_curve(joint_names, points):
    """从全部端点约束构造 1/3/5 次多项式; 不改变时间或中间点."""
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
        for j in range(6):
            q, delta = first.positions[j], last.positions[j] - first.positions[j]
            if degree == 1:
                c = (q, delta)
            elif degree == 3:
                v0, v1 = first.velocities[j] * duration, last.velocities[j] * duration
                c = (q, v0, 3 * delta - 2 * v0 - v1, -2 * delta + v0 + v1)
            else:
                v0, v1 = first.velocities[j] * duration, last.velocities[j] * duration
                a0, a1 = first.accelerations[j] * duration**2, last.accelerations[j] * duration**2
                d, v, a = delta - v0 - a0 / 2, v1 - v0 - a0, a1 - a0
                c = (q, v0, a0 / 2, 10 * d - 4 * v + a / 2,
                     -15 * d + 7 * v - a, 6 * d - 3 * v + a / 2)
            if not all(math.isfinite(value) for value in c):
                raise ValueError('unknown polynomial coefficients: numerical overflow')
            coefficients.append(c)
        segment = PolynomialSegment(first.time, duration, tuple(coefficients))
        for order in range(4):
            segment.at(0., order)
            segment.at(1., order)
        for order, field in enumerate(('positions', 'velocities', 'accelerations')):
            if order and not getattr(first, field):
                continue
            for u, point in ((0., first), (1., last)):
                if not np.allclose(segment.at(u, order), getattr(point, field),
                                   rtol=0, atol=CALC_ATOL):
                    raise ValueError('unknown polynomial endpoints: numerical residual')
        segments.append(segment)
    return PolynomialTrajectory(points, tuple(segments), degree)


def curve_from_payload(payload):
    """校验交换数据与端点契约; 系数必须与当前版本的生成规则精确一致."""
    if payload['version'] != 1:
        raise ValueError('unsupported curve version')
    points = tuple(TrajectoryPoint(p['time_from_start'], p['positions'],
        p.get('velocities', ()), p.get('accelerations', ())) for p in payload['points'])
    curve = build_curve(payload['joint_names'], points)
    if payload['degree'] != curve.degree or len(payload['coefficients']) != len(curve.segments):
        raise ValueError('curve metadata mismatch')
    received = []
    for segment, coefficients in zip(curve.segments, payload['coefficients']):
        c = np.asarray(coefficients, dtype=float)
        if c.shape != (6, curve.degree + 1) or not np.array_equal(c, segment.coefficients):
            raise ValueError('curve coefficient mismatch')
        received.append(PolynomialSegment(segment.start, segment.duration,
                                          tuple(tuple(float(v) for v in row) for row in c)))
    return PolynomialTrajectory(curve.points, tuple(received), curve.degree)

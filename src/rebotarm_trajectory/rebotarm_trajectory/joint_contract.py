"""机械臂与镜像夹爪的纯数值边界契约。

文件职责：规范 FJT 允许的机构集合、镜像关系及具名曲线投影。
对外接口：canonical_input 返回规范点；ArmCurveView 向六轴执行器提供原系数的只读投影。
安全约束：不补齐任意缺轴，不改变时间或导数；不证明限位、碰撞或设备上下文有效。
文件结构：机构名校验 → 从动关节验证 → 冻结六轴曲线视图。
"""

from dataclasses import dataclass
from .polynomial import JOINT_NAMES, COORDINATED_JOINT_NAMES, TrajectoryPoint, ordered_points


def canonical_input(names, points, hold):
    """去除已核验的右指；单夹爪请求只补入冻结的六轴保持，不补造夹爪导数。"""
    names, points = tuple(names), tuple(points)
    if len(names) != len(set(names)) or not points:
        raise ValueError('unique named joints and points required')
    allowed = (set(JOINT_NAMES), set(COORDINATED_JOINT_NAMES),
               {'left_finger_joint'}, {'left_finger_joint', 'right_finger_joint'},
               {*COORDINATED_JOINT_NAMES, 'right_finger_joint'})
    if set(names) not in allowed:
        raise ValueError('unsupported trajectory joint set')
    if any(len(p.positions) != len(names) for p in points):
        raise ValueError('trajectory point dimension differs from joint names')
    if 'right_finger_joint' in names:
        left, right = names.index('left_finger_joint'), names.index('right_finger_joint')
        for p in points:
            for field in ('positions', 'velocities', 'accelerations'):
                values = getattr(p, field)
                if values and (len(values) != len(names) or abs(values[left]+values[right]) > 1e-9):
                    raise ValueError('right finger must mirror left finger including derivatives')
    target = COORDINATED_JOINT_NAMES if 'left_finger_joint' in names else JOINT_NAMES
    result = []
    for p in points:
        fields = []
        for field in ('positions', 'velocities', 'accelerations'):
            values = getattr(p, field)
            if not values:
                fields.append(())
                continue
            fields.append(tuple(values[names.index(n)] if n in names
                else hold[JOINT_NAMES.index(n)] if field == 'positions' else 0. for n in target))
        result.append(TrajectoryPoint(p.time, *fields))
    return target, ordered_points(target, result)


@dataclass(frozen=True)
class ArmCurveView:
    """同一条已检查曲线的六轴投影；不拟合、不生成另一条时间轴。"""

    curve: object

    @property
    def points(self):
        return tuple(TrajectoryPoint(p.time, p.positions[:6], p.velocities[:6],
            p.accelerations[:6]) for p in self.curve.points)

    @property
    def positions(self):
        return tuple(p[:6] for p in self.curve.positions)

    @property
    def times(self):
        return self.curve.times

    @property
    def duration(self):
        return self.curve.duration

    def sample(self, elapsed):
        return tuple(values[:6] for values in self.curve.sample(elapsed))

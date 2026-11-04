"""协同轨道合并与规划阶段统一重定时。

文件职责：将独立关键点轨道合为相同时间轴，解析检查并对全轨一次缩放。
对外接口：merge_tracks、retime_curve 返回可审阅的最终数值曲线与缩放倍数。
安全约束：位置与相对时序不改变；固定时长拒绝超限；纳秒编码后重新检查。
    这里不接纳运动，不读取设备，不替代执行准备中的上下文与碰撞核验。
文件结构：轨道合并 → 边界规范化 → 重定时与最终复核。
"""

from dataclasses import replace
import math
from .polynomial_trajectory import build_curve, TrajectoryPoint, JOINT_NAMES
from .trajectory_preparation import check_curve
from rebotarm_trajectory.polynomial import COORDINATED_JOINT_NAMES
from rebotarm_trajectory.capability import decode_capability, constraint_revision


def effective_limits(config_directory, names, capability):
    """规划与准备共用实际限速；不把 URDF 的理论速度当作夹爪设备输出能力。"""
    from .trajectory_preparation import load_reference_limits
    low, high, limits = load_reference_limits(config_directory, names)
    capability = decode_capability(capability)
    if capability['mode'] != 'prepared_v1':
        raise ValueError('prepared_v1 backend capability required')
    from pathlib import Path
    directory = Path(config_directory)
    revision = constraint_revision(*(directory.joinpath(name).read_bytes()
        for name in ('rebotarm.urdf','joint_limits.yaml')))
    if capability['limits_revision'] != revision:
        raise ValueError('planning/backend model or dynamic limits revision mismatch')
    if 'left_finger_joint' in limits:
        limit = limits['left_finger_joint']
        limits['left_finger_joint'] = replace(limit,
            max_velocity=min(limit.max_velocity, capability['finger_speed_m_s']))
    return low, high, limits


def merge_tracks(arm, finger):
    """在所有节点精确取原轨道 q/v/a；新增节点不会让另一条轨道停顿。"""
    if arm.joint_names != JOINT_NAMES or finger.joint_names != ('left_finger_joint',):
        raise ValueError('arm and independent finger tracks required')
    if arm.degree != 5 or finger.degree != 5:
        raise ValueError('complete q/v/a tracks required for generated coordination')
    stamps = sorted(set((*arm.times, *finger.times)))
    points = []
    for stamp in stamps:
        a, g = arm.sample(stamp), finger.sample(stamp)
        points.append(TrajectoryPoint(stamp, *(a[k]+g[k] for k in range(3))))
    return build_curve(COORDINATED_JOINT_NAMES, points)


def generate_schedule(start, start_width, target, target_width, duration, *, arm_keyframes=None,
                      gripper_keyframes=None):
    """生成两条显式位置轨道；界面的开口为双指总开口，内部只保存左指米制坐标。

    新关键点默认是声明的静止关键点，可显式给出 v/a 通过点；这不是修补旧 FJT 缺失导数。
    末关键点早于全轨终点时补恒定停留，不把另一条轨道的新节点当作停止指令。
    """
    if not math.isfinite(duration) or not 0 < duration <= 300.:
        raise ValueError('schedule duration must be in (0,300] s')
    def track(names, initial, final, keyframes, finger=False):
        zeros = (0.,)*len(names)
        points = [TrajectoryPoint(0., initial, zeros, zeros)]
        records = keyframes if keyframes is not None else [dict(time=duration,positions=final)]
        if not isinstance(records,list) or not 0 < len(records) <= 1000:
            raise ValueError('one to 1000 explicit keyframes required')
        for item in records:
            position = (float(item['width'])/2.,) if finger and 'width' in item else tuple(item['positions'])
            points.append(TrajectoryPoint(float(item['time']),position,
                tuple(item.get('velocities',zeros)),tuple(item.get('accelerations',zeros))))
        if points[-1].time > duration:
            raise ValueError('keyframe after schedule end')
        if points[-1].time < duration:
            if any((*points[-1].velocities,*points[-1].accelerations)):
                raise ValueError('constant dwell requires zero endpoint derivatives')
            points.append(TrajectoryPoint(duration,points[-1].positions,zeros,zeros))
        return build_curve(names,points)
    arm = track(JOINT_NAMES,start,target,arm_keyframes)
    finger = track(('left_finger_joint',),(float(start_width)/2.,),(float(target_width)/2.,),gripper_keyframes,True)
    return merge_tracks(arm,finger)


def curve_message(curve, *, trajectory_factory, point_factory):
    """统一编码纳秒点消息；规划重定时服务随后重建复核，执行端只接纳检查后的系数。"""
    result = trajectory_factory()
    result.joint_names = list(curve.joint_names)
    for p in curve.points:
        point = point_factory()
        point.positions,point.velocities,point.accelerations = list(p.positions),list(p.velocities),list(p.accelerations)
        point.time_from_start.sec,point.time_from_start.nanosec = divmod(round(p.time*1e9),1_000_000_000)
        result.points.append(point)
    return result


def sampled_c2_points(times, positions):
    """对新采样记录生成明确的 q/v/a 约束；旧 FJT 仍按原字段等级处理。

    导数来自非均匀源时间上的中心差分，首末静止连接；最终五次段的极值由重定时逐段复核。
    不把记录中的缺失导数直接填成中间零值，也不声称离散测量本身就是 C2 曲线。
    """
    import numpy as np
    stamps = np.asarray(times,dtype=float)
    values = np.asarray(positions,dtype=float)
    if (len(stamps) < 2 or len(stamps) != len(values) or not np.isfinite(stamps).all()
            or not np.isfinite(values).all() or np.any(np.diff(stamps) <= 0)):
        raise ValueError('finite strictly increasing recorded source time required')
    velocity = np.gradient(values,stamps,axis=0,edge_order=2 if len(stamps) > 2 else 1)
    acceleration = np.gradient(velocity,stamps,axis=0,edge_order=2 if len(stamps) > 2 else 1)
    velocity[[0,-1]],acceleration[[0,-1]] = 0.,0.
    return tuple(TrajectoryPoint(float(t-stamps[0]),tuple(q),tuple(v),tuple(a))
        for t,q,v,a in zip(stamps,values,velocity,acceleration))


def boundary_points(names, points):
    """规划端和执行准备一致：已有首末 v/a 归零，保留中间导数与节点位置。"""
    points = list(points)
    zeros = (0.,)*len(names)
    for index in {0, len(points)-1}:
        p = points[index]
        points[index] = replace(p, velocities=zeros if p.velocities else (),
            accelerations=zeros if p.accelerations else ())
    return tuple(points)


def retime_curve(names, points, lower, upper, limits, *, velocity_scaling=1.,
                 acceleration_scaling=1., fixed=False):
    """按实际规划曲线统一缩放 t/v/a；准备仍复查规范化后的准确曲线。"""
    if any(not math.isfinite(v) or not 0 < v <= 1 for v in (velocity_scaling, acceleration_scaling)):
        raise ValueError('planning scaling must be in (0, 1]')
    if not 0 < len(points) <= 10_000 or points[-1].time > 300.:
        raise ValueError('trajectory planning budget exceeded')
    points = boundary_points(names, points)
    base = build_curve(names, points)
    limits = {name: replace(limit,
        max_velocity=limit.max_velocity*velocity_scaling,
        max_acceleration=limit.max_acceleration*acceleration_scaling)
        for name, limit in limits.items()}
    scale = 1.
    report = check_curve(base, lower, upper, limits)
    if any(issue['kind'] == 'position' for issue in report['issues']):
        raise ValueError('position extremum outside limits; retiming cannot repair geometry')
    for name, peaks in report['peaks'].items():
        for kind, power in (('velocity', 1), ('acceleration', 2), ('jerk', 3)):
            bound = getattr(limits[name], 'max_'+kind)
            if bound is not None:
                if bound <= 0 and peaks[kind]['absolute'] > 1e-9:
                    raise ValueError('motion forbidden by zero dynamic limit')
                if bound > 0:
                    scale = max(scale, (peaks[kind]['absolute']/bound)**(1/power))
    if fixed and scale > 1.+1e-9:
        raise ValueError(f'fixed duration exceeds limits; minimum duration >= {base.duration*scale:.6f} s')
    scale = 1. if fixed else max(1., scale*(1.+1e-6))
    def scaled(candidate):
        return tuple(TrajectoryPoint(round(p.time*scale*1e9)*1e-9, p.positions,
            tuple(v/scale for v in p.velocities), tuple(a/scale**2 for a in p.accelerations)) for p in candidate)
    curve = build_curve(base.joint_names, scaled(points))
    if curve.duration > 300.:
        raise ValueError('retimed duration exceeds execution budget')
    result = check_curve(curve, lower, upper, limits)
    if result['state'] != 'pass':
        raise ValueError(f'quantized retimed curve failed final verification: {result["issues"][:3]}')
    return curve, scale

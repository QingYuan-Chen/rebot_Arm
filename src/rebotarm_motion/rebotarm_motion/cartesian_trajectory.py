"""旧位姿接口需要的 SE(3) 采样, 连续 IK 跟踪与重定时.

职责与位置
    运动层的笛卡尔路径生成与动态限位重定时, 不包含硬件发送.

对外接口
    plan_cartesian 返回关节路径和时长; retime_path 返回可交接的带时间轨迹字典.

安全与并发约束
    上一个 IK 解作为下一个种子; 任一点失败即拒绝. 重定时只延长时间, 不能替代碰撞检查.
    规划导数限位仍用于理想轨迹, 不把量化反馈差分当成同等测量.

文件结构(按定义顺序)
    plan_cartesian: 在 SE(3) 上按五次进度采样, 以前一解为种子逐点 IK, 返回关节路径和时长.
    retime_path: 校验路径后按速度, 加速度与 jerk 限位拉长时间, 返回带相对秒的轨迹字典.
"""

import math
import numpy as np
import pinocchio as pin
from .pose_kinematics import JOINT_NAMES, model_resource
from .paired_trajectory_protocol import build_retimed_path_command, build_quintic_command
from .trajectory_runtime_limits import load_joint_runtime_limits


def plan_cartesian(kinematics, seed, target, duration=0., dt=.02):
    """在 SE(3) 上按五次进度采样, 以前一解为种子逐点 IK, 返回关节路径和时长.

    seed 为规范六轴 rad 向量, duration / dt 为秒; 自动时长由平移距离估算且至少一秒.
    限制规划时长和采样预算, 任一点 IK 不收敛即整条失败; 输出尚未完成碰撞检查.
    """
    if not math.isfinite(duration) or not math.isfinite(dt) or dt <= 0:
        raise ValueError('finite duration and positive sampling period required')
    start = kinematics.pose(seed)
    if duration <= 0:
        duration = max(1., float(np.linalg.norm(target.translation-start.translation))/.1)
    if duration > 120. or duration/dt > 10000:
        raise ValueError('Cartesian path exceeds bounded planning budget')
    count = max(2, math.ceil(duration/dt))
    twist = pin.log6(start.actInv(target))
    joints = [kinematics.joints(seed)]
    for i in range(1, count+1):
        u = i/count
        blend = 10*u**3 - 15*u**4 + 6*u**5
        desired = start * pin.exp6(twist*blend)
        result = kinematics.solve(desired, joints[-1], step_size=.8)
        if not result.success:
            raise ValueError(f'Cartesian IK failed at sample {i}/{count}: {result.error:.4e}')
        joints.append(result.q)
    return joints, duration


def retime_path(kinematics, points, duration, cadence=.02):
    """校验路径后按速度, 加速度与 jerk 限位拉长时间, 返回带相对秒的轨迹字典.

    重新计算理想轨迹的离散导数, 按一/二/三阶时间缩放关系选择放慢倍率.
    最多尝试十二轮且总时长不超过预算; 失败抛异常, 不放宽规划限位.
    """
    path = [kinematics.joints(q) for q in points]
    if len(path) < 2 or not math.isfinite(duration) or duration <= 0:
        raise ValueError('path requires two points and positive duration')
    limits = load_joint_runtime_limits(model_resource('config/joint_limits.yaml'), JOINT_NAMES)
    builder = (build_quintic_command if np.max(np.abs(np.diff(path, axis=0))) < 1e-12
               else build_retimed_path_command)
    for _ in range(12):
        duration = math.ceil(duration/cadence)*cadence
        if duration > 120.:
            raise ValueError('retimed trajectory exceeds 120 second execution budget')
        if builder is build_quintic_command:
            command = builder(path[0], path[-1], duration_sec=duration,
                              cadence_sec=cadence, label='pose compatibility')
        else:
            command = builder(path, duration_sec=duration, cadence_sec=cadence,
                              label='pose compatibility')
        values = np.asarray([p['positions'] for p in command['points']])
        stamps = np.asarray([p['elapsed_sec'] for p in command['points']])
        dt = np.diff(stamps)
        velocity = np.diff(values, axis=0)/dt[:, None]
        acceleration = np.diff(np.vstack((np.zeros(6), velocity, np.zeros(6))), axis=0)
        acceleration /= np.r_[dt[0]/2, (dt[1:]+dt[:-1])/2, dt[-1]/2][:, None]
        jerk = np.diff(acceleration, axis=0)/dt[:, None]
        factor = 1.
        # 时间放慢 k 倍时, 一, 二, 三阶导数分别缩小 k, k², k³, 用对应开方求所需倍率.
        for index, name in enumerate(JOINT_NAMES):
            for derivative, limit, power in ((velocity, limits[name].max_velocity, 1),
                (acceleration, limits[name].max_acceleration, 2), (jerk, limits[name].max_jerk, 3)):
                if limit is not None:
                    if limit <= 0:
                        raise ValueError(f'{name} has no positive motion limit')
                    factor = max(factor, (float(np.max(np.abs(derivative[:,index])))/limit)**(1/power))
        if factor <= 1.000001:
            return command
        duration *= factor*1.05
    raise ValueError('unable to retime path within configured dynamic limits')

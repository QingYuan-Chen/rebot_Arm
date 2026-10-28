"""旧位姿接口需要的 SE(3) 采样、连续 IK 跟踪与重定时。

``plan_cartesian`` 用最小 jerk 进度在 SE(3) 上采样，每点以上一点关节解为
初值做闭环 IK；任一点失败即拒绝整条路径。``retime_path`` 复用现有保路径
重定时，按配置包限位检查速度、加速度及 jerk，必要时只拉长时间。

采样默认 20 ms，自动时长至少 1 s；首末点时间覆盖完整时长，末速度为零。
本模块没有设备对象或 ROS 节点，输出仍需碰撞预检才能交给唯一 FJT 服务端。
"""
import math
import numpy as np
import pinocchio as pin
from .pose_kinematics import JOINT_NAMES, model_resource
from .paired_trajectory_protocol import build_retimed_path_command, build_quintic_command
from .trajectory_runtime_limits import load_joint_runtime_limits


def plan_cartesian(kinematics, seed, target, duration=0., dt=.02):
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

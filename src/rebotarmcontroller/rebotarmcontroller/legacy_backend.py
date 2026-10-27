"""临时纯计算桥接；批次 6 将逆运动学和路径生成迁入运动包。

此模块不持有设备句柄、执行状态或线程。完整许可文本尚未取得，
直接从原目录导入 SDK，暂不复制其源码。
"""
from pathlib import Path
import sys
import numpy as np

def ensure_sdk():
    candidates = [base/'third_party/reBotArm_control_py' for base in
                  (Path.cwd(), *Path(__file__).resolve().parents)]
    for root in candidates:
        if (root/'reBotArm_control_py').is_dir():
            if str(root) not in sys.path: sys.path.insert(0,str(root))
            return root
    raise FileNotFoundError('Cannot find reBotArm_control_py in third_party')

def _target(pose):
    import pinocchio as pin
    values = np.asarray(pose,dtype=float)
    if values.shape!=(6,) or not np.isfinite(values).all():
        raise ValueError('pose needs six finite xyz/rpy values')
    return pin.SE3(pin.rpy.rpyToMatrix(*values[3:]),values[:3])

def solve_pose_ik(kinematics,q,pose):
    ensure_sdk()
    from reBotArm_control_py.kinematics.inverse_kinematics import solve_ik,IKParams
    result = solve_ik(kinematics.model,kinematics.model.createData(),kinematics.frame_id,
                      _target(pose),np.array(q,copy=True),
                      IKParams(max_iter=200,tolerance=1e-4,step_size=.5,damping=1e-6))
    return np.array(result.q,copy=True) if result.success else None

def plan_pose_path(kinematics,q,pose,duration,dt):
    ensure_sdk()
    from reBotArm_control_py.trajectory import (TrajPlanParams,TrajProfile,IKParams,
        plan_cartesian_geodesic_trajectory,track_trajectory)
    if not np.isfinite(duration): raise ValueError('duration must be finite')
    q_end = solve_pose_ik(kinematics,q,pose)
    if q_end is None: return None
    import pinocchio as pin
    start = pin.SE3(kinematics.pose(q)[1],kinematics.pose(q)[0])
    end = pin.SE3(kinematics.pose(q_end)[1],kinematics.pose(q_end)[0])
    if duration<=0: duration = max(1.,float(np.linalg.norm(_target(pose).translation-start.translation))/.1)
    path = plan_cartesian_geodesic_trajectory(start,end,duration,TrajPlanParams(dt=dt,profile=TrajProfile.MIN_JERK))
    joints = track_trajectory(kinematics.model,kinematics.frame_id,path.trajectory,np.array(q,copy=True),
                             IKParams(max_iter=200,tolerance=1e-4,damping=1e-6,step_size=.8),null_gain=.1)
    if not joints or not all(p.ik_success for p in joints): return None
    # 沿用旧发送器的时长与点数调度，由运行时推进轨迹。
    return [np.array(p.q,copy=True) for p in joints],[i*duration/len(joints) for i in range(len(joints))]

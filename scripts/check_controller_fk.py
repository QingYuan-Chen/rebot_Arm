#!/usr/bin/env python3
"""用相同关节输入比较旧模型与规范模型的正运动学，不创建设备对象。"""
from pathlib import Path
import json
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/rebotarmcontroller'))
from rebotarmcontroller.gravity_control import load_gravity_model, GravitySettings
from rebotarmcontroller.controller_kinematics import ArmKinematics

SAMPLES = ((0.,0.,0.,0.,0.,0.), (0.,0.,-.017453292519943295,0.,0.,0.),
           (.2,-.8,-1.1,.3,-.2,.4), (-.4,-1.4,-.5,-.5,.5,-.8))


def compare():
    import pinocchio as pin
    canonical = ArmKinematics(load_gravity_model(
        ROOT/'src/rebotarm_moveit_config/config/rebotarm.urdf',GravitySettings()).model)
    old_path = ROOT/'third_party/reBotArm_control_py/urdf/reBot-DevArm_fixend_description/urdf/reBot-DevArm_fixend.urdf'
    legacy = pin.buildModelFromUrdf(str(old_path))
    results = []
    for q in SAMPLES:
        data = legacy.createData()
        pin.forwardKinematics(legacy,data,np.array(q))
        pin.updateFramePlacements(legacy,data)
        old = data.oMf[legacy.getFrameId('end_link')]
        position,rotation = canonical.pose(q)
        results.append(dict(q=q,translation_delta_m=(position-old.translation).tolist(),
                            position_error_m=float(np.linalg.norm(position-old.translation)),
                            rotation_error_rad=float(np.linalg.norm(pin.log3(old.rotation.T@rotation)))))
    return dict(canonical_joints=list(canonical.model.names)[1:],legacy_joints=list(legacy.names)[1:],samples=results,
                max_position_error_m=max(r['position_error_m'] for r in results),
                max_rotation_error_rad=max(r['rotation_error_rad'] for r in results))

if __name__=='__main__':
    print(json.dumps(compare(),ensure_ascii=False,indent=2))

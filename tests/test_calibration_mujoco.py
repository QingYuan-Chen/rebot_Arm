"""Canonical MuJoCo FK to calibration fit; no rendering or real trajectory."""
import os
from pathlib import Path
import subprocess
import sys
import shutil
import pytest


def test_mujoco_multiaxis_handeye_recovery():
    root=Path(__file__).resolve().parents[1]
    python=shutil.which(os.environ.get('REBOTARM_MUJOCO_PYTHON',sys.executable))
    if python is None: pytest.skip('configured MuJoCo interpreter unavailable')
    script=r'''
import numpy as np
import mujoco
from rebotarm_simulation.core.mujoco_sim import RebotArmMujoco
from rebotarm_calibration.handeye_residual import matrix_transform,transform_matrix
from rebotarm_calibration.handeye_workflow import solve_dataset
sim=RebotArmMujoco();model=sim._model;data=sim._data
rng=np.random.default_rng(77)
handeye=np.eye(4);handeye[:3,3]=[-.08,.01,.04]
marker=np.eye(4);marker[:3,3]=[.3,-.2,.2]
samples=[]
try:
    for index in range(20):
        for name in sim.joint_names:
            joint=model.joint(name)
            lo,hi=joint.range
            data.qpos[joint.qposadr[0]]=rng.uniform(lo*.6,hi*.6)
        mujoco.mj_forward(model,data)
        end=np.eye(4);end[:3,:3]=data.body('end_link').xmat.reshape(3,3);end[:3,3]=data.body('end_link').xpos
        samples.append({'sample_id':str(index),'base_to_end':matrix_transform(end),
                        'camera_to_marker':matrix_transform(np.linalg.inv(end@handeye)@marker)})
    dataset={'schema_version':1,'metadata':dict(base_frame='base_link',end_link_frame='end_link',camera_frame='synthetic_optical',marker_frame='fixed_marker'),
             'training_samples':samples[:14],'validation_samples':samples[14:]}
    report=solve_dataset(dataset)
    assert report['passed'],report
    selected=report['methods'][report['selected_method']]
    assert np.allclose(transform_matrix(selected['end_to_camera']),handeye,atol=1e-7)
    assert selected['validation']['position_residual']['max_m']<1e-7
    print('canonical MuJoCo 14 train + 6 holdout: PASS',report['selected_method'])
finally:sim.close()
'''
    env=dict(os.environ)
    env['PYTHONPATH']=os.pathsep.join([str(root/'src/rebotarm_calibration'),str(root/'src/rebotarm_simulation'),env.get('PYTHONPATH','')])
    result=subprocess.run([str(python),'-c',script],env=env,capture_output=True,text=True,timeout=30)
    assert result.returncode==0,result.stdout+result.stderr


def test_mujoco_projected_marker_detection_and_handeye(tmp_path):
    root=Path(__file__).resolve().parents[1]
    python=shutil.which(os.environ.get('REBOTARM_MUJOCO_PYTHON',sys.executable))
    if python is None: pytest.skip('configured MuJoCo interpreter unavailable')
    script=r'''
import numpy as np,cv2,mujoco,json,sys
from rebotarm_simulation.core.mujoco_sim import RebotArmMujoco
from rebotarm_calibration.handeye_residual import matrix_transform,transform_matrix
from rebotarm_calibration.handeye_workflow import solve_dataset,atomic_write_json
from rebotarm_calibration.aruco_pose import detect_aruco_pose
sim=RebotArmMujoco();model=sim._model;data=sim._data
rng=np.random.default_rng(119)
handeye=np.eye(4);handeye[:3,3]=[-.08,.01,.04]
nominal=data.qpos.copy()
def end_pose():
    pose=np.eye(4);pose[:3,:3]=data.body('end_link').xmat.reshape(3,3);pose[:3,3]=data.body('end_link').xpos
    return pose
mujoco.mj_forward(model,data)
front=np.eye(4);front[:3,:3]=np.diag([1.,-1.,-1.]);front[:3,3]=[0.,0.,.4]
fixed_marker=end_pose()@handeye@front
k=np.array([[1600.,0.,960.],[0.,1600.,720.],[0.,0.,1.]])
obj=np.array([[-.05,.05,0],[.05,.05,0],[.05,-.05,0],[-.05,-.05,0]])
dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
marker=cv2.aruco.drawMarker(dictionary,0,400) if not hasattr(cv2.aruco,'generateImageMarker') else cv2.aruco.generateImageMarker(dictionary,0,400)
# Pixel centers span 0..399; physical outer black edges are at -0.5/399.5.
source=np.array([[-.5,-.5],[399.5,-.5],[399.5,399.5],[-.5,399.5]],np.float32)
samples=[]
try:
    for attempt in range(1000):
        data.qpos[:]=nominal
        for name in sim.joint_names:
            joint=model.joint(name);address=joint.qposadr[0]
            data.qpos[address]=np.clip(nominal[address]+rng.uniform(-.3,.3),joint.range[0]+.01,joint.range[1]-.01)
        mujoco.mj_forward(model,data);end=end_pose()
        camera_marker=np.linalg.inv(end@handeye)@fixed_marker
        if camera_marker[2,3]<.15:continue
        rvec=cv2.Rodrigues(camera_marker[:3,:3])[0]
        corners=cv2.projectPoints(obj,rvec,camera_marker[:3,3],k,np.zeros(5))[0].reshape(4,2).astype(np.float32)
        if np.any(corners<20) or np.any(corners[:,0]>1900) or np.any(corners[:,1]>1420):continue
        image=cv2.warpPerspective(marker,cv2.getPerspectiveTransform(source,corners),(1920,1440),flags=cv2.INTER_LINEAR,borderValue=255)
        try:
            detected=detect_aruco_pose(cv2.cvtColor(image,cv2.COLOR_GRAY2BGR),camera_matrix=k,distortion=[0.]*5,marker_length_m=.1)
        except ValueError:continue
        if detected['reprojection_rmse_px']>1.:continue
        samples.append({'sample_id':str(attempt),'base_to_end':matrix_transform(end),
                        'camera_to_marker':detected['camera_to_marker'],'quality':detected['reprojection_rmse_px']})
        if len(samples)==24:break
    assert len(samples)==24
    dataset={'schema_version':1,'metadata':dict(base_frame='base_link',end_link_frame='end_link',camera_frame='synthetic_optical',marker_frame='fixed_marker'),
             'training_samples':samples[:16],'validation_samples':samples[16:]}
    report=solve_dataset(dataset)
    assert report['passed'],{k:v.get('gates',v.get('error')) for k,v in report['methods'].items()}
    solved=transform_matrix(report['methods'][report['selected_method']]['end_to_camera'])
    error=float(np.linalg.norm(solved[:3,3]-handeye[:3,3]))
    assert error<.005,error
    atomic_write_json(sys.argv[1],dataset)
    print('MuJoCo FK -> projected pixels -> actual ArUco/PnP -> solve PASS',error)
finally:sim.close()
'''
    env=dict(os.environ)
    env['PYTHONPATH']=os.pathsep.join([str(root/'src/rebotarm_calibration'),str(root/'src/rebotarm_simulation'),env.get('PYTHONPATH','')])
    result=subprocess.run([str(python),'-c',script,str(tmp_path/'mujoco_dataset.json')],env=env,capture_output=True,text=True,timeout=40)
    assert result.returncode==0,result.stdout+result.stderr

"""Manufacturer ratings do not override the retained URDF simulation baseline."""
from pathlib import Path
import xml.etree.ElementTree as ET
import pytest
from rebotarm_simulation.control.control_config import load_motor_control_parameters

ROOT = Path(__file__).resolve().parents[1]

def test_24v_specs_preserve_urdf_simulation_limits():
    params = load_motor_control_parameters(
        calibration_path=ROOT/'src/rebotarm_simulation/config/motor_control_calibration.yaml',
        urdf_path=ROOT/'src/rebotarm_moveit_config/config/rebotarm.urdf',
    )
    assert params.arm.effort_limit == (27.,27.,27.,7.,7.,7.)
    assert params.arm.rated_torque == (12.,12.,12.,3.5,3.5,3.5)
    assert params.arm.velocity_limit == (5.,5.,5.,3.,3.,3.)
    root = ET.parse(ROOT/'src/rebotarm_simulation/models/rebotarm/robot.xml')
    urdf = ET.parse(ROOT/'src/rebotarm_moveit_config/config/rebotarm.urdf')
    for i, limit in enumerate(params.arm.effort_limit, 1):
        joint = f'joint{i}'
        for attr in ['ctrlrange','forcerange']:
            assert [float(v) for v in root.find(f'actuator/motor[@joint="{joint}"]').get(attr).split()] == [-limit,limit]
        assert [float(v) for v in root.find(f'.//joint[@name="{joint}"]').get('actuatorfrcrange').split()] == [-limit,limit]
        assert float(urdf.find(f'joint[@name="{joint}"]/limit').get('effort')) == (27 if i<=3 else 7)

def test_compiled_model_retains_urdf_force_limits():
    mj = pytest.importorskip('mujoco')
    model = mj.MjModel.from_xml_path(str(ROOT/'src/rebotarm_simulation/models/rebotarm/scene.xml'))
    for i in range(1,7):
        expected = 27 if i<=3 else 7
        assert model.actuator(f'joint{i}_torque').forcerange.tolist() == [-expected,expected]
        assert model.jnt_actfrcrange[model.joint(f'joint{i}').id].tolist() == [-expected,expected]


def test_finger_actuator_and_joint_limits_share_simulation_setting():
    mj = pytest.importorskip('mujoco')
    model = mj.MjModel.from_xml_path(str(ROOT/'src/rebotarm_simulation/models/rebotarm/scene.xml'))
    for side in ['left', 'right']:
        actuator = model.actuator(f'{side}_finger_force')
        joint = model.joint(f'{side}_finger_joint').id
        assert actuator.ctrlrange.tolist() == [-1.5, 1.5]
        assert actuator.forcerange.tolist() == [-1.5, 1.5]
        assert model.jnt_actfrcrange[joint].tolist() == [-1.5, 1.5]
        assert model.jnt_actfrclimited[joint]
    urdf = ET.parse(ROOT/'src/rebotarm_moveit_config/config/rebotarm.urdf')
    for side in ['left', 'right']:
        assert float(urdf.find(f'joint[@name="{side}_finger_joint"]/limit').get('effort')) == 1.0

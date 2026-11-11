"""不启动 ROS 节点或连接设备,验证组件边界."""
import ast
from dataclasses import FrozenInstanceError
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from rebotarmcontroller.configuration import HardwareSettings, load_configuration, default_config
from rebotarmcontroller.contracts.controller_contracts import MotorFeedback

# 批次 2 切换时,用相同输入对照旧模型和规范模型的正运动学结果.
FK_SAMPLES = ((0.,0.,0.,0.,0.,0.), (0.,0.,-.017453292519943295,0.,0.,0.),
              (.2,-.8,-1.1,.3,-.2,.4), (-.4,-1.4,-.5,-.5,.5,-.8))


def test_configuration_preserves_entrypoint_defaults_and_memory_override(tmp_path):
    root=Path(__file__).resolve().parents[1]
    default=load_configuration()
    bringup=load_configuration(root/'src/rebotarm_bringup/config/arm.yaml',root/'src/rebotarm_bringup/config/gripper.yaml')
    assert (default.rate,bringup.rate)==(500.,100.)
    assert default.joints==bringup.joints and default.gripper==bringup.gripper
    source=default_config('arm').read_bytes()
    override=load_configuration(channel='can23')
    assert override.channel=='can23' and default_config('arm').read_bytes()==source
    assert tuple(j.vlim for j in default.joints)==(5.,5.,5.,3.,3.,3.)
    assert tuple(j.kd for j in default.joints)==(8.,8.,8.,2.,2.,2.)
    assert default.timing.home_velocity==.3 and default.timing.arrival_poll_period==.02


@pytest.mark.parametrize('mutation,match', [
    (lambda a,g: a['joints'][0].update(vendor='robstride'),'only damiao'),
    (lambda a,g: a['joints'].reverse(),'ordered'),
    (lambda a,g: g.update(channel='can1'),'channel must match'),
    (lambda a,g: g['gripper'][0].update(motor_id=1),'duplicate motor_id'),
    (lambda a,g: a['joints'][1]['POS_VEL'].update(vlim=float('nan')),'invalid safety parameter'),
    (lambda a,g: a.update(rate=0),'rate'),
])
def test_configuration_rejects_ambiguous_or_unsafe_bus(mutation,match,tmp_path):
    arm=yaml.safe_load(default_config('arm').read_text())
    grip=yaml.safe_load(default_config('gripper').read_text())
    mutation(arm,grip)
    a,g=tmp_path/'a.yaml',tmp_path/'g.yaml'
    a.write_text(yaml.safe_dump(arm));g.write_text(yaml.safe_dump(grip))
    with pytest.raises(ValueError,match=match): load_configuration(a,g,channel='can23')


def test_feedback_contract_copies_values_and_is_frozen():
    original=SimpleNamespace(pos=1.,vel=2.,torq=3.,status_code=11)
    copy=MotorFeedback.copy_from(original)
    original.pos=99
    assert copy.pos==1. and copy.status_code==11
    with pytest.raises(FrozenInstanceError): copy.pos=99


def test_manager_and_ros_have_no_legacy_private_access():
    root=Path(__file__).resolve().parents[1]/'src/rebotarmcontroller/rebotarmcontroller'
    for name in ('hardware_manager.py','ros/ros_actions.py','ros/ros_services.py','ros/motor_passthrough.py'):
        for node in ast.walk(ast.parse((root/name).read_text())):
            if not isinstance(node, ast.Attribute):
                continue
            assert node.attr not in ('_endpos_ctrl', 'endpos_ctrl', '_motor_map', '_ctrl_map')
            # _arm 现在是六轴职责端口,不再是旧 SDK 对象;禁止穿透其私有成员.
            if (isinstance(node.value, ast.Attribute) and node.value.attr == '_arm'
                    and isinstance(node.value.value, ast.Name) and node.value.value.id == 'self'):
                assert not node.attr.startswith('_'), (name, node.lineno, node.attr)


@pytest.mark.parametrize('mutation,field', [
    (lambda a,g: a['joints'][0].pop('model'),'joints[0].model is required'),
    (lambda a,g: a['joints'][0].pop('MIT'),'joints[0].MIT is required'),
    (lambda a,g: a['joints'][0]['POS_VEL'].pop('pos_kp'),'joints[0].POS_VEL.pos_kp is required'),
    (lambda a,g: g['gripper'][0].pop('vendor'),'gripper[0].vendor is required'),
    (lambda a,g: a.pop('channel'),'root.channel is required'),
    (lambda a,g: a.pop('rate'),'root.rate is required'),
])
def test_configuration_requires_complete_motor_and_bus_fields(mutation,field,tmp_path):
    """缺失字段不得回退为旧型号, 增益或通道默认值, 错误须指出文件与字段."""
    arm=yaml.safe_load(default_config('arm').read_text())
    grip=yaml.safe_load(default_config('gripper').read_text())
    mutation(arm,grip)
    a,g=tmp_path/'a.yaml',tmp_path/'g.yaml'
    a.write_text(yaml.safe_dump(arm));g.write_text(yaml.safe_dump(grip))
    with pytest.raises(ValueError) as error:
        load_configuration(a,g)
    message=str(error.value)
    assert field in message and (str(a) in message or str(g) in message)


def test_configuration_rejects_non_numeric_rate(tmp_path):
    arm=yaml.safe_load(default_config('arm').read_text())
    grip=yaml.safe_load(default_config('gripper').read_text())
    arm['rate']='fast'
    a,g=tmp_path/'a.yaml',tmp_path/'g.yaml'
    a.write_text(yaml.safe_dump(arm));g.write_text(yaml.safe_dump(grip))
    with pytest.raises(ValueError, match='rate must be finite and positive'):
        load_configuration(a,g)

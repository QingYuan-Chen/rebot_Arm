"""Component boundaries exercised without a ROS node or physical devices."""
from dataclasses import FrozenInstanceError
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from rebotarmcontroller.configuration import HardwareSettings, load_configuration, default_config
from rebotarmcontroller.contracts.models import MotorFeedback

# Identical inputs for legacy/canonical FK comparison when switching in batch 2.
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
    for name in ('hardware_manager.py','ros_actions.py','ros_services.py','motor_passthrough.py'):
        source=(root/name).read_text()
        assert 'self._arm.' not in source and 'self._endpos_ctrl.' not in source
        assert '.endpos_ctrl.' not in source


def test_omitted_gripper_gains_keep_legacy_loader_defaults(tmp_path):
    # The installed full YAML overrides MIT to 8/1; omission keeps the old 18/2 fallback.
    grip=yaml.safe_load(default_config('gripper').read_text())
    grip['gripper'][0].pop('MIT')
    grip['gripper'][0].pop('POS_VEL')
    path=tmp_path/'gripper.yaml';path.write_text(yaml.safe_dump(grip))
    cfg=load_configuration(gripper_path=path).gripper
    assert (cfg.kp,cfg.kd,cfg.vel_kp,cfg.vel_ki,cfg.pos_kp,cfg.pos_ki,cfg.vlim)==(18.,2.,.0008,.002,50.,1.,3.)

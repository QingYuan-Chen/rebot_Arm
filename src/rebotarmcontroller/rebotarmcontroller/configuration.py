"""Validated controller configuration; parsing never opens a device.

Safety gains and timing are project baselines. Changes require review and a
separate hardware check; MIT defaults are not gravity-controller gains.
"""
from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
import math
import yaml
from .gravity_control import package_file

JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 7))

@dataclass(frozen=True)
class ProtocolTiming:
    serial_baud: int = 921600
    command_motor_delay: float = .05
    status_retries: int = 10
    status_interval: float = .1
    register_settle: float = .02
    mode_motor_settle: float = .05
    mode_settle: float = .2
    mode_timeout_ms: int = 1000
    disconnect_settle: float = .5
    shutdown_settle: float = .1
    zero_disable_settle: float = .3
    zero_motor_settle: float = .1
    zero_retries: int = 200
    zero_poll_interval: float = .05
    join_timeout: float = 5.
    home_velocity: float = .3
    arrival_poll_period: float = .02

@dataclass(frozen=True)
class HardwareSettings:
    hardware_feedback_rate_hz: float = 50.
    gripper_position_torque_cap_nm: float = 1.
    gripper_position_max_speed_rad_s: float = .5
    gripper_position_timeout_margin_sec: float = 1.5
    gripper_feedback_stale_timeout_sec: float = .15
    grasp_hold_timeout_sec: float = 30.

    def __post_init__(self):
        for name, lo, hi, unit in (
            ('hardware_feedback_rate_hz',20.,100.,'Hz'),
            ('gripper_position_torque_cap_nm',.05,1.5,'N.m'),
            ('gripper_position_max_speed_rad_s',.05,3.,'rad/s'),
            ('gripper_position_timeout_margin_sec',.1,10.,'s'),
            ('gripper_feedback_stale_timeout_sec',.05,2.,'s'),
            ('grasp_hold_timeout_sec',.1,120.,'s')):
            value=float(getattr(self,name))
            if not math.isfinite(value) or not lo <= value <= hi:
                raise ValueError(f"{name} must be within [{lo:g}, {hi:g}] {unit}")
            object.__setattr__(self,name,value)

@dataclass(frozen=True)
class MotorConfig:
    name: str
    motor_id: int
    feedback_id: int
    model: str
    vendor: str
    kp: float
    kd: float
    vel_kp: float
    vel_ki: float
    pos_kp: float
    pos_ki: float
    vlim: float

    def to_mapping(self):
        return dict(name=self.name,motor_id=self.motor_id,feedback_id=self.feedback_id,
                    model=self.model,vendor=self.vendor,MIT=dict(kp=self.kp,kd=self.kd),
                    POS_VEL={key:getattr(self,key) for key in ('vel_kp','vel_ki','pos_kp','pos_ki','vlim')})

@dataclass(frozen=True)
class ControllerConfig:
    name: str
    channel: str
    rate: float
    joints: tuple[MotorConfig,...]
    gripper: MotorConfig
    timing: ProtocolTiming = ProtocolTiming()

    def arm_mapping(self):
        return dict(name=self.name,channel=self.channel,rate=self.rate,
                    joints=[j.to_mapping() for j in self.joints])

def default_config(name: str) -> Path:
    return package_file('rebotarmcontroller',f'config/{name}.yaml')

def _mapping(path):
    with Path(path).expanduser().open(encoding='utf-8') as stream:
        value=yaml.safe_load(stream)
    if not isinstance(value,dict): raise ValueError(f"configuration must be a mapping: {path}")
    return value

def _motor(item, *, gripper=False):
    if not isinstance(item,dict): raise ValueError('motor configuration must be a mapping')
    defaults = dict(kp=8. if gripper else 0.,kd=1. if gripper else 0.,
                    vel_kp=0.,vel_ki=0.,pos_kp=0.,pos_ki=0.,vlim=3. if gripper else 2.)
    name=str(item['name']); vendor=str(item.get('vendor','damiao')).lower()
    if vendor != 'damiao': raise ValueError(f"{name}: only damiao is supported")
    model=str(item.get('model','4310' if gripper else '4340P'))
    if model not in ('4310','4340P'): raise ValueError(f"{name}: unsupported damiao model {model}")
    ids=[]
    for key in ('motor_id','feedback_id'):
        value=item[key]
        if isinstance(value,bool) or not isinstance(value,int) or not 1<=value<=0x7ff:
            raise ValueError(f"{name}: invalid {key}")
        ids.append(value)
    values={}
    for section,keys in (('MIT',('kp','kd')),('POS_VEL',('vel_kp','vel_ki','pos_kp','pos_ki','vlim'))):
        section_value=item.get(section,{})
        if not isinstance(section_value,dict): raise ValueError(f"{name}: {section} must be a mapping")
        for key in keys:
            value=float(section_value.get(key,defaults[key]))
            if not math.isfinite(value) or value<0 or (key=='vlim' and value==0):
                raise ValueError(f"{name}: invalid safety parameter {key}")
            values[key]=value
    # Preserve reviewed defaults (including J1..3 kd=8); gravity MIT has its own bounds.
    return MotorConfig(name,*ids,model,vendor,**values)

def load_configuration(arm_path=None,gripper_path=None,channel='') -> ControllerConfig:
    arm=_mapping(arm_path or default_config('arm'))
    grip=_mapping(gripper_path or default_config('gripper'))
    joints=tuple(_motor(item) for item in arm.get('joints',[]))
    if tuple(j.name for j in joints)!=JOINT_NAMES:
        raise ValueError(f"joints must be ordered {JOINT_NAMES}")
    items=grip.get('gripper',[])
    if not isinstance(items,list) or len(items)!=1: raise ValueError('exactly one gripper is required')
    gripper=_motor(items[0],gripper=True)
    if gripper.name!='gripper': raise ValueError('gripper name must be gripper')
    all_motors=(*joints,gripper)
    for key in ('motor_id','feedback_id'):
        if len({getattr(j,key) for j in all_motors})!=len(all_motors):
            raise ValueError(f"duplicate {key} on shared bus")
    arm_channel=str(arm.get('channel','/dev/ttyACM0')).strip()
    grip_channel=str(grip.get('channel',arm_channel)).strip()
    if not arm_channel or grip_channel!=arm_channel: raise ValueError('arm and gripper channel must match')
    override=str(channel or '').strip()
    effective=arm_channel if not override or override.lower()=='auto' else override
    rate=float(arm.get('rate',500.))
    if not math.isfinite(rate) or rate<=0: raise ValueError('rate must be finite and positive')
    return ControllerConfig(str(arm.get('name','reBotArm')),effective,rate,joints,gripper)

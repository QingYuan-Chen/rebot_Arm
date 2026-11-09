"""版本化执行能力的数值协议。

文件职责：规范后端生效的速度、力矩、会话及系数版本，生成配置摘要。
对外接口：make_capability、decode_capability、constraint_revision；调用方负责发布和接收时刻。
安全约束：摘要只标识配置与会话，不代表使能或运动许可；不读取任何时钟。
文件结构：资源摘要 → 能力构造及校验；最终计划绑定由 planning 模块负责。
"""

import hashlib
import json
import math
from .polynomial import JOINT_NAMES


def constraint_revision(*documents):
    """对调用方提供的模型/限值原始字节求摘要；本库不定位或读取资源文件。"""
    digest = hashlib.sha256()
    for document in documents:
        digest.update(len(document).to_bytes(8, 'big'))
        digest.update(document)
    return digest.hexdigest()


def make_capability(mode, session, speed, torque, limits_revision):
    """速度为夹爪电机 rad/s，力矩为 N·m；与开口单位的转换必须显式。"""
    if mode not in ('legacy', 'continuous') or type(session) is not int or session < 0:
        raise ValueError('invalid capability identity')
    if not all(math.isfinite(v) and v > 0 for v in (speed, torque)):
        raise ValueError('positive finite gripper capabilities required')
    if (not isinstance(limits_revision,str) or len(limits_revision) != 64
            or any(c not in '0123456789abcdef' for c in limits_revision)):
        raise ValueError('sha256 limits revision required')
    data = dict(version=3, mode=mode, session=session, coefficient_versions=[1, 2],
        planning_binding_version=1,
        joint_sets=[list(JOINT_NAMES)], teleop_action_version=1,
        gripper_execution='independent_position',
        gripper_motor_speed_rad_s=float(speed), gripper_torque_cap_nm=float(torque),
        finger_speed_m_s=float(speed)*.009, limits_revision=limits_revision)
    data['revision'] = hashlib.sha256(json.dumps(data, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
    return data


def decode_capability(value):
    """重新构造并比对全部字段，拒绝未知版本、漏字段和伪造的摘要。"""
    data = json.loads(value) if isinstance(value, str) else value
    expected = make_capability(data['mode'], data['session'],
        data['gripper_motor_speed_rad_s'], data['gripper_torque_cap_nm'], data['limits_revision'])
    if data != expected:
        raise ValueError('trajectory capability schema or revision mismatch')
    return expected

"""活动运动工具的数值和参数契约回归."""
import pytest

from rebotarm_motion.pose_math import quaternion_to_rpy, rpy_to_quaternion
from rebotarm_motion.parameter_helpers import build_joint_limits, sensor_qos_kwargs


def test_quaternion_identity_and_yaw_roundtrip():
    assert quaternion_to_rpy(0.,0.,0.,1.) == pytest.approx((0.,0.,0.),abs=1e-6)
    assert quaternion_to_rpy(*rpy_to_quaternion(0.,0.,1.57079632679)) == pytest.approx(
        (0.,0.,1.57079632679),abs=1e-6)


def test_joint_limit_arrays_and_sensor_qos():
    assert build_joint_limits(joint_names=('joint1','joint2'),lower_limits=(-1.,-2.),
        upper_limits=(1.,2.)) == {'joint1': (-1.,1.), 'joint2': (-2.,2.)}
    with pytest.raises(ValueError):
        build_joint_limits(joint_names=('joint1','joint2'),lower_limits=(-1.,),upper_limits=(1.,2.))
    assert sensor_qos_kwargs() == dict(depth=10,reliability='best_effort')

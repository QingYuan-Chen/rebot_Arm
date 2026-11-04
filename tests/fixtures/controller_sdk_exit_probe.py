"""源码或独立安装区的 SDK 退出探针; 只构造组件并计算 FK/IK, 不创建设备会话.

REBOTARM_VERIFY_INSTALL_PREFIX 非空时, 模块及配置和模型必须全部位于该安装区.
旧 SDK 导入始终被阻断; MotorBridge 也被阻断以证明纯构造没有触及通信层.
"""
import importlib.abc
import json
import os
from pathlib import Path
import sys


class BlockDevices(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, *args):
        if fullname.split('.')[0] in ('reBotArm_control_py', 'motorbridge'):
            raise AssertionError('unexpected device dependency during construction: ' + fullname)


sys.meta_path.insert(0, BlockDevices())
import rebotarmcontroller.hardware_manager as hardware
import rebotarm_motion.pose_kinematics as motion
from rebotarmcontroller.configuration import default_config
from rebotarmcontroller.gravity.gravity_control import package_file
from rebotarm_motion.pose_preview_solver import PosePreviewSolver
from rebotarm_motion.cartesian_trajectory import plan_cartesian
from rebotarm_motion.pose_compat_node import PoseCompatNode

manager = hardware.HardwareManager()
assert not manager.lifecycle.connected
assert manager.runtime.control_thread is None
solver = PosePreviewSolver()
q = (0., -1., -1., 0., .3, 0.)
target = solver.kinematics.pose(q)
assert solver.kinematics.solve(target, q).success
assert plan_cartesian(solver.kinematics, q, target, .1)[0]
paths = {
    'controller_module': Path(hardware.__file__).resolve(),
    'motion_module': Path(motion.__file__).resolve(),
    'arm_yaml': default_config('arm').resolve(),
    'gripper_yaml': default_config('gripper').resolve(),
    'gravity_yaml': package_file('rebotarmcontroller', 'config/gravity_compensation.yaml').resolve(),
    'controller_model': package_file('rebotarm_moveit_config', 'config/rebotarm.urdf').resolve(),
    'motion_model': motion.model_resource('config/rebotarm.urdf').resolve(),
}
prefix = os.environ.get('REBOTARM_VERIFY_INSTALL_PREFIX')
if prefix:
    assert all(path.is_relative_to(Path(prefix).resolve()) for path in paths.values()), paths
print(json.dumps(dict(paths={key: str(value) for key, value in paths.items()},
                      connected=False, hardware_thread=False, fk_ik_path=True), indent=2))

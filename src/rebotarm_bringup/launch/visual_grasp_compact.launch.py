"""旧 compact 名称兼容别名：正式总入口为 ``visual_grasp_system``。

``_prepare_stages`` 作为测试/下游兼容符号转发到正式入口；实际 launch 图只包含
正式入口，不再复制一份参数或节点定义。
"""

import importlib.util
from pathlib import Path
import sys


def _system_module():
    path = Path(__file__).with_name("visual_grasp_system.launch.py")
    package_root = path.parent.parent
    if str(package_root) not in sys.path:
        sys.path.insert(0, str(package_root))
    spec = importlib.util.spec_from_file_location("rebotarm_bringup.visual_grasp_system", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _prepare_stages(context):
    return _system_module()._prepare_stages(context)

def generate_launch_description():
    return _system_module().generate_launch_description()

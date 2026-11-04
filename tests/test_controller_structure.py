"""批次 6.5/6.7 的职责门禁；防止门面和请求入口重新积累兼容耦合。"""
import ast
import importlib
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / 'src/rebotarmcontroller/rebotarmcontroller'
MODULES = {
    'arm_controller': 'arm/arm_controller.py',
    'gripper_controller': 'gripper/gripper_controller.py',
    'feedback_reader': 'hardware/feedback_reader.py',
    'gravity_controller': 'gravity/gravity_controller.py',
    'controller_cycle': 'runtime/controller_cycle.py',
    'controller_protection': 'runtime/controller_protection.py',
    'ros_actions': 'ros/ros_actions.py',
    'ros_services': 'ros/ros_services.py',
    'ros_publishers': 'ros/ros_publishers.py',
    'motor_passthrough': 'ros/motor_passthrough.py',
    'lifecycle': 'runtime/lifecycle.py',
}


def test_manager_only_constructs_and_wires_components():
    tree = ast.parse((PACKAGE / 'hardware_manager.py').read_text())
    manager = next(node for node in tree.body if isinstance(node, ast.ClassDef))
    declarations = [node for node in manager.body if not isinstance(node, ast.Expr)]
    assert len(declarations) == 1
    constructor = declarations[0]
    assert isinstance(constructor, ast.FunctionDef) and constructor.name == '__init__'
    assert not constructor.decorator_list
    fields = {node.attr for node in ast.walk(constructor)
              if isinstance(node, ast.Attribute) and isinstance(node.ctx, ast.Store)
              and isinstance(node.value, ast.Name) and node.value.id == 'self'}
    assert fields == {'runtime', 'lifecycle', 'arm', 'gripper', 'feedback',
                      'gravity', 'cycle', 'protection'}
    assert not any(isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                   and node.func.id in ('setattr', 'getattr', 'exec', 'eval')
                   for node in ast.walk(manager))


def test_components_never_receive_manager_or_use_lazy_assembly():
    for name in ('arm_controller', 'gripper_controller', 'feedback_reader',
                 'gravity_controller', 'controller_cycle', 'controller_protection'):
        tree = ast.parse((PACKAGE / MODULES[name]).read_text())
        for node in ast.walk(tree):
            if isinstance(node, ast.arg):
                assert 'manager' not in node.arg
            if isinstance(node, ast.FunctionDef):
                assert node.name not in ('__getattr__', '__setattr__')
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
                assert node.func.attr != '__new__'
                # 构造/一次绑定之外不得在查询或周期中惰性创建执行器和反馈缓存。
                if node.func.attr.startswith('_assemble'):
                    parent = next(method for method in ast.walk(tree)
                                  if isinstance(method, ast.FunctionDef)
                                  and node in list(ast.walk(method)))
                    assert parent.name in ('__init__', 'bind')


def test_stop_confirmation_contains_no_device_polling_or_sleep():
    tree = ast.parse((PACKAGE / 'arm/arm_stop.py').read_text())
    assert not any(isinstance(node, (ast.While, ast.For)) for node in ast.walk(tree))
    for node in ast.walk(tree):
        if isinstance(node, ast.Attribute):
            assert node.attr not in ('sleep', 'device_wait', 'request_feedback', 'poll_feedback_once')


def test_safety_fixtures_use_formal_construction():
    for name in ('test_hardware_manager_p0_safety.py', 'controller_fake_backend.py'):
        assert 'HardwareManager.__new__' not in (ROOT / 'tests' / name).read_text()


def test_peer_access_is_limited_to_declared_public_ports():
    """不只检查门面：bind 的每条边必须是冻结端口，调用不得越过端口字段。"""
    from dataclasses import fields, is_dataclass
    import rebotarmcontroller.contracts.component_ports as ports
    for name in ('arm_controller', 'gripper_controller', 'feedback_reader',
                 'gravity_controller', 'controller_cycle', 'controller_protection'):
        tree = ast.parse((PACKAGE / MODULES[name]).read_text())
        owner = next(node for node in tree.body if isinstance(node, ast.ClassDef))
        bind = next(node for node in owner.body
                    if isinstance(node, ast.FunctionDef) and node.name == 'bind')
        contracts = {}
        for argument in bind.args.kwonlyargs:
            assert isinstance(argument.annotation, ast.Name), (name, argument.arg)
            contract = getattr(ports, argument.annotation.id)
            assert is_dataclass(contract) and contract.__dataclass_params__.frozen
            contracts[argument.arg] = {field.name for field in fields(contract)}
        for node in ast.walk(tree):
            if (isinstance(node, ast.Attribute) and isinstance(node.value, ast.Attribute)
                    and isinstance(node.value.value, ast.Name) and node.value.value.id == 'self'
                    and node.value.attr in contracts):
                assert node.attr in contracts[node.value.attr], (name, node.lineno, node.attr)
                assert not node.attr.startswith('_')
            if (isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                    and node.func.id in ('getattr', 'setattr', 'hasattr')
                    and node.args and isinstance(node.args[0], ast.Attribute)):
                assert node.args[0].attr not in contracts, (name, node.lineno)


def test_manager_never_pierces_component_private_state():
    peers = {'arm', 'gripper', 'feedback', 'gravity', 'cycle', 'protection'}
    tree = ast.parse((PACKAGE / 'hardware_manager.py').read_text())
    for node in ast.walk(tree):
        if (isinstance(node, ast.Attribute) and isinstance(node.value, ast.Attribute)
                and isinstance(node.value.value, ast.Name) and node.value.value.id == 'self'
                and node.value.attr in peers):
            assert not node.attr.startswith('_'), (node.lineno, node.attr)


def test_initialized_controller_fields_do_not_use_fixture_fallbacks():
    for name in ('arm_controller', 'gripper_controller', 'gravity_controller'):
        tree = ast.parse((PACKAGE / MODULES[name]).read_text())
        for node in ast.walk(tree):
            if (isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                    and node.func.id == 'getattr' and node.args):
                assert not (isinstance(node.args[0], ast.Name)
                            and node.args[0].id == 'self'), (name, node.lineno)


def test_only_composition_node_receives_manager_components():
    allowed = {'runtime', 'lifecycle', 'arm', 'gripper', 'feedback',
               'gravity', 'cycle', 'protection'}
    consumers = set()
    for path in (ROOT / 'src').rglob('*.py'):
        tree = ast.parse(path.read_text())
        for node in ast.walk(tree):
            if (isinstance(node, ast.ImportFrom) and node.module
                    and node.module.rsplit('.', 1)[-1] == 'hardware_manager'):
                consumers.add(path)
            if (path == PACKAGE / 'rebotarm_controller.py' and isinstance(node, ast.Attribute)
                    and isinstance(node.value, ast.Attribute) and node.value.attr == 'hardware'):
                assert node.attr in allowed, (node.lineno, node.attr)
    assert consumers == {PACKAGE / 'rebotarm_controller.py'}


def test_ros_adapters_only_access_declared_role_ports():
    """只读端口不能发命令；适配和关闭协调不得穿透组件私有状态或写入属性。"""
    contracts = ast.parse((PACKAGE / 'contracts/ros_ports.py').read_text())
    classes = {node.name: node for node in contracts.body if isinstance(node, ast.ClassDef)}

    def members(name):
        owner = classes[name]
        inherited = set().union(*(members(base.id) for base in owner.bases
                                  if isinstance(base, ast.Name) and base.id in classes))
        return inherited | {node.name for node in owner.body if isinstance(node, ast.FunctionDef)}

    for name in ('ros_actions', 'ros_services', 'ros_publishers', 'motor_passthrough', 'lifecycle'):
        tree = ast.parse((PACKAGE / MODULES[name]).read_text())
        # 构造参数与同名私有字段一一对应；关闭策略直接使用参数。
        functions = [node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef)
                     and node.name in ('__init__', 'shutdown_policy')]
        for function in functions:
            arguments = function.args.args + function.args.kwonlyargs
            roles = {arg.arg: members(arg.annotation.id) for arg in arguments
                     if isinstance(arg.annotation, ast.Name) and arg.annotation.id in classes}
            if not roles:
                continue
            body = tree if function.name == '__init__' else function
            argument_scope = {id(node) for node in ast.walk(function)}

            def role(node):
                if isinstance(node, ast.Name) and node.id in roles and id(node) in argument_scope:
                    return node.id
                if (isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name)
                        and node.value.id == 'self' and node.attr.removeprefix('_') in roles):
                    return node.attr.removeprefix('_')
                return None

            for node in ast.walk(body):
                if isinstance(node, ast.Attribute) and (key := role(node.value)):
                    assert node.attr in roles[key], (name, node.lineno, key, node.attr)
                    assert isinstance(node.ctx, ast.Load), (name, node.lineno, '端口禁止赋值')
                if (isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                        and node.func.id in ('getattr', 'setattr', 'hasattr', 'vars') and node.args):
                    assert role(node.args[0]) is None, (name, node.lineno, '端口禁止动态穿透')


def test_manager_imports_are_used_in_assembly():
    tree = ast.parse((PACKAGE / 'hardware_manager.py').read_text())
    used = {node.id for node in ast.walk(tree) if isinstance(node, ast.Name) and isinstance(node.ctx, ast.Load)}
    for node in tree.body:
        if isinstance(node, ast.ImportFrom) and node.module == '__future__':
            continue
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            for alias in node.names:
                name = alias.asname or alias.name.split('.')[0]
                assert name in used, f'门面禁止为测试或兼容转出导入 {name}'


def test_manager_assembly_does_not_use_transparent_lambdas():
    tree = ast.parse((PACKAGE / 'hardware_manager.py').read_text())
    for node in ast.walk(tree):
        if not isinstance(node, ast.Lambda) or not isinstance(node.body, ast.Call):
            continue
        forwarded = [ast.Name(id=arg.arg, ctx=ast.Load()) for arg in node.args.posonlyargs + node.args.args]
        if node.args.vararg:
            forwarded.append(ast.Starred(value=ast.Name(id=node.args.vararg.arg, ctx=ast.Load()), ctx=ast.Load()))
        keywords = [ast.keyword(arg=arg.arg, value=ast.Name(id=arg.arg, ctx=ast.Load()))
                    for arg in node.args.kwonlyargs]
        if node.args.kwarg:
            keywords.append(ast.keyword(arg=None, value=ast.Name(id=node.args.kwarg.arg, ctx=ast.Load())))
        same = lambda left, right: [ast.dump(value) for value in left] == [ast.dump(value) for value in right]
        assert not (same(node.body.args, forwarded) and same(node.body.keywords, keywords)), node.lineno


def test_request_entries_use_one_component_layer_and_the_central_policy():
    from rebotarmcontroller.runtime.device_requests import REQUEST_POLICIES

    modules = {'ControllerLifecycle': 'runtime/lifecycle.py', 'ArmController': 'arm/arm_controller.py',
               'GripperController': 'gripper/gripper_controller.py',
               'GravityController': 'gravity/gravity_controller.py'}
    found = {}
    for path in PACKAGE.rglob('*.py'):
        tree = ast.parse(path.read_text())
        for owner in (node for node in tree.body if isinstance(node, ast.ClassDef)):
            for method in (node for node in owner.body if isinstance(node, ast.FunctionDef)):
                decorators = [node for node in method.decorator_list if 'device_request' in ast.unparse(node)]
                if not decorators:
                    continue
                assert len(method.decorator_list) == 1
                assert isinstance(decorators[0], ast.Name) and decorators[0].id == 'device_request'
                assert modules.get(owner.name) == path.relative_to(PACKAGE).as_posix()
                key = f'{owner.name}.{method.name}'
                assert key not in found
                module = '.'.join(path.relative_to(PACKAGE).with_suffix('').parts)
                found[key] = getattr(getattr(importlib.import_module(f'rebotarmcontroller.{module}'), owner.name), method.name)
    assert set(found) == set(REQUEST_POLICIES)
    for key, entry in found.items():
        assert entry.request_policy is REQUEST_POLICIES[key]
        assert hasattr(entry, '__wrapped__') and not hasattr(entry.__wrapped__, '__wrapped__'), key
    # 同步编排不能占住唯一设备线程等待自身周期或停止/失能请求。
    for owner, name in (('ArmController', 'safe_home'), ('GripperController', 'grasp_gripper'),
                        ('GripperController', 'set_gripper_position'), ('GripperController', 'wait_gripper_target')):
        assert f'{owner}.{name}' not in found


def test_lifecycle_requires_explicit_request_runtime():
    from rebotarmcontroller.runtime.lifecycle import ControllerLifecycle
    with pytest.raises(TypeError):
        ControllerLifecycle()

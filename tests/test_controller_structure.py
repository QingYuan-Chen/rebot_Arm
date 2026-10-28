"""批次 6.5 的职责门禁；防止兼容门面重新吸收状态和控制事务。"""
import ast
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / 'src/rebotarmcontroller/rebotarmcontroller'


def test_manager_has_only_assembly_and_public_forwarding():
    tree = ast.parse((PACKAGE / 'hardware_manager.py').read_text())
    manager = next(node for node in tree.body if isinstance(node, ast.ClassDef))
    for method in manager.body:
        if not isinstance(method, ast.FunctionDef):
            continue
        assert not any(isinstance(node, ast.Attribute) and node.attr == 'setter'
                       for decorator in method.decorator_list for node in ast.walk(decorator))
        if method.name == '__init__':
            continue
        assert not method.name.startswith('_'), method.name
        body = [node for node in method.body if not (
            isinstance(node, ast.Expr) and isinstance(node.value, ast.Constant)
            and isinstance(node.value.value, str))]
        # 只允许一次查询或转发；分支、锁、轮询和状态赋值须归职责组件。
        assert len(body) == 1 and isinstance(body[0], (ast.Return, ast.Expr)), method.name
        assert not any(isinstance(node, (ast.Assign, ast.AugAssign, ast.If, ast.IfExp,
                                        ast.For, ast.While, ast.Try, ast.With))
                       for node in ast.walk(method)), method.name


def test_components_never_receive_manager_or_use_lazy_assembly():
    for name in ('arm_controller', 'gripper_controller', 'feedback_reader',
                 'gravity_controller', 'controller_cycle', 'controller_protection'):
        tree = ast.parse((PACKAGE / f'{name}.py').read_text())
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
    tree = ast.parse((PACKAGE / 'arm_stop.py').read_text())
    assert not any(isinstance(node, (ast.While, ast.For)) for node in ast.walk(tree))
    for node in ast.walk(tree):
        if isinstance(node, ast.Attribute):
            assert node.attr not in ('sleep', 'device_wait', 'request_feedback', 'poll_feedback_once')


def test_safety_fixtures_use_formal_construction():
    for name in ('test_hardware_manager_p0_safety.py', 'controller_fake_backend.py'):
        assert 'HardwareManager.__new__' not in (ROOT / 'tests' / name).read_text()

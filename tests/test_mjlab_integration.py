from __future__ import annotations

import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PLUGIN = ROOT / "src/rebotarm_simulation/rebotarm_simulation/mjlab_reach.py"


def test_mjlab_plugin_is_optional_and_registered_as_a_task_entrypoint() -> None:
    setup = (ROOT / "src/rebotarm_simulation/setup.py").read_text(encoding="utf-8")
    assert '"mjlab.tasks"' in setup
    assert "rebotarm_reach = rebotarm_simulation.mjlab_reach" in setup
    assert PLUGIN.is_file()


def test_mjlab_plugin_has_no_ros_or_hardware_imports() -> None:
    tree = ast.parse(PLUGIN.read_text(encoding="utf-8"))
    modules: list[str] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            modules.extend(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            modules.append(node.module)
    assert not any(module.startswith(("rclpy", "rebotarmcontroller", "serial")) for module in modules)


def test_mjlab_reference_keeps_cpu_baseline_and_hardware_boundary() -> None:
    doc = (ROOT / "docs/reference/commands/mjlab_rl.md").read_text(encoding="utf-8")
    assert "CPU MuJoCo" in doc
    assert "不启动 ROS" in doc
    assert "真实机械臂" in doc


def test_mjlab_runner_maps_actor_and_critic_to_public_observations() -> None:
    source = PLUGIN.read_text(encoding="utf-8")
    assert 'obs_groups={"actor": ("actor",), "critic": ("actor",)}' in source
    assert 'logger="tensorboard"' in source

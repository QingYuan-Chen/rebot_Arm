"""ROS repository owns models and references the independent training project."""
from pathlib import Path
import ast

ROOT = Path(__file__).resolve().parents[1]


def test_rl_source_and_training_dependencies_are_external():
    assert not (ROOT / "rebotarm_rl").exists()
    assert not (ROOT / "requirements/requirements-mjlab.txt").exists()
    assert '"mjlab.tasks"' not in (ROOT / "src/rebotarm_simulation/setup.py").read_text()
    assert not (ROOT / "src/rebotarm_simulation/rebotarm_simulation/mjlab_reach.py").exists()


def test_training_reference_explains_model_and_execution_boundary():
    doc = (ROOT / "docs/reference/commands/mjlab_rl.md").read_text()
    for text in ("/home/a/project/rebot_Arm_rl/MJLab", "provenance.json",
                 "SHA-256", "CPU MuJoCo", "不启动 ROS", "真实机械臂"):
        assert text in doc


def test_simulation_does_not_import_training_implementation():
    for path in (ROOT / "src/rebotarm_simulation/rebotarm_simulation").rglob("*.py"):
        for node in ast.walk(ast.parse(path.read_text())):
            modules = ([a.name for a in node.names] if isinstance(node, ast.Import)
                       else [node.module or ""] if isinstance(node, ast.ImportFrom) else [])
            assert not any(m.split(".")[0] in {"mjlab", "rebotarm_rl"} for m in modules), path

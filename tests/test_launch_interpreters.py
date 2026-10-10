"""Resolve installed-layout launch descriptions without starting ROS processes."""

from pathlib import Path
import subprocess
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize("mode", ["default", "environment", "argument"])
def test_relocated_launches_select_independent_interpreters(tmp_path, mode):
    # A fresh interpreter also isolates these tests from other tests' ROS stubs.
    result = subprocess.run(
        [sys.executable, "-", str(ROOT), str(tmp_path), mode],
        input=LAUNCH_CHECK,
        text=True,
        capture_output=True,
        timeout=45,
    )
    if result.returncode == 77:
        pytest.skip("ROS 2 launch is not installed in this Python environment")
    assert result.returncode == 0, result.stdout + result.stderr


LAUNCH_CHECK = r'''
import importlib.util
from pathlib import Path
import os
import shutil
import sys
from unittest.mock import patch

try:
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
    from launch_ros.actions import Node
except ImportError:
    sys.exit(77)

root, relocated, mode = Path(sys.argv[1]), Path(sys.argv[2]), sys.argv[3]
sys.path.insert(0, str(root / "src/rebotarm_bringup"))
env_names = {"mujoco": "REBOTARM_MUJOCO_PYTHON", "vision": "REBOTARM_VISION_PYTHON", "graspnet": "GRASPNET_PYTHON"}
expected = {}
for kind, name in env_names.items():
    os.environ.pop(name, None)
    expected[kind] = "python3"
    if mode != "default":
        os.environ[name] = f"/opt/custom-{kind}/bin/python"
        expected[kind] = os.environ[name]
    if mode == "argument":
        expected[kind] = f"/opt/override-{kind}/bin/python"
# The obsolete layouts deliberately exist: they must never override configuration.
for relative in (".venv-graspnet/bin/python", ".venv-vision/lib/python3.12/site-packages/marker", ".venv-mujoco/bin/python", "third_party/rebotarm_mujoco_venv/bin/python"):
    target = relocated / relative
    target.parent.mkdir(parents=True, exist_ok=True)
    target.touch()
os.chdir(relocated)
os.environ["PWD"] = str(relocated)

files = {
    "rebotarm_simulation": ["mujoco_sim.launch.py"],
    "rebotarm_bringup": ["visual_grasp_system.launch.py", "mujoco_moveit_sim.launch.py"],
    "rebotarm_vision": ["vision.launch.py", "vision_ubuntu.launch.py"],
}
seen = set()

def resolve(value, context):
    return perform_substitutions(context, normalize_to_list_of_substitutions(value))

def walk(entities):
    for entity in entities:
        yield entity
        if isinstance(entity, GroupAction):
            yield from walk(entity.get_sub_entities())

for package, names in files.items():
    for name in names:
        target = relocated / "share" / package / "launch" / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(root / "src" / package / "launch" / name, target)
        spec = importlib.util.spec_from_file_location("relocated_launch", target)
        module = importlib.util.module_from_spec(spec)
        with patch("ament_index_python.packages.get_package_share_directory", side_effect=lambda pkg: str(root / "src" / pkg)), patch("launch_ros.substitutions.find_package.get_package_share_directory", side_effect=lambda pkg: str(root / "src" / pkg)):
            spec.loader.exec_module(module)
            description = module.generate_launch_description()
            context = LaunchContext()
            context.environment.update(os.environ)
            if mode == "argument":
                context.launch_configurations.update({
                    "python_executable": expected["mujoco"],
                    "mujoco_python_executable": expected["mujoco"],
                    "vision_python_executable": expected["vision"],
                    "graspnet_python_executable": expected["graspnet"],
                })
            for entity in description.entities:
                if isinstance(entity, DeclareLaunchArgument):
                    entity.execute(context)
            entities = list(walk(description.entities))
            if name == "vision.launch.py":
                entities += module._launch_setup(context)
            if name == "visual_grasp_system.launch.py":
                # Resolve the public profile/default contract, then inspect the
                # two interpreter-owning stages without executing ROS actions.
                module._prepare_stages(context)
                for stage in ("visual_input", "grasp_candidate"):
                    stage_path = target.parent / "includes" / (stage + ".launch.py")
                    stage_path.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copy2(root / "src/rebotarm_bringup/launch/includes" / stage_path.name, stage_path)
                    stage_spec = importlib.util.spec_from_file_location("relocated_" + stage, stage_path)
                    stage_module = importlib.util.module_from_spec(stage_spec)
                    stage_spec.loader.exec_module(stage_module)
                    stage_entities = (stage_module._build(context) if hasattr(stage_module, "_build")
                                      else stage_module.generate_launch_description().entities)
                    entities += list(walk(stage_entities))
            for entity in entities:
                if isinstance(entity, SetEnvironmentVariable):
                    assert resolve(entity.name, context) != "PYTHONPATH", name
                if isinstance(entity, Node):
                    executable = resolve(entity.node_executable, context)
                    kind = {
                        "rebotarm_mujoco_node": "mujoco",
                        "rebotarm_graspnet_baseline_node": "graspnet",
                        "rebotarm_vision_node": "vision",
                        "rebotarm_grasp_tcp_frame": "vision",
                        "rebotarm_offline_yolo_node": "vision",
                    }.get(executable)
                    if kind:
                        assert resolve(entity.process_description.prefix, context) == expected[kind], (name, executable)
                        seen.add((name, kind))
                if isinstance(entity, IncludeLaunchDescription):
                    for key, value in entity.launch_arguments:
                        for kind in ("vision", "graspnet"):
                            if key == f"{kind}_python_executable":
                                assert resolve(value, context) == expected[kind], (name, key)
                                seen.add((name, kind))

assert all(any(filename == name for filename, kind in seen) for names in files.values() for name in names), seen
assert ("visual_grasp_system.launch.py", "vision") in seen
assert ("visual_grasp_system.launch.py", "graspnet") in seen
print("Relocated launch files resolved; no processes started:", mode)
'''

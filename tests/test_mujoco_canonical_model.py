from pathlib import Path
import sys
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
SIM_SRC = ROOT / "src" / "rebotarm_simulation"
if str(SIM_SRC) not in sys.path:
    sys.path.insert(0, str(SIM_SRC))


def test_canonical_robot_model_is_generated_and_contains_current_actuators():
    robot = ROOT / "src/rebotarm_simulation/models/rebotarm/robot.xml"
    assert robot.exists()
    text = robot.read_text(encoding="utf-8")
    assert "AUTO-GENERATED" in text
    root = ET.parse(robot).getroot()
    actuators = {node.get("name") for node in root.findall(".//actuator/*")}
    assert {"joint1_torque", "joint6_torque", "left_finger_force", "right_finger_force"} <= actuators


def test_canonical_model_contains_gemini2_payload_and_collision_defaults():
    robot = ROOT / "src/rebotarm_simulation/models/rebotarm/robot.xml"
    root = ET.parse(robot).getroot()
    assert root.find(".//body[@name='gemini2_camera']") is not None
    assert root.find(".//geom[@class='collision']") is not None


def test_canonical_scene_references_robot_and_has_home_keyframe():
    scene = ROOT / "src/rebotarm_simulation/models/rebotarm/scene.xml"
    root = ET.parse(scene).getroot()
    include = root.find("include")
    assert include is not None and include.get("file") == "robot.xml"
    assert root.find("./keyframe/key[@name='home']") is not None


def test_canonical_model_generator_check_is_available():
    import pytest

    pytest.importorskip("mujoco")
    from rebotarm_simulation.model_tools.urdf_to_mjcf import check_generated_model

    assert check_generated_model(
        ROOT,
        ROOT / "src/rebotarm_simulation/models/rebotarm/robot.xml",
    ) is True

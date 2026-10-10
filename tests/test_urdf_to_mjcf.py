from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET

import rebotarm_simulation.model_tools.urdf_to_mjcf as urdf_to_mjcf
from rebotarm_simulation.model_tools.urdf_to_mjcf import (
    authoritative_urdf_path,
    check_generated_model,
    collision_config_path,
    generate_mjcf_bytes,
    main,
    stage_urdf,
    write_generated_model,
)
import yaml


ROOT = Path(__file__).resolve().parents[1]


def test_authoritative_urdf_is_moveit_model() -> None:
    assert authoritative_urdf_path(ROOT) == (
        ROOT / "src/rebotarm_moveit_config/config/rebotarm.urdf"
    )


def test_stage_urdf_rewrites_package_meshes_without_modifying_source(tmp_path) -> None:
    source = authoritative_urdf_path(ROOT)
    before = source.read_bytes()

    staged = stage_urdf(source, ROOT, tmp_path)

    text = staged.read_text(encoding="utf-8")
    assert "package://" not in text
    assert "assets/base_link.STL" in text
    assert (tmp_path / "assets/base_link.STL").is_file()
    assert source.read_bytes() == before


def test_generation_is_byte_deterministic() -> None:
    assert generate_mjcf_bytes(ROOT) == generate_mjcf_bytes(ROOT)


def test_generated_model_warns_against_manual_edits() -> None:
    assert generate_mjcf_bytes(ROOT).startswith(
        b"<!-- AUTO-GENERATED from rebotarm.urdf; do not edit robot.xml manually. -->\n"
    )


def _generated_root() -> ET.Element:
    return ET.fromstring(generate_mjcf_bytes(ROOT))


def test_generated_model_preserves_named_bodies_and_joint_interface() -> None:
    root = _generated_root()
    body_names = {body.attrib["name"] for body in root.findall("worldbody//body")}
    joint_names = {joint.attrib["name"] for joint in root.findall("worldbody//joint")}

    assert body_names == {
        "base_link", "link1", "link2", "link3", "link4", "link5", "link6",
        "end_link", "left_finger_link", "right_finger_link",
        "gemini2_camera", "gemini2_mount", "gemini2_mount_part0",
    }
    assert joint_names == {
        "joint1", "joint2", "joint3", "joint4", "joint5", "joint6",
        "left_finger_joint", "right_finger_joint",
    }


def test_collision_config_covers_every_body_and_uses_fixed_display_groups() -> None:
    config = yaml.safe_load(collision_config_path(ROOT).read_text(encoding="utf-8"))

    assert config["schema_version"] == 1
    assert config["visual_group"] == 2
    assert config["collision_group"] == 3
    assert set(config["bodies"]) == {
        "base_link", "link1", "link2", "link3", "link4", "link5", "link6",
        "end_link", "left_finger_link", "right_finger_link",
    }


def test_generated_model_preserves_mesh_visuals_and_uses_hybrid_collisions() -> None:
    root = _generated_root()
    expected_meshes = {
        "base_link", "link1", "link2", "link3", "link4", "link5", "link6",
        "gripper_base", "left_finger", "right_finger",
        "gemini2_shell", "gemini2_mount_part0",
    }
    visuals = root.findall('.//geom[@class="visual"]')
    collisions = [geom for geom in root.findall('.//geom[@class="collision"]')
                  if not geom.attrib.get("name", "").startswith("gemini2_")]

    assert {geom.attrib["mesh"] for geom in visuals} == expected_meshes
    assert {geom.attrib["mesh"] for geom in collisions if geom.attrib["type"] == "mesh"} == {
        "left_finger", "right_finger",
    }
    assert {geom.attrib["type"] for geom in collisions} == {
        "box", "capsule", "cylinder", "mesh",
    }
    assert len(collisions) == 10
    visual_defaults = root.find('default/default[@class="visual"]/geom')
    assert all(geom.attrib.get("contype", visual_defaults.attrib["contype"]) == "0" for geom in visuals)
    assert all(geom.attrib.get("conaffinity", visual_defaults.attrib["conaffinity"]) == "0" for geom in visuals)
    assert all(geom.attrib.get("contype") == "1" for geom in collisions)
    assert all(geom.attrib.get("conaffinity") == "1" for geom in collisions)
    assert all("rgba" not in geom.attrib for geom in collisions)


def test_generated_model_adds_mujoco_control_and_observation_contract() -> None:
    root = _generated_root()
    joints = [
        "joint1", "joint2", "joint3", "joint4", "joint5", "joint6",
        "left_finger_joint", "right_finger_joint",
    ]
    actuators = root.findall("actuator/motor")
    assert [actuator.attrib["joint"] for actuator in actuators] == joints
    assert [actuator.attrib["name"] for actuator in actuators] == [
        "joint1_torque", "joint2_torque", "joint3_torque",
        "joint4_torque", "joint5_torque", "joint6_torque",
        "left_finger_force", "right_finger_force",
    ]
    assert all(actuator.attrib["ctrllimited"] == "true" for actuator in actuators)
    assert all(actuator.attrib["forcelimited"] == "true" for actuator in actuators)
    assert actuators[-2].attrib["ctrlrange"] == "-1.5 1.5"
    assert actuators[-1].attrib["forcerange"] == "-1.5 1.5"
    dynamics = {joint.attrib["name"]: joint for joint in root.findall("worldbody//joint")}
    assert dynamics["joint1"].attrib["damping"] == "10"
    assert dynamics["joint4"].attrib["damping"] == "3"
    assert dynamics["joint6"].attrib["damping"] == "1"
    assert dynamics["left_finger_joint"].attrib["damping"] == "20"

    coupling = root.find("equality/joint")
    assert coupling is not None
    assert coupling.attrib["joint1"] == "right_finger_joint"
    assert coupling.attrib["joint2"] == "left_finger_joint"
    assert coupling.attrib["polycoef"] == "0 -1 0 0 0"

    exclusions = root.findall("contact/exclude")
    assert len(exclusions) == 12
    assert root.find('.//site[@name="ee_site"]') is not None
    assert root.find('.//site[@name="wrist_camera_mount"]') is not None
    assert len(root.findall("sensor/jointpos")) == 8
    assert len(root.findall("sensor/jointvel")) == 8
    assert len(root.findall("sensor/actuatorfrc")) == 8
    assert root.find('sensor/framepos[@objname="ee_site"]') is not None
    assert root.find('sensor/framequat[@objname="ee_site"]') is not None


def test_write_generated_model_is_checkable_and_leaves_no_temporary_file(tmp_path) -> None:
    output = tmp_path / "robot.xml"

    write_generated_model(ROOT, output)

    assert output.read_bytes() == generate_mjcf_bytes(ROOT)
    assert check_generated_model(ROOT, output)
    assert list(tmp_path.glob("*.tmp")) == []


def test_check_rejects_stale_generated_model(tmp_path) -> None:
    output = tmp_path / "robot.xml"
    output.write_text("<mujoco/>\n", encoding="utf-8")

    assert not check_generated_model(ROOT, output)


def test_cli_generates_then_checks_explicit_output(tmp_path, capsys) -> None:
    output = tmp_path / "robot.xml"
    common = ["--repo-root", str(ROOT), "--output", str(output)]

    assert main(common) == 0
    assert main([*common, "--check"]) == 0
    assert "up to date" in capsys.readouterr().out

    output.write_text("<mujoco/>\n", encoding="utf-8")
    assert main([*common, "--check"]) == 1
    assert "regenerate" in capsys.readouterr().out


def test_cli_checks_packaged_resources_without_repo_root(capsys) -> None:
    assert main(["--check"]) == 0
    assert "up to date" in capsys.readouterr().out


def test_cli_default_does_not_derive_repo_root_from_installed_module(
    monkeypatch, tmp_path, capsys
) -> None:
    installed_module = (
        tmp_path
        / "install/rebotarm_simulation/lib/python3.12/site-packages"
        / "rebotarm_simulation/urdf_to_mjcf.py"
    )
    monkeypatch.setattr(urdf_to_mjcf, "__file__", str(installed_module))

    assert main(["--check"]) == 0
    assert "up to date" in capsys.readouterr().out

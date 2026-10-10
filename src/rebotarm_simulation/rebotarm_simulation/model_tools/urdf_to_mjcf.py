from __future__ import annotations

import argparse
from pathlib import Path
import shutil
import tempfile
from typing import Sequence
import xml.etree.ElementTree as ET

import mujoco
import yaml

from rebotarm_simulation.core.resource_paths import package_resource
from rebotarm_simulation.model_tools.gemini2_payload import add_gemini2_payload


PACKAGE_MESH_PREFIX = "package://rebotarm_moveit_config/meshes/"
from rebotarm_simulation.core.model_contract import JOINT_NAMES, actuator_name_for_joint

JOINTS = list(JOINT_NAMES)
SUPPORTED_COLLISION_TYPES = {"box", "capsule", "cylinder", "mesh"}


def _package_path(repo_root: Path | None, package: str, relative: str) -> Path:
    if repo_root is not None:
        return repo_root / "src" / package / relative
    return package_resource(package, relative)


def authoritative_urdf_path(repo_root: Path | None) -> Path:
    return _package_path(repo_root, "rebotarm_moveit_config", "config/rebotarm.urdf")


def _model_directory(repo_root: Path | None) -> Path:
    return _package_path(repo_root, "rebotarm_simulation", "models/rebotarm")


def collision_config_path(repo_root: Path | None) -> Path:
    return _package_path(repo_root, "rebotarm_simulation", "config/mujoco_collision.yaml")


def stage_urdf(source: Path, repo_root: Path | None, temporary_dir: Path) -> Path:
    root = ET.parse(source).getroot()
    extension = ET.SubElement(root, "mujoco")
    ET.SubElement(
        extension,
        "compiler",
        {"discardvisual": "false", "fusestatic": "false", "strippath": "false"},
    )
    source_assets = _package_path(repo_root, "rebotarm_moveit_config", "meshes")
    staged_assets = temporary_dir / "assets"
    staged_assets.mkdir(parents=True, exist_ok=True)

    for link in root.findall("link"):
        for role in ("visual", "collision"):
            for index, element in enumerate(link.findall(role)):
                element.set("name", f'{link.attrib["name"]}_{role}_{index}')

    for mesh in root.findall(".//mesh"):
        filename = mesh.attrib.get("filename", "")
        if not filename.startswith(PACKAGE_MESH_PREFIX):
            raise ValueError(f"unsupported URDF mesh URI: {filename}")
        basename = filename.removeprefix(PACKAGE_MESH_PREFIX)
        source_mesh = source_assets / basename
        if not source_mesh.is_file():
            raise FileNotFoundError(source_mesh)
        shutil.copy2(source_mesh, staged_assets / basename)
        mesh.set("filename", f"assets/{basename}")

    staged = temporary_dir / source.name
    ET.ElementTree(root).write(staged, encoding="utf-8", xml_declaration=True)
    return staged


def generate_mjcf_bytes(repo_root: Path | None) -> bytes:
    source = authoritative_urdf_path(repo_root)
    with tempfile.TemporaryDirectory(prefix="rebotarm-urdf-") as directory:
        staged = stage_urdf(source, repo_root, Path(directory))
        spec = mujoco.MjSpec.from_file(str(staged))
        spec.compile()
        root = ET.fromstring(spec.to_xml())
        urdf_root = ET.parse(source).getroot()
        collision_config = _load_collision_config(repo_root)
        _configure_compiler(root, collision_config)
        _classify_geoms(root)
        _replace_collision_geoms(root, collision_config)
        _add_contact_exclusions(root)
        _add_finger_coupling(root)
        _add_sites(root)
        _add_joint_dynamics(root, _load_joint_dynamics(repo_root))
        _add_actuators(root, urdf_root, repo_root)
        _add_sensors(root)
        add_gemini2_payload(root, repo_root)
    return _canonicalize(root)


def _configure_compiler(root: ET.Element, collision_config: dict[str, object]) -> None:
    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(root, "compiler")
    compiler.set("angle", "radian")
    default = ET.Element("default")
    ET.SubElement(
        default,
        "geom",
        {"solref": "0.01 1", "solimp": "0.9 0.95 0.001", "friction": "0.8 0.02 0.001"},
    )
    visual = ET.SubElement(default, "default", {"class": "visual"})
    ET.SubElement(
        visual,
        "geom",
        {
            "contype": "0",
            "conaffinity": "0",
            "group": str(collision_config["visual_group"]),
        },
    )
    collision = ET.SubElement(default, "default", {"class": "collision"})
    ET.SubElement(
        collision,
        "geom",
        {
            "contype": "1",
            "conaffinity": "1",
            "group": str(collision_config["collision_group"]),
            "rgba": "0.2 0.5 0.8 0.12",
        },
    )
    compiler_index = list(root).index(compiler)
    root.insert(compiler_index + 1, default)


def _classify_geoms(root: ET.Element) -> None:
    for body in root.findall("worldbody//body"):
        body_name = body.attrib["name"]
        mesh_counts: dict[str, int] = {}
        for geom in body.findall("geom"):
            mesh = geom.attrib["mesh"]
            index = mesh_counts.get(mesh, 0)
            mesh_counts[mesh] = index + 1
            is_visual = geom.attrib.get("contype") == "0"
            role = "visual" if is_visual else "collision"
            geom.set("name", f"{body_name}_{mesh}_{role}")
            geom.set("class", role)
            geom.set("contype", "0" if is_visual else "1")
            geom.set("conaffinity", "0" if is_visual else "1")
            geom.set("group", "2" if is_visual else "3")
            geom.attrib.pop("density", None)
            if not is_visual:
                geom.attrib.pop("rgba", None)
            if not is_visual and mesh in {"left_finger", "right_finger"}:
                geom.set("friction", "1.2 0.02 0.001")


def _load_collision_config(repo_root: Path | None) -> dict[str, object]:
    path = collision_config_path(repo_root)
    payload = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if payload.get("schema_version") != 1:
        raise ValueError(f"expected schema_version 1 in {path}")
    for field in ("visual_group", "collision_group"):
        value = payload.get(field)
        if not isinstance(value, int) or not 0 <= value <= 5:
            raise ValueError(f"{field} must be an integer from 0 to 5 in {path}")
    bodies = payload.get("bodies")
    if not isinstance(bodies, dict) or not bodies:
        raise ValueError(f"expected non-empty bodies mapping in {path}")
    return payload


def _numbers(values: object, count: int, field: str) -> str:
    if not isinstance(values, list) or len(values) != count:
        raise ValueError(f"collision {field} must contain {count} numbers")
    converted: list[float] = []
    for value in values:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"collision {field} must contain only numbers")
        converted.append(float(value))
    return " ".join(f"{value:g}" for value in converted)


def _sizes(values: object, count: int) -> str:
    result = _numbers(values, count, "size")
    if any(float(value) <= 0.0 for value in result.split()):
        raise ValueError("collision size values must be positive")
    return result


def _collision_geom_attributes(
    body_name: str,
    index: int,
    specification: object,
    collision_group: int,
) -> dict[str, str]:
    if not isinstance(specification, dict):
        raise ValueError(f"collision entry {body_name}[{index}] must be a mapping")
    geom_type = specification.get("type")
    if geom_type not in SUPPORTED_COLLISION_TYPES:
        raise ValueError(f"unsupported collision type for {body_name}[{index}]: {geom_type}")
    suffix = specification.get("name", str(index))
    if not isinstance(suffix, str) or not suffix:
        raise ValueError(f"collision name for {body_name}[{index}] must be non-empty")
    attributes = {
        "name": f"{body_name}_{suffix}_collision",
        "class": "collision",
        "type": str(geom_type),
        "contype": "1",
        "conaffinity": "1",
        "group": str(collision_group),
    }
    if "pos" in specification:
        attributes["pos"] = _numbers(specification["pos"], 3, "pos")
    if "quat" in specification:
        attributes["quat"] = _numbers(specification["quat"], 4, "quat")
    if "friction" in specification:
        attributes["friction"] = _numbers(specification["friction"], 3, "friction")
    if geom_type == "mesh":
        mesh = specification.get("mesh")
        if not isinstance(mesh, str) or not mesh:
            raise ValueError(f"mesh collision {body_name}[{index}] requires mesh")
        attributes["mesh"] = mesh
    elif geom_type == "capsule":
        attributes["fromto"] = _numbers(specification.get("fromto"), 6, "fromto")
        endpoints = tuple(float(value) for value in attributes["fromto"].split())
        if endpoints[:3] == endpoints[3:]:
            raise ValueError(f"capsule collision {body_name}[{index}] has zero length")
        attributes["size"] = _sizes(specification.get("size"), 1)
    elif geom_type == "box":
        attributes["size"] = _sizes(specification.get("size"), 3)
    elif geom_type == "cylinder":
        attributes["size"] = _sizes(specification.get("size"), 2)
    return attributes


def _replace_collision_geoms(root: ET.Element, config: dict[str, object]) -> None:
    bodies = config["bodies"]
    assert isinstance(bodies, dict)
    model_bodies = {
        body.attrib["name"]: body for body in root.findall("worldbody//body")
    }
    if set(bodies) != set(model_bodies):
        missing = sorted(set(model_bodies) - set(bodies))
        unknown = sorted(set(bodies) - set(model_bodies))
        raise ValueError(f"collision config body mismatch: missing={missing}, unknown={unknown}")
    collision_group = int(config["collision_group"])
    visual_group = int(config["visual_group"])
    mesh_names = {mesh.attrib["name"] for mesh in root.findall("asset/mesh")}
    for body_name, body in model_bodies.items():
        for geom in list(body.findall("geom")):
            if geom.attrib.get("class") == "collision":
                body.remove(geom)
            elif geom.attrib.get("class") == "visual":
                geom.set("group", str(visual_group))
        specifications = bodies[body_name]
        if not isinstance(specifications, list) or not specifications:
            raise ValueError(f"collision config for {body_name} must be a non-empty list")
        for index, specification in enumerate(specifications):
            attributes = _collision_geom_attributes(
                body_name, index, specification, collision_group
            )
            mesh = attributes.get("mesh")
            if mesh is not None and mesh not in mesh_names:
                raise ValueError(f"unknown collision mesh for {body_name}: {mesh}")
            ET.SubElement(body, "geom", attributes)


def _add_contact_exclusions(root: ET.Element) -> None:
    contact = ET.SubElement(root, "contact")
    for body1, body2 in (
        ("base_link", "link1"),
        ("link1", "link2"),
        ("link2", "link3"),
        ("link3", "link4"),
        ("link4", "link5"),
        ("link5", "link6"),
        ("link6", "end_link"),
        ("end_link", "left_finger_link"),
        ("end_link", "right_finger_link"),
        ("link2", "link4"),
        ("link4", "link6"),
        ("left_finger_link", "right_finger_link"),
    ):
        ET.SubElement(contact, "exclude", {"body1": body1, "body2": body2})


def _add_finger_coupling(root: ET.Element) -> None:
    equality = ET.SubElement(root, "equality")
    ET.SubElement(
        equality,
        "joint",
        {
            "name": "finger_coupling",
            "joint1": "right_finger_joint",
            "joint2": "left_finger_joint",
            "polycoef": "0 -1 0 0 0",
        },
    )


def _add_sites(root: ET.Element) -> None:
    end_link = root.find('.//body[@name="end_link"]')
    if end_link is None:
        raise ValueError("converted MJCF is missing end_link")
    ET.SubElement(end_link, "site", {"name": "ee_site", "pos": "-0.04 0 0", "size": "0.008"})
    ET.SubElement(
        end_link,
        "site",
        {"name": "wrist_camera_mount", "pos": "-0.04 0 0.04", "quat": "1 0 0 0", "size": "0.005"},
    )


def _load_joint_dynamics(repo_root: Path | None) -> dict[str, dict[str, float]]:
    path = _package_path(
        repo_root, "rebotarm_simulation", "config/motor_control_calibration.yaml"
    )
    payload = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    dynamics = payload.get("model_dynamics")
    if not isinstance(dynamics, dict):
        raise ValueError(f"expected model_dynamics mapping in {path}")
    result: dict[str, dict[str, float]] = {}
    for joint_name in JOINTS:
        values = dynamics.get(joint_name)
        if not isinstance(values, dict):
            raise ValueError(f"missing model_dynamics for {joint_name}")
        result[joint_name] = {
            "damping": float(values["damping"]),
            "armature": float(values["armature"]),
            "frictionloss": float(values["frictionloss"]),
        }
    return result


def _add_joint_dynamics(root: ET.Element, dynamics: dict[str, dict[str, float]]) -> None:
    for joint in root.findall("worldbody//joint"):
        name = joint.attrib["name"]
        if name not in dynamics:
            continue
        for key, value in dynamics[name].items():
            joint.set(key, f"{value:g}")


def _add_actuators(
    root: ET.Element, urdf_root: ET.Element, repo_root: Path | None
) -> None:
    limits = {
        joint.attrib["name"]: joint.find("limit")
        for joint in urdf_root.findall("joint")
        if joint.attrib.get("name") in JOINTS
    }
    actuator = ET.SubElement(root, "actuator")
    finger_force_limit = _load_gripper_force_limit(repo_root)
    for joint_name in JOINTS:
        limit = limits[joint_name]
        if limit is None:
            raise ValueError(f"URDF joint has no limit: {joint_name}")
        effort = (
            finger_force_limit
            if joint_name in {"left_finger_joint", "right_finger_joint"}
            else float(limit.attrib["effort"])
        )
        if joint_name in {"left_finger_joint", "right_finger_joint"}:
            # Keep both force-clamping layers on the same simulation-only setting.
            joint = root.find(f'.//joint[@name="{joint_name}"]')
            if joint is None:
                raise ValueError(f"missing MuJoCo finger joint: {joint_name}")
            joint.set("actuatorfrcrange", f"-{effort:g} {effort:g}")
            joint.set("actuatorfrclimited", "true")
        ET.SubElement(
            actuator,
            "motor",
            {
                "name": actuator_name_for_joint(joint_name),
                "joint": joint_name,
                "gear": "1",
                "ctrlrange": f"-{effort} {effort}",
                "ctrllimited": "true",
                "forcelimited": "true",
                "forcerange": f"-{effort} {effort}",
            },
        )


def _load_gripper_force_limit(repo_root: Path | None) -> float:
    path = _package_path(
        repo_root, "rebotarm_simulation", "config/motor_control_calibration.yaml"
    )
    payload = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    try:
        value = float(payload["gripper"]["finger_force_limit_n"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"expected gripper.finger_force_limit_n in {path}") from exc
    if value <= 0.0:
        raise ValueError(f"gripper.finger_force_limit_n must be positive in {path}")
    return value


def _add_sensors(root: ET.Element) -> None:
    sensor = ET.SubElement(root, "sensor")
    for joint_name in JOINTS:
        prefix = joint_name.removesuffix("_joint")
        ET.SubElement(sensor, "jointpos", {"name": f"{prefix}_pos", "joint": joint_name})
        ET.SubElement(sensor, "jointvel", {"name": f"{prefix}_vel", "joint": joint_name})
    for joint_name in JOINTS:
        prefix = joint_name.removesuffix("_joint")
        ET.SubElement(
            sensor,
            "actuatorfrc",
            {"name": f"{prefix}_force", "actuator": actuator_name_for_joint(joint_name)},
        )
    ET.SubElement(sensor, "framepos", {"name": "ee_position", "objtype": "site", "objname": "ee_site"})
    ET.SubElement(sensor, "framequat", {"name": "ee_orientation", "objtype": "site", "objname": "ee_site"})


def _canonicalize(root: ET.Element) -> bytes:
    for mesh in root.findall("asset/mesh"):
        mesh.attrib.pop("content_type", None)
    ET.indent(root, space="  ")
    xml = ET.tostring(root, encoding="unicode", short_empty_elements=True)
    warning = "<!-- AUTO-GENERATED from rebotarm.urdf; do not edit robot.xml manually. -->\n"
    return (warning + xml.replace("\r\n", "\n").rstrip() + "\n").encode("utf-8")


def check_generated_model(repo_root: Path | None, output: Path) -> bool:
    return output.is_file() and output.read_bytes() == generate_mjcf_bytes(repo_root)


def write_generated_model(repo_root: Path | None, output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_name(f".{output.name}.tmp")
    try:
        temporary.write_bytes(generate_mjcf_bytes(repo_root))
        temporary.replace(output)
    finally:
        temporary.unlink(missing_ok=True)


def _default_output(repo_root: Path | None) -> Path:
    return _model_directory(repo_root) / "robot.xml"


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Generate reBotArm MJCF from the authoritative URDF")
    parser.add_argument("--repo-root", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args(argv)
    repo_root = args.repo_root.resolve() if args.repo_root is not None else None
    output = (args.output or _default_output(repo_root)).resolve()

    if args.check:
        if check_generated_model(repo_root, output):
            print(f"MJCF is up to date: {output}")
            return 0
        source = (
            f'--repo-root "{repo_root}"'
            if repo_root is not None
            else "the installed package resources"
        )
        print(f"MJCF is stale for {source}; regenerate with rebotarm_urdf_to_mjcf")
        return 1

    write_generated_model(repo_root, output)
    print(f"Generated MJCF: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

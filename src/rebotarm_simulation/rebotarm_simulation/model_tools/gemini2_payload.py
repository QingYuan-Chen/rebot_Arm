"""Add a simulation-only Gemini 2 / Seeed bracket with explicit inertias.

CAD assembly coordinates are preserved. Mount mass and mounting pose are
documented estimates, never camera optical TF or calibrated hardware values.
"""
from pathlib import Path
import json
import math
import xml.etree.ElementTree as ET

import yaml


def _vector(value, size, label):
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise ValueError(f"{label} must contain {size} values")
    result = [float(x) for x in value]
    if not all(math.isfinite(x) for x in result):
        raise ValueError(f"{label} must be finite")
    if "quaternion" in label and abs(sum(x*x for x in result) - 1) > 1e-6:
        raise ValueError(f"{label} must have unit length")
    return result


def _text(values):
    return " ".join(f"{x:.12g}" for x in values)


def add_gemini2_payload(root: ET.Element, repo_root: Path) -> None:
    package = repo_root / "src/rebotarm_simulation"
    config = yaml.safe_load((package / "config/gemini2_payload.yaml").read_text())
    if config["enabled"] is False:
        return
    if config["enabled"] is not True:
        raise ValueError("payload enabled must be boolean")
    parent = root.find(f'.//body[@name="{config["parent_body"]}"]')
    if parent is None:
        raise ValueError("Gemini 2 parent body is missing")
    properties = json.loads((package / "models/rebotarm/assets/gemini2/cad_mass_properties.json").read_text())
    vectors = {key: _vector(config[key], size, key) for key, size in (
        ("mount_position_m", 3), ("mount_quaternion_wxyz", 4),
        ("camera_center_in_mount_m", 3), ("camera_quaternion_wxyz", 4),
        ("camera_size_depth_width_height_m", 3), ("camera_mesh_center_m", 3),
        ("mount_part_masses_kg", len(properties)),
    )}
    mass = float(config["camera_mass_kg"])
    if not math.isfinite(mass) or mass <= 0 or any(
        x <= 0 for key in ("camera_size_depth_width_height_m", "mount_part_masses_kg")
        for x in vectors[key]
    ):
        raise ValueError("payload masses and sizes must be positive")
    asset = root.find("asset")
    if asset is None:
        asset = ET.SubElement(root, "asset")
    assembly = ET.SubElement(parent, "body", name="gemini2_mount", pos=_text(vectors["mount_position_m"]),
                             quat=_text(vectors["mount_quaternion_wxyz"]))
    for i, part in enumerate(properties):
        name = f"gemini2_mount_part{i}"
        ET.SubElement(asset, "mesh", name=name, file=f"assets/gemini2/mount_part{i}.stl")
        body = ET.SubElement(assembly, "body", name=name)
        part_mass = vectors["mount_part_masses_kg"][i]
        inertia = [v * part_mass / part["mass_kg"] for v in part["fullinertia_kg_m2"]]
        ET.SubElement(body, "inertial", pos=_text(part["com_m"]), mass=str(part_mass), fullinertia=_text(inertia))
        # Separate render mesh and conservative convex-hull contact mesh.
        ET.SubElement(body, "geom", name=name+"_visual", type="mesh", mesh=name,
                      attrib={"class": "visual"}, rgba="0.50 0.50 0.50 1", mass="0")
        ET.SubElement(body, "geom", name=name+"_collision", type="mesh", mesh=name,
                      attrib={"class": "collision"}, mass="0")
    ET.SubElement(asset, "mesh", name="gemini2_shell", file="assets/gemini2/camera.STL")
    camera = ET.SubElement(assembly, "body", name="gemini2_camera", pos=_text(vectors["camera_center_in_mount_m"]),
                           quat=_text(vectors["camera_quaternion_wxyz"]))
    x, y, z = vectors["camera_size_depth_width_height_m"]
    inertia = [mass*(y*y+z*z)/12, mass*(x*x+z*z)/12, mass*(x*x+y*y)/12]
    ET.SubElement(camera, "inertial", pos="0 0 0", mass=str(mass), diaginertia=_text(inertia))
    ET.SubElement(camera, "geom", name="gemini2_camera_visual", type="mesh", mesh="gemini2_shell",
                  pos=_text([-v for v in vectors["camera_mesh_center_m"]]),
                  attrib={"class": "visual"}, rgba="0.23 0.25 0.28 1", mass="0")
    ET.SubElement(camera, "geom", name="gemini2_camera_collision", type="box", size=_text([x/2, y/2, z/2]),
                  attrib={"class": "collision"}, mass="0")

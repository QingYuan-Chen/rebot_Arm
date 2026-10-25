"""Apply provisional payload inertials to the simulation's staged URDF only."""
from __future__ import annotations

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


def apply_d405_inertials(root: ET.Element, config_path: Path) -> None:
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if set(config) != {"d405_mount_link", "d405_link"}:
        raise ValueError("D405 inertia config must cover mount and camera")
    for name, properties in config.items():
        link = root.find(f'link[@name="{name}"]')
        if link is None:
            raise ValueError(f"D405 payload link is missing: {name}")
        mass = float(properties["mass_kg"])
        com = properties["com_m"]
        inertia = properties["inertia_kg_m2"]
        if (len(com) != 3 or len(inertia) != 6
                or not all(math.isfinite(v) for v in [mass, *com, *inertia])
                or mass <= 0 or any(v <= 0 for v in inertia[:3])):
            raise ValueError(f"invalid D405 inertial estimate: {name}")
        if link.find("inertial") is not None:
            raise ValueError(f"D405 real inertial must not be overwritten: {name}")
        element = ET.SubElement(link, "inertial")
        ET.SubElement(element, "origin", xyz=" ".join(f"{v:.15g}" for v in com))
        ET.SubElement(element, "mass", value=f"{mass:.15g}")
        ET.SubElement(element, "inertia", **dict(zip(
            ("ixx", "iyy", "izz", "ixy", "ixz", "iyz"),
            (f"{v:.15g}" for v in inertia),
        )))

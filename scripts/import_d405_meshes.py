#!/usr/bin/env python3
"""Convert the checked D405 sources to metre STL assets without changing CAD axes.

Uses the optional CAD environment in docs/setup/dependencies/cad.md.
Assembly transforms are in the canonical URDF; see features/d405_wrist_model.md.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import struct

import numpy as np

from import_soft_gripper_meshes import STL_RECORD, binary_stl, step_triangles_mm

ROOT = Path(__file__).resolve().parents[1]
SOURCE_HASHES = {
    "D405_305_Mount.step": "99df53e11fcdd192830ef205d080db310448cc39b891b62a8f64316f6565c28d",
    "official_d405_mm.stl": "a248f41149d12b28311829feecbe7a80cf1481fd05e0f5df2c4c7ecd556edd48",
}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-dir", type=Path, default=ROOT / "src/rebotarm_moveit_config/cad/d405")
    parser.add_argument("--output-dir", type=Path, action="append")
    args = parser.parse_args()
    for name, expected in SOURCE_HASHES.items():
        if hashlib.sha256((args.source_dir / name).read_bytes()).hexdigest() != expected:
            raise ValueError(f"source changed; recheck assembly: {name}")
    mount = step_triangles_mm(args.source_dir / "D405_305_Mount.step")
    content = (args.source_dir / "official_d405_mm.stl").read_bytes()
    count = struct.unpack_from("<I", content, 80)[0]
    if len(content) != 84 + 50 * count:
        raise ValueError("expected binary official D405 STL")
    camera = np.frombuffer(content, STL_RECORD, count=count, offset=84)["vertices"]
    for destination in args.output_dir or [ROOT / "src/rebotarm_moveit_config/meshes"]:
        destination.mkdir(parents=True, exist_ok=True)
        for name, triangles in (("d405_mount", mount), ("d405", camera)):
            data = binary_stl(triangles * .001)
            data = b"reBotArm D405; metres; import_d405_meshes.py".ljust(80, b" ") + data[80:]
            path = destination / f"{name}.stl"
            path.write_bytes(data)
            print(f"{path}: {len(triangles)} triangles")

    # Geometric inertia per unit mass, independent of material/infill assumptions.
    from OCP.STEPControl import STEPControl_Reader
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps
    reader = STEPControl_Reader()
    reader.ReadFile(str(args.source_dir / "D405_305_Mount.step"))
    reader.TransferRoots()
    properties = GProp_GProps()
    BRepGProp.VolumeProperties_s(reader.OneShape(), properties)
    inertia = properties.MatrixOfInertia()
    matrix = np.array([[inertia.Value(i, j) for j in range(1, 4)] for i in range(1, 4)])
    geometry = {
        "volume_m3": properties.Mass() * 1e-9,
        "com_m": (np.array(properties.CentreOfMass().Coord()) * .001).tolist(),
        "inertia_per_kg_m2": (matrix / properties.Mass() * 1e-6).tolist(),
    }
    (args.source_dir / "mount_geometry.json").write_text(json.dumps(geometry, indent=2) + "\n")


if __name__ == "__main__":
    main()

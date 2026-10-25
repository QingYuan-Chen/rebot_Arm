#!/usr/bin/env python3
"""Build metre-based gripper meshes from the checked CAD sources, without ROS.

Requires the optional CAD dependencies in docs/setup/dependencies/cad.md.
Transforms and calibration boundaries: docs/implemented/features/soft_gripper_model.md.
"""
from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
import struct

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
SOURCE_HASHES = {
    "Soft_Gripper_Finger.step": "007f79d8b666973f831d02dce1e2ff127290681b24b350fc6428c83ff69cc33f",
    "Soft_Gripper_Mount.step": "ae4d101d26aece0dc0730f84f89cde908a1653fc7b66baa28ea55f436121af99",
    "original_left_finger.stl": "cf8a3acbbfbcaf15e3de0663e80956a1601d3595ea451107174620e8bfcbeb5c",
    "original_right_finger.stl": "e4b5ebef66e0b2b3e5d6e0ab59342716cca585db94c5bd4fc95b79f21c6657ef",
}
STL_RECORD = np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
# CAD millimetres. Finger's three hole axes match the mount after Rx(pi).
FINGER_TO_MOUNT_ROTATION = np.diag([1.0, -1.0, -1.0])
FINGER_TO_MOUNT_TRANSLATION_MM = np.array([-94.209253443758, 141.125760638401, 0.0])
# Mount back face Y=-15 -> original slider front X=-70.60932368 mm.
# Mount hole X=-5.15/+4.85 -> slider Y=24.8/14.8 mm; Z=+/-7.5 mm.
MOUNT_TO_LEFT_ROTATION = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
MOUNT_TO_LEFT_TRANSLATION_MM = np.array([-55.60932368, 19.65, 0.0])


def retained_drive_triangles(path: Path) -> np.ndarray:
    """Keep the rack and slider, preserving their original float32 vertices."""
    content = path.read_bytes()
    count = struct.unpack_from("<I", content, 80)[0]
    if len(content) != 84 + 50 * count:
        raise ValueError(f"not a binary STL: {path}")
    triangles = np.frombuffer(content, STL_RECORD, count=count, offset=84)["vertices"]
    vertices, inverse = np.unique(np.round(triangles.reshape(-1, 3), 7), axis=0, return_inverse=True)
    parents = list(range(len(vertices)))

    def find(index: int) -> int:
        while parents[index] != index:
            parents[index] = parents[parents[index]]
            index = parents[index]
        return index

    for a, b, c in inverse.reshape(-1, 3):
        parents[find(int(b))] = find(int(a))
        parents[find(int(c))] = find(int(a))
    labels = np.array([find(int(i)) for i in inverse.reshape(-1, 3)[:, 0]])
    components = [triangles[labels == label] for label in np.unique(labels)]
    if sorted(len(component) for component in components) != [1080, 1542, 2148]:
        raise ValueError(f"unexpected original rack/slider/finger topology: {path}")
    # Identified by the checked source topology; 1542 triangles form the old finger.
    return np.concatenate([component for component in components if len(component) != 1542])


def step_triangles_mm(path: Path) -> np.ndarray:
    from OCP.BRep import BRep_Tool
    from OCP.BRepCheck import BRepCheck_Analyzer
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.STEPControl import STEPControl_Reader
    from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopLoc import TopLoc_Location
    from OCP.TopoDS import TopoDS

    reader = STEPControl_Reader()
    if reader.ReadFile(str(path)) != IFSelect_RetDone:
        raise ValueError(f"cannot read STEP: {path}")
    reader.TransferRoots()
    shape = reader.OneShape()
    if not BRepCheck_Analyzer(shape).IsValid():
        raise ValueError(f"invalid STEP shape: {path}")
    mesher = BRepMesh_IncrementalMesh(shape, 0.08, False, 0.25, False)
    mesher.Perform()
    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    triangles = []
    while explorer.More():
        face = TopoDS.Face_s(explorer.Current())
        location = TopLoc_Location()
        mesh = BRep_Tool.Triangulation_s(face, location)
        if mesh is None:
            raise ValueError(f"unmeshed STEP face: {path}")
        transform = location.Transformation()
        nodes = np.array([mesh.Node(i).Transformed(transform).Coord() for i in range(1, mesh.NbNodes() + 1)])
        indices = np.array([mesh.Triangle(i).Get() for i in range(1, mesh.NbTriangles() + 1)]) - 1
        if face.Orientation() == TopAbs_REVERSED:
            indices = indices[:, [0, 2, 1]]
        triangles.append(nodes[indices])
        explorer.Next()
    return np.concatenate(triangles)


def binary_stl(triangles_m: np.ndarray) -> bytes:
    normals = np.cross(triangles_m[:, 1] - triangles_m[:, 0], triangles_m[:, 2] - triangles_m[:, 0])
    lengths = np.linalg.norm(normals, axis=1)
    if not np.isfinite(triangles_m).all() or np.any(lengths <= 0):
        raise ValueError("non-finite or degenerate mesh triangles")
    records = np.zeros(len(triangles_m), dtype=STL_RECORD)
    records["normal"] = normals / lengths[:, None]
    records["vertices"] = triangles_m
    header = b"reBotArm soft gripper; metres; import_soft_gripper_meshes.py".ljust(80, b" ")
    return header + struct.pack("<I", len(records)) + records.tobytes()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-dir", type=Path, default=ROOT / "src/rebotarm_moveit_config/cad/soft_gripper")
    parser.add_argument("--output-dir", type=Path, action="append", help="Repeat to synchronize model asset directories")
    args = parser.parse_args()
    destinations = args.output_dir or [ROOT / "src/rebotarm_moveit_config/meshes"]
    for name, expected in SOURCE_HASHES.items():
        if hashlib.sha256((args.source_dir / name).read_bytes()).hexdigest() != expected:
            raise ValueError(f"CAD source changed; recheck mounting transforms: {name}")
    mount = step_triangles_mm(args.source_dir / "Soft_Gripper_Mount.step")
    finger = step_triangles_mm(args.source_dir / "Soft_Gripper_Finger.step")
    finger = finger @ FINGER_TO_MOUNT_ROTATION.T + FINGER_TO_MOUNT_TRANSLATION_MM
    assets = {}
    for side in ("left", "right"):
        assets[f"{side}_finger.stl"] = binary_stl(retained_drive_triangles(args.source_dir / f"original_{side}_finger.stl"))
        side_rotation = np.eye(3) if side == "left" else np.diag([1.0, -1.0, -1.0])
        for part, triangles in (("mount", mount), ("finger", finger)):
            placed = (triangles @ MOUNT_TO_LEFT_ROTATION.T + MOUNT_TO_LEFT_TRANSLATION_MM) @ side_rotation.T
            assets[f"{side}_soft_gripper_{part}.stl"] = binary_stl(placed * 0.001)
    for destination in destinations:
        destination.mkdir(parents=True, exist_ok=True)
        for name, content in assets.items():
            (destination / name).write_bytes(content)
            print(f"{destination / name}: {len(content)} bytes")


if __name__ == "__main__":
    main()

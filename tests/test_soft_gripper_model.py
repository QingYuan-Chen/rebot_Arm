"""Geometry and contact checks for the replacement, without hardware or CAD tools."""
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "src/rebotarm_moveit_config"
MODEL = ROOT / "src/rebotarm_simulation/models/rebotarm"


def _triangles(path):
    data = path.read_bytes()
    count = struct.unpack_from("<I", data, 80)[0]
    assert len(data) == 84 + count * 50
    records = np.frombuffer(data, dtype=np.dtype([
        ("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2"),
    ]), count=count, offset=84)
    return records["vertices"]


def test_original_drive_triangles_are_retained_without_the_old_finger():
    for side in ("left", "right"):
        original = _triangles(PACKAGE / f"cad/soft_gripper/original_{side}_finger.stl")
        retained = _triangles(PACKAGE / f"meshes/{side}_finger.stl")
        source_faces = {triangle.tobytes() for triangle in original}
        assert len(retained) == 2148 + 1080
        assert all(triangle.tobytes() in source_faces for triangle in retained)
        # No part of the original forward-extending finger remains.
        assert retained[:, :, 0].max() < -0.070


def test_both_backends_cover_all_jaw_parts_with_identical_metre_meshes():
    urdf = ET.parse(PACKAGE / "config/rebotarm.urdf").getroot()
    for side in ("left", "right"):
        filenames = {f"{side}_finger.stl", f"{side}_soft_gripper_mount.stl", f"{side}_soft_gripper_finger.stl"}
        link = urdf.find(f'link[@name="{side}_finger_link"]')
        for role in ("visual", "collision"):
            assert {Path(mesh.attrib["filename"]).name for mesh in link.findall(f"{role}/geometry/mesh")} == filenames
        for filename in filenames:
            assert (PACKAGE / "meshes" / filename).read_bytes() == (MODEL / "assets" / filename).read_bytes()
            vertices = _triangles(PACKAGE / "meshes" / filename).reshape(-1, 3)
            assert np.isfinite(vertices).all()
            assert np.max(np.abs(vertices)) < 0.15


def test_soft_fingers_are_symmetric_and_have_clearance_across_the_slide_range():
    left = _triangles(PACKAGE / "meshes/left_soft_gripper_finger.stl").reshape(-1, 3)
    right = _triangles(PACKAGE / "meshes/right_soft_gripper_finger.stl").reshape(-1, 3)
    np.testing.assert_allclose(right, left * [1, -1, -1], atol=1e-8)
    for half_travel in (0.0, 0.015, 0.045):
        gap = left[:, 1].min() + half_travel - (right[:, 1].max() - half_travel)
        assert gap == pytest.approx(0.003765764 + 2 * half_travel, abs=1e-6)
    mount = _triangles(PACKAGE / "meshes/left_soft_gripper_mount.stl").reshape(-1, 3)
    assert mount[:, 0].min() == pytest.approx(-0.07060932368, abs=1e-8)
    assert left[:, 0].max() - mount[:, 0].min() == pytest.approx(0.093389279, abs=1e-6)


def test_new_soft_contact_surfaces_touch_cube_then_release_on_opening():
    mujoco = pytest.importorskip("mujoco")
    model = mujoco.MjModel.from_xml_path(str(MODEL / "scene.xml"))
    data = mujoco.MjData(model)
    end = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "end_link")
    cube = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "test_cube")
    cube_start = model.jnt_qposadr[model.body_jntadr[cube]]
    for half_travel, expected in ((0.010, True), (0.045, False)):
        data.qpos[6:8] = (half_travel, -half_travel)
        mujoco.mj_forward(model, data)
        # At the new forward finger surface, away from the retained drive/mount.
        data.qpos[cube_start:cube_start + 3] = data.xpos[end] + data.xmat[end].reshape(3, 3) @ np.array([0.005, 0.0, 0.0])
        data.qpos[cube_start + 3:cube_start + 7] = data.xquat[end]
        mujoco.mj_forward(model, data)
        touched = set()
        for contact in data.contact:
            first, second = int(contact.geom1), int(contact.geom2)
            if int(model.geom_bodyid[first]) == cube:
                touched.add(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, second))
            if int(model.geom_bodyid[second]) == cube:
                touched.add(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, first))
        for side in ("left", "right"):
            assert (f"{side}_finger_link_finger_collision" in touched) is expected


def test_collision_mesh_copies_do_not_double_the_inferred_finger_mass():
    mujoco = pytest.importorskip("mujoco")
    model = mujoco.MjModel.from_xml_path(str(MODEL / "robot.xml"))
    root = ET.parse(MODEL / "robot.xml").getroot()
    for mesh in root.findall("asset/mesh"):
        mesh.set("file", str(MODEL / mesh.attrib["file"]))
    for side in ("left", "right"):
        body = root.find(f'.//body[@name="{side}_finger_link"]')
        for geom in list(body.findall('geom[@class="collision"]')):
            body.remove(geom)
    visual_only = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    for side in ("left", "right"):
        name = f"{side}_finger_link"
        body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        comparison = mujoco.mj_name2id(visual_only, mujoco.mjtObj.mjOBJ_BODY, name)
        assert model.body_mass[body] == pytest.approx(visual_only.body_mass[comparison])
        np.testing.assert_allclose(model.body_inertia[body], visual_only.body_inertia[comparison])

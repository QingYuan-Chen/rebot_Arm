from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from rebotarm_simulation.model_tools.d405_payload import apply_d405_inertials
from rebotarm_simulation.model_tools.urdf_to_mjcf import generate_mjcf_bytes

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "src/rebotarm_simulation"
MODEL = PACKAGE / "models/rebotarm/robot.xml"
URDF = ROOT / "src/rebotarm_moveit_config/config/rebotarm.urdf"


def test_d405_shared_geometry_and_simulation_inertia_boundary():
    source = ET.parse(URDF).getroot()
    payload = ET.parse(MODEL).find('.//body[@name="d405_mount_link"]')
    assert payload is not None
    assert not payload.findall('.//joint')
    assert not payload.findall('.//freejoint')
    assert len(payload.findall('.//geom[@class="visual"]')) == 2
    assert sum(float(i.get("mass")) for i in payload.findall('.//inertial')) == pytest.approx(.107)
    for name, mesh in (("d405_mount_link", "d405_mount"), ("d405_link", "d405")):
        link = source.find(f'link[@name="{name}"]')
        assert link.find('inertial') is None  # no guessed hardware load retuning
        assert len(link.findall('visual')) == len(link.findall('collision')) == 1
        assert (ROOT / f"src/rebotarm_moveit_config/meshes/{mesh}.stl").read_bytes() == (
            MODEL.parent / f"assets/{mesh}.stl").read_bytes()
    assert ET.parse(MODEL).find('.//body[@name="gemini2_camera"]') is None


def test_missing_payload_link_fails_instead_of_silently_dropping_mass():
    with pytest.raises(ValueError, match="missing"):
        apply_d405_inertials(ET.fromstring('<robot/>'), PACKAGE / "config/d405_payload.yaml")


def test_d405_mounting_holes_and_viewing_direction_match_independent_cad_data():
    mujoco = pytest.importorskip("mujoco")
    model = mujoco.MjModel.from_xml_path(str(MODEL.parent / 'scene.xml'))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    ids = {name: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
           for name in ('end_link', 'd405_mount_link', 'd405_link')}
    end, mount, camera = (ids[n] for n in ('end_link', 'd405_mount_link', 'd405_link'))
    Re, Rm, Rc = (data.xmat[i].reshape(3, 3) for i in (end, mount, camera))
    assert Re.T @ Rm[:, 2] == pytest.approx([1, 0, 0], abs=1e-8)
    for cad_x, end_y in [(-.0112079068932795, -.011), (.0107920931067205, .011)]:
        hole = np.array([cad_x, .0299992412736477, -.0169])
        registered = Re.T @ (Rm @ hole + data.xpos[mount] - data.xpos[end])
        # MuJoCo's XML serializer retains six significant digits in body poses.
        assert registered == pytest.approx([-.10320933, end_y, .03], abs=1e-7)
    # Native official STL front normal +Z, rear M3 holes at Z=-23 mm.
    assert Rm.T @ Rc[:, 2] == pytest.approx([0, -.25881601644697405, .9659266378097872], abs=1e-6)
    for x, cad_x in [(-.010, -.00899854447587028), (.010, .0110014555241297)]:
        registered = Rm.T @ (Rc @ np.array([x, 0, -.023]) + data.xpos[camera] - data.xpos[mount])
        assert registered == pytest.approx([cad_x, .0689723991877944, -.0106118025980252], abs=1e-7)


def test_d405_loads_without_false_assembly_contacts_and_still_collides_with_objects():
    mujoco = pytest.importorskip("mujoco")
    assert generate_mjcf_bytes(ROOT) == MODEL.read_bytes()
    model = mujoco.MjModel.from_xml_path(str(MODEL.parent / 'scene.xml'))
    assert model.nu == 8
    data = mujoco.MjData(model)
    for pose in ('zero', 'home'):
        key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, pose)
        mujoco.mj_resetDataKeyframe(model, data, key)
        mujoco.mj_forward(model, data)
        for contact in data.contact:
            names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(g)) for g in contact.geom]
            assert not any(name.startswith('d405_') for name in names), names
    for _ in range(100):
        mujoco.mj_step(model, data)
        assert np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()
    # Place scene cube across the front of the camera, independent of assembly exclusions.
    camera = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'd405_link')
    cube = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'test_cube')
    address = model.jnt_qposadr[model.body_jntadr[cube]]
    data.qpos[address:address+3] = data.xpos[camera] + data.xmat[camera].reshape(3, 3) @ [0, 0, .005]
    data.qpos[address+3:address+7] = data.xquat[camera]
    mujoco.mj_forward(model, data)
    pairs = {frozenset(int(model.geom_bodyid[g]) for g in c.geom) for c in data.contact}
    assert frozenset((camera, cube)) in pairs

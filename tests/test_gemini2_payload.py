from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from rebotarm_simulation.gemini2_payload import add_gemini2_payload

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / "src/rebotarm_simulation/models/rebotarm/robot.xml"


def candidate_root(monkeypatch):
    return ET.parse(MODEL).getroot()


def test_payload_has_explicit_mass_and_no_extra_control_dofs(monkeypatch):
    root = candidate_root(monkeypatch)
    payload = root.find('.//body[@name="end_link"]/body[@name="gemini2_mount"]')
    assert payload is not None
    assert len(payload.findall('body')) == 2  # one old bracket plus camera
    assert root.find('.//mesh[@name="gemini2_mount_part1"]') is None
    assert not payload.findall('.//joint') and not payload.findall('.//freejoint')
    assert sum(float(i.get('mass')) for i in payload.findall('.//inertial')) == pytest.approx(.120)
    assert all(g.get('mass') == '0' for g in payload.findall('.//geom'))
    camera = payload.find('body[@name="gemini2_camera"]')
    assert camera.find('geom[@name="gemini2_camera_collision"]').get('size') == '0.015 0.045 0.0125'
    for mesh in root.findall('asset/mesh'):
        assert (MODEL.parent / mesh.get('file')).is_file()


def test_missing_mount_parent_fails_instead_of_silently_dropping_payload(monkeypatch):
    candidate_root(monkeypatch)
    with pytest.raises(ValueError, match='parent'):
        add_gemini2_payload(ET.fromstring('<mujoco><worldbody/></mujoco>'), ROOT)


def test_generated_model_loads_and_has_no_payload_contact_at_home(monkeypatch, tmp_path):
    mujoco = pytest.importorskip('mujoco')
    import numpy as np
    from rebotarm_simulation.urdf_to_mjcf import generate_mjcf_bytes
    assert generate_mjcf_bytes(ROOT) == MODEL.read_bytes()
    assert ET.parse(MODEL).find('.//body[@name="gemini2_camera"]') is not None
    candidate = candidate_root(monkeypatch)
    for mesh in candidate.findall('asset/mesh'):
        mesh.set('file', str(MODEL.parent / mesh.get('file')))
    robot = tmp_path / 'robot.xml'
    ET.ElementTree(candidate).write(robot)
    scene = ET.parse(MODEL.parent / 'scene.xml')
    scene.find('include').set('file', str(robot))
    scene.write(tmp_path / 'scene.xml')
    model = mujoco.MjModel.from_xml_path(str(tmp_path / 'scene.xml'))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    for contact in data.contact:
        names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(g)) for g in contact.geom]
        assert not any(name.startswith('gemini2_') for name in names), names
    camera = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'gemini2_camera')
    assert model.body_mass[camera] == pytest.approx(.098)
    end = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'end_link')
    mount = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'gemini2_mount')
    end_rotation = data.xmat[end].reshape(3, 3)
    camera_position = end_rotation.T @ (data.xpos[camera] - data.xpos[end])
    mount_position = end_rotation.T @ (data.xpos[mount] - data.xpos[end])
    # Camera back is attached by its rear M3 pair, not its bottom face.
    mount_rotation = data.xmat[mount].reshape(3, 3)
    camera_rotation = data.xmat[camera].reshape(3, 3)
    normal = np.array([0, -.25881601644697405, .9659266378097872])
    assert mount_rotation.T @ camera_rotation[:, 0] == pytest.approx(normal, abs=1e-8)
    bracket_holes = np.array([
        [22.4167124852992, 60.8364373210517, -12.7917996680817],
        [-22.4984216150612, 60.9798127357473, -12.7533828233282],
    ]) * .001
    # Local shell back x=-15mm, rear screw spacing=45mm.
    camera_holes = np.array([[-.015, .0225, 0], [-.015, -.0225, 0]])
    in_mount = ((camera_holes @ camera_rotation.T + data.xpos[camera]
                 - data.xpos[mount]) @ mount_rotation)
    assert np.max(np.linalg.norm(in_mount - bracket_holes, axis=1)) < .00015
    # Bracket CAD circular seat axis is coaxial with the end X axis.
    # Inner recess plane, not the z=0 opening, mates to the gripper.
    axis = end_rotation.T @ mount_rotation[:, 2]
    assert axis == pytest.approx([1, 0, 0], abs=1e-8)
    center = np.array([-.0002079068932795, .0299992412736477, -.0169])
    seat = end_rotation.T @ (mount_rotation @ center + data.xpos[mount] - data.xpos[end])
    assert seat == pytest.approx([-.10320933, 0, .03], abs=1e-8)
    # Independent CAD hole locations must be coaxial with gripper holes.
    for cad_x, end_y in [(-.0112079068932795, -.011), (.0107920931067205, .011)]:
        hole = np.array([cad_x, .0299992412736477, -.0169])
        registered = end_rotation.T @ (mount_rotation @ hole + data.xpos[mount] - data.xpos[end])
        assert registered == pytest.approx([-.10320933, end_y, .03], abs=1e-8)
    for _ in range(100):
        mujoco.mj_step(model, data)
    assert np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()

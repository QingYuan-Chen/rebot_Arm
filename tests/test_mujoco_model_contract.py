from __future__ import annotations

from pathlib import Path
import math
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from rebotarm_simulation.control.control_config import load_motor_control_parameters

from rebotarm_simulation.control.motor_control import PosVelController


ROOT = Path(__file__).resolve().parents[1]
URDF_PATH = ROOT / "src/rebotarm_moveit_config/config/rebotarm.urdf"
MODEL_DIR = ROOT / "src/rebotarm_simulation/models/rebotarm"
ROBOT_PATH = MODEL_DIR / "robot.xml"
SCENE_PATH = MODEL_DIR / "scene.xml"

ARM_JOINTS = [f"joint{index}" for index in range(1, 7)]
FINGER_JOINTS = ["left_finger_joint", "right_finger_joint"]
JOINTS = ARM_JOINTS + FINGER_JOINTS
LINKS = ["base_link"] + [f"link{index}" for index in range(1, 7)] + ["end_link"]


def _numbers(value: str) -> tuple[float, ...]:
    return tuple(float(part) for part in value.split())


def _assert_close(actual: str, expected: str) -> None:
    assert _numbers(actual) == pytest.approx(_numbers(expected), abs=1e-7)


def _robot_root() -> ET.Element:
    return ET.parse(ROBOT_PATH).getroot()


def _urdf_root() -> ET.Element:
    return ET.parse(URDF_PATH).getroot()


def _inertia_components(inertia: ET.Element) -> tuple[float, ...]:
    return tuple(
        float(inertia.attrib[name])
        for name in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")
    )


def _is_positive_definite(components: tuple[float, ...]) -> bool:
    ixx, iyy, izz, ixy, ixz, iyz = components
    determinant = (
        ixx * iyy * izz
        + 2 * ixy * ixz * iyz
        - ixx * iyz * iyz
        - iyy * ixz * ixz
        - izz * ixy * ixy
    )
    return ixx > 0 and ixx * iyy - ixy * ixy > 0 and determinant > 0


def _matmul(left: tuple[tuple[float, ...], ...], right: tuple[tuple[float, ...], ...]) -> tuple[tuple[float, ...], ...]:
    return tuple(
        tuple(sum(left[row][k] * right[k][column] for k in range(4)) for column in range(4))
        for row in range(4)
    )


def _urdf_origin_matrix(origin: ET.Element) -> tuple[tuple[float, ...], ...]:
    x, y, z = _numbers(origin.attrib["xyz"])
    roll, pitch, yaw = _numbers(origin.attrib["rpy"])
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, x),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, y),
        (-sp, cp * sr, cp * cr, z),
        (0.0, 0.0, 0.0, 1.0),
    )


def test_robot_model_uses_urdf_extrinsic_xyz_euler_convention() -> None:
    compiler = _robot_root().find("compiler")
    assert compiler is not None
    assert compiler.attrib["angle"] == "radian"


def test_robot_model_preserves_canonical_joint_chain_transforms_axes_and_ranges() -> None:
    robot = _robot_root()
    urdf = _urdf_root()
    urdf_joints = {joint.attrib["name"]: joint for joint in urdf.findall("joint")}
    mjcf_joints = {joint.attrib["name"]: joint for joint in robot.findall("worldbody//joint")}

    assert list(mjcf_joints) == JOINTS
    for name in JOINTS:
        source = urdf_joints[name]
        target = mjcf_joints[name]
        body = next(body for body in robot.findall(".//body") if target in list(body))
        _assert_close(body.attrib.get("pos", "0 0 0"), source.find("origin").attrib["xyz"])
        _assert_close(target.attrib["axis"], source.find("axis").attrib["xyz"])
        limit = source.find("limit")
        _assert_close(target.attrib["range"], f'{limit.attrib["lower"]} {limit.attrib["upper"]}')
        assert target.attrib.get("type", "hinge") == ("slide" if name in FINGER_JOINTS else "hinge")
        assert float(target.attrib["damping"]) > 0.0
        assert float(target.attrib["armature"]) > 0.0
        assert float(target.attrib["frictionloss"]) >= 0.0


def test_robot_model_preserves_link_masses_and_has_valid_explicit_finger_inertials() -> None:
    robot = _robot_root()
    urdf = _urdf_root()
    source_masses = {
        link.attrib["name"]: float(link.find("inertial/mass").attrib["value"])
        for link in urdf.findall("link")
        if link.find("inertial/mass") is not None
    }
    bodies = {body.attrib["name"]: body for body in robot.findall(".//body")}

    for link_name in LINKS:
        inertial = bodies[link_name].find("inertial")
        assert inertial is not None
        assert float(inertial.attrib["mass"]) == pytest.approx(source_masses[link_name])
        source = next(link for link in urdf.findall("link") if link.attrib["name"] == link_name)
        diagonal = _numbers(inertial.attrib["diaginertia"])
        assert all(value > 0 for value in diagonal)

    mujoco = pytest.importorskip("mujoco")
    model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
    for finger in ("left_finger_link", "right_finger_link"):
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, finger)
        assert float(model.body_mass[body_id]) > 0
        assert all(float(value) > 0 for value in model.body_inertia[body_id])


def test_robot_model_couples_finger_joints_with_equal_and_opposite_motion() -> None:
    coupling = _robot_root().find("equality/joint")
    assert coupling is not None
    assert coupling.attrib == {
        "name": "finger_coupling",
        "joint1": "right_finger_joint",
        "joint2": "left_finger_joint",
        "polycoef": "0 -1 0 0 0",
    }


def test_robot_model_uses_original_meshes_for_visuals_and_hybrid_collisions() -> None:
    robot = _robot_root()
    assets = {mesh.attrib["name"]: mesh for mesh in robot.findall("asset/mesh")}
    expected_assets = {
        "base_link", "link1", "link2", "link3", "link4", "link5", "link6",
        "gripper_base", "left_finger", "right_finger",
        "gemini2_shell", "gemini2_mount_part0",
    }
    assert set(assets) == expected_assets
    for mesh in assets.values():
        path = MODEL_DIR / mesh.attrib["file"]
        assert path.is_file()

    visual_geoms = robot.findall('.//geom[@class="visual"]')
    collision_geoms = [geom for geom in robot.findall('.//geom[@class="collision"]')
                       if not geom.attrib.get("name", "").startswith("gemini2_")]
    assert {geom.attrib["mesh"] for geom in visual_geoms} == expected_assets
    assert collision_geoms
    assert {geom.attrib["mesh"] for geom in collision_geoms if geom.attrib["type"] == "mesh"} == {
        "left_finger", "right_finger",
    }
    assert {geom.attrib["type"] for geom in collision_geoms} == {
        "box", "capsule", "cylinder", "mesh",
    }
    assert len(collision_geoms) == 10
    assert all(geom.attrib.get("contype") == "1" for geom in collision_geoms)
    assert all(geom.attrib.get("conaffinity") == "1" for geom in collision_geoms)
    assert all("rgba" not in geom.attrib for geom in collision_geoms)

    bodies = {body.attrib["name"]: body for body in robot.findall("worldbody//body")}
    for link_name in LINKS:
        # Explicit URDF inertials remain authoritative; collision proxies must
        # never contribute inferred mass or inertia.
        assert bodies[link_name].find("inertial") is not None
    assert all("mass" not in geom.attrib and "density" not in geom.attrib for geom in collision_geoms)


def test_robot_model_filters_adjacent_chain_and_gripper_parent_contacts() -> None:
    robot = _robot_root()
    excluded_pairs = {
        frozenset((exclude.attrib["body1"], exclude.attrib["body2"]))
        for exclude in robot.findall("contact/exclude")
    }
    expected_pairs = {
        frozenset(pair)
        for pair in (
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
        )
    }

    assert excluded_pairs == expected_pairs


def test_robot_model_has_eight_torque_actuators_end_effector_site_and_state_sensors() -> None:
    robot = _robot_root()
    actuators = robot.findall("actuator/motor")
    assert [actuator.attrib["joint"] for actuator in actuators] == JOINTS

    ee_site = robot.find('.//site[@name="ee_site"]')
    assert ee_site is not None
    sensors = robot.find("sensor")
    assert sensors is not None
    assert {element.attrib["joint"] for element in sensors.findall("jointpos")} == set(JOINTS)
    assert {element.attrib["joint"] for element in sensors.findall("jointvel")} == set(JOINTS)
    assert sensors.find('framepos[@objname="ee_site"]') is not None
    assert sensors.find('framequat[@objname="ee_site"]') is not None


def test_robot_model_exposes_named_wrist_camera_mount_frame() -> None:
    robot = _robot_root()
    end_link = robot.find('.//body[@name="end_link"]')
    assert end_link is not None
    mount = end_link.find('site[@name="wrist_camera_mount"]')
    assert mount is not None
    assert _numbers(mount.attrib["pos"]) == pytest.approx((-0.04, 0.0, 0.04))
    assert _numbers(mount.attrib["quat"]) == pytest.approx((1.0, 0.0, 0.0, 0.0))


def test_torque_actuator_force_limits_match_urdf_effort_limits() -> None:
    robot = _robot_root()
    urdf = _urdf_root()
    efforts = {
        joint.attrib["name"]: float(joint.find("limit").attrib["effort"])
        for joint in urdf.findall("joint")
        if joint.attrib["name"] in JOINTS
    }

    actuators = {actuator.attrib["joint"]: actuator for actuator in robot.findall("actuator/motor")}
    assert set(actuators) == set(JOINTS)
    for joint_name, effort in efforts.items():
        actuator = actuators[joint_name]
        expected_effort = 1.5 if joint_name in FINGER_JOINTS else effort
        assert actuator.attrib["forcelimited"] == "true"
        assert _numbers(actuator.attrib["forcerange"]) == pytest.approx((-expected_effort, expected_effort))
        assert actuator.attrib["ctrllimited"] == "true"
        assert _numbers(actuator.attrib["ctrlrange"]) == pytest.approx((-expected_effort, expected_effort))


def test_scene_defines_world_fixture_free_cube_camera_and_simulation_options() -> None:
    scene = ET.parse(SCENE_PATH).getroot()
    include = scene.find("include")
    assert include is not None and include.attrib["file"] == "robot.xml"
    option = scene.find("option")
    _assert_close(option.attrib["gravity"], "0 0 -9.81")
    assert float(option.attrib["timestep"]) == pytest.approx(0.002)
    statistic = scene.find("statistic")
    assert statistic is not None
    _assert_close(statistic.attrib["center"], "0.15 0 0.18")
    assert float(statistic.attrib["extent"]) == pytest.approx(0.8)
    world = scene.find("worldbody")
    floor = world.find('geom[@name="floor"]')
    assert floor is not None
    _assert_close(floor.attrib["pos"], "0 0 -0.75")
    table = world.find('body[@name="table"]')
    assert table is not None
    _assert_close(table.attrib["pos"], "0.20 0 -0.025")
    assert table.find('geom[@name="table_top"]') is not None
    assert len([geom for geom in table.findall("geom") if geom.attrib["name"].startswith("table_leg_")]) == 4
    cube = world.find('body[@name="test_cube"]')
    assert cube is not None and cube.find("freejoint") is not None
    _assert_close(cube.attrib["pos"], "0.28 0 0.04")
    assert cube.find("geom").attrib["type"] == "box"
    assert world.find('light[@name="key_light"]') is not None
    target = world.find('body[@name="ee_target"]')
    assert target is not None and target.attrib["mocap"] == "true"
    target_handle = target.find('geom[@name="ee_target_handle"]')
    assert target_handle is not None
    assert target_handle.attrib["contype"] == "0"
    assert target_handle.attrib["conaffinity"] == "0"
    assert target_handle.attrib["group"] == "1"
    for axis, color_index in (("x", 0), ("y", 1), ("z", 2)):
        marker = target.find(f'geom[@name="ee_target_axis_{axis}"]')
        assert marker is not None
        assert marker.attrib["group"] == "1"
        assert marker.attrib["contype"] == "0"
        assert marker.attrib["conaffinity"] == "0"
        rgba = _numbers(marker.attrib["rgba"])
        assert rgba[color_index] == max(rgba[:3])
    assert target.find('site[@name="ee_target_frame"]') is not None
    camera = world.find('camera[@name="fixed_camera"]')
    assert camera is not None
    _assert_close(camera.attrib["pos"], "0.85 -0.90 0.55")
    home = scene.find('keyframe/key[@name="home"]')
    assert home is not None
    assert len(_numbers(home.attrib["qpos"])) == 15


def test_scene_compiles_with_mujoco_when_runtime_is_available() -> None:
    mujoco = pytest.importorskip("mujoco")
    model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    for _ in range(100):
        mujoco.mj_step(model, data)
    assert model.nu == 8
    assert data.time == pytest.approx(100 * model.opt.timestep)
    assert all(math.isfinite(float(value)) for value in data.qpos)
    assert all(math.isfinite(float(value)) for value in data.qvel)
    assert all(math.isfinite(float(value)) for value in data.actuator_force)
    assert max((abs(float(value)) for value in data.qvel), default=0.0) < 1_000.0


def test_mujoco_zero_pose_fk_matches_independent_urdf_rpy_math_when_runtime_is_available() -> None:
    mujoco = pytest.importorskip("mujoco")
    model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    urdf = _urdf_root()
    identity = (
        (1.0, 0.0, 0.0, 0.0),
        (0.0, 1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )
    transforms = {"base_link": identity}
    remaining = list(urdf.findall("joint"))
    while remaining:
        progressed = False
        for joint in remaining[:]:
            parent = joint.find("parent").attrib["link"]
            if parent not in transforms:
                continue
            child = joint.find("child").attrib["link"]
            transforms[child] = _matmul(transforms[parent], _urdf_origin_matrix(joint.find("origin")))
            remaining.remove(joint)
            progressed = True
        assert progressed, "URDF joint graph must be connected"

    for body_name, expected in transforms.items():
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        assert body_id >= 0
        assert tuple(float(value) for value in data.xpos[body_id]) == pytest.approx(
            (expected[0][3], expected[1][3], expected[2][3]), abs=1e-6
        )
        expected_rotation = tuple(expected[row][column] for row in range(3) for column in range(3))
        assert tuple(float(value) for value in data.xmat[body_id]) == pytest.approx(expected_rotation, abs=2e-6)


def _robot_self_contact_pairs(mujoco, model, data) -> set[frozenset[str]]:
    robot_bodies = {
        "base_link", "link1", "link2", "link3", "link4", "link5", "link6",
        "end_link", "left_finger_link", "right_finger_link",
    }
    pairs: set[frozenset[str]] = set()
    for contact in data.contact:
        body1 = mujoco.mj_id2name(
            model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[contact.geom1])
        )
        body2 = mujoco.mj_id2name(
            model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[contact.geom2])
        )
        if body1 in robot_bodies and body2 in robot_bodies:
            pairs.add(frozenset((body1, body2)))
    return pairs


def _body_contact_pairs(mujoco, model, data) -> set[frozenset[str]]:
    pairs: set[frozenset[str]] = set()
    for contact in data.contact:
        names = []
        for geom_id in (int(contact.geom1), int(contact.geom2)):
            body_id = int(model.geom_bodyid[geom_id])
            names.append(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id))
        pairs.add(frozenset(names))
    return pairs


def test_default_cube_falls_onto_table_and_settles_when_runtime_is_available() -> None:
    mujoco = pytest.importorskip("mujoco")
    model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
    data = mujoco.MjData(model)
    cube_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "test_cube")
    cube_joint_id = int(model.body_jntadr[cube_body_id])
    cube_qpos = int(model.jnt_qposadr[cube_joint_id])
    cube_dof = int(model.jnt_dofadr[cube_joint_id])

    for _ in range(2_000):
        mujoco.mj_step(model, data)

    assert float(data.xpos[cube_body_id][2]) == pytest.approx(0.02, abs=0.002)
    assert max(abs(float(value)) for value in data.qvel[cube_dof:cube_dof + 6]) < 1e-6
    assert frozenset(("table", "test_cube")) in _body_contact_pairs(mujoco, model, data)
    table_cube_distances = []
    for contact in data.contact:
        body_names = frozenset(
            mujoco.mj_id2name(
                model,
                mujoco.mjtObj.mjOBJ_BODY,
                int(model.geom_bodyid[int(geom_id)]),
            )
            for geom_id in (contact.geom1, contact.geom2)
        )
        if body_names == frozenset(("table", "test_cube")):
            table_cube_distances.append(float(contact.dist))
    assert table_cube_distances
    assert min(table_cube_distances) > -1e-3


def test_finger_to_cube_contact_is_detectable_when_runtime_is_available() -> None:
    mujoco = pytest.importorskip("mujoco")
    model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
    data = mujoco.MjData(model)
    end_link_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "end_link")
    cube_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "test_cube")
    cube_joint_id = int(model.body_jntadr[cube_body_id])
    cube_qpos = int(model.jnt_qposadr[cube_joint_id])

    data.qpos[6:8] = (0.015, -0.015)
    mujoco.mj_forward(model, data)
    local_cube_position = (-0.105, 0.0, -0.025)
    rotation = data.xmat[end_link_id].reshape(3, 3)
    world_cube_position = data.xpos[end_link_id] + rotation @ local_cube_position
    data.qpos[cube_qpos:cube_qpos + 3] = world_cube_position
    data.qpos[cube_qpos + 3:cube_qpos + 7] = data.xquat[end_link_id]
    mujoco.mj_forward(model, data)

    pairs = _body_contact_pairs(mujoco, model, data)
    assert frozenset(("left_finger_link", "test_cube")) in pairs
    assert frozenset(("right_finger_link", "test_cube")) in pairs


def test_robot_to_table_collision_is_enabled_when_runtime_is_available() -> None:
    mujoco = pytest.importorskip("mujoco")
    model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
    data = mujoco.MjData(model)
    arm_qpos = (
        -0.73059666,
        -3.12827448,
        -0.53365013,
        -1.33865388,
        -0.72973818,
        2.38848593,
    )
    assert all(
        float(model.jnt_range[index][0]) <= value <= float(model.jnt_range[index][1])
        for index, value in enumerate(arm_qpos)
    )
    data.qpos[:8] = (*arm_qpos, 0.045, -0.045)
    cube_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "test_cube")
    cube_joint_id = int(model.body_jntadr[cube_body_id])
    cube_qpos = int(model.jnt_qposadr[cube_joint_id])
    data.qpos[cube_qpos:cube_qpos + 7] = (2.0, 2.0, 2.0, 1.0, 0.0, 0.0, 0.0)
    mujoco.mj_forward(model, data)

    assert frozenset(("link3", "table")) in _body_contact_pairs(mujoco, model, data)


def test_canonical_zero_pose_has_no_false_robot_self_contacts_when_runtime_is_available() -> None:
    mujoco = pytest.importorskip("mujoco")
    model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert _robot_self_contact_pairs(mujoco, model, data) == set()


def test_home_pose_has_no_false_robot_self_contacts_when_runtime_is_available() -> None:
    mujoco = pytest.importorskip("mujoco")
    model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
    data = mujoco.MjData(model)
    home_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    assert home_id >= 0
    data.qpos[:] = model.key_qpos[home_id]
    mujoco.mj_forward(model, data)
    assert _robot_self_contact_pairs(mujoco, model, data) == set()


def test_pos_vel_torque_controller_can_drive_mujoco_motor_inputs_when_runtime_is_available() -> None:
    mujoco = pytest.importorskip("mujoco")
    model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
    data = mujoco.MjData(model)
    target = (0.0, -0.05, -0.1, 0.1, 0.0, 0.0, 0.02, -0.02)
    motor_parameters = load_motor_control_parameters(ROOT)
    controller = PosVelController(motor_parameters.arm)
    arm_joint_ids = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        for name in ARM_JOINTS
    ]
    qpos_addresses = [int(model.jnt_qposadr[joint_id]) for joint_id in arm_joint_ids]
    dof_addresses = [int(model.jnt_dofadr[joint_id]) for joint_id in arm_joint_ids]

    for step in range(100):
        if step % 5 == 0:
            data.ctrl[:6] = controller.compute(
                target=np.asarray(target[:6], dtype=float),
                position=np.asarray([data.qpos[address] for address in qpos_addresses]),
                velocity=np.asarray([data.qvel[address] for address in dof_addresses]),
                dt=1.0 / motor_parameters.control_rate_hz,
        )
            mujoco.mj_step(model, data)

    assert all(math.isfinite(float(value)) for value in data.ctrl)
    assert all(math.isfinite(float(value)) for value in data.qpos)
    assert np.all(np.abs(data.ctrl[:6]) <= np.asarray(motor_parameters.arm.effort_limit))


def test_folded_valid_pose_still_detects_non_adjacent_self_collision_when_runtime_is_available() -> None:
    mujoco = pytest.importorskip("mujoco")
    model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
    data = mujoco.MjData(model)
    folded_arm_q = (
        2.644410583,
        -2.207802494,
        -0.455216834,
        0.883963023,
        -0.764293556,
        -0.833907491,
    )
    data.qpos[:6] = folded_arm_q
    data.qpos[6:8] = (0.02, -0.02)
    mujoco.mj_forward(model, data)

    pairs = _robot_self_contact_pairs(mujoco, model, data)
    assert frozenset(("base_link", "link6")) in pairs

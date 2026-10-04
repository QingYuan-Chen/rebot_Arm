"""Versioned, simulation-only Reach-and-Hold model and target catalogue."""
import json
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from .contracts import contract_hash, sha256
from .runtime import PACKAGE

TASK_ID = "RebotArm-Reach-Hold-v1"
XML = PACKAGE / "assets/rebotarm/robot.xml"
CATALOGUE = PACKAGE / "configs/reach_targets.json"
HOME = np.array([0., -1.2, -1.5, 0., 0., 0.])
LOW = np.array([-2.8, -3.14, -3.14, -1.87, -1.57, -3.14])
HIGH = np.array([2.8, .02, .02, 1.57, 1.57, 3.14])
KP = np.array([100., 100., 100., 40., 40., 20.])
KD = np.array([4., 4., 4., 1.5, 1.5, .5])
TORQUE = np.array([27., 27., 27., 7., 7., 7.])
DT = .02
PHYSICS_DT = .002
DELTA = .01  # rad per policy step, i.e. target rate <= 0.5 rad/s
MARGIN = .05
POSITION_TOL = .01
SPEED_TOL = .02
JOINT_SPEED_TOL = .05
HOLD_STEPS = 50
EPISODE_STEPS = 500


def model_spec():
    """Keep original snapshot intact. Freeze fingers at 40 mm opening, add floor/goal."""
    root = ET.parse(XML).getroot()
    assets = {mesh.get("file"): (XML.parent/mesh.get("file")).read_bytes()
              for mesh in root.findall("asset/mesh")}
    for side, offset in (("left", .02), ("right", -.02)):
        body = root.find(f".//body[@name='{side}_finger_link']")
        body.set("pos", f"0 {offset} 0")
        body.remove(body.find("joint"))
    root.remove(root.find("equality"))
    for section in ("actuator", "sensor"):
        parent = root.find(section)
        for item in list(parent):
            if "finger" in item.get("name", ""):
                parent.remove(item)
    world = root.find("worldbody")
    ET.SubElement(world, "geom", name="floor", type="plane", size="2 2 .05",
                  contype="1", conaffinity="1", rgba=".2 .2 .2 1")
    spec = mujoco.MjSpec.from_string(ET.tostring(root, encoding="unicode"), assets=assets)
    spec.option.timestep = PHYSICS_DT
    return spec


def model_identity():
    provenance = json.loads((XML.parent / "provenance.json").read_text())
    for name, expected in provenance["files"].items():
        if sha256(XML.parent/name) != expected:
            raise ValueError(f"Reach model asset checksum mismatch: {name}")
    return {"source_xml_sha256": sha256(XML), "assets": provenance,
            "adaptation": "reach-v1: fingers frozen +/-0.02m; floor; noncontact goal marker"}


def load_catalogue():
    bank = json.loads(CATALOGUE.read_text())
    if bank["model"] != model_identity():
        raise ValueError("Reach target catalogue model mismatch; regenerate and revalidate")
    return bank


def valid_pose(model, data, q):
    q = np.asarray(q)
    if q.shape != (6,) or not np.isfinite(q).all() or np.any(q < LOW + MARGIN) or np.any(q > HIGH - MARGIN):
        return False
    data.qpos[:] = q
    data.qvel[:] = 0
    mujoco.mj_forward(model, data)
    site = model.site("ee_site").id
    return data.site_xpos[site, 2] >= .10 and data.ncon == 0


def valid_path(model, data, start, end):
    # Discrete geometric precheck; runtime collision guard is still required.
    return all(valid_pose(model, data, (1-t)*start + t*end) for t in np.linspace(0, 1, 61))


def generate_catalogue():
    model = model_spec().compile()
    data = mujoco.MjData(model)
    if not valid_pose(model, data, HOME):
        raise ValueError("Reach home is invalid or in collision")
    rng = np.random.default_rng(20261004)
    bank = {"model": model_identity(), "seed": 20261004, "train": [], "eval": []}
    site = model.site("ee_site").id
    for split, count in (("train", 128), ("eval", 16)):
        for attempt in range(20000):
            start = HOME + rng.uniform(-.08, .08, 6)
            end = HOME + rng.uniform(-.35, .35, 6)
            if not valid_path(model, data, start, end):
                continue
            target = data.site_xpos[site].copy()
            valid_pose(model, data, start)
            if not .04 <= np.linalg.norm(target - data.site_xpos[site]) <= .22:
                continue
            # Held-out points must be at least 1 cm from the finite training set.
            if split == "eval" and any(np.linalg.norm(target - np.array(row["target_m"])) < .01 for row in bank["train"]):
                continue
            bank[split].append({"id": f"{split}-{len(bank[split]):03d}",
                                "start_rad": start.tolist(), "target_m": target.tolist(),
                                "fk_witness_rad": end.tolist()})
            if len(bank[split]) == count:
                break
        if len(bank[split]) != count:
            raise RuntimeError("could not generate sufficient collision-free targets")
    return bank


def deployment_contract():
    return {"schema_version": 1, "input_dim": 34, "output_dim": 6,
            "input_groups": [{"name": "actor", "size": 34}],
            "semantics": {
                "task": TASK_ID, "model": model_identity(),
                "implementation_sha256": {name: sha256(PACKAGE/name) for name in
                    ("reach.py", "reach_model.py", "reach_eval.py", "tasks.py")},
                "catalogue_sha256": sha256(CATALOGUE),
                "observations": ["joint_pos_rad[6]", "joint_vel_rad_s[6]",
                    "target_minus_tcp_m[3]", "tcp_linear_velocity_m_s[3]",
                    "command_minus_joint_pos_rad[6]", "previous_clipped_action[6]",
                    "target_position_base_m[3]", "continuous_hold_fraction[1]"],
                "normalization": "none outside actor; RSL actor normalizer exported with weights",
                "tcp": "ee_site: end_link offset [-0.04,0,0] m; base_link/world axes",
                "action": "clip [-1,1]; integrate command += 0.01*action once per 0.02s; clamp joint margin 0.05rad",
                "controller": {"kp": KP.tolist(), "kd": KD.tolist(), "torque_cap_nm": TORQUE.tolist(),
                    "feedforward": "MuJoCo qfrc_bias (simulation model only)", "physics_dt": PHYSICS_DT},
                "position_tolerance_m": POSITION_TOL, "tcp_speed_tolerance_m_s": SPEED_TOL,
                "joint_speed_tolerance_rad_s": JOINT_SPEED_TOL,
                "hold_steps": HOLD_STEPS, "episode_steps": EPISODE_STEPS,
                "joint_low": LOW.tolist(), "joint_high": HIGH.tolist(), "policy_dt": DT,
                "gripper": "fixed 0.040m opening; orientation unconstrained",
            }}


def protocol():
    return {"task": TASK_ID, "contract": contract_hash(deployment_contract()),
            "split": "eval", "catalogue_sha256": sha256(CATALOGUE),
            "dt": DT, "horizon_steps": EPISODE_STEPS,
            "success": "50 consecutive samples: position<=0.01m, TCP speed<=0.02m/s, all joint speed<=0.05rad/s; no violation",
            "missing_times": "10s horizon used when reach or success is absent",
            "hold_window": "last min(50, episode length) samples; inspect success alongside error"}

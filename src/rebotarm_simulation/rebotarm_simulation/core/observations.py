"""Read-only extraction of immutable simulation observations.

The owner must hold its simulation lock while calling these functions. No step,
reset or control mutation is performed here; model/data ownership stays in the runtime.
"""
from types import MappingProxyType
import numpy as np
from rebotarm_simulation.core.model_contract import JOINT_NAMES
from rebotarm_simulation.core.mujoco_types import SimulationState, ContactInfo


def read_state(mj, model, data, joint_ids, actuator_ids, ee_site_id, free_bodies):
    positions = []
    velocities = []
    for joint_id in joint_ids:
        positions.append(float(data.qpos[int(model.jnt_qposadr[joint_id])]))
        velocities.append(float(data.qvel[int(model.jnt_dofadr[joint_id])]))
    # 自由关节 qpos 内部顺序为 (pos_x,pos_y,pos_z, qw,qx,qy,qz)；
    # 这里重排为对外的 (x,y,z, qx,qy,qz,qw)，即把 w 分量从第 4 位挪到末位。
    object_poses = {
        name: (
            *(float(value) for value in data.qpos[address : address + 3]),
            *(float(value) for value in data.qpos[address + 4 : address + 7]),
            float(data.qpos[address + 3]),
        )
        for name, (_, address) in free_bodies.items()
    }
    ee_quaternion_wxyz = np.empty(4, dtype=float)
    mj.mju_mat2Quat(
        ee_quaternion_wxyz, data.site_xmat[ee_site_id]
    )
    return SimulationState(
        joint_names=JOINT_NAMES,
        joint_positions=tuple(positions),
        joint_velocities=tuple(velocities),
        actuator_forces=tuple(float(data.actuator_force[index]) for index in actuator_ids),
        end_effector_position=tuple(float(value) for value in data.site_xpos[ee_site_id]),
        # mju_mat2Quat 输出 (w,x,y,z)，对外统一改排为 (x,y,z,w)。
        end_effector_orientation=(
            *(float(value) for value in ee_quaternion_wxyz[1:]),
            float(ee_quaternion_wxyz[0]),
        ),
        # 张开宽度 = 左指位置 - 右指位置，并按夹爪机械行程 [0, 0.09] m 夹紧，
        # 以吸收数值抖动与接触穿透带来的瞬时越界。
        gripper_width=max(0.0, min(0.09, positions[-2] - positions[-1])),
        object_poses=MappingProxyType(object_poses),
        simulation_time=float(data.time),
    )


def read_contacts(mj, model, data):
    contacts = []
    force = np.zeros(6, dtype=float)
    for index in range(int(data.ncon)):
        contact = data.contact[index]
        geom1, geom2 = int(contact.geom1), int(contact.geom2)
        body1, body2 = int(model.geom_bodyid[geom1]), int(model.geom_bodyid[geom2])
        force.fill(0.0)
        mj.mj_contactForce(model, data, index, force)
        contacts.append(ContactInfo(
            body1=str(mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, body1) or "world"),
            body2=str(mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, body2) or "world"),
            geom1=str(mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, geom1) or f"geom{geom1}"),
            geom2=str(mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, geom2) or f"geom{geom2}"),
            position=tuple(float(value) for value in contact.pos),
            force=float(np.linalg.norm(force[:3])),
            penetration_depth=max(0.0, -float(contact.dist)),
            normal=tuple(float(value) for value in contact.frame[:3]),
        ))
    return tuple(contacts)

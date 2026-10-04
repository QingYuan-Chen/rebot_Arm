"""Load versioned simulation control parameters without changing controller math."""
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
from rebotarm_simulation.core.resource_paths import package_resource
from rebotarm_simulation.control.motor_control import MotorSpec, ArmControlParameters, GripperControlParameters, MotorControlParameters


def load_motor_control_parameters(
    *, calibration_path: Path | None = None, urdf_path: Path | None = None,
) -> MotorControlParameters:
    """读取标定文件与 URDF 限幅，组装出完整控制参数。

    参数（均为仅关键字）：
        calibration_path: 电机标定 YAML 路径；默认取本包 config 目录下的标定文件。
        urdf_path: 权威 URDF 路径；默认取运动规划配置包中的 rebotarm.urdf，
            其 joint/limit@effort 作为各关节力矩上限。

    返回：组装好的 :class:`MotorControlParameters`。

    异常：标定值缺失或类型不符时由字典访问/``float()`` 抛出；
    ``torque_lowpass_alpha`` 不在 (0, 1] 区间时抛 ``ValueError``。

    说明：这里同时读取「固件参考整定」与「仿真换算标定」两段，
    关节顺序完全由 firmware_reference.arm 的键序决定。
    """
    calibration = _read_yaml(calibration_path or package_resource(
        "rebotarm_simulation", "config/motor_control_calibration.yaml",
    ))
    reference = calibration["firmware_reference"]
    urdf_efforts = _load_urdf_efforts(urdf_path or package_resource(
        "rebotarm_moveit_config", "config/rebotarm.urdf",
    ))

    # 电机型号规格：额定/峰值力矩与减速比，供换算与重力补偿使用
    motor_specs = {
        name: MotorSpec(
            name=name,
            rated_torque_nm=float(values["rated_torque_nm"]),
            peak_torque_nm=float(values["peak_torque_nm"]),
            reduction_ratio=float(values["reduction_ratio"]),
        )
        for name, values in calibration["motor_specs"].items()
    }

    # 按固件参考的关节顺序逐个展开成等长元组，保证索引对齐
    arm_calibration = calibration["arm"]
    joint_names: list[str] = []
    motor_models: list[str] = []
    pos_kp: list[float] = []
    pos_ki: list[float] = []
    vel_kp: list[float] = []
    vel_ki: list[float] = []
    velocity_limit: list[float] = []
    effort_limit: list[float] = []
    rated_torque: list[float] = []
    firmware_to_torque_scale: list[float] = []
    torque_rate_limit_nm_s: list[float] = []
    torque_lowpass_alpha: list[float] = []
    for name, pos_vel in reference["arm"].items():
        calibration_entry = arm_calibration[name]
        motor_model = str(calibration_entry["motor_model"])
        motor_spec = motor_specs[motor_model]
        joint_names.append(str(name))
        motor_models.append(motor_model)
        pos_kp.append(float(pos_vel["pos_kp"]))
        pos_ki.append(float(pos_vel["pos_ki"]))
        vel_kp.append(float(pos_vel["vel_kp"]))
        vel_ki.append(float(pos_vel["vel_ki"]))
        velocity_limit.append(float(pos_vel["vlim"]))
        effort_limit.append(float(urdf_efforts[name]))
        rated_torque.append(float(motor_spec.rated_torque_nm))
        firmware_to_torque_scale.append(float(calibration_entry["firmware_to_torque_scale"]))
        torque_rate_limit_nm_s.append(float(calibration_entry["torque_rate_limit_nm_s"]))
        alpha = float(calibration_entry["torque_lowpass_alpha"])
        if not 0.0 < alpha <= 1.0:
            raise ValueError(f"torque_lowpass_alpha for {name} must be in (0, 1]")
        torque_lowpass_alpha.append(alpha)

    # 夹爪：固件默认增益取自固件参考段，其余换算标定取自仿真标定段
    source_gripper = reference["gripper"]
    gripper_calibration = calibration["gripper"]
    modes = gripper_calibration["modes"]
    displacement_range = gripper_calibration["displacement_range_m"]
    # 组装：控制周期取固件参考频率；积分限幅与重力补偿系数取自仿真标定段
    return MotorControlParameters(
        control_rate_hz=float(reference["rate_hz"]),
        motor_specs=motor_specs,
        arm=ArmControlParameters(
            joint_names=tuple(joint_names),
            motor_models=tuple(motor_models),
            pos_kp=tuple(pos_kp),
            pos_ki=tuple(pos_ki),
            vel_kp=tuple(vel_kp),
            vel_ki=tuple(vel_ki),
            velocity_limit=tuple(velocity_limit),
            effort_limit=tuple(effort_limit),
            rated_torque=tuple(rated_torque),
            firmware_to_torque_scale=tuple(firmware_to_torque_scale),
            torque_rate_limit_nm_s=tuple(torque_rate_limit_nm_s),
            torque_lowpass_alpha=tuple(torque_lowpass_alpha),
            gravity_compensation_scale=float(arm_calibration["gravity_compensation_scale"]),
            position_integral_limit=float(arm_calibration["position_integral_limit_rad_s"]),
            velocity_integral_limit=float(arm_calibration["velocity_integral_limit_rad"]),
        ),
        gripper=GripperControlParameters(
            firmware_default_kp=float(source_gripper["kp"]),
            firmware_default_kd=float(source_gripper["kd"]),
            motor_model=str(gripper_calibration["motor_model"]),
            move_kp=float(modes["move"]["kp"]),
            move_kd=float(modes["move"]["kd"]),
            closing_kp=float(modes["closing"]["kp"]),
            closing_kd=float(modes["closing"]["kd"]),
            hold_kp=float(modes["hold"]["kp"]),
            hold_kd=float(modes["hold"]["kd"]),
            motor_radians_per_opening_m=float(
                gripper_calibration["motor_radians_per_opening_m"]
            ),
            transmission_efficiency=float(gripper_calibration["transmission_efficiency"]),
            motor_torque_limit_nm=float(gripper_calibration["motor_torque_limit_nm"]),
            finger_force_limit_n=float(gripper_calibration["finger_force_limit_n"]),
            sim_force_kp_n_per_m=float(gripper_calibration["sim_force_kp_n_per_m"]),
            sim_force_kd_n_s_per_m=float(gripper_calibration["sim_force_kd_n_s_per_m"]),
            sim_force_deadband_m=float(gripper_calibration["sim_force_deadband_m"]),
            sim_velocity_deadband_m_s=float(gripper_calibration["sim_velocity_deadband_m_s"]),
            displacement_min_m=float(displacement_range[0]),
            displacement_max_m=float(displacement_range[1]),
        ),
    )


def _read_yaml(path: Path) -> dict:
    """读取 UTF-8 YAML 并要求顶层为映射，否则抛 ``ValueError``。"""
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"expected YAML mapping in {path}")
    return payload


def _load_urdf_efforts(path: Path) -> dict[str, float]:
    """从 URDF 提取「关节名 → limit@effort（N·m）」，作为关节力矩硬上限。"""
    root = ET.parse(path).getroot()
    return {
        joint.attrib["name"]: float(limit.attrib["effort"])
        for joint in root.findall("joint")
        for limit in [joint.find("limit")]
        if limit is not None and "effort" in limit.attrib
    }

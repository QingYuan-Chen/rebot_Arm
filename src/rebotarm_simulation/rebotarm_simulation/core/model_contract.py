"""Lightweight names shared by model generation, control and execution."""
ARM_JOINT_NAMES = tuple(f"joint{index}" for index in range(1, 7))
FINGER_JOINT_NAMES = ("left_finger_joint", "right_finger_joint")
JOINT_NAMES = ARM_JOINT_NAMES + FINGER_JOINT_NAMES


def actuator_name_for_joint(joint_name: str) -> str:
    """关节名 → MuJoCo 执行器名。

    命名约定：机械臂关节加 ``_torque`` 后缀，手指关节去掉 ``_joint`` 后加 ``_force``。
    该名字由仿真运行时按名查找执行器，属于对外接口，不可更改。
    """
    if joint_name in {f"joint{index}" for index in range(1, 7)}:
        return f"{joint_name}_torque"
    return f"{joint_name.removesuffix('_joint')}_force"

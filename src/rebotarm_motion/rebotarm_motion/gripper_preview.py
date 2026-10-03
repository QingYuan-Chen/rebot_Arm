"""仅生成显示用轨迹：不导入控制器、不发布状态、不调用动作或服务。"""

from copy import deepcopy
import math

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


FINGER_NAMES = ["left_finger_joint", "right_finger_joint"]


def with_gripper_preview(trajectories, indices, openings):
    """按阶段插入半秒开合动画，并在机械臂运动时保持手指开口。

    起始开口为显示用闭合状态，不冒充控制器反馈；左右指按 URDF 正负半开口换算。
    原轨迹不修改。事件为空时保持原六轴预览兼容行为。
    """
    if len(indices) != len(openings):
        raise ValueError("gripper preview event arrays differ in length")
    if not indices:
        return trajectories
    if any(i > len(trajectories) for i in indices) or list(indices) != sorted(indices):
        raise ValueError("gripper preview event indices are invalid or unordered")
    if any(not math.isfinite(w) or not 0.0 <= w <= 0.09 for w in openings):
        raise ValueError("gripper preview opening outside URDF limits [0, 0.09] m")
    names = list(trajectories[0].joint_names)
    if len(names) != 6 or set(names) != {f"joint{i}" for i in range(1, 7)}:
        raise ValueError("gripper preview requires six arm joints")
    position = list(trajectories[0].points[0].positions)
    width = 0.0
    event = 0
    result = []
    for index in range(len(trajectories) + 1):
        while event < len(indices) and indices[event] == index:
            target = float(openings[event])
            animation = JointTrajectory()
            animation.joint_names = names + FINGER_NAMES
            for sample in range(11):
                opening = width + (target - width) * sample / 10.0
                point = JointTrajectoryPoint()
                point.positions = position + [opening / 2.0, -opening / 2.0]
                point.time_from_start.nanosec = sample * 50_000_000
                animation.points.append(point)
            result.append(animation)
            width = target
            event += 1
        if index == len(trajectories):
            break
        arm = deepcopy(trajectories[index])
        arm.joint_names = names + FINGER_NAMES
        for point in arm.points:
            point.positions = list(point.positions) + [width / 2.0, -width / 2.0]
            for field in ("velocities", "accelerations", "effort"):
                values = list(getattr(point, field))
                if values:
                    if len(values) != 6:
                        raise ValueError(f"invalid arm preview {field} dimension")
                    setattr(point, field, values + [0.0, 0.0])
        position = list(trajectories[index].points[-1].positions)
        result.append(arm)
    return result

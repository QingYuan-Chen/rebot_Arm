"""控制器离线测试的连续曲线夹具,只替代 motion 准备服务的消息交接."""
import time

from rebotarm_motion.polynomial_trajectory import JOINT_NAMES, TrajectoryPoint, build_curve


def curve(initial, target, duration=1.):
    """构造具名六轴位置曲线,测试仍使用生产数值内核."""
    return build_curve(JOINT_NAMES, (
        TrajectoryPoint(0., tuple(initial)), TrajectoryPoint(duration, tuple(target))))


def submit_curve(manager, target, duration=1., *, settle=2.):
    """预约,解码及接纳均调用正式所有者;夹具只提供冻结系数和上下文."""
    manager.arm.update_trajectory_scene('scene-a')
    if not manager.runtime.arm_resource_reserved:
        deadline = time.monotonic()+2.
        while not manager.arm.admission_snapshot()['ready']:
            assert time.monotonic() < deadline, manager.arm.admission_snapshot()
            time.sleep(.005)
    token = manager.arm.reserve_arm_preparation(time.monotonic()+2.)
    try:
        prepared = curve(token.hold, target, duration)
        payload = dict(curve=prepared.to_payload(), context=dict(session=token.session,
            generation=token.generation, hold=token.hold), gripper_positions=token.gripper_positions,
            gripper_generation=token.gripper_generation, scene_revision=token.scene_revision,
            capability_revision=token.capability, gripper_collision_policy=token.collision_policy)
        task = manager.arm.admit_prepared_arm_trajectory(token,
            manager.arm.decode_prepared_arm_trajectory(payload))
        # 定向故障测试缩短末点等待,不改变设备发送和反馈到位判定.
        task.deadline += settle-2.
        return task
    except Exception:
        manager.arm.cancel_arm_preparation(token)
        raise

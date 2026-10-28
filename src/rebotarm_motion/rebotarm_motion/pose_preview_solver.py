"""位姿预览的兼容接口：规范模型上的 FK 与带重试 IK。

``solve_pose`` 把 xyz/rpy 目标和当前六轴种子转换成预览结果；
``compute_pose_target`` 将六轴反馈转换为末端位姿。这里只计算，不连接设备，
不发送 ROS 目标；不可达时保留原种子并报告失败。关节顺序必须是 joint1..joint6。
旧构造参数 workspace_root 保留兼容，资源由配置包安装索引或源码路径定位。
"""
from .command_models import PoseTarget, PreviewSolveResult
from .pose_kinematics import JOINT_NAMES, PoseKinematics, pose_target
import pinocchio as pin


class PosePreviewSolver:
    def __init__(self, workspace_root=None, end_frame_name='end_link'):
        self.kinematics = PoseKinematics(end_frame=end_frame_name)
        self._end_frame_name = end_frame_name

    def solve_pose(self, pose_target_value, seed_positions, joint_names):
        if tuple(joint_names) != JOINT_NAMES:
            raise ValueError('preview requires canonical joint order')
        value = pose_target_value
        target = pose_target((value.x, value.y, value.z, value.roll, value.pitch, value.yaw))
        result = self.kinematics.solve(target, seed_positions, retries=5)
        return PreviewSolveResult(success=result.success,
            joint_positions=tuple(float(v) for v in result.q) if result.success else seed_positions,
            message='ik preview ok' if result.success else f'target pose unreachable: error={result.error:.4e}')

    def compute_pose_target(self, joint_positions):
        transform = self.kinematics.pose(joint_positions)
        roll, pitch, yaw = pin.rpy.matrixToRpy(transform.rotation)
        x, y, z = transform.translation
        return PoseTarget(x=float(x), y=float(y), z=float(z),
                          roll=float(roll), pitch=float(pitch), yaw=float(yaw))

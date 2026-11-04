"""位姿预览的兼容接口: 规范模型上的 FK 与带重试 IK.

职责与位置
    把规范模型 FK/IK 包装成预览层的值类型接口.

对外接口
    PosePreviewSolver.solve_pose 返回 PreviewSolveResult; compute_pose_target 返回 PoseTarget.

安全与并发约束
    只计算可达性, 不发送运动; 失败保留种子, 调用方仍须执行路径与碰撞预检.

文件结构(按定义顺序)
    PosePreviewSolver: 预览数据契约与规范 FK/IK 之间的薄适配, 不负责执行准入.
"""

from .command_models import PoseTarget, PreviewSolveResult
from .pose_kinematics import JOINT_NAMES, PoseKinematics, pose_target
import pinocchio as pin


class PosePreviewSolver:
    """预览数据契约与规范 FK/IK 之间的薄适配, 不负责执行准入."""

    def __init__(self, workspace_root=None, end_frame_name='end_link'):
        """按末端帧建立规范运动学适配; workspace_root 仅保留旧签名兼容, 不定位 SDK."""
        self.kinematics = PoseKinematics(end_frame=end_frame_name)
        self._end_frame_name = end_frame_name

    def solve_pose(self, pose_target_value, seed_positions, joint_names):
        """校验固定关节顺序, 求解 xyz/RPY 目标并封装结果; 不收敛时保留种子而非失败候选角."""
        if tuple(joint_names) != JOINT_NAMES:
            raise ValueError('preview requires canonical joint order')
        value = pose_target_value
        target = pose_target((value.x, value.y, value.z, value.roll, value.pitch, value.yaw))
        result = self.kinematics.solve(target, seed_positions, retries=5)
        return PreviewSolveResult(success=result.success,
            joint_positions=tuple(float(v) for v in result.q) if result.success else seed_positions,
            message='ik preview ok' if result.success else f'target pose unreachable: error={result.error:.4e}')

    def compute_pose_target(self, joint_positions):
        """以规范 FK 把六轴角度转换为基座中的米制位置与固定轴 RPY 预览值."""
        transform = self.kinematics.pose(joint_positions)
        roll, pitch, yaw = pin.rpy.matrixToRpy(transform.rotation)
        x, y, z = transform.translation
        return PoseTarget(x=float(x), y=float(y), z=float(z),
                          roll=float(roll), pitch=float(pitch), yaw=float(yaw))

"""Candidate policy adapter with explicit configuration and joint-state inputs."""
from __future__ import annotations

from types import SimpleNamespace
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from geometry_msgs.msg import Pose
    from sensor_msgs.msg import JointState

    from rebotarm_msgs.msg import GraspCandidateArray

from tf2_ros import TransformException

from .candidate_gate_policy import CandidateGateConfig, evaluate_candidate_gate
from .candidate_motion_policy import (
    JointMotionPolicyConfig,
    evaluate_joint_motion,
    joint_positions_by_name,
)
from .candidate_precheck_policy import CandidatePrecheckConfig, evaluate_candidate_precheck
from .candidate_target_policy import CandidateTargetPolicyConfig, build_candidate_target_variants
from .pose_variant_policy import PoseVariantConfig
from ..visual_grasp_sequence import PoseTarget


def _parameter(context, name):
    config = getattr(context, "_config", None)
    if config is not None:
        return SimpleNamespace(value=config.values[name])
    return context.get_parameter(name)


class CandidateIkPolicy:
    def __init__(self, *, config, joint_state, transform_pose, logger):
        self._config = config
        self._latest_joint_state = joint_state
        self._transform_pose_to_target_frame = transform_pose
        self._logger = logger

    def get_logger(self):
        return self._logger

    def _tuple3(self, name: str) -> tuple[float, float, float]:
        """读取三元组向量参数并转为 float；长度不是 3 时立即抛错，避免静默使用错参数。"""
        values = list(_parameter(self, name).value)
        if len(values) != 3:
            raise ValueError(f"{name} must contain exactly 3 values")
        return (float(values[0]), float(values[1]), float(values[2]))


    def _tuple4(self, name: str) -> tuple[float, float, float, float]:
        """读取四元数参数并转为 float；长度不是 4 时立即抛错（四元数必须完整给出 x,y,z,w）。"""
        values = list(_parameter(self, name).value)
        if len(values) != 4:
            raise ValueError(f"{name} must contain exactly 4 values")
        return (float(values[0]), float(values[1]), float(values[2]), float(values[3]))


    def _candidate_precheck_allows(self, candidate) -> bool:
        """候选级预检：置信度有限且不低于下限，夹爪宽度有限且落在可信行程内。"""
        result = evaluate_candidate_precheck(
            confidence=float(getattr(candidate, "confidence", 0.0)),
            jaw_width_m=float(getattr(candidate, "jaw_width", 0.0)),
            config=CandidatePrecheckConfig(
                min_confidence=float(_parameter(self, "candidate_min_confidence").value),
                min_jaw_width_m=float(_parameter(self, "candidate_min_jaw_width_m").value),
                max_jaw_width_m=float(_parameter(self, "candidate_max_jaw_width_m").value),
            ),
        )
        return result.accepted


    def _candidate_gate_allows(self, candidate, *, grasp: PoseTarget) -> bool:
        """几何闸门：夹爪宽度、抓取点最低高度，以及可选的工作空间盒检查。

        工作空间闸门默认关闭：关闭时既不判断坐标范围，也不查询 TF 求目标中心，省掉一次
        变换；开启时把候选自身位姿变换到 target_frame 当作目标中心，并用它与抓取点的
        距离判断抓取点是否已明显偏离物体。
        """
        try:
            workspace_enabled = bool(_parameter(self, "candidate_workspace_gate_enabled").value)
        except (KeyError, AttributeError):
            # 参数缺失或类型异常时按"不启用"处理，与默认配置保持一致
            workspace_enabled = False
        object_center_xyz = None
        # 闸门关闭时使用与参数默认值相同的占位范围（该分支下不会被用到）
        workspace_min_xyz = (-0.35, -0.64, 0.0)
        workspace_max_xyz = (0.35, -0.18, 0.45)
        max_grasp_to_object_center_m = 0.15
        if workspace_enabled:
            object_center_xyz = self._candidate_object_center_in_target_frame(candidate)
            workspace_min_xyz = self._tuple3("candidate_workspace_min_xyz")
            workspace_max_xyz = self._tuple3("candidate_workspace_max_xyz")
            max_grasp_to_object_center_m = float(
                _parameter(self, "candidate_max_grasp_to_object_center_m").value
            )
        result = evaluate_candidate_gate(
            jaw_width_m=float(getattr(candidate, "jaw_width", 0.0)),
            grasp_position_xyz=tuple(float(v) for v in grasp.position),
            object_center_xyz=object_center_xyz,
            config=CandidateGateConfig(
                min_jaw_width_m=float(_parameter(self, "candidate_min_jaw_width_m").value),
                max_jaw_width_m=float(_parameter(self, "candidate_max_jaw_width_m").value),
                min_grasp_z_m=float(_parameter(self, "candidate_min_grasp_z_m").value),
                workspace_gate_enabled=workspace_enabled,
                workspace_min_xyz=workspace_min_xyz,
                workspace_max_xyz=workspace_max_xyz,
                max_grasp_to_object_center_m=max_grasp_to_object_center_m,
            ),
        )
        if not result.accepted:
            self.get_logger().warn(f"candidate IK filter rejected reachable grasp: {result.reason}")
            return False
        return True


    def _candidate_object_center_in_target_frame(self, candidate) -> tuple[float, float, float] | None:
        """取候选位姿作为目标中心并变换到 target_frame；变换失败返回 None。

        候选消息没有独立的目标中心字段，这里用候选位姿近似目标中心，仅用于"抓取点是否
        离物体太远"这一粗粒度判断。
        """
        pose = getattr(candidate, "pose", None)
        if pose is None:
            return None
        source_frame = str(getattr(getattr(candidate, "header", None), "frame_id", "") or "")
        try:
            stamp = getattr(getattr(candidate, "header", None), "stamp", None)
            object_pose = self._transform_pose_to_target_frame(pose, source_frame, stamp)
        except (TransformException, RuntimeError, ValueError) as exc:
            self.get_logger().warn(f"candidate object center transform failed: source_frame={source_frame}: {exc}")
            return None
        return (
            float(object_pose.position.x),
            float(object_pose.position.y),
            float(object_pose.position.z),
        )


    def _list_float_parameter(self, name: str) -> list[float]:
        """读取浮点数组参数（偏航角偏移、高度偏移等），统一转换为 float 列表。"""
        values = list(_parameter(self, name).value)
        return [float(value) for value in values]


    def _candidate_target_variants(
        self,
        candidates: GraspCandidateArray,
        pose: Pose,
    ) -> list[tuple[PoseTarget, PoseTarget, str]]:
        """生成该候选全部待校验的 (接近点, 抓取点, 变体标签) 组合。

        先把候选位姿从消息声明的坐标系变换到 target_frame，再交给位姿策略按 pose_policy、
        偏航角偏移、高度偏移、TCP 偏移与平行夹爪对称配置展开变体；标签用于日志与打分，
        其中 yaw/z 下标对应参数列表下标。
        """
        source_frame = str(candidates.header.frame_id)
        stamp = getattr(candidates.header, "stamp", None)
        grasp_pose = self._transform_pose_to_target_frame(pose, source_frame, stamp)
        position_xyz = (
            float(grasp_pose.position.x),
            float(grasp_pose.position.y),
            float(grasp_pose.position.z),
        )
        variants = build_candidate_target_variants(
            grasp_position_xyz=position_xyz,
            candidate_orientation_xyzw=(
                float(grasp_pose.orientation.x),
                float(grasp_pose.orientation.y),
                float(grasp_pose.orientation.z),
                float(grasp_pose.orientation.w),
            ),
            config=CandidateTargetPolicyConfig(
                pose_policy=str(_parameter(self, "pose_policy").value),
                fixed_grasp_orientation_xyzw=self._tuple4("fixed_grasp_orientation_xyzw"),
                base_approach_axis_xyz=self._tuple3("base_approach_axis_xyz"),
                base_pregrasp_distance_m=float(_parameter(self, "base_pregrasp_distance_m").value),
                tcp_offset_xyz=self._tuple3("tcp_offset_xyz"),
                target_base_offset_xyz=self._tuple3("target_base_offset_xyz"),
                pregrasp_min_z_m=float(_parameter(self, "candidate_pregrasp_min_z_m").value),
                grasp_base_z_offset_m=float(_parameter(self, "grasp_base_z_offset_m").value),
                orientation_yaw_offsets_rad=tuple(self._list_float_parameter("orientation_yaw_offsets_rad")),
                candidate_grasp_z_offsets_m=tuple(self._list_float_parameter("candidate_grasp_z_offsets_m")),
                pose_variant_config=PoseVariantConfig(
                    joint6_symmetry_enabled=bool(_parameter(self, "candidate_joint6_symmetry_enabled").value),
                    joint6_symmetry_angle_rad=float(_parameter(self, "candidate_joint6_symmetry_angle_rad").value),
                ),
            ),
        )
        return [(variant.pregrasp, variant.grasp, variant.label) for variant in variants]


    def _joint_motion_penalty(self, robot_state) -> tuple[float | None, str]:
        """计算逆解相对当前关节状态的位移代价，并拦截 joint6 过大的解。

        返回 (代价, 说明)；当 joint6 变化超过 candidate_max_joint6_delta_rad 时返回
        (None, 说明) 表示该解不可行——这类解虽然能到达目标位姿，但腕部需要转过很大的角度，
        通常说明还存在更省力的等效解（例如平行夹爪的 180° 对称姿态）。当前状态缺失或
        找不到公共关节名时返回 0 代价，即"无法比较就不惩罚"，避免因为反馈缺失而否决全部候选。
        """
        current = self._joint_positions_by_name(self._latest_joint_state)
        solution_joint_state = getattr(robot_state, "joint_state", None)
        target = self._joint_positions_by_name(solution_joint_state)
        # 只统计以 joint 开头的臂关节，夹爪等执行器关节不参与位移代价
        common_names = [name for name in current if name in target and name.startswith("joint")]
        if not common_names:
            return 0.0, "joint_delta=unknown"
        evaluation = evaluate_joint_motion(
            current_positions=current,
            target_positions=target,
            config=JointMotionPolicyConfig(
                joint_distance_weight=float(_parameter(self, "candidate_score_joint_distance_weight").value),
                joint6_weight=float(_parameter(self, "candidate_score_joint6_weight").value),
                max_joint6_delta_rad=float(_parameter(self, "candidate_max_joint6_delta_rad").value),
            ),
        )
        if not evaluation.accepted:
            self.get_logger().warn(
                "candidate IK filter rejected reachable grasp: "
                f"joint6 delta too large "
                f"({evaluation.joint6_delta:.3f}rad > "
                f"{float(_parameter(self, 'candidate_max_joint6_delta_rad').value):.3f}rad)"
            )
            return None, evaluation.reason
        return evaluation.penalty, evaluation.reason


    def _joint_positions_by_name(self, joint_state: JointState | None) -> dict[str, float]:
        """把关节状态整理成 {关节名: 弧度} 字典，非有限值直接跳过。"""
        if joint_state is None:
            return {}
        names = list(getattr(joint_state, "name", []))
        positions = list(getattr(joint_state, "position", []))
        return joint_positions_by_name(names, positions)

"""MoveIt precheck adaptation for teach replay workflow."""

from __future__ import annotations

from collections.abc import Sequence

from rebotarm_motion.collision_precheck import CollisionPrecheckConfig
from rebotarm_motion.teach_replay_start_align_precheck import MoveItStartAlignPrecheckConfig
from rebotarm_motion.teach_replay_start_alignment import MoveItStartAlignmentConfig
from trajectory_msgs.msg import JointTrajectory


class ReplayMoveItPrecheckMixin:
    def _moveit_align_summary(
        self, info_payload: dict, samples: Sequence[object] | None = None, *, plan: bool = False
    ) -> dict:
        """汇总"是否需要并能否完成 MoveIt 起始对齐"的判定结果。

        ``plan=False`` 只做可用性检查（服务是否就绪、起始误差是否已小于跳过阈值），
        用于 execute 前的轻量评估；``plan=True`` 时额外真的规划一次到记录首点的关节空间
        轨迹，用于 dry-run 给出可信结论。起始误差小于 ``moveit_start_skip_threshold``
        （rad）时直接判为 skipped，不调用规划服务。
        """
        return self._moveit_start_align_prechecker.summary(
            info_payload,
            config=MoveItStartAlignPrecheckConfig(
                enabled=bool(self.get_parameter("use_moveit_start_align").value),
                service=str(self.get_parameter("moveit_planning_service").value),
                skip_threshold=float(self.get_parameter("moveit_start_skip_threshold").value),
                # 关节目标容差 rad，以及规划的速度/加速度缩放（0~1，越小越慢越稳）。
                joint_goal_tolerance=float(self.get_parameter("moveit_joint_goal_tolerance").value),
                velocity_scaling=float(self.get_parameter("moveit_velocity_scaling").value),
                acceleration_scaling=float(self.get_parameter("moveit_acceleration_scaling").value),
            ),
            samples=samples,
            plan=plan,
        )

    def _collision_precheck(self, samples: Sequence[object]) -> dict:
        """对预处理后的样本序列做碰撞预检（逐采样点检查关节位置是否有效）。"""
        if not samples:
            # 没有样本时仍走统一入口：由预检器返回 unknown，避免"空输入=无碰撞"的误判。
            return self._collision_precheck_positions((), [])
        first = samples[0]
        positions = [tuple(sample.positions) for sample in samples]
        return self._collision_precheck_positions(tuple(first.joint_names), positions)

    def _collision_precheck_trajectory(self, trajectory: JointTrajectory) -> dict:
        """对最终要下发的轨迹点做碰撞预检（真实回放前的最后一道门）。"""
        positions = [
            tuple(point.positions)
            for point in getattr(trajectory, "points", [])
            if getattr(point, "positions", None)
        ]
        return self._collision_precheck_positions(tuple(trajectory.joint_names), positions)

    def _collision_precheck_positions(self, joint_names: tuple[str, ...], positions_list: list[tuple[float, ...]]) -> dict:
        """以节点参数构造碰撞预检配置并执行检查（关节位置单位 rad）。"""
        default_joint_positions = self._collision_default_joint_positions(joint_names)
        return self._collision_prechecker.check_positions(
            joint_names=joint_names,
            positions_list=positions_list,
            config=CollisionPrecheckConfig(
                enabled=bool(self.get_parameter("collision_check_enabled").value),
                service=str(self.get_parameter("collision_check_service").value),
                group_name=str(self.get_parameter("collision_group_name").value),
                # 采样上限至少 1 个；上限越大越保险，但每次调用服务的次数线性增加。
                max_samples=max(int(self.get_parameter("collision_check_max_samples").value), 1),
                # 整轮预检总超时秒数，下限 0.1 s，防止配置成 0 导致必然超时。
                timeout_sec=max(float(self.get_parameter("collision_check_timeout_sec").value), 0.1),
                default_joint_positions=default_joint_positions,
            ),
        )

    def _append_moveit_start_alignment(
        self,
        trajectory: JointTrajectory,
        *,
        current_positions: tuple[float, ...],
        first_positions: tuple[float, ...],
    ) -> float:
        """起始对齐回调：把规划出的对齐段追加到轨迹前部，返回对齐段结束时刻（秒）。

        起始误差小于 ``moveit_start_skip_threshold``（rad）时跳过规划，只做保持点；
        规划失败会抛异常向上传播，由调用方按"构造失败=阻断"处理。
        """
        return self._moveit_start_aligner.append(
            trajectory,
            current_positions=current_positions,
            first_positions=first_positions,
            config=MoveItStartAlignmentConfig(
                start_hold_sec=float(self.get_parameter("start_hold_sec").value),
                first_hold_sec=float(self.get_parameter("first_hold_sec").value),
                skip_threshold=float(self.get_parameter("moveit_start_skip_threshold").value),
                joint_goal_tolerance=float(self.get_parameter("moveit_joint_goal_tolerance").value),
                velocity_scaling=float(self.get_parameter("moveit_velocity_scaling").value),
                acceleration_scaling=float(self.get_parameter("moveit_acceleration_scaling").value),
            ),
        )


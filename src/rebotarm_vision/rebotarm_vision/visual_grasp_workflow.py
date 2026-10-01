from __future__ import annotations

import time
from copy import deepcopy
from types import SimpleNamespace

import rclpy
from geometry_msgs.msg import Pose

from rebotarm_msgs.msg import GraspCandidateArray, GraspPlan

from .policies.grasp_retry_policy import RetryPolicyConfig, ordered_candidate_indices
from .policies.grasp_verification_policy import (
    GraspVerificationConfig,
    GraspVerificationInput,
    verify_grasp_after_close,
)
from .policies.gripper_policy import GripperPolicyConfig, resolve_gripper_command
from .utils.message_freshness import is_message_fresh, message_age_sec
from .policies.place_task_policy import PlaceTaskConfig, build_place_stages
from .policies.retreat_policy import RetreatPolicyConfig
from .utils.tf_message_adapter import (
    apply_tcp_offset_to_pose,
    transform_from_msg,
    transform_pose_message,
)
from .visual_grasp_io import io_gateway_for as _io_gateway_for
from .utils.visual_grasp_messages import pose_to_target
from .policies.visual_grasp_pose_policy import BaseAxisGraspPolicyConfig, build_base_axis_grasp_targets
from .policies.visual_grasp_runtime_policy import (
    detected_jaw_width,
    detected_object_length,
)
from .visual_grasp_sequence import (
    PoseTarget,
    VisualGraspSequenceConfig,
    VisualGraspStage,
    build_visual_grasp_sequence,
)
from .visual_grasp_service_gateway import VisualGraspServiceGateway
from .visual_grasp_state import _state_for, plan_snapshot
from .policies.visual_servo_policy import VisualServoApproachConfig, build_visual_servo_step


def _parameter(context, name: str):
    if hasattr(context, "_config") and context._config is not None:
        return SimpleNamespace(value=context._config.get(name))
    return context.get_parameter(name)




class VisualGraspWorkflow:
    """Stage coordination with explicit resources and a fixed task configuration.

    This object never holds a ROS Node. Node callbacks replace the shared state;
    all service dispatch goes through the injected I/O gateway.
    """

    def __init__(self, *, state, config, io_gateway, tf_buffer, clock, logger, monotonic=time.monotonic, sleep=time.sleep, context_ok=rclpy.ok):
        self._state = state
        self._config = config
        self._io_gateway = io_gateway
        self._tf_buffer = tf_buffer
        self._clock = clock
        self._logger = logger
        self._monotonic = monotonic
        self._sleep = sleep
        self._context_ok = context_ok
        self._tf_cache: dict[tuple[str, str, int], object] = {}
        self._services = VisualGraspServiceGateway(state=state, config=config, io_gateway=io_gateway)
        self._input_topic = config.input_topic
        self._max_plan_age_sec = config.max_plan_age_sec
        self._target_frame = str(config.get("target_frame", "base_link"))
        self._tcp_offset_xyz = tuple(config.get("tcp_offset_xyz", (0.0, 0.0, 0.0)))
        self._target_base_offset_xyz = tuple(config.get("target_base_offset_xyz", (0.0, 0.0, 0.0)))
        self._grasp_base_z_offset_m = float(config.get("grasp_base_z_offset_m", 0.0))
        self._service_timeout_sec = float(config.get("service_timeout_sec", 20.0))
        self._motion_result_timeout_sec = float(config.get("motion_result_timeout_sec", 45.0))
        self._execution_mode = str(config.get("execution_mode", "plan_only")).strip().lower()
        self._stage_waits = {
            "move_to_pregrasp": float(config.get("pregrasp_wait_sec", 0.5)),
            "approach_grasp": float(config.get("approach_wait_sec", 0.2)),
            "close_gripper": float(config.get("gripper_wait_sec", 1.0)),
            "open_gripper": float(config.get("gripper_wait_sec", 1.0)),
            "safe_retreat": float(config.get("retreat_wait_sec", 0.5)),
        }

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return self._clock

    def _tuple3(self, name: str) -> tuple[float, float, float]:
        values = self._tuple_n(name, 3)
        return (values[0], values[1], values[2])


    def _tuple_n(self, name: str, expected_len: int) -> tuple[float, ...]:
        values = list(_parameter(self, name).value)
        if len(values) != expected_len:
            raise ValueError(f"{name} must contain exactly {expected_len} values")
        return tuple(float(value) for value in values)


    def _plan_is_fresh(self, plan: GraspPlan | None) -> bool:
        """判断计划是否可作为执行依据：时间戳必须已设置，且时延不超过 `max_plan_age_sec`。

        时间戳为 0（未设置）一律视为不新鲜；允许轻微的"未来时间戳"以容忍时钟偏差。
        拒绝时打印 age 与阈值，便于排查"有检测但执行器说没有计划"的问题。
        """

        if plan is None:
            return False
        header = getattr(plan, "header", None)
        stamp = getattr(header, "stamp", None)
        now_ns = int(self.get_clock().now().nanoseconds)
        age_sec = message_age_sec(
            stamp,
            now_ns=now_ns,
        )
        if not is_message_fresh(
            stamp,
            now_ns=now_ns,
            max_age_sec=self._max_plan_age_sec,
        ):
            self.get_logger().warn(
                "visual grasp executor rejected stale or unset plan: "
                f"age_sec={age_sec} max_plan_age_sec={self._max_plan_age_sec}"
            )
            return False
        return True


    def _build_sequence_from_plan(self, plan: GraspPlan) -> list[VisualGraspStage]:
        """按抓取计划与当前参数生成完整阶段序列（不含可选的放置阶段）。

        流程：先算接近点/抓取点（位姿策略），再让夹爪策略根据检测宽度解析出张开、闭合宽度
        与力矩（可能直接拒绝抓取），最后交给序列构建策略生成阶段列表。若策略判定"目标
        过宽不可抓"，序列构建会抛 ValueError，由上游执行回调的异常分支统一处理。
        """

        pregrasp, grasp = self._build_motion_targets(plan)
        gripper_command = resolve_gripper_command(
            jaw_width_m=self._detected_jaw_width(plan),
            object_length_m=self._detected_object_length(plan),
            class_name=str(getattr(plan.candidate, "class_name", "") or ""),
            config=GripperPolicyConfig(
                auto_width=bool(_parameter(self, "auto_gripper_width").value),
                auto_effort=bool(_parameter(self, "auto_gripper_effort").value),
                default_open_width_m=float(_parameter(self, "open_position_m").value),
                default_close_width_m=float(_parameter(self, "close_position_m").value),
                default_max_effort=float(_parameter(self, "close_max_effort").value),
                open_clearance_m=float(_parameter(self, "open_clearance_m").value),
                close_margin_m=float(_parameter(self, "close_margin_m").value),
                min_open_width_m=float(_parameter(self, "min_open_position_m").value),
                max_open_width_m=float(_parameter(self, "max_open_position_m").value),
                min_close_width_m=float(_parameter(self, "min_close_position_m").value),
                max_close_width_m=float(_parameter(self, "max_close_position_m").value),
                min_effort=float(_parameter(self, "min_gripper_effort").value),
                max_effort=float(_parameter(self, "max_gripper_effort").value),
                max_allowed_width_m=float(_parameter(self, "max_allowed_grasp_width_m").value),
            ),
        )
        config = VisualGraspSequenceConfig(
            open_before_approach=bool(_parameter(self, "open_before_approach").value),
            open_position_m=float(_parameter(self, "open_position_m").value),
            close_position_m=float(_parameter(self, "close_position_m").value),
            close_max_effort=float(_parameter(self, "close_max_effort").value),
            min_grasp_z_m=float(_parameter(self, "min_grasp_z_m").value),
            auto_gripper_width=bool(_parameter(self, "auto_gripper_width").value),
            detected_jaw_width_m=self._detected_jaw_width(plan),
            open_clearance_m=float(_parameter(self, "open_clearance_m").value),
            close_margin_m=float(_parameter(self, "close_margin_m").value),
            min_open_position_m=float(_parameter(self, "min_open_position_m").value),
            max_open_position_m=float(_parameter(self, "max_open_position_m").value),
            min_close_position_m=float(_parameter(self, "min_close_position_m").value),
            max_close_position_m=float(_parameter(self, "max_close_position_m").value),
            gripper_command=gripper_command,
            retreat_policy=RetreatPolicyConfig(
                enabled=bool(_parameter(self, "safe_retreat_enabled").value),
                retreat_distance_m=float(_parameter(self, "safe_retreat_distance_m").value),
            ),
            include_safe_home=bool(_parameter(self, "safe_home_after_grasp").value),
        )
        return build_visual_grasp_sequence(pregrasp, grasp, config)


    def _candidate_plans_for_attempts(self) -> list[tuple[int, GraspPlan]]:
        """组装本轮执行的尝试序列，元素为 (候选下标, 计划)。

        第 0 项永远是上游最优计划本身，候选下标记为 -1；若候选数组可用且启用了自动重试，
        再按重试策略给出的顺序追加其它候选（跳过最优候选本身，避免重复执行同一目标）。
        重试策略关闭时排序函数只返回最优下标，而这里会把它过滤掉，结果仍只有第 0 项，
        等价于"只执行一次"。
        """

        state = _state_for(self)
        with state.lock:
            plan = deepcopy(state.latest_plan)
            candidates = deepcopy(state.latest_candidates)
        if plan is None:
            return []
        attempts: list[tuple[int, GraspPlan]] = [(-1, plan)]
        if candidates is None or not candidates.candidates:
            return attempts
        indices = ordered_candidate_indices(
            candidate_count=len(candidates.candidates),
            best_index=int(candidates.best_index),
            failed_indices=set(),
            config=RetryPolicyConfig(
                enabled=bool(_parameter(self, "auto_retry_enabled").value),
                max_attempts=int(_parameter(self, "auto_retry_max_attempts").value),
            ),
        )
        for index in indices:
            if index == int(candidates.best_index):
                continue
            attempts.append((index, self._plan_from_candidate(candidates, index)))
        return attempts


    def _plan_from_candidate(self, candidates: GraspCandidateArray, index: int) -> GraspPlan:
        """把候选数组中的第 index 个候选包装成可直接执行的抓取计划。

        候选本身只给一个 TCP 位姿，没有独立接近点，因此这里把接近点与抓取点都设为该位姿；
        真正的进给路径由下游运动层规划决定，不代表本节点会走一条直线接近段。
        计划来源标记为 `visual_grasp_executor_retry`，方便在日志里区分"上游计划"与"重试计划"。
        """

        candidate = candidates.candidates[int(index)]
        plan = GraspPlan()
        plan.header = candidates.header
        plan.candidate = deepcopy(candidate)
        plan.pregrasp_pose = deepcopy(candidate.pose)
        plan.grasp_pose = deepcopy(candidate.pose)
        plan.jaw_width = float(candidate.jaw_width)
        plan.valid = True
        plan.source = "visual_grasp_executor_retry"
        plan.reason = ""
        return plan


    def _execute_stages(self, stages: list[VisualGraspStage]) -> tuple[bool, str, str]:
        """顺序执行阶段列表，返回 (是否全部成功, 说明, 失败阶段名)。

        阶段列表在运行中可能被替换：到达接近点后若拿到更新的计划，就用新计划重建剩余阶段；
        若启用了视觉伺服逼近，则删掉原来的 approach_grasp 阶段（已由伺服步替代）。替换只
        影响尚未执行的阶段，已完成阶段不会重放。

        两个特殊阶段：
          - move_to_pregrasp：记录本阶段为"重试前撤回点"，并按参数等待更新的计划；刷新被
            设为必需却等不到时，本次尝试直接判失败。
          - close_gripper：闭合完成后立即用接触与闭合行程做抓取验证；验证通过后才撤退。
        """

        stage_index = 0
        while stage_index < len(stages):
            if not _state_for(self).is_running():
                return False, "stopped", stages[stage_index].name
            stage = stages[stage_index]
            state = _state_for(self)
            with state.lock:
                stage_start_revision = state.plan_revision
            ok, message = self._run_stage(stage)
            if not ok:
                return False, message, stage.name
            if stage.name == "move_to_pregrasp":
                _state_for(self).retry_retreat_stage = stage
                refreshed_plan = self._wait_for_refreshed_plan(stage_start_revision)
                if (
                    refreshed_plan is None
                    and bool(_parameter(self, "refresh_plan_at_pregrasp_enabled").value)
                    and bool(_parameter(self, "refresh_plan_at_pregrasp_required").value)
                ):
                    return False, "fresh grasp plan unavailable after pregrasp", stage.name
                if refreshed_plan is not None:
                    refreshed_stages = self._append_place_stages(self._build_sequence_from_plan(refreshed_plan))
                    stages = self._replace_remaining_after_pregrasp(stages, refreshed_stages, stage_index)
                if self._approach_visual_servo_enabled() and stage.pose is not None:
                    ok, message = self._run_visual_servo_approach(stage.pose)
                    if not ok:
                        return False, message, "visual_servo_approach"
                    stages = self._remove_approach_after_pregrasp(stages, stage_index)
            if stage.name == "close_gripper":
                verified, reason = self._verify_after_close()
                if not verified:
                    return False, reason, stage.name
            stage_index += 1
        return True, "ok", ""


    def _diagnostic_prefix(self, stage: str) -> str:
        """构造统一日志前缀，把轮次、尝试序号、候选下标与阶段名串起来便于排障。"""

        return (
            f"[visual_grasp][run={_state_for(self).current_run_id}]"
            f"[attempt={_state_for(self).current_attempt_index}]"
            f"[candidate={_state_for(self).current_candidate_index}]"
            f"[stage={stage}]"
        )


    def _log_diagnostic(self, stage: str, status: str, details: str = "") -> None:
        """输出一条带前缀的诊断日志；`details` 为空时不追加冒号后缀。"""

        suffix = f": {details}" if details else ""
        self.get_logger().info(f"{self._diagnostic_prefix(stage)} {status}{suffix}")


    def _log_plan_snapshot(self, plan: GraspPlan) -> None:
        """打印计划快照：有效性、来源、原因、类别、置信度、夹爪宽度与接近/抓取位姿。

        计划与候选字段都按可选字段读取（缺失回落到候选值或 0），因此对缺字段的旧消息也安全。
        """

        snapshot = plan_snapshot(plan)
        self.get_logger().info(
            f"{self._diagnostic_prefix('plan')} "
            f"valid={snapshot.valid}, source={snapshot.source}, reason={snapshot.reason}, "
            f"frame_id={snapshot.frame_id}, "
            f"stamp={snapshot.stamp_ns // 1_000_000_000}.{snapshot.stamp_ns % 1_000_000_000:09d}, "
            f"class={snapshot.class_name}, confidence={snapshot.confidence:.3f}, "
            f"jaw_width={snapshot.jaw_width_m:.4f}"
        )
        self.get_logger().info(f"{self._diagnostic_prefix('pregrasp_pose')} {self._format_pose(plan.pregrasp_pose)}")
        self.get_logger().info(f"{self._diagnostic_prefix('grasp_pose')} {self._format_pose(plan.grasp_pose)}")


    def _log_failure_snapshot(self, failed_stage: str, message: str) -> None:
        """失败时的汇总日志：失败阶段与原因 + 计划快照 + 夹爪证据。

        夹爪证据用于区分"没夹到"与"夹到但被判定不合规"：`last_gripper_reached_position`
        是最近一次张爪的实际到位开口（m），从未张过爪时打印 unknown；
        `closure_distance` 是本次闭合行程（m）。两者配合接触标志判断失败原因。
        """

        self.get_logger().error(f"{self._diagnostic_prefix(failed_stage)} fail: {message}")
        if _state_for(self).current_attempt_plan is not None:
            self._log_plan_snapshot(_state_for(self).current_attempt_plan)
        reached = "unknown" if _state_for(self).last_gripper_reached_position is None else f"{_state_for(self).last_gripper_reached_position:.4f}"
        self.get_logger().error(
            f"{self._diagnostic_prefix('failure_summary')} "
            f"failed_stage={failed_stage}, message={message}, "
            f"last_gripper_reached_position={reached}, "
            f"contact={_state_for(self).last_grasp_contact_detected}, "
            f"closure_distance={_state_for(self).last_grasp_closure_distance_m:.4f}"
        )


    def _format_pose(self, pose: Pose) -> str:
        """把位姿格式化成定长小数文本（位置 4 位小数，单位 m），便于日志逐行比对。"""

        return (
            "position=("
            f"{float(pose.position.x):.4f}, {float(pose.position.y):.4f}, {float(pose.position.z):.4f}"
            "), orientation=("
            f"{float(pose.orientation.x):.4f}, {float(pose.orientation.y):.4f}, "
            f"{float(pose.orientation.z):.4f}, {float(pose.orientation.w):.4f}"
            ")"
        )


    def _wait_for_refreshed_plan(self, min_revision: int) -> GraspPlan | None:
        """轮询等待"比 min_revision 更新的计划"，返回深拷贝；未启用、超时或被中止时返回 None。

        等待条件是计划版本号增大（而不是时间），因此只要订阅回调接受了新计划就能立刻返回；
        轮询间隔 20 ms 是对实时性与占用之间的折中。整个等待循环受 `rclpy.ok()` 与
        `_running` 控制，停止服务可在等待期间立即打断本次执行。
        """

        if not bool(_parameter(self, "refresh_plan_at_pregrasp_enabled").value):
            return None
        timeout_sec = float(_parameter(self, "refresh_plan_timeout_sec").value)
        deadline = self._monotonic() + max(0.0, timeout_sec)
        while self._context_ok() and _state_for(self).is_running() and self._monotonic() < deadline:
            state = _state_for(self)
            with state.lock:
                revision = state.plan_revision
                plan = deepcopy(state.latest_plan)
            if revision > min_revision and plan is not None and self._plan_is_fresh(plan):
                self.get_logger().info(
                    f"using refreshed grasp plan after pregrasp: revision={revision}, source={plan.source}"
                )
                return plan
            self._sleep(0.02)
        return None


    def _replace_remaining_after_pregrasp(
        self,
        current_stages: list[VisualGraspStage],
        refreshed_stages: list[VisualGraspStage],
        completed_pregrasp_index: int,
    ) -> list[VisualGraspStage]:
        """用新计划生成的阶段替换"接近点之后"的剩余阶段。

        保留已经执行完的部分（含接近点阶段本身），只替换其后内容；若新阶段列表里找不到
        接近点阶段（即没有可替换的剩余部分），则原样返回，避免把整个序列清空。
        """

        refreshed_remaining = self._remaining_stages_after_pregrasp(refreshed_stages)
        if not refreshed_remaining:
            return current_stages
        return current_stages[: completed_pregrasp_index + 1] + refreshed_remaining


    def _remaining_stages_after_pregrasp(self, stages: list[VisualGraspStage]) -> list[VisualGraspStage]:
        """截取接近点阶段之后的阶段；列表中若没有接近点阶段，则整体返回（视为全部都是剩余部分）。"""

        for index, stage in enumerate(stages):
            if stage.name == "move_to_pregrasp":
                return stages[index + 1 :]
        return stages


    def _remove_approach_after_pregrasp(
        self,
        stages: list[VisualGraspStage],
        completed_pregrasp_index: int,
    ) -> list[VisualGraspStage]:
        """删掉接近点之后紧跟的 approach_grasp 阶段。

        仅在视觉伺服已经用小步逼近代替了这一步时调用；只删紧跟的那一个，其余阶段顺序不变。
        """

        remaining = stages[completed_pregrasp_index + 1 :]
        if remaining and remaining[0].name == "approach_grasp":
            remaining = remaining[1:]
        return stages[: completed_pregrasp_index + 1] + remaining


    def _approach_visual_servo_enabled(self) -> bool:
        """是否启用接近段视觉伺服（用迭代小步替代一次到位的接近）。"""

        return bool(_parameter(self, "approach_visual_servo_enabled").value)


    def _run_visual_servo_approach(self, current: PoseTarget) -> tuple[bool, str]:
        """迭代逼近抓取点，返回 (是否收敛, 说明)。

        每次迭代先用最新计划算出期望抓取点，再由伺服策略给出"朝目标走一步"的位姿：
        误差进入容差即判收敛；否则按单步上限截断后执行一次移动。要求"每步都用新计划"
        时，等不到新计划立即失败，避免拿旧观测反复逼近同一点。

        失败语义：单步移动失败直接透传原因；迭代次数用尽仍未收敛返回 not converged，
        并在消息里带上最后一次误差。抛出的异常由上层执行回调统一处理。
        """

        max_iterations = max(1, int(_parameter(self, "approach_visual_servo_max_iterations").value))
        config = VisualServoApproachConfig(
            max_step_m=float(_parameter(self, "approach_visual_servo_max_step_m").value),
            position_tolerance_m=float(_parameter(self, "approach_visual_servo_position_tolerance_m").value),
        )
        last_error = 0.0
        require_fresh_plan = bool(_parameter(self, "approach_visual_servo_require_fresh_plan").value)
        for iteration in range(max_iterations):
            state = _state_for(self)
            with state.lock:
                plan_revision = state.plan_revision
            refreshed_plan = self._wait_for_refreshed_plan(plan_revision)
            if require_fresh_plan and refreshed_plan is None:
                return False, "fresh visual servo plan unavailable"
            with state.lock:
                plan = refreshed_plan if refreshed_plan is not None else deepcopy(state.latest_plan)
            if plan is None:
                return False, "no refreshed grasp plan available"
            _, desired_grasp = self._build_motion_targets(plan)
            step = build_visual_servo_step(current, desired_grasp, config)
            last_error = step.error_m
            if step.reached:
                return True, f"target reached: error={step.error_m:.4f}"
            ok, message = self._run_stage(
                VisualGraspStage(name="visual_servo_approach", kind="move", pose=step.target)
            )
            if not ok:
                return False, message
            current = step.target
            self.get_logger().info(
                "visual servo approach step "
                f"{iteration + 1}/{max_iterations}: error={step.error_m:.4f}"
            )
        return False, f"not converged after {max_iterations} steps: error={last_error:.4f}"


    def _verify_after_close(self) -> tuple[bool, str]:
        """闭合后判定本次抓取是否成功，返回 (是否通过, 原因)。

        只在"真正执行 + 夹爪命令未禁用"时才有意义：plan_only 或夹爪被禁用时直接放行并注明跳过，
        因为此时根本没有真实夹持行为可供判定。

        证据来源是夹爪是否检出接触（力闭合服务由堵转推断）以及闭合行程是否够大。
        """

        if not self._execution_enabled():
            return True, "plan_only: grasp verification skipped"
        if not self._gripper_execution_enabled():
            return True, "gripper disabled: grasp verification skipped"
        result = verify_grasp_after_close(
            GraspVerificationInput(
                gripper_contact_detected=bool(_state_for(self).last_grasp_contact_detected),
                closure_distance_m=float(_state_for(self).last_grasp_closure_distance_m),
            ),
            GraspVerificationConfig(
                enabled=bool(_parameter(self, "grasp_verification_enabled").value),
                min_closure_distance_m=float(_parameter(self, "grasp_verification_min_closure_distance_m").value),
                require_gripper_contact=bool(_parameter(self, "grasp_verification_require_contact").value),
            ),
        )
        return bool(result.success), str(result.reason)


    def _append_place_stages(self, stages: list[VisualGraspStage]) -> list[VisualGraspStage]:
        """在抓取序列末尾追加放置阶段（移动到位 → 松爪 → 抬升撤开）；未启用时追加空列表。"""

        return stages + build_place_stages(
            PlaceTaskConfig(
                enabled=bool(_parameter(self, "place_after_grasp_enabled").value),
                place_position_xyz=self._tuple3("place_position_xyz"),
                place_orientation_xyzw=self._tuple_n("place_orientation_xyzw", 4),
                open_position_m=float(_parameter(self, "place_open_position_m").value),
                open_max_effort=float(_parameter(self, "place_open_max_effort").value),
                retreat_z_m=float(_parameter(self, "place_retreat_z_m").value),
            )
        )


    def _request_retry_retreat(self) -> tuple[bool, str]:
        """重试前撤回：把末端退回本轮的接近点，避免贴着目标换位形。

        撤回点直接复用已执行过的 `move_to_pregrasp` 阶段位姿；若该阶段还没跑过（例如首个
        阶段就失败），只告警不动作。撤回本身失败也只记录告警：此时已经决定重试或放弃，
        不应该因为撤回失败再改变上层决策。
        """

        if _state_for(self).retry_retreat_stage is None:
            self.get_logger().warn("safe retreat before retry requested, but no pregrasp retreat stage is available")
            return False, "no pregrasp retreat stage is available"
        retreat = VisualGraspStage(
            name="retry_safe_retreat",
            kind="move",
            pose=_state_for(self).retry_retreat_stage.pose,
        )
        ok, message = self._run_stage(retreat)
        if not ok:
            self.get_logger().warn(f"retry safe retreat failed: {message}")
            return False, message
        return True, message


    def _build_motion_targets(self, plan: GraspPlan) -> tuple[PoseTarget, PoseTarget]:
        """按计划与策略算出 (接近点, 抓取点) 两个目标。

        三种分支：
          - 计划来源为 `candidate_ik_filter`：上游已做过 IK 与碰撞过滤，位姿就是最终答案，
            只做坐标系换算，不再叠加任何策略（见 `_build_motion_targets_from_filtered_plan`）。
          - `pose_policy` 为 visual_pose / source_pose / legacy：直接使用计划自带位姿，
            经坐标系换算、TCP 偏移扣减后再叠加各自的 z 偏移。
          - `pose_policy` 为 base_axis（默认）：只用计划里的抓取位置，姿态取固定值，
            接近点由基座接近轴与距离现算，因此不依赖上游给出的接近点质量。
        其它取值一律抛 ValueError，交由上层按执行失败处理。
        """

        if str(getattr(plan, "source", "")).strip() == "candidate_ik_filter":
            return self._build_motion_targets_from_filtered_plan(plan)
        policy = str(_parameter(self, "pose_policy").value).strip().lower()
        if policy in ("visual_pose", "source_pose", "legacy"):
            return (
                self._convert_plan_pose(plan, plan.pregrasp_pose, 0.0),
                self._convert_plan_pose(plan, plan.grasp_pose, self._grasp_base_z_offset_m),
            )
        if policy != "base_axis":
            raise ValueError(f"unsupported pose_policy: {policy}")
        grasp_pose = self._transform_plan_pose_to_target_frame(plan, plan.grasp_pose)
        return build_base_axis_grasp_targets(
            grasp_position_xyz=(
                float(grasp_pose.position.x),
                float(grasp_pose.position.y),
                float(grasp_pose.position.z),
            ),
            config=BaseAxisGraspPolicyConfig(
                fixed_orientation_xyzw=self._tuple_n("fixed_grasp_orientation_xyzw", 4),
                approach_axis_xyz=self._tuple3("base_approach_axis_xyz"),
                pregrasp_distance_m=float(_parameter(self, "base_pregrasp_distance_m").value),
                tcp_offset_xyz=self._tcp_offset_xyz,
                target_base_offset_xyz=self._target_base_offset_xyz,
                grasp_z_offset_m=self._grasp_base_z_offset_m,
            ),
        )


    def _build_motion_targets_from_filtered_plan(self, plan: GraspPlan) -> tuple[PoseTarget, PoseTarget]:
        """已过滤计划专用：直接采用计划里的接近点与抓取点位姿，只做坐标系换算。

        该分支不套用固定姿态、不叠加 TCP/基座偏移——上游过滤节点已经把这些处理完并做过
        可达性校验，再处理一次会让执行位姿偏离被校验过的那一个。
        """

        return (
            pose_to_target(self._transform_plan_pose_to_target_frame(plan, plan.pregrasp_pose)),
            pose_to_target(self._transform_plan_pose_to_target_frame(plan, plan.grasp_pose)),
        )


    def _transform_plan_pose_to_target_frame(self, plan: GraspPlan, pose: Pose) -> Pose:
        """把计划中的位姿换算到 `target_frame`。

        计划未带坐标系（空 frame_id）或本来就与目标坐标系相同时原样返回（仍做深拷贝，
        避免修改调用方持有的消息）；否则查询最新可用变换，查询超时 0.2 s，超时抛出
        异常（TF 查不到变换属于不可恢复状态，不能拿未换算的位姿去执行）。
        """

        converted = deepcopy(pose)
        source_frame = str(plan.header.frame_id)
        if self._target_frame and source_frame and source_frame != self._target_frame:
            tf_msg = self._lookup_plan_transform(plan)
            converted = transform_pose_message(converted, transform_from_msg(tf_msg))
        return converted


    def _convert_plan_pose(self, plan: GraspPlan, pose: Pose, base_z_offset_m: float) -> PoseTarget:
        """旧式策略的位姿处理：坐标系换算 → 扣减 TCP 偏移 → 叠加基座偏移与 z 偏移。

        位置偏移结果保留 6 位小数，避免浮点累加产生 1e-17 级别的噪声位姿；
        z 方向同时叠加 `target_base_offset_xyz[2]` 与传入的 `base_z_offset_m`
        （接近点与抓取点使用不同的 z 偏移参数）。
        """

        converted = deepcopy(pose)
        source_frame = str(plan.header.frame_id)
        if self._target_frame and source_frame and source_frame != self._target_frame:
            tf_msg = self._lookup_plan_transform(plan)
            converted = transform_pose_message(converted, transform_from_msg(tf_msg))
        converted = apply_tcp_offset_to_pose(converted, self._tcp_offset_xyz)
        converted.position.x = round(float(converted.position.x) + self._target_base_offset_xyz[0], 6)
        converted.position.y = round(float(converted.position.y) + self._target_base_offset_xyz[1], 6)
        converted.position.z = round(float(converted.position.z) + self._target_base_offset_xyz[2] + base_z_offset_m, 6)
        return pose_to_target(converted)

    def _lookup_plan_transform(self, plan: GraspPlan):
        """Look up and reuse TF at the plan's acquisition timestamp."""
        stamp = plan.header.stamp
        stamp_ns = int(getattr(stamp, "sec", 0)) * 1_000_000_000 + int(
            getattr(stamp, "nanosec", 0)
        )
        if stamp_ns <= 0:
            raise ValueError("grasp plan has no acquisition timestamp for TF lookup")
        key = (self._target_frame, str(plan.header.frame_id), stamp_ns)
        if key not in self._tf_cache:
            self._tf_cache[key] = self._tf_buffer.lookup_transform(
                self._target_frame,
                str(plan.header.frame_id),
                rclpy.time.Time.from_msg(stamp),
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
        return self._tf_cache[key]


    def _run_stage(self, stage: VisualGraspStage) -> tuple[bool, str]:
        """执行单个阶段并返回 (是否成功, 说明)。

        按 `kind` 分派：
          - move：调用运动执行服务；plan_only 模式下只规划干跑。
          - gripper：未开启执行时跳过；开启时闭合阶段优先走力闭合抓取服务，其余走位置服务。
          - safe_home：回安全位（未开启执行时直接跳过）。
        未知 `kind` 视为失败，避免静默漏掉某个阶段。

        真正执行时按 `_stage_waits` 做机械稳定等待；plan_only 不发生机械动作，因此不使用
        这些等待，只保留可选的 `plan_only_stage_pause_sec` 诊断节拍（默认 0）。等待结束后
        再检查一次 `_running`：若期间收到停止请求，本阶段返回失败 `stopped`。
        """

        self._log_diagnostic(stage.name, "start")
        if stage.kind == "move":
            if stage.pose is None:
                return False, "missing move pose"
            ok, message = self._call_execute_pose(stage)
        elif stage.kind == "gripper":
            if not self._execution_enabled():
                ok, message = True, "plan_only: gripper command skipped"
            elif self._gripper_execution_enabled():
                if stage.name == "close_gripper" and bool(_parameter(self, "gripper_grasp_enabled").value):
                    ok, message = self._call_grasp_gripper(stage)
                else:
                    ok, message = self._call_gripper(stage)
            else:
                ok, message = True, "simulation: gripper command skipped"
        elif stage.kind == "safe_home":
            ok, message = self._call_safe_home()
        else:
            return False, f"unsupported stage kind: {stage.kind}"
        if not ok:
            self._log_diagnostic(stage.name, "fail", message)
            return False, message
        if self._execution_enabled():
            wait_sec = self._stage_waits.get(stage.name, 0.0)
        else:
            wait_sec = float(_parameter(self, "plan_only_stage_pause_sec").value)
        deadline = self._monotonic() + max(0.0, wait_sec)
        while self._context_ok() and _state_for(self).is_running() and self._monotonic() < deadline:
            self._sleep(min(0.02, max(0.0, deadline - self._monotonic())))
        if not _state_for(self).is_running():
            return False, "stopped"
        self._log_diagnostic(stage.name, "ok", message)
        return True, message


    def _call_execute_pose(self, stage: VisualGraspStage) -> tuple[bool, str]:
        """把运动阶段交给运动执行服务。

        真正执行且开启了 `trajectory_precheck_enabled` 时，先用 `execute=False` 干跑一次做
        预检：规划不通过就不下发执行请求，避免"先动了才发现后半段规划失败"。
        预检通过（或未启用）后，再按执行模式发出正式请求。
        """

        if stage.pose is None:
            return False, "missing move pose"
        if self._execution_enabled() and bool(_parameter(self, "trajectory_precheck_enabled").value):
            ok, message = self._precheck_execute_pose(stage)
            if not ok:
                return False, f"trajectory precheck failed: {message}"
        return self._send_execute_pose(stage, execute=self._execution_enabled())


    def _precheck_execute_pose(self, stage: VisualGraspStage) -> tuple[bool, str]:
        """执行前的规划干跑：只规划、不执行。"""

        return self._send_execute_pose(stage, execute=False)


    def _send_execute_pose(self, *args, **kwargs):
        return self._services._send_execute_pose(*args, **kwargs)


    def _publish_preview_sequence(self, *args, **kwargs):
        return self._services._publish_preview_sequence(*args, **kwargs)


    def _execution_enabled(self, *args, **kwargs):
        return self._services._execution_enabled(*args, **kwargs)


    def _gripper_execution_enabled(self, *args, **kwargs):
        return self._services._gripper_execution_enabled(*args, **kwargs)


    def _detected_jaw_width(self, plan: GraspPlan) -> float:
        """取检测到的夹爪开口宽度（m）：计划字段优先，缺失或非正时回落到候选字段，仍无则为 0。"""

        return detected_jaw_width(plan)


    def _detected_object_length(self, plan: GraspPlan) -> float:
        """取候选给出的目标长度估计（m）；缺失时为 0。"""

        return detected_object_length(plan)


    def _velocity_scaling_for_stage(self, *args, **kwargs):
        return self._services._velocity_scaling_for_stage(*args, **kwargs)


    def _call_gripper(self, *args, **kwargs):
        return self._services._call_gripper(*args, **kwargs)


    def _call_safe_home(self, *args, **kwargs):
        return self._services._call_safe_home(*args, **kwargs)


    def _call_grasp_gripper(self, *args, **kwargs):
        return self._services._call_grasp_gripper(*args, **kwargs)


    def _wait_for_future(self, future, timeout_sec: float) -> bool:
        """轮询等待异步调用完成，返回是否已完成。

        返回 False 有两种含义：超时，或等待期间被停止（`_running` 变假）；调用方把两者
        统一按失败处理，调用结果由调用方自行读取。轮询间隔 20 ms，避免忙等占满 CPU。
        """

        return _io_gateway_for(self).wait(future, timeout_sec)


    def _request_stop(self) -> None:
        self._io_gateway.stop("motion_stop")
        self._io_gateway.stop("trajectory_stop")

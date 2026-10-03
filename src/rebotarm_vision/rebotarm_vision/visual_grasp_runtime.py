"""视觉抓取执行流程的运行时编排。"""

from __future__ import annotations

from copy import deepcopy

from .utils.message_freshness import message_age_sec
from .policies.trajectory_recovery_policy import RecoveryConfig, recovery_decision_for_stage
from .visual_grasp_state import _state_for


class VisualGraspRuntime:
    """执行阶段与恢复流程的编排器；ROS 具体调用仍由 workflow gateway 方法提供。"""

    def __init__(self, workflow) -> None:
        self.workflow = workflow

    def execute(self, _request, response, *, admitted: bool = False):
        workflow = self.workflow
        """`/visual_grasp/execute` 服务回调：同步执行整条抓取序列并返回最终结论。

        请求内容被忽略（Trigger 无字段），执行对象是缓存的最近计划。返回的 `success`
        表示"整条序列走完且抓取验证通过"，`message` 为英文结果说明。

        进入前有两道门：已有任务在跑（`_running`）直接拒绝，避免两个抓取序列并发；
        没有新鲜有效的计划也拒绝，避免用过期的检测结果驱动机械臂。进入后按候选顺序
        逐个尝试，每次尝试都会重建完整阶段序列（含可选的放置阶段）。停止请求一旦
        发出，I/O 门控阻止继续运动；停止确认由外层节点在旧回调退出后完成。
        无论成功、失败还是抛异常，`finally` 都会清空本轮计划快照并复位运行标志；
        异常路径额外请求运动停止；只有后续确认成功才允许新任务。
        """

        state = _state_for(workflow)
        with state.lock:
            plan = deepcopy(state.latest_plan)
        if plan is None:
            response.success = False
            response.message = state.last_plan_rejection or "no grasp plan received yet"
            return response
        if not workflow._plan_is_fresh(plan):
            age = message_age_sec(
                plan.header.stamp, now_ns=int(workflow.get_clock().now().nanoseconds)
            )
            response.success = False
            response.message = f"cached grasp plan expired: age_sec={age}, max_plan_age_sec={workflow._max_plan_age_sec}"
            return response
        if not admitted and not state.begin_run():
            response.success = False
            response.message = "visual grasp already running"
            return response
        try:
            config = getattr(workflow, "_config", None)
            input_topic = config.input_topic if config is not None else workflow._input_topic
            workflow._log_diagnostic("detect", "ok", f"input_topic={input_topic}, plan_revision={_state_for(workflow).plan_revision}")
            attempts = workflow._candidate_plans_for_attempts()
            if not attempts:
                response.success = False
                response.message = "no candidate attempts available"
                workflow._log_diagnostic("filter", "fail", response.message)
                return response
            workflow._log_diagnostic("filter", "ok", f"attempts={len(attempts)}")
            for attempt_index, (candidate_index, plan) in enumerate(attempts):
                with state.lock:
                    _state_for(workflow).preview_start_joint_state = None
                    _state_for(workflow).preview_trajectories = []
                    _state_for(workflow).preview_gripper_events = []
                    _state_for(workflow).current_attempt_index = attempt_index + 1
                    _state_for(workflow).current_candidate_index = int(candidate_index)
                    _state_for(workflow).current_attempt_plan = deepcopy(plan)
                workflow.get_logger().info(
                    f"{workflow._diagnostic_prefix('attempt')} start: "
                    f"attempt={attempt_index + 1}/{len(attempts)}, candidate={candidate_index}"
                )
                workflow._log_plan_snapshot(plan)
                # 每轮尝试都把上一轮的抓取证据清零，防止上一轮的结果误判本轮
                _state_for(workflow).last_grasp_contact_detected = False
                _state_for(workflow).last_grasp_closure_distance_m = 0.0
                _state_for(workflow).retry_retreat_stage = None
                stages = workflow._append_place_stages(workflow._build_sequence_from_plan(plan))
                ok, message, failed_stage = workflow._execute_stages(stages)
                if ok:
                    if not workflow._execution_enabled():
                        ok, message = workflow._publish_preview_sequence()
                        if not ok:
                            response.success = False
                            response.message = f"preview publication failed: {message}"
                            workflow._log_diagnostic("preview", "fail", message)
                            return response
                    response.success = True
                    response.message = "visual grasp sequence finished"
                    workflow._log_diagnostic("result", "success", response.message)
                    return response
                # 走到这里说明某个阶段失败：先按恢复策略收拾现场，再决定是否换候选
                if not state.is_running():
                    # 本地超时或停止不会终止远端服务，运动服务端可能仍在执行。
                    # 返回前必须通过独立的停止通道发送停止请求。
                    workflow._request_stop()
                    response.success = False
                    response.message = state.abort_reason or "stopped"
                    return response
                remaining_attempts = len(attempts) - attempt_index - 1
                decision = recovery_decision_for_stage(
                    failed_stage,
                    attempt_index=attempt_index,
                    remaining_attempts=remaining_attempts,
                    config=RecoveryConfig(
                        auto_retry_enabled=(
                            workflow._config.auto_retry_enabled
                            if getattr(workflow, "_config", None) is not None
                            else bool(workflow.get_parameter("auto_retry_enabled").value)
                        ),
                        safe_retreat_before_retry=(
                            workflow._config.safe_retreat_before_retry
                            if getattr(workflow, "_config", None) is not None
                            else bool(workflow.get_parameter("safe_retreat_before_retry").value)
                        ),
                    ),
                )
                if decision.request_stop:
                    workflow._request_stop()
                if decision.request_safe_retreat:
                    retreat_ok, retreat_message = workflow._request_retry_retreat()
                    if not retreat_ok:
                        workflow._request_stop()
                        response.success = False
                        response.message = f"retry safe retreat failed: {retreat_message}"
                        workflow._log_failure_snapshot("retry_safe_retreat", retreat_message)
                        return response
                if decision.retry:
                    workflow.get_logger().warn(f"{decision.reason}: {message}")
                    continue
                response.success = False
                response.message = f"{failed_stage} failed: {message}"
                workflow._log_failure_snapshot(failed_stage, message)
                return response
            # 循环体每条路径都会 return（成功、失败或换候选继续），能走到这里说明没有可执行的尝试
            response.success = True
            response.message = "visual grasp sequence finished"
            workflow._log_diagnostic("result", "success", response.message)
            return response
        except Exception as exc:
            # 任何未预料的异常都必须先停运动再返回，避免机械臂停在未知状态
            workflow._request_stop()
            response.success = False
            response.message = f"visual grasp failed: {exc}"
            workflow._log_failure_snapshot("executor", str(exc))
            return response
        finally:
            with state.lock:
                _state_for(workflow).current_attempt_plan = None
                _state_for(workflow).preview_start_joint_state = None
                _state_for(workflow).preview_trajectories = []
                if not admitted:
                    state.finish_run(preserve_abort=state.phase in ("STOP_REQUESTED", "ABORTING"))

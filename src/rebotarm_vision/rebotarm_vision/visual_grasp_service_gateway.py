"""Request construction and response adaptation for visual grasp ROS services."""
from __future__ import annotations

import math
from copy import deepcopy
from types import SimpleNamespace

from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from rebotarm_msgs.srv import ExecutePose, GraspGripper, PublishTrajectoryPreview, SetGripper

from .gripper_quality import close_contact_success
from .visual_grasp_io import io_gateway_for as _io_gateway_for
from .utils.visual_grasp_messages import target_to_pose_stamped
from .policies.visual_grasp_runtime_policy import velocity_scaling_for_stage
from .visual_grasp_sequence import VisualGraspStage
from .visual_grasp_state import _state_for


def _parameter(context, name):
    config = getattr(context, "_config", None)
    if config is not None:
        return SimpleNamespace(value=config.get(name))
    return context.get_parameter(name)




class VisualGraspServiceGateway:
    def __init__(self, *, state, config, io_gateway):
        self._state = state
        self._config = config
        self._io_gateway = io_gateway
        self._target_frame = str(config.get("target_frame", "base_link"))
        self._service_timeout_sec = float(config.get("service_timeout_sec", 20.0))
        self._motion_result_timeout_sec = float(config.get("motion_result_timeout_sec", 45.0))
        self._execution_mode = str(config.get("execution_mode", "plan_only")).strip().lower()

    def _send_execute_pose(self, stage: VisualGraspStage, *, execute: bool) -> tuple[bool, str]:
        """组装并发送一次运动执行请求，返回 (是否成功, 阶段说明)。

        `execute=False` 即干跑。速度缩放按阶段选择，加速度缩放全阶段共用；超时字段同时作为
        服务端执行超时下发（`motion_result_timeout_sec`），客户端等待时间取
        "服务可用超时 + 执行超时"之和，保证等待窗口不小于服务端可能花费的时间。
        返回文本拼上服务端返回的阶段名，便于从日志判断失败发生在规划还是执行。
        """

        if stage.pose is None:
            return False, "missing move pose"
        request = ExecutePose.Request()
        request.target_pose = target_to_pose_stamped(stage.pose, self._target_frame)
        request.velocity_scaling = self._velocity_scaling_for_stage(stage.name)
        request.acceleration_scaling = float(_parameter(self, "acceleration_scaling").value)
        request.timeout_sec = self._motion_result_timeout_sec
        request.execute = bool(execute)
        request.suppress_preview = bool(not execute and not self._execution_enabled())
        state = _state_for(self)
        with state.lock:
            if not execute and not self._execution_enabled() and state.preview_start_joint_state is not None:
                request.preview_start_joint_state = deepcopy(state.preview_start_joint_state)
        result, error = _io_gateway_for(self).call(
            "execute_pose", request, self._service_timeout_sec + self._motion_result_timeout_sec
        )
        if result is None:
            return False, error
        if result.success and not execute and not self._execution_enabled():
            trajectory = result.planned_trajectory
            names = list(trajectory.joint_names)
            points = list(trajectory.points)
            if (
                len(names) != 6
                or set(names) != {f"joint{i}" for i in range(1, 7)}
                or not points
                or len(points[-1].positions) != 6
                or not all(math.isfinite(value) for value in points[-1].positions)
            ):
                return False, "plan-only response has no valid arm trajectory"
            next_start = JointState()
            next_start.name = names
            next_start.position = list(points[-1].positions)
            with state.lock:
                state.preview_start_joint_state = next_start
                state.preview_trajectories.append(deepcopy(trajectory))
        return bool(result.success), f"{result.stage}: {result.message}"


    def _publish_preview_sequence(self) -> tuple[bool, str]:
        """全部阶段规划成功后，请运动层一次发布完整 RViz 预览序列。"""

        state = _state_for(self)
        with state.lock:
            trajectories = deepcopy(state.preview_trajectories)
        if not trajectories:
            return False, "no planned trajectories collected"
        request = PublishTrajectoryPreview.Request()
        request.trajectories = trajectories
        result, error = _io_gateway_for(self).call("publish_preview", request, self._service_timeout_sec)
        if result is None:
            return False, error
        return bool(result.success), str(result.message)


    def _execution_enabled(self) -> bool:
        """是否处于真正下发动作的模式：只有 execute / real 算数，其余（含 plan_only）都只做规划。"""

        return self._execution_mode in ("execute", "real")


    def _gripper_execution_enabled(self) -> bool:
        """夹爪命令是否真正下发：要求处于执行模式且 `execute_gripper` 未被关闭。"""

        return self._execution_enabled() and bool(_parameter(self, "execute_gripper").value)


    def _velocity_scaling_for_stage(self, name: str) -> float:
        """按阶段名选择速度缩放：接近最慢，撤退居中，其余用常规值。"""

        return velocity_scaling_for_stage(
            name,
            approach=float(_parameter(self, "approach_velocity_scaling").value),
            retreat=float(_parameter(self, "retreat_velocity_scaling").value),
            normal=float(_parameter(self, "move_velocity_scaling").value),
        )


    def _call_gripper(self, stage: VisualGraspStage) -> tuple[bool, str]:
        """调用夹爪位置服务开到指定开口，返回 (是否成功, 说明)。

        副作用（供后续抓取验证使用）：
          - 张开阶段成功时记录实际到位开口 `_last_gripper_reached_position`，它是计算
            闭合行程的基准；张开失败则不更新，避免用错误基准估算行程。
          - 闭合阶段失败但启用"接触判据"时，若"停在了目标外侧一定余量"且"相对上次张开
            位有明显行程"，则按"夹到了目标"处理并返回成功；这是无接触传感器下的替代判据，
            返回文本会标注 contact assumed。
        失败信息统一带上实际到位开口（m），便于区分"没动"与"动到一半"。
        """

        if stage.gripper_position_m is None or stage.gripper_max_effort is None:
            return False, "missing gripper target"
        request = SetGripper.Request()
        request.position = float(stage.gripper_position_m)
        request.max_effort = float(stage.gripper_max_effort)
        result, error = _io_gateway_for(self).call("gripper", request, self._service_timeout_sec)
        if result is None:
            return False, error
        reached_position = float(result.reached_position)
        command_success = bool(result.success)
        if stage.name == "open_gripper" and command_success:
            _state_for(self).last_gripper_reached_position = reached_position
        if stage.name == "open_gripper_at_place" and command_success:
            _state_for(self).last_gripper_reached_position = reached_position
        if stage.name == "close_gripper" and not command_success:
            contact_ok = bool(_parameter(self, "close_contact_success_enabled").value) and close_contact_success(
                command_success=command_success,
                target_position_m=float(stage.gripper_position_m),
                reached_position_m=reached_position,
                previous_open_position_m=_state_for(self).last_gripper_reached_position,
                contact_margin_m=float(_parameter(self, "close_contact_margin_m").value),
                min_closure_delta_m=float(_parameter(self, "close_contact_min_closure_delta_m").value),
            )
            if contact_ok:
                return True, (
                    f"contact assumed: reached_position={reached_position:.4f}, "
                    f"target={float(stage.gripper_position_m):.4f}"
                )
        return command_success, f"reached_position={reached_position:.4f}"


    def _call_safe_home(self) -> tuple[bool, str]:
        """请求运动层回到安全位；非执行模式下直接跳过（返回成功并注明 skipped）。

        这是序列末尾可选的大范围回零动作，等待窗口同样按"服务可用超时 + 执行超时"计算。
        """

        if not self._execution_enabled():
            return True, "plan_only: safe_home skipped"
        result, error = _io_gateway_for(self).call(
            "safe_home", Trigger.Request(), self._service_timeout_sec + self._motion_result_timeout_sec
        )
        if result is None:
            return False, error
        return bool(result.success), str(result.message)


    def _call_grasp_gripper(self, stage: VisualGraspStage) -> tuple[bool, str]:
        """调用力闭合抓取服务闭合夹爪并保持，返回 (是否成功, 说明)。

        请求字段映射：闭合力矩取 `gripper_grasp_close_force`（夹持力旋钮），保持力矩取阶段
        计算出的最大力矩（两者都截断为非负）；超时、最短闭合时间、停住速度阈值与最小闭合
        行程全部来自抓取参数。注意本夹爪没有力传感器，"接触"是闭合行程 + 速度堵转推断的
        结果，不是实测接触力。

        副作用（供闭合后抓取验证使用）：记录是否检出接触，并把闭合行程记为
        "上次张爪到位开口 - 本次到位开口"，结果截断到非负；若从未张过爪（基准为 None），
        基准按 0 处理，因此行程会偏大——正常序列里闭合前一定先张开过。
        """

        if stage.gripper_position_m is None or stage.gripper_max_effort is None:
            return False, "missing gripper grasp target"
        request = GraspGripper.Request()
        request.close_force = max(float(_parameter(self, "gripper_grasp_close_force").value), 0.0)
        request.hold_force = max(float(stage.gripper_max_effort), 0.0)
        request.close_timeout_sec = float(_parameter(self, "gripper_grasp_timeout_sec").value)
        request.min_close_time_sec = float(_parameter(self, "gripper_grasp_min_close_time_sec").value)
        request.velocity_threshold = float(_parameter(self, "gripper_grasp_velocity_threshold").value)
        request.min_closure_distance_m = float(_parameter(self, "gripper_grasp_min_closure_distance_m").value)
        result, error = _io_gateway_for(self).call(
            "gripper_grasp", request, self._service_timeout_sec + request.close_timeout_sec
        )
        if result is None:
            return False, error
        _state_for(self).last_grasp_contact_detected = bool(result.contact_detected)
        _state_for(self).last_grasp_closure_distance_m = max(
            0.0,
            float(_state_for(self).last_gripper_reached_position or 0.0) - float(result.reached_position),
        )
        if result.success:
            return True, (
                f"{result.message}: contact={result.contact_detected}, "
                f"contact_position={result.contact_position:.4f}, "
                f"hold_force={result.hold_force:.3f}"
            )
        return False, (
            f"{result.message}: contact={result.contact_detected}, "
            f"reached_position={result.reached_position:.4f}"
        )

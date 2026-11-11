"""Web 面板点对点关节运动与夹爪命令的适配层.

系统位置与职责:
- 本模块属于"操作者意图 → ROS 命令"的交互层,只做参数校验,轨迹组装与动作下发,
  不直接访问硬件,不做碰撞检查,也不实现示教回放算法.
- 输入是 Web 面板传来的 JSON 字典(payload),输出是可直接序列化回前端的 dict;
  其中 ``goal_future`` / ``cancel_future`` 由调用方注册回调来跟踪动作结果.
- 所有动作客户端,消息工厂与触发器服务客户端都由调用方注入,因此本模块不导入任何
  ROS 消息类型,可以在没有 ROS 运行时的环境下做单元测试.

对外交互(客户端由调用方注入):
- 关节动作:``/<ns>/execute_teleop_motion`` 提交六轴五次约束和可选独立夹爪目标;
- 夹爪动作 ``/<ns>/gripper/command``:接收目标开口与最大力矩;
- 触发器服务 ``/<ns>/trajectory_stop``:请求硬件控制器停止当前轨迹.

安全语义:
- 校验不通过时绝不发送动作目标,直接返回 ``accepted=False``;
- 动作服务器在 0.1 s 内不可用即判为失败,不排队等待;
- 正式入口注入执行能力核对回调;配置与后端不匹配时拒绝,不因失败回退到旧模式;
- 停止流程总是先请求控制器级 trajectory_stop,再取消动作目标,两条路径互不阻塞.

文件结构:响应编码 → WebTeleopClient 依赖注入 → 点对点轨迹组装 → 停止与夹爪适配.
"""

from __future__ import annotations

from typing import Any, Callable

from .service_call_helpers import call_trigger_service
from .web_execute import (
    WebExecuteDecision,
    WebGripperDecision,
    validate_web_gripper_request,
    validate_web_execute_request,
)


def decision_response(decision: WebExecuteDecision) -> dict:
    """把关节运动校验结果转成 Web 面板使用的 JSON 契约.

    字段与前端一一对应:
    - ``accepted`` / ``message``:是否放行及人类可读说明;
    - ``positions``:关节名 → 目标角度(rad)映射,供前端回显;
    - ``duration``:轨迹总时长(s);
    - ``max_delta`` / ``max_delta_limit``:本次请求的最大单关节增量及其上限(rad).
    ``zip`` 依赖 joint_names 与 positions 一一对应且顺序一致,这是 Decision 的构造约定.
    """
    return {
        "accepted": bool(decision.accepted),
        "message": decision.message,
        "positions": dict(zip(decision.joint_names, decision.positions)),
        "duration": decision.duration,
        "max_delta": decision.max_delta,
        "max_delta_limit": decision.max_delta_limit,
    }


def gripper_decision_response(decision: WebGripperDecision) -> dict:
    """把夹爪命令校验结果转成 Web 面板使用的 JSON 契约.

    ``position`` 是夹爪开口(m),``max_effort`` 是力矩上限(归一化值,量纲由夹爪固件定义).
    """
    return {
        "accepted": bool(decision.accepted),
        "message": decision.message,
        "position": decision.position,
        "max_effort": decision.max_effort,
    }


class WebTeleopClient:
    """把 Web 面板的点对点命令组装成 ROS 动作目标并下发.

    职责边界:只做"校验 → 组装轨迹/夹爪目标 → 发送",不保存执行状态;
    目标句柄,取消句柄与执行锁都由调用方(状态面板节点)维护.
    本类不自带锁,调用方需要自行串行化对 execute/stop/set_gripper 的调用.
    """

    def __init__(
        self,
        *,
        action_client: Any,
        joint_names: tuple[str, ...],
        joint_limits: dict[str, tuple[float, float]],
        joint_velocity_limits: dict[str, float],
        trajectory_factory: Callable[[], Any],
        trajectory_point_factory: Callable[[], Any],
        follow_goal_factory: Callable[[], Any],
        gripper_action_client: Any | None = None,
        gripper_goal_factory: Callable[[], Any] | None = None,
        require_trajectory_capability: Callable[[], None] = lambda: None,
        finalize_trajectory=None,
    ) -> None:
        """注入动作客户端与消息工厂(全部为关键字参数).

        - ``action_client``:跟随关节轨迹动作客户端;
        - ``joint_names``:关节顺序,必须与轨迹消息里 ``joint_names`` 的顺序一致,
          它同时决定位置向量的下标含义;
        - ``joint_limits``:关节名 → (下限, 上限),单位 rad,用于目标限位校验;
        - ``joint_velocity_limits``:关节名 → 速度上限,单位 rad/s,与全局上限取小;
        - ``trajectory_factory`` / ``trajectory_point_factory`` / ``follow_goal_factory``:
          无参工厂,用于延迟构造消息对象,使本模块在导入期不依赖 ROS 消息类型;
        - ``gripper_action_client`` / ``gripper_goal_factory``:夹爪动作客户端与目标工厂;
          为 None 时,真实硬件模式下的夹爪命令会返回 unavailable.
        """
        self._action_client = action_client
        self._joint_names = tuple(joint_names)
        self._joint_limits = dict(joint_limits)
        self._joint_velocity_limits = dict(joint_velocity_limits)
        self._trajectory_factory = trajectory_factory
        self._trajectory_point_factory = trajectory_point_factory
        self._follow_goal_factory = follow_goal_factory
        self._gripper_action_client = gripper_action_client
        self._gripper_goal_factory = gripper_goal_factory
        self._require_trajectory_capability = require_trajectory_capability
        self._finalize_trajectory = finalize_trajectory

    def execute(
        self,
        payload: dict,
        *,
        current_positions: dict[str, float],
        max_delta_rad: float,
        min_duration: float,
        max_duration: float,
        max_joint_speed_rad_s: float,
    ) -> dict:
        """校验并下发一次点对点关节运动.

        参数:
        - ``payload``:Web 请求体,须含 ``confirm="EXECUTE"`` 与 ``joint_positions``
          (关节名 → 目标角度 rad);可选 ``duration``(s),``max_delta_rad``(rad),
          ``max_joint_speed_rad_s``(rad/s);
        - ``current_positions``:当前实测关节角(rad),用于计算增量并作为插值起点;
        - ``max_delta_rad``:单关节最大允许增量(rad)的全局上限;
        - ``min_duration`` / ``max_duration``:轨迹时长(s)的允许区间,参与限速校验;
        - ``max_joint_speed_rad_s``:全局关节速度上限(rad/s).

        返回值 dict 的 ``status.state`` 取值:
        - ``rejected``:校验未通过(``goal_future`` 为 None);
        - ``unavailable``:动作服务器 0.1 s 内未就绪(``goal_future`` 为 None);
        - ``active``:目标已发出,``goal_future`` 交由调用方注册回调.

        校验通过后构造首末 q/v/a 约束,并绑定最终规划;连续采样由执行后端负责.
        """
        decision = validate_web_execute_request(
            payload,
            joint_names=self._joint_names,
            current_positions=current_positions,
            joint_limits=self._joint_limits,
            max_delta_rad=max_delta_rad,
            min_duration=min_duration,
            max_duration=max_duration,
            joint_velocity_limits=self._joint_velocity_limits,
            max_joint_speed_rad_s=max_joint_speed_rad_s,
        )
        if not decision.accepted:
            return {
                "accepted": False,
                "decision": decision,
                "response": decision_response(decision),
                "status": {"state": "rejected", "message": decision.message},
                "goal_future": None,
            }
        try:
            self._require_trajectory_capability()
        except ValueError as exc:
            return dict(accepted=False, decision=decision, goal_future=None,
                response=dict(accepted=False, message=str(exc)),
                status=dict(state='rejected', message=str(exc)))
        # 只等 0.1 s:Web 命令背后是人在等反馈,宁可快速失败也不要阻塞面板线程
        if not self._action_client.wait_for_server(timeout_sec=0.1):
            message = "execute_teleop_motion action unavailable"
            return {
                "accepted": False,
                "decision": decision,
                "response": {"accepted": False, "message": message},
                "status": {"state": "unavailable", "message": message},
                "goal_future": None,
            }

        # 起点必须与决策里的关节顺序对齐,否则位置会挂到错误的关节上
        current = tuple(current_positions[name] for name in decision.joint_names)
        # 两点包含 q/v/a,连续曲线由运动层准备;此处不离散化或修改请求时长.
        from rebotarm_motion.trajectory_messages import point_to_point_message
        trajectory = point_to_point_message(decision.joint_names, current,
            decision.positions, decision.duration, trajectory_factory=self._trajectory_factory,
            point_factory=self._trajectory_point_factory)
        if self._finalize_trajectory is None:
            raise ValueError('final planning binding required')
        trajectory = self._finalize_trajectory(trajectory)
        goal = self._follow_goal_factory()
        goal.trajectory = trajectory
        future = self._action_client.send_goal_async(goal)
        return {
            "accepted": True,
            "decision": decision,
            "response": decision_response(decision),
            "status": {
                "state": "active",
                "message": decision.message,
                "max_delta": decision.max_delta,
                "max_delta_limit": decision.max_delta_limit,
                "duration": decision.duration,
                "max_joint_speed_rad_s": float(max_joint_speed_rad_s),
                "points": len(trajectory.points),
            },
            "goal_future": future,
            "trajectory": trajectory,
        }

    def send_planned(self, trajectory, decision, *, gripper=(), feedback_callback=None):
        """提交已预览的准确联合消息;不再插值,改时长或发送独立夹爪 Action."""
        self._require_trajectory_capability()
        if not self._action_client.wait_for_server(timeout_sec=.1):
            raise ValueError('execute_teleop_motion action unavailable')
        goal = self._follow_goal_factory()
        goal.trajectory = trajectory
        goal.move_gripper = bool(gripper)
        if gripper:
            goal.gripper_position, goal.gripper_max_effort = gripper
        return dict(accepted=True,decision=decision,response=decision_response(decision),
            status=dict(state='submitted',message='联合轨迹已提交，等待控制器接纳',duration=decision.duration),
            trajectory=trajectory,goal_future=self._action_client.send_goal_async(goal,
                feedback_callback=feedback_callback))

    def stop(self, goal_handle: Any | None, *, trajectory_stop_client: Any) -> dict:
        """停止 Web 触发的运动:先请求控制器级停止,再取消动作目标.

        先有界等待 trajectory_stop 服务,再提交异步取消;取消响应由调用方继续跟踪.
        即使动作层取消抛异常,只要停止服务成功仍返回 ``accepted=True``.

        参数:
        - ``goal_handle``:当前活动的动作目标句柄,None 表示没有 Web 运动在跑;
        - ``trajectory_stop_client``:控制器触发器服务客户端.

        返回值约定:
        - ``accepted``:停止服务成功或取消请求已提交;目标终态由后续回调确认;
        - ``status.state``:``cancel_requested`` / ``idle`` / ``failed``;
        - ``cancel_future``:取消请求的 future,供调用方注册回调;
        - ``clear_goal_handle``:True 表示调用方应清空保存的目标句柄.
        """
        # 0.2 s 超时:停止属于紧急路径,服务不可用时应立刻返回而不是卡住调用线程
        stop_requested, _stop_message = call_trigger_service(
            trajectory_stop_client,
            timeout_sec=0.2,
        )
        if goal_handle is None:
            message = (
                "no active web execute goal; controller trajectory_stop requested"
                if stop_requested
                else "no active web execute goal; controller trajectory_stop unavailable"
            )
            state = "cancel_requested" if stop_requested else "idle"
            return {
                "accepted": bool(stop_requested),
                "state": state,
                "message": message,
                "trajectory_stop_requested": stop_requested,
                "status": {
                    "state": state,
                    "message": message,
                    "trajectory_stop_requested": stop_requested,
                },
                "cancel_future": None,
                "clear_goal_handle": False,
            }
        try:
            future = goal_handle.cancel_goal_async()
        except Exception as exc:
            # 保存取消异常,同时返回已确认的停止服务结果,供调用方继续核对目标状态.
            message = f"failed to request trajectory cancel: {exc}"
            if stop_requested:
                message = f"{message}; controller trajectory_stop requested"
                return {
                    "accepted": True,
                    "message": message,
                    "trajectory_stop_requested": True,
                    "status": {
                        "state": "cancel_requested",
                        "message": message,
                        "trajectory_stop_requested": True,
                    },
                    "cancel_future": None,
                    "clear_goal_handle": False,
                }
            return {
                "accepted": False,
                "message": message,
                "trajectory_stop_requested": False,
                "status": {"state": "failed", "message": message},
                "cancel_future": None,
                "clear_goal_handle": False,
            }
        message = (
            "trajectory cancel requested; controller trajectory_stop requested"
            if stop_requested
            else "trajectory cancel requested; controller trajectory_stop unavailable"
        )
        return {
            "accepted": True,
            "message": message,
            "trajectory_stop_requested": stop_requested,
            "status": {
                "state": "cancel_requested",
                "message": message,
                "trajectory_stop_requested": stop_requested,
            },
            "cancel_future": future,
            "clear_goal_handle": True,
        }

    def set_gripper(
        self,
        payload: dict,
        *,
        gripper_limits: tuple[float, float],
        default_max_effort: float,
        max_effort_limit: float,
    ) -> dict:
        """校验并下发夹爪命令(真实硬件或仿真).

        参数:
        - ``payload``:Web 请求体,须含 ``confirm="SET_GRIPPER"`` 与 ``position``(开口,m);
          可选 ``max_effort``(力矩上限,超过硬上限时会被夹紧);
        - ``gripper_limits``:夹爪开口允许区间 (下限, 上限),单位 m;
        - ``default_max_effort``:请求未给 ``max_effort`` 时使用的默认力矩;
        - ``max_effort_limit``:力矩硬上限,任何请求都不会越过它.

        返回值 dict 的 ``status.state`` 取值:
        - ``rejected``:校验失败;
        - ``unavailable``:夹爪动作不可用;
        - ``active``:目标已下发,``goal_future`` 交由调用方注册回调.
        """
        decision = validate_web_gripper_request(
            payload,
            gripper_limits=gripper_limits,
            default_max_effort=default_max_effort,
            max_effort_limit=max_effort_limit,
        )
        if not decision.accepted:
            return {
                "accepted": False,
                "decision": decision,
                "response": gripper_decision_response(decision),
                "status": {"state": "rejected", "message": decision.message},
                "goal_future": None,
            }
        if self._gripper_action_client is None or self._gripper_goal_factory is None:
            message = "gripper command action unavailable"
            return {
                "accepted": False,
                "decision": decision,
                "response": {"accepted": False, "message": message},
                "status": {"state": "unavailable", "message": message},
                "goal_future": None,
            }
        # 同 execute:只等 0.1 s,避免 Web 请求线程被动作发现过程阻塞
        if not self._gripper_action_client.wait_for_server(timeout_sec=0.1):
            message = "gripper command action unavailable"
            return {
                "accepted": False,
                "decision": decision,
                "response": {"accepted": False, "message": message},
                "status": {"state": "unavailable", "message": message},
                "goal_future": None,
            }
        goal = self._gripper_goal_factory()
        # 位置单位为 m;max_effort 已在校验环节夹紧到硬上限,这里只做数值类型归一
        goal.command.position = float(decision.position)
        goal.command.max_effort = float(decision.max_effort)
        future = self._gripper_action_client.send_goal_async(goal)
        return {
            "accepted": True,
            "decision": decision,
            "response": gripper_decision_response(decision),
            "status": {
                "state": "active",
                "message": decision.message,
                "position": decision.position,
                "max_effort": decision.max_effort,
            },
            "goal_future": future,
        }

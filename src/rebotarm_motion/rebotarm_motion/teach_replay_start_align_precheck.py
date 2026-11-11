"""示教回放的起始对齐预检:只汇总"能不能对齐",不产生任何运动.

回放前的诊断环节.它读取当前位姿与首个示教点的偏差,按顺序给出一个状态摘要,供状态
面板与控制流程决定是否需要规划起始对齐段:

- ``disabled``:配置未启用对齐;
- ``unknown``:偏差不可用(反馈尚未到位或不是有限数),或在仅诊断模式下缺少示教样本;
- ``skipped``:偏差小于阈值,已经在示教起点附近,不需要规划;
- ``unavailable``:规划服务不可用;
- ``ready``:服务可用且本次只做诊断(``plan=False``),尚未实际规划;
- ``planned`` / ``failed``:实际调用规划后的成功/失败结论,并附带规划点数.

本模块只调用"是否就绪"查询与规划请求,不发送执行指令;返回的字典会被直接发布到
状态面板,因此键名视为对外接口,保持英文.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class MoveItStartAlignPrecheckConfig:
    """起始对齐预检的配置.

    - ``enabled``:是否启用起始对齐(关闭时预检直接返回 disabled);
    - ``service``:规划服务名,仅用于摘要展示,便于在面板上确认连的是哪个服务;
    - ``skip_threshold``:跳过对齐的最大关节偏差(rad),偏差小于它即认为已在起点;
    - ``joint_goal_tolerance``:规划目标的关节容差(rad);
    - ``velocity_scaling``:规划速度缩放系数(0~1);
    - ``acceleration_scaling``:规划加速度缩放系数(0~1).
    """

    enabled: bool
    service: str
    skip_threshold: float
    joint_goal_tolerance: float
    velocity_scaling: float
    acceleration_scaling: float


class MoveItStartAlignPrechecker:
    """汇总起始对齐是否就绪.

    ``planner`` 需同时提供 ``plan_joint_positions(...)`` 与
    ``service_is_ready()``/``wait_for_service(timeout_sec=...)``;若规划端口与服务查询
    端口不是同一对象,可通过 ``service_client`` 单独注入(``None`` 时复用 ``planner``).
    本类不保存状态,可被反复调用.
    """

    def __init__(self, *, planner: Any, service_client: Any = None) -> None:
        self._planner = planner
        self._service_client = planner if service_client is None else service_client

    def summary(
        self,
        info_payload: dict,
        *,
        config: MoveItStartAlignPrecheckConfig,
        samples=None,
        plan: bool = False,
    ) -> dict:
        """生成对齐预检摘要字典.

        参数:
        - ``info_payload``:外部汇总信息,至少可含 ``max_error``-当前位姿与首个示教
          点的最大关节偏差(rad);缺失或非有限数时返回 ``unknown``;
        - ``config``:预检配置,见 :class:`MoveItStartAlignPrecheckConfig`;
        - ``samples``:示教样本序列,仅在实际规划时使用其**第一个**样本的关节名与位置
          作为规划目标(与回放起点一致);
        - ``plan``:为 ``True`` 时真正发起一次规划做可行性验证,为 ``False`` 时只做
          就绪性诊断,不占用规划资源.

        返回字典的 ``state`` 取值见模块说明;``max_error``/``skip_threshold``/``service``
        等字段会随状态一同返回,便于面板直接展示判定依据.规划失败不抛异常,而是以
        ``state="failed"`` 返回规划器给出的消息.
        """
        max_error = info_payload.get("max_error")
        threshold = float(config.skip_threshold)
        service = str(config.service)
        if not bool(config.enabled):
            return {"state": "disabled", "message": "MoveIt start alignment disabled"}
        if not self._is_number_like(max_error):
            return {"state": "unknown", "message": "current start error unavailable"}
        if float(max_error) < threshold:
            return {
                "state": "skipped",
                "message": "already near teach start; MoveIt alignment not required",
                "max_error": float(max_error),
                "skip_threshold": threshold,
            }
        if not self._service_available():
            return {
                "state": "unavailable",
                "message": "MoveIt planning service unavailable",
                "max_error": float(max_error),
                "skip_threshold": threshold,
                "service": service,
            }
        if not plan:
            return {
                "state": "ready",
                "message": "MoveIt planning service ready",
                "max_error": float(max_error),
                "skip_threshold": threshold,
                "service": service,
            }
        # 没有样本就无法确定对齐目标,只能报 unknown,绝不能凭空构造目标去规划.
        if not samples:
            return {
                "state": "unknown",
                "message": "no teach samples for MoveIt start alignment precheck",
                "max_error": float(max_error),
                "skip_threshold": threshold,
                "service": service,
            }
        first = samples[0]
        result = self._planner.plan_joint_positions(
            joint_names=tuple(first.joint_names),
            target_positions=tuple(first.positions),
            tolerance=float(config.joint_goal_tolerance),
            velocity_scaling=float(config.velocity_scaling),
            acceleration_scaling=float(config.acceleration_scaling),
        )
        # 规划点数为 0 或轨迹缺失都说明没有可用对齐段,但仍按规划器返回的成功标志
        # 区分 planned/failed,避免掩盖真实失败原因.
        points = len(getattr(result.trajectory, "points", [])) if result.trajectory is not None else 0
        return {
            "state": "planned" if result.success else "failed",
            "message": result.message,
            "max_error": float(max_error),
            "skip_threshold": threshold,
            "service": service,
            "points": points,
        }

    def _service_available(self) -> bool:
        """查询规划服务是否可用(先查就绪标志,再做 0 秒等待探测,不阻塞)."""
        try:
            available = bool(self._service_client.service_is_ready())
            if not available:
                # timeout_sec=0.0:只做一次即时探测,预检路径绝不允许阻塞主循环.
                available = bool(self._service_client.wait_for_service(timeout_sec=0.0))
            return available
        except Exception:
            # 服务客户端处于异常状态(未连接/已销毁)时按不可用处理,而不是抛出异常
            # 中断状态面板刷新.
            return False

    @staticmethod
    def _is_number_like(value) -> bool:
        """判断取值能否转换为有限浮点数(``None``,字符串,inf,nan 均视为不可用)."""
        try:
            number = float(value)
        except (TypeError, ValueError):
            return False
        return math.isfinite(number)

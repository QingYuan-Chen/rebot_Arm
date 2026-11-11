"""示教回放的运行期跟踪守卫.

职责与位置
    有状态的回放监测策略, 位于纯跟踪判定与上层取消/停止编排之间.

对外接口
    ReplayRuntimeMonitor.check 返回 should_stop 和状态字典; 每轮回放前 reset.

安全与并发约束
    宽限按同一单调时钟计算; 连续违规锁存后只请求一次停止, 执行停止由调用方确认.
    本类无锁, 反馈新鲜度与消息合法性须由调用链检查.

文件结构(按定义顺序)
    ReplayRuntimeMonitorConfig: 运行期守卫的判定阈值.
    ReplayRuntimeMonitorDecision: 一次运行期检查的结论.
    ReplayRuntimeMonitor: 面向活动示教回放的有状态运行期守卫.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from .trajectory_safety_monitor import ReplayTrackingResult, evaluate_replay_tracking


@dataclass(frozen=True)
class ReplayRuntimeMonitorConfig:
    """运行期守卫的判定阈值.

    - ``enabled``: 总开关, 关闭后 ``check`` 恒返回"不停";
    - ``start_grace_sec``: 轨迹开始后的启动宽限期(秒), 此窗口内完全不做判定;
    - ``violation_grace_sec``: 违规容忍时长(秒), 超差需连续持续这么久才停机;
    - ``max_tracking_error_rad``: 允许的最大跟踪误差(rad), 建议远小于相邻关节的
      机械间隙与碰撞距离余量;
    - ``max_live_velocity_rad_s``: 允许的最大实时关节速度(rad/s), 用于识别"反馈
      速度异常"这类误差尚未体现但已经失控的情形.
    """

    enabled: bool
    start_grace_sec: float
    violation_grace_sec: float
    max_tracking_error_rad: float
    max_live_velocity_rad_s: float


@dataclass(frozen=True)
class ReplayRuntimeMonitorDecision:
    """一次运行期检查的结论.

    ``should_stop`` 为 ``True`` 时 ``status`` 必带 ``state="safety_stop"`` 的状态字典,
    调用方应据此请求控制器停止并取消回放目标; 为 ``False`` 时 ``status`` 为空字典.
    """

    should_stop: bool
    status: dict = field(default_factory=dict)


class ReplayRuntimeMonitor:
    """面向活动示教回放的有状态运行期守卫.

    生命周期: 一次回放开始前调用 :meth:`reset` 清空历史, 回放期间周期性调用
    :meth:`check`; 一旦 :attr:`stop_requested` 变为 ``True``, 后续 ``check`` 直接短路,
    必须先 ``reset`` 才能用于下一次回放. 守卫只提出停止请求, 实际停止动作与目标取消
    由上层完成.
    """

    def __init__(self) -> None:
        """保存本轮监测配置并初始化违规计时与停止锁存; 不创建定时器或控制客户端."""
        # 首次发现违规的时刻(调用方时钟, 秒); 达标后清零, 用于实现连续违规计时.
        self.violation_since: float | None = None
        # 已请求过停止的锁存标志: 置位后不再重复判定, 避免重复下发停止请求.
        self.stop_requested = False

    def reset(self) -> None:
        """清空违规计时与停止锁存, 供下一次回放复用同一实例."""
        self.violation_since = None
        self.stop_requested = False

    def check(
        self,
        *,
        trajectory,
        started_at: float | None,
        joints: dict,
        now: float,
        config: ReplayRuntimeMonitorConfig,
    ) -> ReplayRuntimeMonitorDecision:
        """执行一次运行期跟踪检查.

        参数:
        - ``trajectory``: 活动回放轨迹, 需带 ``joint_names`` 与 ``points``(每点含
          ``positions`` 与 ``time_from_start``); ``None`` 表示当前没有回放;
        - ``started_at``: 轨迹下发的起始时刻(秒, 与 ``now`` 同一时钟); ``None`` 表示
          尚未开始计时;
        - ``joints``: 最新关节状态, 形如 ``{关节名: {"position": rad, "velocity": rad/s}}``,
          位置和速度均必填,缺字段按持续违规处理;
        - ``now``: 本次检查时刻(秒, 需单调时钟);
        - ``config``: 阈值配置, 见 :class:`ReplayRuntimeMonitorConfig`.

        返回"是否停止"及可发布的状态负载. 超差时先记录首次违规时刻并放行, 只有违规持
        续超过 ``violation_grace_sec`` 才置位 :attr:`stop_requested` 并返回停止请求; 一旦
        恢复达标立即清除计时, 该宽限用于减少短时超差触发的停止, 不保证消除所有误判.
        """
        if not bool(config.enabled) or trajectory is None or started_at is None or self.stop_requested:
            return ReplayRuntimeMonitorDecision(False)
        elapsed = float(now) - float(started_at)
        if elapsed < float(config.start_grace_sec):
            return ReplayRuntimeMonitorDecision(False)
        try:
            positions = tuple(item["position"] for item in joints.values())
            velocities = tuple(item["velocity"] for item in joints.values())
        except KeyError as exc:
            result = ReplayTrackingResult(False, "missing_joint_state", f"joint_states missing field: {exc}")
        else:
            result = evaluate_replay_tracking(
                trajectory,
                joint_names=tuple(joints.keys()),
                positions=positions,
                velocities=velocities,
                elapsed_sec=elapsed,
                max_tracking_error_rad=float(config.max_tracking_error_rad),
                max_live_velocity_rad_s=float(config.max_live_velocity_rad_s),
            )
        if result.ok:
            self.violation_since = None
            return ReplayRuntimeMonitorDecision(False)
        # 首次违规只记录起始时刻并放行, 等待违规是否连续, 避免单帧抖动触发停机.
        if self.violation_since is None:
            self.violation_since = float(now)
            return ReplayRuntimeMonitorDecision(False)
        if float(now) - float(self.violation_since) < float(config.violation_grace_sec):
            return ReplayRuntimeMonitorDecision(False)
        self.stop_requested = True
        return ReplayRuntimeMonitorDecision(
            True,
            {
                "state": "safety_stop",
                "message": f"runtime replay monitor stopped trajectory: {result.message}",
                "runtime_monitor": {
                    # 停机原因(tracking_error / live_velocity), 供面板与日志区分.
                    "reason": result.reason,
                    "worst_joint": result.worst_joint,
                    "max_tracking_error_rad": result.max_tracking_error_rad,
                    "max_live_velocity_rad_s": result.max_live_velocity_rad_s,
                    "tracking_error": result.reason == "tracking_error",
                    "live_velocity": result.reason == "live_velocity",
                },
                # 真实回放的运行期停机, 不是干跑演练, 上层据此保留安全停机状态.
                "dry_run": False,
            },
        )

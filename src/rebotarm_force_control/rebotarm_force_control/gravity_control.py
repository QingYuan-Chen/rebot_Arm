"""六轴重力补偿运行期状态与抗漂移积分步进.

职责与位置
    维护重力补偿期间的目标角,积分累计,静止锁定计数,拖动跟随及故障保持状态;
    独立于硬件总线,ROS 节点与 Pinocchio 模型对象.

对外接口
    GravityExecutionState: `step(q, dt, gravity, spatial_velocity, settings)` 返回前馈力矩.

安全与并发约束
    周期计算由唯一设备线程串行推进;上一周期总力矩在整批发送确认成功后由调用方更新.
    拖动或异常反馈校验由调用方在进入 step 前完成.

文件结构(按定义顺序)
    GravityExecutionState.
"""

from __future__ import annotations

import numpy as np

from .contracts import vector


class GravityExecutionState:
    """六轴重力补偿独立运行状态;周期计算不访问电机或 ROS.

    启停由 GravityController 协调,模式设备事务经生命周期与运行时执行.
    `step` 接收已验证反馈,重力力矩和末端空间速度,按实际时间积分并返回前馈;
    最后总力矩在发送成功后才更新.
    """

    def __init__(self) -> None:
        """初始化补偿目标,积分,跟随及故障保持状态,不执行模式切换或发送."""
        self.active = False
        self.target = None
        self.integral = None
        self.lock_counter = 0
        self.last_position = None
        self.last_total = np.zeros(6)
        self.fault = None
        self.hold_feedforward = None

    def step(self, q, dt: float, gravity, spatial_velocity, settings) -> np.ndarray:
        """更新抗漂移积分和拖动跟随目标,返回本周期前馈力矩."""
        q = vector(q, 6, "gravity feedback")
        gravity = vector(gravity, 6, "gravity torque")
        spatial_velocity = vector(spatial_velocity, 6, "tool velocity")
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError("gravity control dt must be finite and positive")
        if self.target is None:
            raise RuntimeError("gravity target unavailable")
        if self.integral is None:
            self.integral = np.zeros_like(q)
        self.integral += (self.target - q) * settings.integral_gain * dt
        cap = settings.integral_limit_nm
        np.clip(self.integral, -cap, cap, out=self.integral)
        if np.linalg.norm(spatial_velocity[:3]) > 0.04 or np.linalg.norm(spatial_velocity[3:]) > 0.08:
            self.target = q.copy()
            self.lock_counter = 0
            self.integral *= np.exp(-dt / 0.2)
        else:
            self.lock_counter += 1
        return gravity + self.integral

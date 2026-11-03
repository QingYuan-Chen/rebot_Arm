"""反馈, 夹爪接纳与执行查询的跨组件值契约.

职责与位置
    跨组件交接只读数值, 隔离执行器内部的数组, 锁和任务对象.

对外接口
    JointSnapshot, GripperTarget, GripperResult 分别用于反馈, 目标接纳和代次查询.

安全与并发约束
    反馈批次时间取六轴最旧观测时间; 目标接纳与释放完成分开表达.

文件结构(按定义顺序)
    JointSnapshot: 同一次缓存锁内复制的六轴反馈; 观测时间为本批最旧源时间.
    GripperTarget: 准入完成后的夹爪位置目标; 超时是相对接纳时间的秒数.
    GripperResult: 只属于指定代次的结果; 释放中不提前报告成功.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class PreparedArmHandoff:
    """调用线程已解码的冻结交接，设备线程只做常量规模身份复核。"""

    curve: object
    session: int
    generation: int
    hold: tuple
    gripper_positions: tuple
    gripper_generation: int
    scene_revision: str

    def __deepcopy__(self, memo):
        return self


@dataclass(frozen=True)
class ArmPreparation:
    """六轴所有者颁发的准备身份，跨线程复制仍保留同一令牌。"""

    session: int
    generation: int
    hold: tuple
    feedback: tuple
    observed_at: float
    gripper_positions: tuple
    gripper_generation: int
    gripper_raw_position: float
    scene_revision: str
    deadline: float

    def __deepcopy__(self, memo):
        return self


@dataclass(frozen=True)
class JointSnapshot:
    """同一次缓存锁内复制的六轴反馈; 观测时间为本批最旧源时间."""

    positions: tuple[float, ...]
    velocities: tuple[float, ...]
    torques: tuple[float, ...]
    statuses: tuple[int, ...]
    sequences: tuple[int, ...]
    observed_at: float


@dataclass(frozen=True)
class GripperTarget:
    """准入完成后的夹爪位置目标; 超时是相对接纳时间的秒数."""

    start_angle: float
    goal_angle: float
    effort: float
    timeout: float


@dataclass(frozen=True)
class GripperResult:
    """只属于指定代次的结果; 释放中不提前报告成功."""

    generation: int
    state: str
    message: str | None = None

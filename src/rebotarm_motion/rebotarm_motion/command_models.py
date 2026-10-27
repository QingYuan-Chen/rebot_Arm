"""运动命令与预览执行的纯数据模型。

本模块是「预览 → 执行」链路的公共词汇表，只定义枚举与不可变数据类，不含任何
运行时逻辑，也不依赖 ROS 或硬件：

- 预览侧：位姿目标、预览求解结果、预览命令，供预览管理器与位姿求解器实现共享；
- 执行侧：模式枚举、执行状态、执行请求与执行决策，供协调器判定「本次目标是否
  允许下发」；
- 求解器协议：把「按位姿反解关节角」抽象成接口，便于用真实求解器或测试替身注入。

约定:所有关节角单位为弧度(rad)，关节名称顺序由调用方给出的元组确定；位姿为
基坐标系下的位置(m)+ 固定轴 RPY 姿态(rad)。本模块只描述数据，不负责校验，
真正的关节限位与安全判定在预览管理器、协调器及上层运动守卫中完成。
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class ControlMode(str, Enum):
    """目标下发模式。

    继承 ``str`` 便于直接序列化到 JSON 或与字符串比较。
    - ``SIMULATION``：仅走仿真链路，不驱动物理机械臂；
    - ``REAL``：真实机械臂模式，执行前仍需上层安全门与使能确认。
    """

    SIMULATION = "simulation"
    REAL = "real"


class ExecutionState(str, Enum):
    """预览/执行状态机的状态。

    - ``IDLE``：无可用预览（或执行已结束/已停止）；
    - ``PREVIEW_READY``：存在一次成功预览，等待执行请求；
    - ``EXECUTING``：已受理执行，未收到结束或停止回调前不再受理新请求。
    """

    IDLE = "idle"
    PREVIEW_READY = "preview_ready"
    EXECUTING = "executing"


@dataclass(frozen=True)
class PoseTarget:
    """末端目标位姿（基坐标系）。

    位置单位为米，姿态为固定轴 RPY 欧拉角（弧度），顺序与
    ``roll``/``pitch``/``yaw`` 字段一致，由求解器负责转换为旋转矩阵或四元数。
    """

    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float


@dataclass(frozen=True)
class PreviewSolveResult:
    """一次逆解预览的结果。

    失败时 ``joint_positions`` 仍填当前（种子）关节角，调用方可直接沿用而不必处理
    ``None``;``message`` 为失败原因，会原样透传到预览命令与状态面板。
    """

    success: bool
    joint_positions: tuple[float, ...]
    message: str


class PoseSolverProtocol:
    """位姿求解器协议：实现者按目标位姿反解关节角。

    只要求实现 :meth:`solve_pose`；预览管理器通过该协议解耦，真实实现可基于外部
    运动学 SDK,测试可用替身。
    """

    def solve_pose(
        self,
        pose_target: "PoseTarget",
        seed_positions: tuple[float, ...],
        joint_names: tuple[str, ...],
    ) -> "PreviewSolveResult":
        """按目标位姿反解关节角。

        ``seed_positions`` 是当前关节角（弧度），作为数值迭代初值；它同时决定解的
        分支，因此同一目标位姿在不同构型下可能得到不同关节角。``joint_names`` 给出
        ``seed_positions`` 各元素的关节顺序。返回 :class:`PreviewSolveResult`，失败
        时不应抛异常，而应返回 ``success=False`` 以便上层降级为「不可达预览」。
        """
        raise NotImplementedError


@dataclass(frozen=True)
class PreviewCommand:
    """一次预览命令及其可达性结论。

    - ``command_type``:``"joint"`` 表示关节空间预览，``"pose"`` 表示位姿预览；
    - ``reachable``：目标是否可执行；为 ``False`` 时 ``joint_positions`` 保持当前
      关节角，避免调用方误用无效解；
    - ``message``：可读结论（如关节限位超限、未知关节名、逆解不可达）；
    - ``pose_target``：仅位姿预览携带，用于后续 MoveIt 规划与碰撞预检。
    """

    command_type: str
    reachable: bool
    message: str
    joint_names: tuple[str, ...]
    joint_positions: tuple[float, ...]
    pose_target: PoseTarget | None = None


@dataclass(frozen=True)
class ExecutionRequest:
    """一次已受理的执行请求。

    由协调器在预览校验通过后构造，是下发给控制器/执行节点的唯一凭证。

    - ``duration``：目标轨迹的总时长（秒），需大于 0;
    - ``preview_only``：为 ``True`` 时只做预览/仿真，不得驱动真实硬件（仿真模式下
      由协调器置位）；
    - ``preview_command``：产生本次请求的原始预览命令，供执行侧复核可达性与关节
      顺序，避免预览与执行之间发生状态漂移。
    """

    mode: ControlMode
    joint_names: tuple[str, ...]
    joint_positions: tuple[float, ...]
    duration: float
    preview_only: bool
    preview_command: PreviewCommand


@dataclass(frozen=True)
class ExecutionDecision:
    """执行受理结论。

    ``accepted`` 为 ``False`` 时 ``request`` 必为 ``None``,``message`` 说明拒绝原因
    （如「已有执行在进行中」「没有可用预览」「目标不可达」）；为 ``True`` 时
    ``request`` 一定非空，调用方可安全下发。
    """

    accepted: bool
    message: str
    request: ExecutionRequest | None

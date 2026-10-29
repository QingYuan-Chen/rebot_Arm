"""运动命令与预览执行的纯数据模型.

职责与位置
    预览, 求解与执行准入之间的公共数据契约, 无设备或 ROS 状态.

对外接口
    模式/状态枚举, 位姿与预览值对象, 执行请求及决策; PoseSolverProtocol 声明注入能力.

安全与并发约束
    冻结数据只是接口边界; accepted / reachable 不能替代碰撞, 反馈, 显式使能和实际终态验证.

文件结构(按定义顺序)
    ControlMode: 目标下发模式.
    ExecutionState: 预览/执行状态机的状态.
    PoseTarget: 末端目标位姿(基坐标系).
    PreviewSolveResult: 一次逆解预览的结果.
    PoseSolverProtocol: 位姿求解器协议: 实现者按目标位姿反解关节角.
    PreviewCommand: 一次预览命令及其可达性结论.
    ExecutionRequest: 一次已受理的执行请求.
    ExecutionDecision: 执行受理结论.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class ControlMode(str, Enum):
    """目标下发模式.

    继承 ``str`` 便于直接序列化到 JSON 或与字符串比较.
    - ``SIMULATION``: 仅走仿真链路, 不驱动物理机械臂;
    - ``REAL``: 真实机械臂模式, 执行前仍需上层安全门与使能确认.
    """

    SIMULATION = "simulation"
    REAL = "real"


class ExecutionState(str, Enum):
    """预览/执行状态机的状态.

    - ``IDLE``: 无可用预览(或执行已结束/已停止);
    - ``PREVIEW_READY``: 存在一次成功预览, 等待执行请求;
    - ``EXECUTING``: 已受理执行, 未收到结束或停止回调前不再受理新请求.
    """

    IDLE = "idle"
    PREVIEW_READY = "preview_ready"
    EXECUTING = "executing"


@dataclass(frozen=True)
class PoseTarget:
    """末端目标位姿(基坐标系).

    位置单位为米, 姿态为固定轴 RPY 欧拉角(弧度), 顺序与
    ``roll``/``pitch``/``yaw`` 字段一致, 由求解器负责转换为旋转矩阵或四元数.
    """

    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float


@dataclass(frozen=True)
class PreviewSolveResult:
    """一次逆解预览的结果.

    失败时 ``joint_positions`` 仍填当前(种子)关节角, 调用方可直接沿用而不必处理
    ``None``; ``message`` 为失败原因, 会原样透传到预览命令与状态面板.
    """

    success: bool
    joint_positions: tuple[float, ...]
    message: str


class PoseSolverProtocol:
    """位姿求解器协议: 实现者按目标位姿反解关节角.

    只要求实现 :meth:`solve_pose`; 预览管理器通过该协议解耦, 真实实现可基于外部
    运动学 SDK, 测试可用替身.
    """

    def solve_pose(
        self,
        pose_target: "PoseTarget",
        seed_positions: tuple[float, ...],
        joint_names: tuple[str, ...],
    ) -> "PreviewSolveResult":
        """按目标位姿反解关节角.

        ``seed_positions`` 是当前关节角(弧度), 作为数值迭代初值; 它同时决定解的
        分支, 因此同一目标位姿在不同构型下可能得到不同关节角. ``joint_names`` 给出
        ``seed_positions`` 各元素的关节顺序. 返回 :class:`PreviewSolveResult`, 失败
        时不应抛异常, 而应返回 ``success=False`` 以便上层降级为"不可达预览".
        """
        raise NotImplementedError


@dataclass(frozen=True)
class PreviewCommand:
    """一次预览命令及本层可达性结论.

    - command_type: "joint" 为关节预览, "pose" 为位姿预览.
    - reachable: 预览求解和限位检查是否通过; 不能代替路径, 碰撞或硬件准入.
      不可达时 joint_positions 保留当前关节角, 供界面展示, 不得当作执行授权.
    - message: 求解, 关节名或限位等原因.
    - pose_target: 位姿预览的原始目标, 供后续规划和碰撞检查使用.
    """

    command_type: str
    reachable: bool
    message: str
    joint_names: tuple[str, ...]
    joint_positions: tuple[float, ...]
    pose_target: PoseTarget | None = None


@dataclass(frozen=True)
class ExecutionRequest:
    """一次已受理的执行请求.

    由协调器在预览校验通过后构造, 是下发给控制器/执行节点的唯一凭证.

    - ``duration``: 目标轨迹的总时长(秒), 需大于 0;
    - ``preview_only``: 为 ``True`` 时只做预览/仿真, 不得驱动真实硬件(仿真模式下
      由协调器置位);
    - ``preview_command``: 产生本次请求的原始预览命令, 供执行侧复核可达性与关节
      顺序, 避免预览与执行之间发生状态漂移.
    """

    mode: ControlMode
    joint_names: tuple[str, ...]
    joint_positions: tuple[float, ...]
    duration: float
    preview_only: bool
    preview_command: PreviewCommand


@dataclass(frozen=True)
class ExecutionDecision:
    """执行受理结论.

    ``accepted`` 为 ``False`` 时 ``request`` 必为 ``None``, ``message`` 说明拒绝原因
    (如"已有执行在进行中""没有可用预览""目标不可达"); 为 ``True`` 时
    ``request`` 一定非空, 调用方可安全下发.
    """

    accepted: bool
    message: str
    request: ExecutionRequest | None

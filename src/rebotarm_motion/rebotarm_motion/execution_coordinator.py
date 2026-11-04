"""预览与执行之间的协调器: 模式切换, 预览缓存和执行门控.

职责与位置
    交互预览到执行请求之间的纯状态协调器, 不持有设备或 ROS 客户端.
    PreviewManager 负责预览求解和关节限位, 本模块只缓存结论, 判断准入并打包请求.

对外接口
    set_mode 选择仿真或真实模式; preview_* 更新最近一次预览.
    execute_preview 返回 ExecutionDecision, 接纳时附带 ExecutionRequest.
    execution_finished 或 stop_execution 释放本层的执行占用.

主要流程
    选择模式 -> 更新预览 -> 校验执行准入 -> 返回请求 -> 等待调用方报告结束.
    同一实例只允许一项执行在途, 执行来源必须是最近一次可达预览.

安全与并发约束
    实例无锁, 由调用方串行驱动. accepted 只表示本层准入, 不证明硬件已接纳或到位.
    仿真模式把 preview_only 置为 True, 执行适配器须遵守该标志.
    本模块不发控制指令; stop_execution 仅更新本地状态, 实际取消和停止由调用链完成.
    执行, 碰撞, 反馈和显式使能条件仍须由各自所有者验证.

文件结构(按定义顺序)
    InteractiveCoordinator: 持有模式, 最近预览和执行占用, 生成执行决策.
"""

from __future__ import annotations

from .command_models import (
    ControlMode,
    ExecutionDecision,
    ExecutionRequest,
    ExecutionState,
    PoseTarget,
    PreviewCommand,
)
from .preview_manager import PreviewManager


class InteractiveCoordinator:
    """负责第一阶段的模式切换, 预览缓存与执行门控.

    生命周期: 构造后先 :meth:`set_mode` 选定模式(默认仿真), 再反复"预览 -> 执行".
    预览成功会把状态推进到 ``PREVIEW_READY``, 预览不可达则退回 ``IDLE``;
    :meth:`execute_preview` 受理后进入 ``EXECUTING``, 必须由结束/停止回调释放.
    本类不加锁, 假设由单线程(例如定时器或状态机回调)串行驱动.
    """

    def __init__(
        self,
        *,
        preview_manager: PreviewManager,
        default_mode: ControlMode = ControlMode.SIMULATION,
    ) -> None:
        """注入预览管理器, 初始化默认仿真模式和空闲状态; 不启动执行."""
        # 默认为仿真模式: 即使调用方未显式选择, 也不会把运动意外下发到真实机械臂.
        self._preview_manager = preview_manager
        self._mode = default_mode
        self._execution_state = ExecutionState.IDLE
        self._last_preview: PreviewCommand | None = None

    @property
    def mode(self) -> ControlMode:
        """读取本协调器选择的模式, 不能据此推断真实硬件状态."""
        return self._mode

    @property
    def execution_state(self) -> ExecutionState:
        """读取本层请求占用状态, 实际动作终态由调用链另行确认."""
        return self._execution_state

    @property
    def last_preview(self) -> PreviewCommand | None:
        """返回最近一次预览值对象, 可能为空或不可达, 执行仍须经过准入."""
        return self._last_preview

    def set_mode(self, mode: ControlMode) -> None:
        """切换控制模式(仿真/真实).

        只改模式本身, 不清空已有预览, 也不影响进行中的执行; 切换模式后如需按新模式
        重新决策, 应由上层重新发起预览.
        """
        self._mode = mode

    def preview_joint_target(self, joint_targets: dict[str, float]) -> PreviewCommand:
        """关节空间预览并缓存结果.

        ``joint_targets`` 为 ``{关节名: 目标角(rad)}``, 可只给部分关节(未给出的沿用
        当前角, 由预览管理器负责合并与限位校验). 预览成功后状态置为
        ``PREVIEW_READY``, 失败(未知关节, 超限)退回 ``IDLE``, 两种情况下缓存都会被
        更新, 因此后续执行请求一定对应最近一次预览.
        """
        preview = self._preview_manager.preview_joint_target(joint_targets)
        self._last_preview = preview
        self._execution_state = (
            ExecutionState.PREVIEW_READY if preview.reachable else ExecutionState.IDLE
        )
        return preview

    def preview_pose_target(self, pose_target: PoseTarget) -> PreviewCommand:
        """位姿预览(逆解)并缓存结果.

        与关节预览的差别在于可达性由逆解给出; 位姿预览会携带 ``pose_target``, 供后续
        规划与碰撞预检复用. 状态迁移规则与 :meth:`preview_joint_target` 一致.
        """
        preview = self._preview_manager.preview_pose_target(pose_target)
        self._last_preview = preview
        self._execution_state = (
            ExecutionState.PREVIEW_READY if preview.reachable else ExecutionState.IDLE
        )
        return preview

    def stop_execution(self) -> None:
        """外部停止回调: 把状态机复位为 ``IDLE``, 使后续预览/执行可以继续.

        本方法只改状态, 不发送停止指令; 真正的停机由调用方所在的执行链路负责.
        """
        self._execution_state = ExecutionState.IDLE

    def execution_finished(self) -> None:
        """执行正常结束回调: 把状态机复位为 ``IDLE``, 与停止路径做同样的释放."""
        self._execution_state = ExecutionState.IDLE

    def execute_preview(self, *, duration: float) -> ExecutionDecision:
        """请求执行最近一次预览目标.

        ``duration`` 为目标轨迹总时长(秒), 需为正数, 由执行侧换算成轨迹时间戳.

        三道门按顺序拒绝: 已有执行在途(防重入), 没有预览(防跳过预览), 预览不可达
        (防执行无效目标); 全部通过才构造 :class:`ExecutionRequest` 并把状态推进到
        ``EXECUTING``. ``preview_only`` 在仿真模式下恒为 ``True``, 即仿真模式不会产生
        真实硬件运动; 被拒绝时 ``request`` 必为 ``None``.
        """
        if self._execution_state == ExecutionState.EXECUTING:
            return ExecutionDecision(
                accepted=False,
                message="execution already in progress",
                request=None,
            )
        if self._last_preview is None:
            return ExecutionDecision(
                accepted=False,
                message="no preview available",
                request=None,
            )
        if not self._last_preview.reachable:
            return ExecutionDecision(
                accepted=False,
                message="target pose unreachable",
                request=None,
            )

        request = ExecutionRequest(
            mode=self._mode,
            joint_names=self._last_preview.joint_names,
            joint_positions=self._last_preview.joint_positions,
            duration=float(duration),
            # 仿真模式一律标记为仅预览, 真实运动必须由真实模式显式发起.
            preview_only=self._mode == ControlMode.SIMULATION,
            # 带上原始预览命令, 供执行侧复核可达性与关节顺序.
            preview_command=self._last_preview,
        )
        self._execution_state = ExecutionState.EXECUTING
        return ExecutionDecision(
            accepted=True,
            message="execution accepted",
            request=request,
        )

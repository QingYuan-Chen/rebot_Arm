"""无头 reBotArm MuJoCo 仿真的 ROS 2 安全适配节点。

模块职责：把 MuJoCo 物理仿真包装成符合 ROS 2 约定的仿真控制器后端，向上层提供
与真实控制器一致的接口，使运动规划、示教回放等上层代码无需区分真机与仿真：

- 动作 `/<namespace>/follow_joint_trajectory`：接收关节轨迹并在仿真时间轴上执行；
- 服务 `/<namespace>/trajectory_stop`：请求停止当前轨迹并原地保持；
- 服务 `/<namespace>/gripper/set`：设置仿真夹爪开口宽度；
- 话题 `/<namespace>/joint_states`、`/<namespace>/gripper/state`、`/clock`；
- 可选的虚拟 RGB-D 相机话题（只有显式开启虚拟相机时才创建）。

安全约束：本节点只驱动 MuJoCo 仿真，不导入也不调用任何真实电机 SDK；仿真启动
不会打开硬件通道。所有外部输入（轨迹、夹爪宽度、仿真时间）先经本模块顶部的
校验助手做有限性、范围与规模检查，非法输入一律拒绝而不是静默接受；属于物理
行程范围的裁剪（如夹爪开口）由仿真侧按模型限位完成。

本模块顶部的校验助手刻意不导入任何 ROS 依赖，因此可以在未安装 ROS 的开发主机上
对轨迹输入做模糊测试与单元测试；ROS 类型只在构造节点时延迟导入。
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from enum import Enum, auto
from typing import Any, Sequence

from .trajectory_sampler import ARM_JOINT_NAMES, NamedTrajectoryPoint, TrajectorySampler


# 单条轨迹允许的最大路点数：防止畸形/恶意目标用超大点数耗尽内存与规划时间。
DEFAULT_MAX_TRAJECTORY_POINTS = 10_000
# 单条轨迹允许的最长执行时长（秒），与点数上限一起构成轨迹规模的硬边界。
DEFAULT_MAX_TRAJECTORY_DURATION_SEC = 300.0


@dataclass(frozen=True)
class GoalSettlingPolicy:
    """节点级"到位判定"策略：轨迹走完后判断仿真是否已稳定停住。

    三个阈值都是不可被单次目标请求覆盖的节点级默认值（构造节点时由参数注入）。
    只做判定，不发命令；`evaluate` 是纯函数，便于离线测试。
    """

    # 各关节位置误差上限，单位 rad（取六轴中的最大绝对误差）。
    position_tolerance: float = 0.02
    # 各关节速度上限，单位 rad/s（取六轴中的最大绝对速度）。
    velocity_tolerance: float = 0.05
    # 到达轨迹终点后允许的等待时间，单位 s；超时仍未满足上面两个阈值即判超时。
    time_tolerance_sec: float = 5.0

    def __post_init__(self) -> None:
        # 阈值必须为正的有限值：0 或负数会让"到位"判定永不可能满足，使目标
        # 只能等到超时，等于人为制造执行失败。
        values = (self.position_tolerance, self.velocity_tolerance, self.time_tolerance_sec)
        if any(not math.isfinite(float(value)) or float(value) <= 0.0 for value in values):
            raise ValueError("goal tolerances must be positive finite values")

    def evaluate(self, desired, actual, velocities, settle_elapsed: float) -> str:
        """判定当前是否到位。

        参数：
            desired: 目标关节角，单位 rad，必须是 6 个手臂关节值；
            actual: 当前关节角，单位 rad，长度同上；
            velocities: 当前关节角速度，单位 rad/s，长度同上；
            settle_elapsed: 轨迹走完后已等待的时间，单位 s，必须非负。

        返回三态字符串（调用方据此决定继续等待/成功/超时）：
            "succeeded" —— 最大位置误差与最大速度都在阈值内；
            "timed_out" —— 等待时间已达 time_tolerance_sec 仍未满足阈值；
            "settling"  —— 仍在容差内等待。
        """
        vectors = (tuple(desired), tuple(actual), tuple(velocities))
        if any(len(vector) != len(ARM_JOINT_NAMES) for vector in vectors):
            raise ValueError("goal state must contain six arm values")
        numeric = tuple(tuple(float(value) for value in vector) for vector in vectors)
        elapsed = float(settle_elapsed)
        if any(not math.isfinite(value) for vector in numeric for value in vector) or not math.isfinite(elapsed):
            raise ValueError("goal state must be finite")
        if elapsed < 0.0:
            raise ValueError("settling time must be non-negative")
        # 位置与速度都用"最差关节"（各轴绝对值取最大）作为判据，任一轴不达标即未稳定。
        position_error = max(abs(target - reached) for target, reached in zip(numeric[0], numeric[1]))
        max_velocity = max(abs(value) for value in numeric[2])
        # 先判成功再判超时：恰好同时满足时按成功处理，避免边界抖动导致误报超时。
        if position_error <= self.position_tolerance and max_velocity <= self.velocity_tolerance:
            return "succeeded"
        if elapsed >= self.time_tolerance_sec:
            return "timed_out"
        return "settling"


def duration_to_seconds(duration: Any) -> float:
    """把带 sec/nanosec 字段的 ROS 时长对象换算为秒（float）。

    仅按属性读取，不依赖具体消息类型，因此可以在无 ROS 环境下测试。字段缺失、
    类型错误或结果非有限值时抛 ValueError（由调用方转成拒绝目标）。
    """
    try:
        value = float(duration.sec) + float(duration.nanosec) * 1e-9
    except (AttributeError, TypeError, ValueError) as exc:
        raise ValueError("trajectory time is invalid") from exc
    if not math.isfinite(value):
        raise ValueError("trajectory time must be finite")
    return value


def trajectory_to_sampler(
    trajectory: Any,
    *,
    initial_positions: Sequence[float],
    max_points: int = DEFAULT_MAX_TRAJECTORY_POINTS,
    max_duration_sec: float = DEFAULT_MAX_TRAJECTORY_DURATION_SEC,
) -> TrajectorySampler:
    """校验外部传入的"类 JointTrajectory"对象并构造采样器。

    这是动作目标的第一道安全门：先把不可信输入挡在仿真之外，再交给采样器。
    本函数只做规模与结构层面的拒绝式校验（点数上限、时长上限、时长为有限值），
    关节名合法性、重复点、时间严格递增等语义校验由采样器负责。

    参数：
        trajectory: 具有 joint_names 与 points 属性的对象（points 需含
            time_from_start 与 positions）；
        initial_positions: 六个手臂关节的初始角，单位 rad；用于补齐目标中未列出的关节；
        max_points: 允许的最大路点数，正整数；
        max_duration_sec: 允许的最长轨迹时长，单位 s，正的有限值。

    异常：结构不可读、超规模或超时长时抛 ValueError。
    """
    if isinstance(max_points, bool) or int(max_points) <= 0:
        raise ValueError("max points must be positive")
    duration_cap = float(max_duration_sec)
    if not math.isfinite(duration_cap) or duration_cap <= 0.0:
        raise ValueError("max duration must be finite and positive")
    try:
        names = tuple(trajectory.joint_names)
        raw_points = tuple(trajectory.points)
    except (AttributeError, TypeError) as exc:
        raise ValueError("trajectory structure is invalid") from exc
    if len(raw_points) > int(max_points):
        raise ValueError("trajectory contains too many points")

    points = tuple(
        NamedTrajectoryPoint(duration_to_seconds(point.time_from_start), point.positions)
        for point in raw_points
    )
    # 规范关节名校验、重复名/长度、非空、有限值与时间严格递增这些语义约束统一由
    # 采样器负责，本文只在上游控制规模。
    sampler = TrajectorySampler(names, points, initial_positions=initial_positions)
    # 时长上限在采样器规范化之后才能确知（此时 duration 已含补齐初始点的语义）。
    if sampler.duration > duration_cap:
        raise ValueError("trajectory duration exceeds configured limit")
    return sampler


def validate_gripper_width(width: Any) -> float:
    """校验夹爪目标开口宽度并返回 float。

    参数 width 单位为 m（0 = 完全闭合，正值 = 张开）。这里只拒绝非有限值，
    行程裁剪（夹到可信开度内）交由仿真侧完成，与本模块的尺度无关。
    """
    try:
        value = float(width)
    except (TypeError, ValueError) as exc:
        raise ValueError("gripper width must be finite") from exc
    if not math.isfinite(value):
        raise ValueError("gripper width must be finite")
    return value


def seconds_to_stamp_parts(simulation_time: Any) -> tuple[int, int]:
    """把仿真时间（秒，float）拆成 ROS 时间戳的 (sec, nanosec) 整数对。

    仿真时间必须有限且非负；四舍五入到纳秒后若进位到 1e9 则向秒进位，
    保证返回的纳秒字段始终落在 [0, 1e9) 内。
    """
    try:
        value = float(simulation_time)
    except (TypeError, ValueError) as exc:
        raise ValueError("simulation time must be finite and non-negative") from exc
    if not math.isfinite(value) or value < 0.0:
        raise ValueError("simulation time must be finite and non-negative")
    seconds = math.floor(value)
    # 先按纳秒取整，再处理进位，避免 nanosec 越界导致下游时间戳非法。
    nanoseconds = int(round((value - seconds) * 1_000_000_000))
    if nanoseconds >= 1_000_000_000:
        seconds += 1
        nanoseconds -= 1_000_000_000
    return int(seconds), nanoseconds


class MonotonicStamp:
    """单调时间戳生成器：保证对外发布的仿真时间只增不减。

    物理步进与虚拟相机线程可能并发取时间戳，若直接透传仿真时间，时钟会在
    重启/回退时倒退，使依赖时间戳的 TF、消息缓存与上层校验失效。这里用锁
    维护已发布的最大值，任何更小的候选都被夹到该值。
    """

    def __init__(self) -> None:
        self._nanoseconds = 0
        self._lock = threading.Lock()

    def update(self, simulation_time: Any) -> tuple[int, int]:
        """登记一个仿真时间并返回不小于此前已返回值的 (sec, nanosec)。"""
        seconds, nanoseconds = seconds_to_stamp_parts(simulation_time)
        candidate = seconds * 1_000_000_000 + nanoseconds
        with self._lock:
            self._nanoseconds = max(self._nanoseconds, candidate)
            return divmod(self._nanoseconds, 1_000_000_000)


class FeedbackRateLimiter:
    """按固定频率限制动作反馈发布，避免执行循环每个物理步都发反馈。

    上限 200 Hz 是刻意的保护：反馈频率若超过真实控制周期只会制造无意义的总线
    与订阅端压力，这里直接把配置错误拒绝掉。`should_publish` 使用调用方传入的
    单调时钟（time.monotonic），与仿真时间无关。
    """

    def __init__(self, rate_hz: Any) -> None:
        try:
            rate = float(rate_hz)
        except (TypeError, ValueError) as exc:
            raise ValueError("feedback rate must be finite and in (0, 200]") from exc
        if not math.isfinite(rate) or rate <= 0.0 or rate > 200.0:
            raise ValueError("feedback rate must be finite and in (0, 200]")
        self.rate_hz = rate
        self._period = 1.0 / rate
        self._last_publish: float | None = None

    def should_publish(self, monotonic_time: Any, *, final: bool = False) -> bool:
        """判断此刻是否应发布反馈。

        final=True 用于终态反馈（成功/超时），无论是否已到周期都强制放行，
        确保订阅端一定收到最后一条状态；首次调用（_last_publish 为 None）
        也一定放行。
        """
        now = float(monotonic_time)
        if not math.isfinite(now):
            raise ValueError("feedback clock must be finite")
        due = self._last_publish is None or now - self._last_publish >= self._period
        if final or due:
            self._last_publish = now
            return True
        return False


class ActiveTrajectory:
    """线程安全的"单目标准入 + 协作式取消"状态。

    仿真一次只允许执行一条轨迹。`_token` 就是被准入的目标对象（当前实现里是
    目标请求），为 None 表示空闲；所有读取与迁移都在同一把可重入锁下完成，
    使"取消"与"开始下一个目标"不会交错。取消采用协作式：这里只置标志，
    真正的停止与保持由持有仿真锁的命令闸门完成。
    """

    def __init__(self, lock: threading.RLock | None = None) -> None:
        self._lock = lock or threading.RLock()
        self._token: object | None = None
        self._cancel_requested = False

    @property
    def busy(self) -> bool:
        with self._lock:
            return self._token is not None

    @property
    def cancel_requested(self) -> bool:
        with self._lock:
            return self._cancel_requested

    @property
    def token(self) -> object | None:
        with self._lock:
            return self._token

    def try_start(self, token: object) -> bool:
        """尝试占用执行权：空闲时登记 token 并清零取消标志，忙时返回 False。"""
        with self._lock:
            if self._token is not None:
                return False
            self._token = token
            self._cancel_requested = False
            return True

    def stop(self) -> bool:
        """仅置取消标志（不触碰仿真状态）；无活动目标时返回 False。"""
        with self._lock:
            if self._token is None:
                return False
            self._cancel_requested = True
            return True

    def finish(self, token: object) -> None:
        """释放执行权。只有 token 与当前登记一致才清空，避免误释放他人的目标。"""
        with self._lock:
            if self._token is token:
                self._token = None
                self._cancel_requested = False


class GateOutcome(Enum):
    """命令闸门对一次操作给出的裁决结果（决定目标如何终止）。"""

    # 命令已真正下发给仿真，执行循环继续。
    APPLIED = auto()
    # 客户端取消了动作目标：保持当前位置并按 canceled 终止。
    ACTION_CANCEL = auto()
    # 收到停止服务请求：保持当前位置并按 aborted 终止。
    SERVICE_STOP = auto()
    # token 已不匹配（目标已被其他路径终结），本次调用什么都不做。
    INACTIVE = auto()
    # 目标成功收敛，动作按 succeeded 终止。
    SUCCEEDED = auto()


def terminal_disposition(outcome: GateOutcome, action_cancel_requested: bool) -> str:
    """把闸门裁决映射为动作终态字符串。

    只有"闸门判为取消"且客户端确实请求过取消时才算 canceled；停止服务或目标
    被替换等情况一律 aborted，避免把服务停止误报成用户取消。
    """
    if outcome is GateOutcome.ACTION_CANCEL and bool(action_cancel_requested):
        return "canceled"
    return "aborted"


class TrajectoryCommandGate:
    """在持锁状态下原子仲裁"继续下发轨迹"与"停止/保持"两类操作。

    设计要点：所有钩子（apply/hold/succeed 以及取消回调）都在可重入锁内执行，
    因此"取消到达"与"成功收敛"只会有一个赢家；执行循环每轮都必须经过本闸门，
    一旦 token 失效或出现取消请求就立刻停手并保持当前位置，绝不再下发新目标。
    """

    def __init__(self, active: ActiveTrajectory) -> None:
        self._active = active

    def apply_if_active(self, token, cancel_requested, apply, hold) -> bool:
        """下发目标的布尔封装；未下发时返回 False。"""
        return self.apply_with_reason(token, cancel_requested, apply, hold) is GateOutcome.APPLIED

    def apply_with_reason(self, token, action_cancel_requested, apply, hold) -> GateOutcome:
        """在确认目标仍有效且未取消时执行 apply，否则执行 hold 并给出原因。"""
        with self._active._lock:
            action_cancel = bool(action_cancel_requested())
            if self._active._token is not token:
                return GateOutcome.INACTIVE
            if action_cancel:
                self._active._cancel_requested = True
                hold()
                return GateOutcome.ACTION_CANCEL
            # 服务停止（而非动作取消）：已经置过取消标志，同样只保持不再前进。
            if self._active._cancel_requested:
                hold()
                return GateOutcome.SERVICE_STOP
            apply()
            return GateOutcome.APPLIED

    def stop_and_hold(self, hold) -> bool:
        """请求停止：有活动目标时置取消标志并立即保持当前位置。

        返回是否确实停止了某个目标，供取消回调/停止服务决定 ACCEPT 还是 REJECT。
        """
        with self._active._lock:
            stopped = self._active._token is not None
            if stopped:
                self._active._cancel_requested = True
                hold()
            return stopped

    def complete_if_active(self, token, cancel_requested, hold, succeed) -> bool:
        """完成目标的布尔封装；真正成功收敛时返回 True。"""
        return self.complete_with_reason(
            token, cancel_requested, hold, succeed
        ) is GateOutcome.SUCCEEDED

    def complete_with_reason(self, token, action_cancel_requested, hold, succeed) -> GateOutcome:
        """把"取消"与"成功终态迁移"线性化，保证二者互斥。"""
        with self._active._lock:
            # 先执行外部回调再重新读取内部状态：取消回调可能通过可重入的测试
            # 钩子置位，也可能刚好在进入临界区之前完成，因此必须以锁内的
            # 最新标志为准。
            action_cancel = bool(action_cancel_requested())
            if self._active._token is not token:
                return GateOutcome.INACTIVE
            if action_cancel:
                self._active._cancel_requested = True
                hold()
                return GateOutcome.ACTION_CANCEL
            if self._active._cancel_requested:
                hold()
                return GateOutcome.SERVICE_STOP
            # 调用 ROS 终态迁移时不持有仿真锁，避免在锁内触发订阅端回调而死锁。
            succeed()
            self._active._token = None
            self._active._cancel_requested = False
            return GateOutcome.SUCCEEDED


class ExecutionLifecycle:
    """异常路径的兜底清理：保证已准入的 token 一定被释放。

    执行回调抛异常时，若只 abort 而不释放执行权，动作服务器会永久处于 busy
    状态、后续所有目标都被拒绝。这里先尽量把当前位置保持住（失败也不影响
    清理），再 abort，最后无论如何都 finish 掉 token。
    """

    def __init__(self, active: ActiveTrajectory, gate: TrajectoryCommandGate) -> None:
        self._active = active
        self._gate = gate

    def fail(self, token, hold, abort) -> None:
        try:
            self._gate.stop_and_hold(hold)
        except Exception:
            pass
        try:
            abort()
        finally:
            self._active.finish(token)


class SerializedSimulationAccess:
    """串行化所有触碰同一仿真实例的操作。

    物理步进（定时器线程）、轨迹目标下发（动作执行线程）和虚拟相机渲染
    （相机工作线程）都会访问同一个 MuJoCo data，必须通过这把锁互斥；否则
    会出现读写竞态导致的非物理跳变。
    """

    def __init__(self, simulation: Any, lock: threading.RLock | None = None) -> None:
        self._simulation = simulation
        self._lock = lock or threading.RLock()

    def run(self, operation):
        with self._lock:
            return operation(self._simulation)


def create_node_class():
    """延迟导入 ROS 依赖并返回具体的节点类。

    之所以做成工厂函数而不是模块级类定义：本模块顶部的校验逻辑需要在没有 ROS
    的环境里被导入和测试，因此所有 ROS 消息、执行器与 QoS 类型都推迟到真正
    构造节点时才导入。
    """
    import rclpy
    from control_msgs.action import FollowJointTrajectory
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from rclpy.action import ActionServer, CancelResponse, GoalResponse
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
    from rclpy.clock import Clock as RclpyClock
    from rclpy.clock import ClockType
    from rclpy.node import Node
    from rebotarm_msgs.msg import JointMotorState
    from rebotarm_msgs.srv import SetGripper
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState
    from std_srvs.srv import Trigger
    from trajectory_msgs.msg import JointTrajectoryPoint

    from .mujoco_sim import RebotArmMujoco
    from .ros_diagnostics import build_control_diagnostic

    class RebotArmMujocoNode(Node):
        """仿真控制器节点：唯一持有 MuJoCo 实例并对外提供假硬件接口。

        线程模型：
        - 定时器回调（互斥回调组）负责按固定周期步进物理、发布 `/clock`、
          关节状态与夹爪状态，并触发虚拟相机渲染；
        - 动作执行回调（可重入回调组）在自己的线程里按仿真时间采样轨迹；
        - 虚拟相机工作线程负责离屏渲染，其生命周期完全在单独线程上；
        - 三者对仿真的访问统一经 `_sim_access` 串行化，锁序固定为
          "命令闸门锁 → 仿真锁"，避免死锁。

        安全语义：只接受 backend=mujoco 且 headless=true；同一时刻只服务一条
        轨迹；取消/停止/异常都会保持当前位置，绝不继续下发目标。
        """

        def __init__(self) -> None:
            super().__init__("rebotarm_mujoco_node")
            # 后端选择：本节点只实现 mujoco，其他取值（例如指向真实硬件）必须走别的节点。
            self.declare_parameter("backend", "mujoco")
            # 必须为 true：ROS 适配只允许无头模式，查看器由启动脚本按需另开进程。
            self.declare_parameter("headless", True)
            # 是否在执行线程之外同步一个 MuJoCo 被动查看器窗口（桌面调试用）。
            self.declare_parameter("show_viewer", False)
            # 场景模型路径；空字符串表示用包内默认场景。
            self.declare_parameter("model_path", "")
            # 话题与服务命名空间前缀（去除首尾斜杠后使用），决定所有接口的名字。
            self.declare_parameter("arm_namespace", "rebotarm")
            # 物理步进定时器频率，单位 Hz；决定 /clock 与关节状态的发布周期。
            self.declare_parameter("publish_rate_hz", 30.0)
            # 单条轨迹点数上限与时长上限（单位 s），见模块顶部常量说明。
            self.declare_parameter("max_trajectory_points", DEFAULT_MAX_TRAJECTORY_POINTS)
            self.declare_parameter("max_trajectory_duration_sec", DEFAULT_MAX_TRAJECTORY_DURATION_SEC)
            # 六个手臂关节的初始角，单位 rad；启动即把仿真复位到该位形。
            self.declare_parameter("initial_joint_positions", [0.0] * 6)
            # 到位判定阈值：位置单位 rad、速度单位 rad/s、等待时间单位 s。
            self.declare_parameter("goal_position_tolerance", 0.02)
            self.declare_parameter("goal_velocity_tolerance", 0.05)
            self.declare_parameter("goal_time_tolerance_sec", 5.0)
            # 动作反馈发布频率，单位 Hz（上限 200，见 FeedbackRateLimiter）。
            self.declare_parameter("feedback_rate_hz", 20.0)
            self.declare_parameter("diagnostic_rate_hz", 1.0)
            self.declare_parameter("max_contact_force_n", 200.0)
            self.declare_parameter("max_contact_penetration_m", 0.005)
            for name in ("diagnostic_rate_hz", "max_contact_force_n", "max_contact_penetration_m"):
                value = float(self.get_parameter(name).value)
                if not math.isfinite(value) or value <= 0.0:
                    raise ValueError(f"{name} must be positive and finite")
            # 以下三项是启动前的硬门：参数不合法直接抛错终止，绝不带着错误配置跑仿真。
            if self.get_parameter("backend").value != "mujoco":
                raise ValueError("simulation backend must be mujoco")
            if self.get_parameter("headless").value is not True:
                raise ValueError("ROS adapter requires headless=true")
            self._show_viewer = bool(self.get_parameter("show_viewer").value)

            # 命名空间只允许去掉首尾斜杠的普通标识；含 "//" 或空格会拼出非法接口名。
            namespace = str(self.get_parameter("arm_namespace").value).strip("/")
            if not namespace or any(part in namespace for part in ("//", " ")):
                raise ValueError("arm namespace is invalid")
            self._arm_namespace = namespace
            # 步进频率上限 1000 Hz：再高只会空转 CPU 且不改善物理精度（精度由 timestep 决定）。
            rate = float(self.get_parameter("publish_rate_hz").value)
            if not math.isfinite(rate) or rate <= 0.0 or rate > 1000.0:
                raise ValueError("publish rate must be finite and in (0, 1000]")
            self._max_points = int(self.get_parameter("max_trajectory_points").value)
            self._max_duration = float(self.get_parameter("max_trajectory_duration_sec").value)
            # 刻意不解释单个目标自带的 JointTolerance 数组：本仿真后端只使用
            # 上述有界的节点级默认值，防止客户端在单次目标里放宽到位保证。
            self._settling_policy = GoalSettlingPolicy(
                float(self.get_parameter("goal_position_tolerance").value),
                float(self.get_parameter("goal_velocity_tolerance").value),
                float(self.get_parameter("goal_time_tolerance_sec").value),
            )
            self._feedback_rate_hz = float(self.get_parameter("feedback_rate_hz").value)
            # 构造函数内部完成有限/正数/范围校验，非法值在此直接终止启动。
            FeedbackRateLimiter(self._feedback_rate_hz)
            # 执行循环的休眠步长：上限 10 ms，避免忙等占满 CPU，同时保证反馈与
            # 取消请求的响应延迟不超过一个反馈周期。
            self._execute_wait_sec = min(0.01, 1.0 / self._feedback_rate_hz)
            # 初始位形必须是六个有限值，否则拒绝启动（不静默补零）。
            initial = tuple(float(v) for v in self.get_parameter("initial_joint_positions").value)
            if len(initial) != 6 or any(not math.isfinite(v) for v in initial):
                raise ValueError("initial positions must contain six finite values")

            model_path = str(self.get_parameter("model_path").value).strip()
            # 空路径交给仿真类去解析包内默认场景，语义与显式传路径一致。
            self._sim = RebotArmMujoco(model_path or None)
            self._lock = threading.RLock()
            self._sim_access = SerializedSimulationAccess(self._sim, self._lock)
            self._sim_access.run(lambda sim: sim.reset_joint_positions(initial))
            # 命令闸门锁与仿真锁刻意分开：加锁顺序恒为"先闸门、后仿真"，而定时器
            # 只取仿真锁，因此不存在反向持锁路径，不会死锁。
            self._active = ActiveTrajectory()
            self._command_gate = TrajectoryCommandGate(self._active)
            self._lifecycle = ExecutionLifecycle(self._active, self._command_gate)
            # 准入时校验好的轨迹采样器，由目标回调暂存、执行回调取走（一次性交接）。
            self._pending_sampler: TrajectorySampler | None = None
            # 动作/服务回调放在可重入组里：执行回调自身会长时间运行，且停止服务
            # 必须在执行期间仍能被派发，否则"停止"会被执行回调阻塞住。
            self._callback_group = ReentrantCallbackGroup()
            # 定时器组互斥：避免多次步进回调重入同一个物理实例。
            self._timer_callback_group = MutuallyExclusiveCallbackGroup()
            # 物理时钟用 STEADY_TIME，保证定时器周期不受系统时间跳变影响。
            self._physics_clock = RclpyClock(clock_type=ClockType.STEADY_TIME)
            # 对外时间戳单调递增（见 MonotonicStamp 说明）。
            self._stamp = MonotonicStamp()
            # 只读反馈话题：关节状态、夹爪状态与仿真时钟。队列深度 10 足以吸收
            # 短暂抖动，仿真时间由 /clock 统一对外，供 use_sim_time 的节点对齐。
            self._joint_pub = self.create_publisher(
                JointState, f"/{self._arm_namespace}/joint_states", 10
            )
            self._gripper_pub = self.create_publisher(
                JointMotorState, f"/{self._arm_namespace}/gripper/state", 10
            )
            self._clock_pub = self.create_publisher(Clock, "/clock", 10)
            self._diagnostic_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
            self._diagnostic_last_wall = time.monotonic()
            self._diagnostic_last_sim = 0.0
            # 与真实控制器同名的轨迹动作接口：上层运动/示教代码无需区分真机与仿真。
            self._action_server = ActionServer(
                self,
                FollowJointTrajectory,
                f"/{self._arm_namespace}/follow_joint_trajectory",
                goal_callback=self._goal_callback,
                cancel_callback=self._cancel_callback,
                execute_callback=self._execute_goal,
                callback_group=self._callback_group,
            )
            # 停止服务：让上层以"服务调用"的方式软停（保持当前位置），不依赖动作取消。
            self.create_service(
                Trigger,
                f"/{self._arm_namespace}/trajectory_stop",
                self._stop_service,
                callback_group=self._callback_group,
            )
            # 夹爪服务沿用 SetGripper 的调用约定（开口宽度单位 m），便于仿真链路复用。
            self.create_service(
                SetGripper,
                f"/{self._arm_namespace}/gripper/set",
                self._gripper_service,
                callback_group=self._callback_group,
            )
            # 每次定时器回调推进足够多的固定物理步，使其时长与配置的发布周期一致；
            # 轨迹采样始终以仿真时间（而非墙钟）为准，因此慢机器上轨迹不会"变慢"。
            self._steps_per_tick = max(1, round((1.0 / rate) / self._sim.timestep))
            self.create_timer(
                1.0 / rate,
                self._timer_callback,
                callback_group=self._timer_callback_group,
                clock=self._physics_clock,
            )
            self.create_timer(
                1.0 / float(self.get_parameter("diagnostic_rate_hz").value),
                self._publish_diagnostics,
                callback_group=self._timer_callback_group,
                clock=self._physics_clock,
            )

        @property
        def show_viewer(self) -> bool:
            return self._show_viewer

        def viewer_handles(self):
            """在仿真锁内取出 MuJoCo 原生 model/data 句柄供查看器使用。

            句柄只在仿真未关闭时有效，调用方不得长期持有，也不得绕过本节点
            直接改动物理状态。
            """
            return self._sim_access.run(lambda sim: sim._unsafe_viewer_handles())

        def sync_viewer(self, viewer) -> None:
            """在仿真锁内把最新物理状态同步给查看器（避免读到半更新状态）。"""
            self._sim_access.run(lambda _sim: viewer.sync())

        def _current_arm_positions(self) -> tuple[float, ...]:
            """读取当前六个手臂关节角（rad），供轨迹准入时补齐缺省关节。"""
            return self._sim_access.run(
                lambda sim: tuple(sim.get_state().joint_positions[:6])
            )

        def _hold_current_position(self) -> None:
            """线程安全地"原地保持"：把当前位置同时设为目标位置。"""
            self._sim_access.run(lambda _sim: self._hold_current_position_unlocked())

        def _hold_current_position_unlocked(self) -> None:
            """已持有仿真锁时的保持实现，供闸门在同一临界区内调用。"""
            current = tuple(self._sim.get_state().joint_positions[:6])
            self._sim.set_joint_position_targets(current)

        def _apply_target_threadsafe(self, operation) -> None:
            """在仿真锁内执行一个会改写物理目标的操作。"""
            self._sim_access.run(lambda _sim: operation())

        def _goal_callback(self, goal_request):
            """动作目标准入：先校验轨迹，再抢占执行权；任一步失败都返回 REJECT。

            校验在此完成而不是等到执行阶段，是为了让非法目标尽快得到明确的
            拒绝响应，也避免执行线程被畸形输入拖住。通过后把采样器暂存，由
            执行回调一次性取走。
            """
            token = goal_request
            try:
                sampler = trajectory_to_sampler(
                    goal_request.trajectory,
                    initial_positions=self._current_arm_positions(),
                    max_points=self._max_points,
                    max_duration_sec=self._max_duration,
                )
            except (TypeError, ValueError):
                self.get_logger().warning("rejected invalid trajectory goal")
                return GoalResponse.REJECT
            # 单目标闸门：忙时直接拒绝新目标，不做排队，避免隐式覆盖正在执行的轨迹。
            if not self._active.try_start(token):
                self.get_logger().warning("rejected trajectory goal while controller is busy")
                return GoalResponse.REJECT
            self._pending_sampler = sampler
            return GoalResponse.ACCEPT

        def _cancel_callback(self, _goal_handle):
            """动作取消回调：能停下活动目标才 ACCEPT，否则 REJECT（不改动仿真）。"""
            stopped = self._command_gate.stop_and_hold(self._hold_current_position)
            return CancelResponse.ACCEPT if stopped else CancelResponse.REJECT

        def _stop_service(self, _request, response):
            """停止服务：软停（保持当前位置），始终返回 success=True 以区分"无活动目标"。"""
            stopped = self._command_gate.stop_and_hold(self._hold_current_position)
            response.success = True
            response.message = "simulation trajectory stop requested" if stopped else "no active trajectory"
            return response

        def _publish_diagnostics(self):
            state, contacts, status = self._sim_access.run(
                lambda sim: (
                    sim.get_state(),
                    sim.get_contacts(),
                    {"mode": sim.control_mode, "joint_targets": sim.control_targets[:6]},
                )
            )
            now = time.monotonic()
            elapsed = max(now - self._diagnostic_last_wall, 1e-9)
            physics_rate = (state.simulation_time - self._diagnostic_last_sim) / elapsed / self._sim.timestep
            self._diagnostic_last_wall, self._diagnostic_last_sim = now, state.simulation_time
            summary = build_control_diagnostic(
                arm_namespace=self._arm_namespace,
                configured_rate_hz=1.0 / self._sim.timestep,
                measured_rate_hz=physics_rate,
                state=state,
                status=status,
                contacts=contacts,
                max_contact_force_n=self.get_parameter("max_contact_force_n").value,
                max_contact_penetration_m=self.get_parameter("max_contact_penetration_m").value,
            )
            msg = DiagnosticArray()
            msg.header.stamp = self.get_clock().now().to_msg()
            item = DiagnosticStatus()
            item.level = DiagnosticStatus.WARN if summary.warning else DiagnosticStatus.OK
            item.name, item.hardware_id, item.message = summary.name, summary.hardware_id, summary.message
            item.values = [KeyValue(key=value.key, value=value.value) for value in summary.values]
            msg.status = [item]
            self._diagnostic_pub.publish(msg)

        def _gripper_service(self, request, response):
            """夹爪服务：校验开口宽度后交给仿真裁剪到可信行程并返回实际到位值。"""
            try:
                width = validate_gripper_width(request.position)
                reached = self._sim_access.run(lambda sim: sim.set_gripper_width(width))
            except (TypeError, ValueError):
                # 请求非法：明确报失败，并回填 0.0 而不是伪造一个"已到位"的位置。
                response.success = False
                response.reached_position = 0.0
                return response
            response.success = True
            response.reached_position = float(reached)
            return response

        @staticmethod
        def _terminate_goal(goal_handle, result, outcome):
            """按闸门裁决把目标终止为 canceled 或 aborted 并填好错误码。

            注意：取消成功时动作错误码仍填 SUCCESSFUL——取消是客户端主动行为，
            不是执行故障，错误信息只用于日志区分。
            """
            disposition = terminal_disposition(outcome, goal_handle.is_cancel_requested)
            if disposition == "canceled":
                goal_handle.canceled()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                result.error_string = "simulation trajectory canceled"
                return result
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            if outcome is GateOutcome.SERVICE_STOP:
                result.error_string = "simulation trajectory stopped by service"
            else:
                result.error_string = "simulation trajectory is no longer active"
            return result

        def _execute_goal(self, goal_handle):
            """动作执行回调：在仿真时间轴上跟踪轨迹、发反馈并判定终态。

            每轮循环先经命令闸门下发目标（闸门会同时处理取消与 token 失效），
            再按反馈频率发布 desired/actual/error；轨迹走完后进入到位判定，
            只有位置与速度都收敛才 succeed，超时则 abort 并保持当前位置。
            """
            # rclpy 不保证"传给准入门的目标请求包装对象"与"目标句柄暴露的对象"
            # 是同一个。这里保留准入时的 token，确保清理阶段不会因为对象不一致
            # 而漏释放执行权、让动作服务器在一个合法目标之后永久 busy。
            token = self._active.token
            result = FollowJointTrajectory.Result()
            # 一次性交接：取走准入时校验好的采样器并清空暂存位。
            sampler, self._pending_sampler = self._pending_sampler, None
            if sampler is None:
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = "trajectory was not admitted"
                goal_handle.abort()
                self._active.finish(token)
                return result
            try:
                # 以目标开始执行时的仿真时间为轨迹时间原点；仿真时间只随物理步进前进，
                # 因此慢机器不会让轨迹被执行得更快或更慢。
                start_time = self._sim_access.run(lambda sim: sim.get_state().simulation_time)
                # 每个目标一个限流器实例：首个反馈立即发出，之后按反馈频率节流。
                feedback_limiter = FeedbackRateLimiter(self._feedback_rate_hz)
                while rclpy.ok():
                    # 闭包无法直接返回结果，用一个临时字典把本轮的仿真状态与采样值
                    # 从 apply_target 里带出来。
                    command: dict[str, Any] = {}

                    def apply_target() -> None:
                        state = self._sim.get_state()
                        elapsed = max(0.0, state.simulation_time - start_time)
                        # 超过轨迹时长后夹到终点，保持终点位姿直到判定结束。
                        desired = sampler.sample(min(elapsed, sampler.duration))
                        self._sim.set_joint_position_targets(desired)
                        command.update(state=state, elapsed=elapsed, desired=desired)

                    # 闸门回调里再查一次取消标志：取消可能刚好在上一轮循环之后到达。
                    outcome = self._command_gate.apply_with_reason(
                        token,
                        lambda: goal_handle.is_cancel_requested,
                        lambda: self._apply_target_threadsafe(apply_target),
                        self._hold_current_position,
                    )
                    if outcome is not GateOutcome.APPLIED:
                        return self._terminate_goal(goal_handle, result, outcome)
                    state = command["state"]
                    elapsed = command["elapsed"]
                    desired = command["desired"]
                    # 反馈统一使用规范的六个关节名与名称顺序，保证订阅端能按名对齐。
                    feedback = FollowJointTrajectory.Feedback()
                    feedback.joint_names = list(ARM_JOINT_NAMES)
                    feedback.desired = JointTrajectoryPoint()
                    feedback.actual = JointTrajectoryPoint()
                    feedback.error = JointTrajectoryPoint()
                    feedback.desired.positions = list(desired)
                    actual = tuple(state.joint_positions[:6])
                    feedback.actual.positions = list(actual)
                    feedback.actual.velocities = list(state.joint_velocities[:6])
                    # error 定义为"期望 - 实际"，符号与上层监控约定一致。
                    feedback.error.positions = [d - a for d, a in zip(desired, actual)]
                    # 只有在轨迹走完后才开始到位判定；在此之前统一报告 "tracking"
                    # （非终态伪状态），settle_elapsed 也才从 0 起算，不会把跟踪
                    # 过程中的减速误判为"已到位"。
                    settling = self._settling_policy.evaluate(
                        sampler.sample(sampler.duration),
                        actual,
                        tuple(state.joint_velocities[:6]),
                        max(0.0, elapsed - sampler.duration),
                    ) if elapsed >= sampler.duration else "tracking"
                    # 终态反馈强制发布（final=True），确保订阅端不会漏掉最后一次状态。
                    if feedback_limiter.should_publish(
                        time.monotonic(), final=settling in ("succeeded", "timed_out")
                    ):
                        goal_handle.publish_feedback(feedback)
                    if settling == "succeeded":
                        # 成功迁移与取消判定在同一临界区完成：若期间到达取消请求，
                        # 则按取消/停止终止，绝不把已取消的目标报成成功。
                        completion = self._command_gate.complete_with_reason(
                            token,
                            lambda: goal_handle.is_cancel_requested,
                            self._hold_current_position,
                            goal_handle.succeed,
                        )
                        if completion is not GateOutcome.SUCCEEDED:
                            return self._terminate_goal(goal_handle, result, completion)
                        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                        result.error_string = "simulation trajectory finished"
                        return result
                    if settling == "timed_out":
                        # 超时视为失败：先软停并保持当前位置，再以容差违规错误码
                        # 上报，便于上层区分"没走到位"与"轨迹本身非法"。
                        self._command_gate.stop_and_hold(self._hold_current_position)
                        goal_handle.abort()
                        result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                        result.error_string = "simulation goal did not settle within configured tolerance"
                        return result
                    time.sleep(self._execute_wait_sec)
                # 退出 while 说明 rclpy 已停止（进程正在关闭）。
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = "simulation shutting down"
                return result
            except Exception as exc:
                self.get_logger().error(
                    f"trajectory execution exception: {type(exc).__name__}: {exc}"
                )
                # 兜底清理：先保持当前位置，再在句柄仍活动时 abort，最后一定会
                # 释放 token（见 ExecutionLifecycle），避免异常后节点永久 busy。
                self._lifecycle.fail(
                    token,
                    self._hold_current_position,
                    lambda: goal_handle.abort() if getattr(goal_handle, "is_active", True) else None,
                )
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = "simulation trajectory execution failed"
                return result
            finally:
                # 正常/取消/异常路径都从这里释放执行权；finish 只在 token 匹配时生效，
                # 因此不会误释放后续目标。
                self._active.finish(token)

        def _timer_callback(self) -> None:
            """固定周期回调：步进物理、发布时钟/关节/夹爪状态并驱动虚拟相机。

            物理步数与发布周期对齐（见 _steps_per_tick），因此仿真时间大致与墙钟
            同步；本回调在互斥回调组中执行，不会与自身重入。
            """
            state = self._sim_access.run(lambda sim: sim.step(self._steps_per_tick))
            # 先发布 /clock：使用 use_sim_time 的订阅端据此驱动自己的定时器，
            # 因此时钟必须先于同批次的状态消息发出。
            stamp = Clock()
            seconds, nanoseconds = self._stamp.update(state.simulation_time)
            stamp.clock.sec = seconds
            stamp.clock.nanosec = nanoseconds
            self._clock_pub.publish(stamp)

            # 关节状态包含八个关节：joint1..joint6 单位 rad；effort 取执行器输出力
            # （手臂为 N·m，直线手指关节为 N），供上层监控但不可当作真机测量值。
            joint = JointState()
            joint.header.stamp = stamp.clock
            joint.name = list(state.joint_names)
            joint.position = list(state.joint_positions)
            joint.velocity = list(state.joint_velocities)
            joint.effort = list(state.actuator_forces)
            self._joint_pub.publish(joint)

            # 夹爪状态：position 承载开口宽度（m，0 = 完全闭合），velocity 不建模固定为 0；
            # torque 用两个手指执行器出力绝对值之和近似夹持力（N）；status_code 固定 0
            # 表示"正常"，不代表真机意义的电机使能位。
            gripper = JointMotorState()
            gripper.header.stamp = stamp.clock
            gripper.joint_name = "gripper"
            gripper.position = float(state.gripper_width)
            gripper.velocity = 0.0
            gripper.torque = float(sum(abs(v) for v in state.actuator_forces[-2:]))
            gripper.status_code = 0
            self._gripper_pub.publish(gripper)


        def destroy_node(self):
            """释放动作服务端与仿真实例。"""
            self._action_server.destroy()
            self._sim_access.run(lambda sim: sim.close())
            return super().destroy_node()

    return RebotArmMujocoNode


def main(args=None) -> None:
    """进程入口：创建节点并驱动执行器（可选同步 MuJoCo 被动查看器）。

    线程数取 3：定时器回调、动作执行回调、以及动作/服务回调各占一路，避免
    长时间运行的执行回调把步进定时器饿死。开启查看器时，MuJoCo 的窗口循环
    必须占用主线程，因此把执行器放到后台线程，主线程只做 viewer.sync()。
    """
    import importlib
    import rclpy
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor

    rclpy.init(args=args)
    node = create_node_class()()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    executor_thread = None
    viewer = None
    try:
        if node.show_viewer:
            # 被动查看器直接复用仿真内的 model/data 句柄，不做物理推进（由定时器负责）。
            model, data = node.viewer_handles()
            launch_passive = importlib.import_module("mujoco.viewer").launch_passive
            viewer = launch_passive(model, data)
            executor_thread = threading.Thread(
                target=executor.spin,
                name="rebotarm-mujoco-ros-executor",
                daemon=True,
            )
            executor_thread.start()
            # 主线程以约 100 Hz 同步渲染画面，直到窗口关闭或收到关闭信号。
            while rclpy.ok() and viewer.is_running():
                node.sync_viewer(viewer)
                time.sleep(0.01)
            return
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl-C 与外部关闭都按正常退出处理，不打印堆栈。
        pass
    finally:
        # 关闭顺序：查看器 → 执行线程 → 执行器 → 节点 → rclpy，确保没有线程
        # 仍在访问已被销毁的节点或仿真对象。
        if viewer is not None:
            viewer.close()
        if executor_thread is not None:
            executor_thread.join(timeout=2.0)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

"""硬件管理核心：真实电机的唯一访问点与最后一道执行安全。

本模块属于硬件包，向上层（服务/动作/话题适配层）暴露一个
``HardwareManager``；它独占厂商控制库的机械臂实例、串口/总线通道与
唯一的配置频率硬件控制循环，是整条链路中唯一真正向电机写命令的地方。

主要职责：
- 连接/断开、使能/失能（真机默认失能，必须显式 enable 才能运动）；
- 反馈采集与校验：位置/速度/力矩、状态码、软限位、时效性、接收序号；
- 位置-速度（pos_vel）轨迹跟踪、safe_home、trajectory_stop 的落地点；
- 夹爪的位置移动、抓取闭合与有界保持；
- 重力补偿（MIT 模式 + 广义重力前馈）与低层 JointMotorCmd 直通；
- 拒绝不安全的低层命令：未连接/未使能、反馈过期、越限、状态码异常。

并发的关键约定：
- 电机命令与反馈读写共享同一条总线，因此所有写操作都通过
  ``_patch_controller_bus`` / ``_wrap_motor_bus`` 打上的总线重入锁串行化；
- ``_motor_lifecycle_lock`` 保护连接/使能/失能/置零等生命周期迁移，
  与运动命令互斥；
- ``_gripper_lock`` 保护夹爪状态机；``_feedback_lock`` 保护已验证反馈样本。

安全约束（务必保持）：
- 真机启动后处于失能态，只有新鲜反馈 + 显式使能才允许运动；
- 反馈过期（超过 ``_gripper_feedback_stale_timeout_sec``）或状态码异常时，
  控制循环会执行保护性失能；
- 任何安全校验失败都必须抛出异常，不得静默降级为"继续运动"。
"""

from __future__ import annotations

from dataclasses import dataclass
import logging
import threading
import time
import sys
from pathlib import Path
from typing import Optional, Sequence

import numpy as np
import yaml

from .conversions import fk_to_pose
from .gravity_control import (GravitySettings, LoopTiming, bounded_mit_command,
                              load_gravity_model, package_file, vector)


class GravityCommandError(RuntimeError):
    """Motor command transport failed; holding torque cannot be trusted."""

_LOG = logging.getLogger(__name__)

_G_MAX_DIST_M = 0.09
# 结构设计/软件标称行程为 90 mm，但当前安装的夹爪在齿轮损坏后，经操作员确认
# 的机械极限只有 85 mm。保留上面的映射量程用于反馈解释，但绝不下发超过真实
# 硬件极限的命令。
_G_VERIFIED_OPEN_LIMIT_M = 0.085
# 全开位置对应的电机原始角度（rad，负方向为张开）。位置-角度线性映射的另一端。
_G_ANGLE_OPEN = -5.0
# DM4310 的 16 位反馈覆盖 +/-12.5 rad。这里给张开端点与置零验证留两个量化步长，
# 它是"反馈有效性"容差，不是运动到位容差。
_G_COORDINATE_TOL_RAD = 2.0 * 25.0 / 65535.0
# 操作员批准的 1 mm 闭合反馈容差（换算成电机角），不是命令容差或置零余量。
_G_CLOSED_FEEDBACK_TOL_RAD = 0.001 * abs(_G_ANGLE_OPEN) / _G_MAX_DIST_M
_G_ZERO_VERIFY_TIMEOUT_SEC = 0.5
_G_ZERO_VERIFY_SAMPLES = 3
_G_OPEN_SOFT_LIMIT = -4.9
_G_ARRIVE_TOL = 0.12
_G_TAU_MAX = 1.5
# 常规位置移动使用有界的位置斜坡命令。默认 0.5 rad/s 与既有 Web/键盘遥操作
# 速度基线一致；它仍是独立的夹爪参数，以便两条机构分别整定。
_G_POSITION_MAX_SPEED_RAD_S = 0.5
_G_POSITION_MAX_SPEED_MIN_RAD_S = 0.05
_G_POSITION_MAX_SPEED_MAX_RAD_S = 3.0
_G_POSITION_TIMEOUT_MARGIN_SEC = 1.5
_G_POSITION_TIMEOUT_MARGIN_MAX_SEC = 10.0
_G_FEEDBACK_STALE_TIMEOUT_SEC = 0.15
_G_FEEDBACK_STALE_TIMEOUT_MAX_SEC = 2.0
_HARDWARE_FEEDBACK_RATE_HZ = 50.0
_HARDWARE_FEEDBACK_RATE_MIN_HZ = 20.0
_HARDWARE_FEEDBACK_RATE_MAX_HZ = 100.0
# 下面两个是 MOVE 阶段专用的电机侧力矩上限；到位后防止继续顶住机械限位的，
# 是 _release_gripper_position_target 里的"中性 + 空闲"释放。
#
# 2026-08-14：齿轮修复后在真机上实测。0.40 N.m 上限下，0 -> 10 -> 0 mm 空载
# 循环三段分别在 7.86-8.02 mm 与 1.83-1.93 mm 处堵转；三段都需要
# kp*err = 0.536-0.594 N.m，即都是撞到上限饱和而非到达目标，留下约 2 mm
# 稳态误差，仅以最小 0.02 mm 的余量通过 _G_ARRIVE_TOL。可见传动阻力确实很大。
# 2026-09-04 操作员决定：采用 Web 遥操作已验证可用的默认值 1.0 N.m，仅把
# 1.5 N.m 保留为可配置上限，并用上面的 0.5 rad/s 斜坡限制运动速度。
_G_LARGE_MOVE_MAX_TAU_NM = 1.0
_G_LARGE_MOVE_MAX_TAU_CAP_NM = 1.5
_G_KP_MOVE = 5.0
_G_KD_MOVE = 1.0
_G_DEFAULT_FORCE = 0.40
_G_GRASP_CLOSE_KP = 0.0
_G_GRASP_CLOSE_KD = 0.5
_G_GRASP_HOLD_KP = 5.0
_G_GRASP_HOLD_KD = 1.0
# 关于夹持力的说明：grasp_holding 下发的是
#   kp*(hold_angle - pos) + kd*(-vel) + hold_force
# 其中 hold_angle 冻结在堵转位置，因此位置项占主导，hold_force 只是前馈叠加。
# 实际夹持力取决于闭合阶段 close_force 把夹爪推到多深才堵转，而不是 hold_force。
# 2026-08-14 真机、两次 hold_force 均为 0.4：
#   close_force 0.4 -> 推进 2.02 mm 后堵转（50 mm 瓶子），保持 0.271-0.286 N.m
#                      （10 s 后仍在缓慢闭合；夹不住）
#   close_force 1.0 -> 推进 15.21 mm（压缩 30%），保持 0.423-0.437 N.m
#                      （弹性回弹 0.158 mm；夹住了）
# 整定时不要把 hold_force 当成夹持力旋钮。
_G_GRASP_CLOSE_FORCE_DEFAULT = 0.40
_G_GRASP_CLOSE_FORCE_MAX = 1.0
_G_GRASP_HOLD_FORCE_DEFAULT = 0.40
# grasp_holding 会在控制循环中持续加载电机。上游从不退出该状态，因此一次成功
# 抓取会把力矩一直顶到下一条命令或控制器关闭——这正是 2026-08-12 Joint4 过温
# 所涉及的持续加载机理。故保持必须有时限，且总以中性释放收尾。
_G_GRASP_HOLD_TIMEOUT_DEFAULT_SEC = 30.0
_G_GRASP_HOLD_TIMEOUT_MAX_SEC = 120.0
_G_GRASP_VEL_THRESHOLD = 0.04
_G_GRASP_MIN_CLOSE_TIME = 0.08
_G_GRASP_MIN_CLOSURE_M = 0.006
_G_GRASP_TIMEOUT = 2.0
_G_CTRL_RATE = 500.0
_GC_VEL_THRESHOLD = 0.04
_GC_W_VEL_THRESHOLD = 0.08
_GC_EE_FRAME = "end_link"
_GC_KP = 7.0
_GC_KD = 0.8
_GC_TAU_SCALE = np.ones(6, dtype=np.float64)
_FEEDBACK_REFRESH_RETRIES = 3
# DM 串口桥可能分多个轮询周期才把六帧电机反馈取回。5 ms 短于实测的帧节拍，
# 会把首帧晚于 joint1/joint2 到达的健康关节误判为失败。
_FEEDBACK_RETRY_INTERVAL_SEC = 0.05
_UINT64_MAX = (1 << 64) - 1
_UINT64_HALF_RANGE = 1 << 63

# 各关节硬件软限位（rad）。这是"接受反馈"的合法性区间，同时用于 safe_home 目标
# 校验；超出即视为反馈异常并触发保护逻辑，而不是放宽到 URDF 的完整量程。
_JOINT_POSITION_LIMITS_RAD = {
    "joint1": (-2.8, 2.8),
    # joint2 的标称上端点即已确认的机械零点。这里保留很小的正向软件余量以吸收
    # 编码器量化/回差，并不是额外的可命令工作空间。
    "joint2": (-3.14, 0.02),
    # 操作员批准的 J3 上余量，与 J2 相同；不施加任何编码器偏置。
    "joint3": (-3.14, 0.02),
    "joint4": (-1.87, 1.57),
    "joint5": (-1.57, 1.57),
    "joint6": (-3.14, 3.14),
}

# 厂商 ArmEndPos.safe_home 会把每个关节都驱到 0 rad。对本机械臂而言那是"折叠"
# 姿态而非张开姿态：joint3 原点在 x 方向为 -0.264 m，joint4 为 +0.2426 m，
# 因此当 2-5 关节接近零时，前臂会折回贴上臂，link2/link5 间距缩到约 1.6 mm。
# MoveIt 的 CheckStartStateCollision 随后会否决从该处发出的每一个规划请求
# （"1 contact(s) detected : link2 - link5"），即零位不可用于规划，也就无法作为
# 停放姿态。这里改为停在"视觉就绪"姿态：link2/link5 实测间隙 33.2 mm，且机械
# 上与零位同样是紧凑站姿（end_link 水平伸展 0.254 m 对 0.260 m，高 24 mm），
# 因此不增加机械代价，也不会让手臂在重力下长期外伸。该姿态同时是
# tests/test_paired_trajectory_protocol.py 视为标准工作位姿的姿态，并与 MoveIt
# SRDF 中的 "safe_home" 命名状态以及 MuJoCo keyframe 一致。
_SAFE_HOME_JOINT_POSITIONS = (0.0, 0.0, -0.017453292519943295, 0.0, 0.0, 0.0)
# safe_home 到位判据：所有关节残差的最大值需小于该阈值（rad）。
_SAFE_HOME_ARRIVE_TOL_RAD = 0.02
_SAFE_HOME_TIMEOUT_SEC = 30.0

# 生命周期状态集合。真机路径固定按
# DISCONNECTED -> CONNECTED_DISABLED -> ENABLING -> ENABLED_HOLD
# ->（TRAJECTORY_RUNNING）-> DISABLING 迁移；没有"自动使能"的捷径。
_LIFECYCLE_STATES = {
    "DISCONNECTED",
    "CONNECTED_DISABLED",
    "ENABLING",
    "ENABLED_HOLD",
    "TRAJECTORY_RUNNING",
    "DISABLING",
}


@dataclass(frozen=True)
class _VerifiedFeedbackSample:
    """一条已验证的电机反馈快照（不可变）。

    ``state`` 是厂商 SDK 的状态对象（pos/vel/torq/status_code），``sequence``
    是该电机反馈帧的接收序号（uint64），``observed_at`` 是本地单调时钟的观测
    时刻。序号用于证明"这是一帧新反馈"而非缓存旧值；时间戳用于时效性判断。
    """

    state: object
    sequence: int
    observed_at: float


def apply_gravity_compensation_tau_scale(tau: np.ndarray) -> np.ndarray:
    """对广义重力力矩逐关节施加补偿增益。

    参数 ``tau``：形状为 (6,) 的关节力矩（N.m）。返回值是副本，形状匹配
    ``_GC_TAU_SCALE`` 时按增益缩放，否则原样返回（调用方不应依赖就地修改）。
    """
    scaled = np.array(tau, dtype=np.float64, copy=True)
    if scaled.shape == _GC_TAU_SCALE.shape:
        scaled *= _GC_TAU_SCALE
    return scaled


class HardwareManager:
    """持有 ROS 驱动所用的唯一 RobotArm 实例。

    生命周期：``connect()`` -> ``enable()`` -> 运动/夹爪命令 -> ``disable()``
    -> ``shutdown()``。构造阶段只解析配置与参数上下限、把厂商 SDK 加入模块
    搜索路径，并给控制器/电机打上总线锁；真正的串口打开发生在 ``connect()``。

    并发的回调模型：使能与运动都跑在厂商的唯一控制线程（``start_control_loop``
    启动）里，该线程是总线上唯一的写入者；ROS 定时器线程只允许读取已缓存的
    已验证反馈（``refresh_feedback_if_due`` 会拒绝在循环运行期间做同步刷新）。
    因此任何跨线程状态都由 ``_motor_lifecycle_lock`` / ``_gripper_lock`` /
    ``_feedback_lock`` 保护。
    """

    def __init__(
        self,
        arm_cfg: Optional[str] = None,
        gripper_cfg: Optional[str] = None,
        channel: str = "",
        hardware_feedback_rate_hz: float = _HARDWARE_FEEDBACK_RATE_HZ,
        gripper_position_torque_cap_nm: float = _G_LARGE_MOVE_MAX_TAU_NM,
        gripper_position_max_speed_rad_s: float = _G_POSITION_MAX_SPEED_RAD_S,
        gripper_position_timeout_margin_sec: float = _G_POSITION_TIMEOUT_MARGIN_SEC,
        gripper_feedback_stale_timeout_sec: float = _G_FEEDBACK_STALE_TIMEOUT_SEC,
        grasp_hold_timeout_sec: float = _G_GRASP_HOLD_TIMEOUT_DEFAULT_SEC,
        gravity_config: Optional[str] = None,
    ) -> None:
        requested_feedback_rate = float(hardware_feedback_rate_hz)
        if not (
            _HARDWARE_FEEDBACK_RATE_MIN_HZ
            <= requested_feedback_rate
            <= _HARDWARE_FEEDBACK_RATE_MAX_HZ
        ):
            raise ValueError(
                "hardware_feedback_rate_hz must be within "
                f"[{_HARDWARE_FEEDBACK_RATE_MIN_HZ:g}, "
                f"{_HARDWARE_FEEDBACK_RATE_MAX_HZ:g}] Hz"
            )
        self._hardware_feedback_rate_hz = requested_feedback_rate
        self._hardware_feedback_period_sec = 1.0 / requested_feedback_rate
        requested_cap = float(gripper_position_torque_cap_nm)
        if not 0.05 <= requested_cap <= _G_LARGE_MOVE_MAX_TAU_CAP_NM:
            raise ValueError(
                "gripper_position_torque_cap_nm must be within "
                f"[0.05, {_G_LARGE_MOVE_MAX_TAU_CAP_NM:g}] N.m"
            )
        self._gripper_position_torque_cap_nm = requested_cap
        requested_speed = float(gripper_position_max_speed_rad_s)
        if not _G_POSITION_MAX_SPEED_MIN_RAD_S <= requested_speed <= _G_POSITION_MAX_SPEED_MAX_RAD_S:
            raise ValueError(
                "gripper_position_max_speed_rad_s must be within "
                f"[{_G_POSITION_MAX_SPEED_MIN_RAD_S:g}, "
                f"{_G_POSITION_MAX_SPEED_MAX_RAD_S:g}] rad/s"
            )
        self._gripper_position_max_speed_rad_s = requested_speed
        requested_timeout_margin = float(gripper_position_timeout_margin_sec)
        if not 0.1 <= requested_timeout_margin <= _G_POSITION_TIMEOUT_MARGIN_MAX_SEC:
            raise ValueError(
                "gripper_position_timeout_margin_sec must be within "
                f"[0.1, {_G_POSITION_TIMEOUT_MARGIN_MAX_SEC:g}] s"
            )
        self._gripper_position_timeout_margin_sec = requested_timeout_margin
        requested_stale_timeout = float(gripper_feedback_stale_timeout_sec)
        if not 0.05 <= requested_stale_timeout <= _G_FEEDBACK_STALE_TIMEOUT_MAX_SEC:
            raise ValueError(
                "gripper_feedback_stale_timeout_sec must be within "
                f"[0.05, {_G_FEEDBACK_STALE_TIMEOUT_MAX_SEC:g}] s"
            )
        self._gripper_feedback_stale_timeout_sec = requested_stale_timeout
        requested_hold_timeout = float(grasp_hold_timeout_sec)
        if not 0.1 <= requested_hold_timeout <= _G_GRASP_HOLD_TIMEOUT_MAX_SEC:
            raise ValueError(
                "grasp_hold_timeout_sec must be within "
                f"[0.1, {_G_GRASP_HOLD_TIMEOUT_MAX_SEC:g}] s"
            )
        self._grasp_hold_timeout_sec = requested_hold_timeout
        self._sdk_root = self._ensure_rebot_sdk_in_syspath()

        from reBotArm_control_py.actuator import RobotArm
        from reBotArm_control_py.controllers import ArmEndPos
        import pinocchio as pin

        settings_path = (Path(gravity_config).expanduser() if gravity_config else
                         package_file('rebotarmcontroller', 'config/gravity_compensation.yaml'))
        self._gc_settings = GravitySettings.from_yaml(settings_path)
        self._gravity_model = load_gravity_model(
            package_file('rebotarm_moveit_config', 'config/rebotarm.urdf'), self._gc_settings)
        self._gc_torque_limits = self._gravity_model.effort
        cfg_path = Path(arm_cfg).expanduser() if arm_cfg else self.default_arm_cfg()
        cfg_path = self._arm_cfg_with_channel(cfg_path, channel)
        self._arm = RobotArm(cfg_path=str(cfg_path))
        # 重力补偿用的动力学模型：pinocchio 模型 + 数据缓冲 + 末端帧 id，
        # 计算函数和数据缓冲来自同一个权威模型，不再使用 SDK 的另一份 URDF。
        self._gc_model = self._gravity_model.model
        self._gc_data = self._gc_model.createData()
        self._gc_ee_frame_id = self._gc_model.getFrameId(_GC_EE_FRAME)
        self._gc_compute_generalized_gravity = self._gravity_model.gravity
        self._gc_pin = pin

        self._gripper_cfg_path = (
            Path(gripper_cfg).expanduser() if gripper_cfg else self.default_gripper_cfg()
        )
        self._gripper_cfg = None
        self._gripper_mot = None
        self._gripper_ctrl = None
        # 夹爪角度语义：0 rad = 完全闭合，_G_ANGLE_OPEN = 完全张开；
        # _gripper_target_angle 是斜坡中间值，_gripper_goal_angle 是最终目标。
        self._gripper_target_angle = 0.0
        self._gripper_goal_angle = 0.0
        self._gripper_target_effort = _G_DEFAULT_FORCE
        self._gripper_close_force = _G_GRASP_CLOSE_FORCE_DEFAULT
        self._gripper_hold_force = _G_GRASP_HOLD_FORCE_DEFAULT
        self._gripper_hold_angle = 0.0
        self._gripper_hold_deadline: float | None = None
        self._gripper_hold_release_reason: str | None = None
        self._gripper_mode = "idle"
        self._gripper_active = False
        self._gripper_pos = 0.0
        self._gripper_vel = 0.0
        self._gripper_torque = 0.0
        # 255 是"状态未知/无效"的哨兵值，不是电机上报的状态码。
        self._gripper_status_code = 255
        self._gripper_feedback_updated_monotonic: float | None = None
        self._gripper_feedback_error: str | None = "gripper feedback not received"
        self._gripper_zero_error: str | None = None
        self._gripper_command_error: str | None = None
        self._gripper_position_result = "idle"
        # 待发送的中性命令：(角度, 原因, 是否标记成功)，由硬件循环消费。
        self._gripper_neutral_pending: tuple[float, str, bool] | None = None
        self._gripper_target_timeout_sec = 0.0
        self._gripper_target_deadline_monotonic: float | None = None
        # 机械臂与夹爪命令共用厂商的同一个硬件循环。以下兼容字段只保留可观测性
        # 用于诊断，不会创建独立的夹爪线程。
        self._gripper_loop_thread: None = None
        self._gripper_loop_running = False
        self._gripper_last_tick_monotonic: float | None = None
        self._gripper_lock = threading.RLock()
        self._feedback_lock = threading.RLock()
        self._feedback_next_refresh_monotonic: float | None = None
        self._verified_feedback_by_label: dict[str, _VerifiedFeedbackSample] = {}
        # 请求-响应式反馈验证状态：baseline 记录请求前的接收序号，deadline 是
        # 该次请求的截止时刻，只有序号前进才算收到新帧。
        self._feedback_request_baseline_by_label: dict[str, int] = {}
        self._feedback_request_deadline_by_label: dict[str, float] = {}
        self._feedback_error_by_label: dict[str, str] = {}
        # 启动时的强制刷新可能要跨多个轮询周期才收齐电机帧；在这个有界收集窗口
        # 内不要发布中间态的 "pending" 错误。
        self._feedback_force_refresh_active = False
        self._arm_feedback_updated_monotonic: float | None = None
        self._arm_feedback_error: str | None = "arm feedback not received"
        self._motor_lifecycle_lock = threading.RLock()

        self._endpos_ctrl = ArmEndPos(self._arm)
        self._connected = False
        self._enabled = False
        self._lifecycle_state = "DISCONNECTED"
        self._state_machine = "IDLE"
        self._error_codes: list[str] = []
        # 重力补偿运行态：目标关节角、误差积分（抗漂移）、锁定计数与上一帧角度。
        self._gravity_comp_active = False
        self._gravity_comp_q_target: np.ndarray | None = None
        self._gravity_comp_integral: np.ndarray | None = None
        self._gravity_comp_lock_counter = 0
        self._gravity_comp_q_last: np.ndarray | None = None
        self._gravity_comp_last_total = np.zeros(6)
        self._gravity_comp_fault: str | None = None
        self._gravity_comp_hold_feedforward = None
        self._gravity_timing = None
        self._gravity_feedback_metrics = {}
        self._gravity_next_log = 0.

        self._patch_arm_bus_lock()

    def default_arm_cfg(self) -> Path:
        return self._sdk_root / "config" / "arm.yaml"

    def default_gripper_cfg(self) -> Path:
        return self._sdk_root / "config" / "gripper.yaml"

    @staticmethod
    def _workspace_root() -> Path:
        # 本文件位于 <workspace>/src/rebotarmcontroller/rebotarmcontroller/，
        # 因此 parents[3] 即工作空间根目录。
        return Path(__file__).resolve().parents[3]

    @classmethod
    def _sdk_candidates(cls) -> list[Path]:
        """厂商 SDK 的候选根目录（按优先级排列，含开发机历史路径）。"""
        workspace = cls._workspace_root()
        return [
            workspace / "third_party" / "reBotArm_control_py",
            workspace / "sdk" / "reBotArm_control_py",
            Path.cwd() / "third_party" / "reBotArm_control_py",
            Path.cwd() / "sdk" / "reBotArm_control_py",
            Path.home() / "robotarm_ros2" / "third_party" / "reBotArm_control_py",
            Path.home() / "robotarm_ros2" / "sdk" / "reBotArm_control_py",
            Path.home() / "seeed" / "cameraws" / "sdk" / "reBotArm_control_py",
        ]

    @classmethod
    def _ensure_rebot_sdk_in_syspath(cls) -> Path:
        """把厂商 SDK 根目录插入 sys.path 并返回该目录。

        插入位置为 0，确保本地 SDK 优先于任何同名安装包；找不到时抛出
        FileNotFoundError 并列出全部候选路径，便于现场排查。
        """
        for root in cls._sdk_candidates():
            if (root / "reBotArm_control_py").is_dir():
                root_str = str(root)
                if root_str not in sys.path:
                    sys.path.insert(0, root_str)
                return root
        candidates = "\n".join(f"  - {path}" for path in cls._sdk_candidates())
        raise FileNotFoundError(
            "Cannot find reBotArm_control_py. Clone it into one of:\n"
            f"{candidates}"
        )

    @staticmethod
    def _arm_cfg_with_channel(cfg_path: Path, channel: str) -> Path:
        """按需把串口通道写进机械臂配置，返回实际使用的配置文件路径。

        ``channel`` 为空或为 "auto" 时原样返回原配置（由厂商自动选择）；否则把
        ``channel`` 键覆盖后写到 /tmp 下的临时 YAML，避免改动仓库内的配置。
        """
        normalized_channel = str(channel or "").strip()
        if not normalized_channel or normalized_channel.lower() == "auto":
            return cfg_path
        with open(cfg_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        data["channel"] = normalized_channel
        tmp_dir = Path("/tmp") / "rebotarm_ros2"
        tmp_dir.mkdir(parents=True, exist_ok=True)
        tmp_path = tmp_dir / "arm_channel_override.yaml"
        with open(tmp_path, "w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, sort_keys=False)
        return tmp_path

    @property
    def arm(self):
        return self._arm

    @property
    def endpos_ctrl(self):
        return self._endpos_ctrl

    @property
    def joint_names(self) -> list[str]:
        return list(self._arm.joint_names)

    @property
    def mode(self) -> str:
        return str(self._arm.mode)

    @property
    def enabled(self) -> bool:
        return self._enabled

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def control_loop_active(self) -> bool:
        return bool(self._arm.control_loop_active)

    @property
    def has_gripper(self) -> bool:
        return self._gripper_mot is not None

    @property
    def state_machine(self) -> str:
        return self._state_machine

    @property
    def lifecycle_state(self) -> str:
        return self._lifecycle_state

    @property
    def ready_for_motion(self) -> bool:
        # 只有"已连接 + 已使能 + 位于保持或轨迹执行状态"才允许运动；
        # 上层必须以此为准，不得只看 enabled。
        return bool(
            self._connected
            and self._enabled
            and self._lifecycle_state in ("ENABLED_HOLD", "TRAJECTORY_RUNNING")
        )

    @property
    def gripper_active(self) -> bool:
        with self._gripper_lock:
            return bool(self._gripper_active)

    @property
    def gripper_mode(self) -> str:
        with self._gripper_lock:
            return str(self._gripper_mode)

    @property
    def error_codes(self) -> list[str]:
        """累计错误码 + 当前反馈健康状态派生出的错误项。

        反馈类错误是在读取时按最新时效性实时派生的，不会写入 ``_error_codes``；
        这样"反馈恢复"后无需清理历史即可自动消失。
        """
        codes = list(self._error_codes)
        arm_feedback_failure = self._arm_feedback_failure_reason()
        if arm_feedback_failure is not None:
            codes.append(f"ARM_FEEDBACK: {arm_feedback_failure}")
        if self._gripper_mot is not None:
            with self._gripper_lock:
                gripper_feedback_failure = (
                    self._gripper_feedback_failure_reason_locked()
                )
            if gripper_feedback_failure is not None:
                codes.append(f"GRIPPER_FEEDBACK: {gripper_feedback_failure}")
        return codes

    def set_state_machine(self, state: str) -> None:
        """设置对外可见的子系统状态机，并在必要时同步生命周期状态。

        TRAJ_RUNNING 需要已使能才会切到 TRAJECTORY_RUNNING；回到 IDLE 且已使能
        时切到 ENABLED_HOLD（即"使能但保持当前位置"）。
        """
        if state not in ("IDLE", "TRAJ_RUNNING", "LOWLEVEL_STREAMING", "GRAVITY_COMP"):
            raise ValueError(f"unsupported state machine value: {state}")
        self._state_machine = state
        if state == "TRAJ_RUNNING" and self._enabled:
            self._set_lifecycle_state("TRAJECTORY_RUNNING")
        elif state == "IDLE" and self._enabled:
            self._set_lifecycle_state("ENABLED_HOLD")

    def _set_lifecycle_state(self, state: str) -> None:
        if state not in _LIFECYCLE_STATES:
            raise ValueError(f"unsupported lifecycle state: {state}")
        self._lifecycle_state = state

    def _require_connected(self) -> None:
        if not self._connected:
            raise RuntimeError("hardware is not connected")

    def _require_enabled(self) -> None:
        """运动类命令的统一安全门：未连接或未显式使能一律拒绝。"""
        self._require_connected()
        if not self._enabled:
            raise RuntimeError(
                "hardware is disabled; call explicit /rebotarm/enable first"
            )

    def _refresh_all_feedback(self) -> None:
        self.refresh_feedback_if_due(force=True)

    def _feedback_controller_groups(self):
        """把关节与夹爪按所属控制器分组，返回 [(ctrl, [(label, motor), ...]), ...]。

        同一控制器下的电机共享一条总线，因此每个控制器只需一次
        request_feedback + poll_feedback_once 事务即可取回其全部电机反馈。
        缺少电机/控制器映射时抛错，宁可失败也不要静默漏采反馈。
        """
        groups: list[tuple[object, list[tuple[str, object]]]] = []

        def add(ctrl, label: str, motor) -> None:
            for existing_ctrl, entries in groups:
                if existing_ctrl is ctrl:
                    entries.append((label, motor))
                    return
            groups.append((ctrl, [(label, motor)]))

        ctrl_map = getattr(self._arm, "_ctrl_map", {})
        motor_map = getattr(self._arm, "_motor_map", {})
        for joint in getattr(self._arm, "_joints", []):
            ctrl = ctrl_map.get(getattr(joint, "vendor", None))
            motor = motor_map.get(joint.name)
            if motor is None:
                raise RuntimeError(f"{joint.name} feedback motor unavailable")
            if ctrl is None:
                raise RuntimeError(f"{joint.name} feedback controller unavailable")
            add(ctrl, joint.name, motor)
        if self._gripper_mot is not None and self._gripper_ctrl is None:
            raise RuntimeError("gripper feedback controller unavailable")
        if self._gripper_mot is not None:
            add(self._gripper_ctrl, "gripper", self._gripper_mot)
        if not groups:
            raise RuntimeError("hardware feedback controller map unavailable")
        return groups

    @staticmethod
    def _feedback_state_with_sequence(label: str, motor) -> tuple[object, int]:
        """读取 (状态, 接收序号)；依赖打过补丁的 MotorBridge 提供序号接口。

        没有 ``get_state_with_sequence`` 就无法证明反馈是"新帧"，因此在采集路径
        上直接报错，而不是退化为读缓存。
        """
        getter = getattr(motor, "get_state_with_sequence", None)
        if not callable(getter):
            raise RuntimeError(
                f"{label} feedback requires patched MotorBridge "
                "get_state_with_sequence()"
            )
        state, sequence_value = getter()
        sequence = int(sequence_value)
        if not 0 <= sequence <= _UINT64_MAX:
            raise RuntimeError(f"{label} feedback sequence outside uint64 range")
        return state, sequence

    @staticmethod
    def _feedback_sequence_advanced(sequence: int, baseline: int) -> bool:
        """判断接收序号是否相对基线"向前"推进，可容忍 uint64 回绕。

        采用模 2^64 差值：差值落在 (0, 2^63) 视为推进，落在 (2^63, 2^64) 视为
        落后（回绕的另一半）。序号从 0 且基线非 0 时判为未推进——那是计数器
        复位，不能当作新帧。
        """
        current = int(sequence)
        previous = int(baseline)
        if not 0 <= current <= _UINT64_MAX or not 0 <= previous <= _UINT64_MAX:
            raise RuntimeError("feedback sequence outside uint64 range")
        if current == 0 and previous != 0:
            return False
        delta = (current - previous) & _UINT64_MAX
        return 0 < delta < _UINT64_HALF_RANGE

    def _verified_feedback_sample(self, label: str) -> _VerifiedFeedbackSample:
        """返回一条不可变的、由控制循环写入的已验证反馈快照。

        读 MotorBridge 缓存只属于采集路径。消费者在 ``_feedback_lock`` 下取用
        已经验证过的样本；读取它不得改变其接收序号或观测时间。
        """
        with self._feedback_lock:
            sample = self._verified_feedback_by_label.get(label)
        if sample is None:
            raise RuntimeError(f"{label} verified feedback unavailable")
        return sample

    def _verified_feedback_samples(
        self,
        labels: Sequence[str],
    ) -> tuple[_VerifiedFeedbackSample, ...]:
        # 一次性原子读取多个标签，避免同一控制周期内取到不同批次的数据；
        # 任一标签缺失即整体失败（多关节命令不能基于部分反馈执行）。
        with self._feedback_lock:
            samples = tuple(
                self._verified_feedback_by_label.get(label) for label in labels
            )
        for label, sample in zip(labels, samples):
            if sample is None:
                raise RuntimeError(f"{label} verified feedback unavailable")
        return samples  # type: ignore[return-value]

    def _feedback_response_window_sec(self) -> float:
        """一次反馈请求允许的响应窗口：取两种节拍假设与重试次数的较大者。"""
        return max(
            self._hardware_feedback_period_sec * _FEEDBACK_REFRESH_RETRIES,
            _FEEDBACK_RETRY_INTERVAL_SEC * _FEEDBACK_REFRESH_RETRIES,
        )

    def _validate_feedback_sample(self, label: str, state) -> None:
        """校验单条反馈的合法性：有限数值 + 位于该关节硬件软限位内。

        关节越限或非有限值都说明编码器/通信异常，必须报错，绝不能当作有效
        位置参与运动命令。
        """
        if label == "gripper":
            self._validated_gripper_feedback_values(state)
            return
        if state is None:
            raise RuntimeError(f"{label} feedback unavailable")
        values = (float(state.pos), float(state.vel), float(state.torq))
        if not all(np.isfinite(value) for value in values):
            raise RuntimeError(f"{label} feedback contains non-finite values")
        limits = _JOINT_POSITION_LIMITS_RAD.get(label)
        if limits is None:
            raise RuntimeError(f"no hardware soft limit configured for {label}")
        lower, upper = limits
        if values[0] < lower or values[0] > upper:
            raise RuntimeError(
                f"{label} position {values[0]:.6f} rad outside "
                f"hardware soft limit [{lower:.6f}, {upper:.6f}]"
            )

    def _record_feedback_label_error(self, label: str, reason: str) -> None:
        self._feedback_error_by_label[label] = str(reason)

    def _record_verified_feedback(
        self,
        label: str,
        state,
        sequence: int,
        observed_at: float,
    ) -> None:
        """校验通过后登记为"已验证反馈"，并清除该标签的错误标记。"""
        self._validate_feedback_sample(label, state)
        sample = _VerifiedFeedbackSample(
            state=state,
            sequence=int(sequence),
            observed_at=float(observed_at),
        )
        self._verified_feedback_by_label[label] = sample
        self._feedback_error_by_label.pop(label, None)
        if label == "gripper":
            self._record_gripper_feedback(state, observed_at=observed_at)

    def _sync_feedback_health(self) -> None:
        """把逐标签的反馈状态汇总为机械臂/夹爪级健康状态。

        机械臂侧只要任一关节有错误就报错；缺少样本且不在强制刷新窗口内时标为
        "verified feedback pending"。夹爪只在自己的标签有错误或样本缺失时报错。
        """
        arm_labels = list(self.joint_names)
        arm_errors = [
            self._feedback_error_by_label[label]
            for label in arm_labels
            if label in self._feedback_error_by_label
        ]
        missing_arm = [
            label for label in arm_labels if label not in self._verified_feedback_by_label
        ]
        if arm_errors:
            self._record_arm_feedback_error("; ".join(arm_errors))
        elif missing_arm and not getattr(self, "_feedback_force_refresh_active", False):
            self._record_arm_feedback_error(
                "verified feedback pending: " + ",".join(missing_arm)
            )
        else:
            # 机械臂新鲜度取所有关节中"最旧"的观测时刻，即短板决定整体时效。
            self._record_arm_feedback_success(
                min(
                    self._verified_feedback_by_label[label].observed_at
                    for label in arm_labels
                )
            )

        if self._gripper_mot is None:
            return
        gripper_error = self._feedback_error_by_label.get("gripper")
        if gripper_error is not None:
            self._record_gripper_feedback_error(gripper_error)
        elif "gripper" not in self._verified_feedback_by_label:
            self._record_gripper_feedback_error("gripper feedback not received")

    def _inspect_pending_feedback(
        self,
        observations: dict[str, tuple[object, int]],
        *,
        observed_at: float,
    ) -> None:
        """结算所有未完成的反馈请求：序号前进则登记样本，超时则记录错误。

        每个待决请求只结算一次；无论成功还是超时都会清掉 baseline/deadline，
        避免过期的截止时间影响下一条命令。
        """
        for label, baseline in list(
            self._feedback_request_baseline_by_label.items()
        ):
            if label not in observations:
                continue
            state, sequence = observations[label]
            if self._feedback_sequence_advanced(sequence, baseline):
                try:
                    self._record_verified_feedback(
                        label,
                        state,
                        sequence,
                        observed_at,
                    )
                except Exception as exc:
                    self._record_feedback_label_error(label, f"{label} feedback invalid: {exc}")
                self._feedback_request_baseline_by_label.pop(label, None)
                self._feedback_request_deadline_by_label.pop(label, None)
                continue
            deadline = self._feedback_request_deadline_by_label[label]
            if observed_at >= deadline:
                self._record_feedback_label_error(
                    label,
                    f"{label} feedback deadline expired: sequence did not advance "
                    f"beyond baseline={baseline}",
                )
                self._feedback_request_baseline_by_label.pop(label, None)
                self._feedback_request_deadline_by_label.pop(label, None)

    def _read_feedback_observations(
        self,
        groups,
    ) -> dict[str, tuple[object, int]]:
        observations: dict[str, tuple[object, int]] = {}
        for _ctrl, entries in groups:
            for label, motor in entries:
                observations[label] = self._feedback_state_with_sequence(label, motor)
        return observations

    @staticmethod
    def _require_feedback_sequence_api(groups) -> None:
        """确认全部电机都支持带序号的反馈读取，否则反馈无法被证明为"新帧"。"""
        for _ctrl, entries in groups:
            for label, motor in entries:
                if not callable(getattr(motor, "get_state_with_sequence", None)):
                    raise RuntimeError(
                        f"{label} feedback requires patched MotorBridge "
                        "get_state_with_sequence()"
                    )

    def _record_arm_feedback_success(self, observed_at: float) -> None:
        recovered = self._arm_feedback_error is not None
        self._arm_feedback_updated_monotonic = float(observed_at)
        self._arm_feedback_error = None
        if recovered:
            _LOG.info("arm feedback recovered updated=%.6f", observed_at)

    def _record_arm_feedback_error(self, reason: str) -> None:
        # 只在错误内容变化时打日志，避免硬件回路上刷屏。
        message = str(reason)
        changed = message != self._arm_feedback_error
        self._arm_feedback_error = message
        if changed:
            _LOG.error("arm feedback error: %s", message)

    def _arm_feedback_failure_reason(self, *, now: float | None = None) -> str | None:
        """机械臂反馈的失效原因；健康时返回 None。

        先看是否有显式错误；否则用单调时钟计算反馈年龄，超过
        ``_gripper_feedback_stale_timeout_sec`` 即判定为过期（运动命令必须被拒绝）。
        """
        if self._arm_feedback_error is not None:
            return f"arm feedback unavailable: {self._arm_feedback_error}"
        updated = self._arm_feedback_updated_monotonic
        current = time.monotonic() if now is None else float(now)
        age = float("inf") if updated is None else max(current - updated, 0.0)
        if age > self._gripper_feedback_stale_timeout_sec:
            return (
                "arm feedback stale: "
                f"age={age:.3f}s "
                f"limit={self._gripper_feedback_stale_timeout_sec:.3f}s"
            )
        return None

    def _refresh_feedback_batch(
        self,
        *,
        observed_at: float,
        inspect_after_poll: bool = False,
    ) -> None:
        """执行一轮共享总线反馈事务：先结算旧请求，再请求新帧。

        流程：按控制器分组读一次缓存观测 -> 结算上一轮待决请求 -> 为尚无基线
        的标签建立 baseline/deadline -> 持总线锁执行 request_feedback +
        poll_feedback_once。``inspect_after_poll`` 为真时在轮询结束后立即再读一次
        并结算（用于强制刷新），否则只结算轮询前的观测。任一控制器组失败都会
        记录该组电机错误，并在最后以异常抛出，绝不静默继续。
        """
        groups = self._feedback_controller_groups()
        self._require_feedback_sequence_api(groups)
        observations: dict[str, tuple[object, int]] = {}
        group_errors: list[str] = []
        readable_groups = []
        for ctrl, entries in groups:
            try:
                for label, motor in entries:
                    observations[label] = self._feedback_state_with_sequence(label, motor)
                readable_groups.append((ctrl, entries))
            except Exception as exc:
                labels = ",".join(label for label, _motor in entries)
                message = f"controller={type(ctrl).__name__} motors={labels}: {exc}"
                group_errors.append(message)
                for label, _motor in entries:
                    self._record_feedback_label_error(
                        label, f"shared feedback batch failed before request: {message}"
                    )

        self._inspect_pending_feedback(observations, observed_at=observed_at)
        response_window = self._feedback_response_window_sec()
        for _ctrl, entries in readable_groups:
            for label, _motor in entries:
                if label not in self._feedback_request_baseline_by_label:
                    _state, sequence = observations[label]
                    self._feedback_request_baseline_by_label[label] = sequence
                    self._feedback_request_deadline_by_label[label] = (
                        observed_at + response_window
                    )

        successful_groups = []
        for ctrl, entries in readable_groups:
            lock = getattr(ctrl, "_bus_lock", None)

            def transaction() -> None:
                # 同一控制器上的"请求 + 轮询"必须在一次持锁事务内完成，
                # 否则别的线程可能在两者之间插入自己的帧，导致错配。
                for _label, motor in entries:
                    motor.request_feedback()
                ctrl.poll_feedback_once()

            try:
                if lock is None:
                    transaction()
                else:
                    with lock:
                        transaction()
                successful_groups.append((ctrl, entries))
            except Exception as exc:
                labels = ",".join(label for label, _motor in entries)
                message = (
                    f"controller={type(ctrl).__name__} motors={labels}: {exc}"
                )
                group_errors.append(message)
                for label, _motor in entries:
                    self._record_feedback_label_error(
                        label,
                        f"shared feedback batch failed: {message}",
                    )
                    self._feedback_request_baseline_by_label.pop(label, None)
                    self._feedback_request_deadline_by_label.pop(label, None)

        if inspect_after_poll:
            completed_at = time.monotonic()
            observations = {}
            for ctrl, entries in successful_groups:
                try:
                    for label, motor in entries:
                        observations[label] = self._feedback_state_with_sequence(label, motor)
                except Exception as exc:
                    labels = ",".join(label for label, _motor in entries)
                    message = f"controller={type(ctrl).__name__} motors={labels}: {exc}"
                    group_errors.append(message)
                    for label, _motor in entries:
                        self._record_feedback_label_error(label, message)
            self._inspect_pending_feedback(
                observations,
                observed_at=completed_at,
            )
        self._sync_feedback_health()
        if group_errors:
            raise RuntimeError("shared feedback batch failed: " + "; ".join(group_errors))

    def _force_feedback_refresh(self) -> None:
        """同步强制刷新：必须拿到"请求之后新到"的反馈，否则报错。

        用于使能、置零、夹爪命令等需要当下真实状态的场合。先记录每个标签的
        基线序号与上一份样本，再最多重试 ``_FEEDBACK_REFRESH_RETRIES`` 次，
        直到样本对象被替换且序号相对基线前进；全部失败时抛出最后一个错误或
        汇总缺失原因。
        """
        groups = self._feedback_controller_groups()
        self._require_feedback_sequence_api(groups)
        initial: dict[str, tuple[object, int]] = {}
        initial_errors: list[str] = []
        for ctrl, entries in groups:
            try:
                for label, motor in entries:
                    initial[label] = self._feedback_state_with_sequence(label, motor)
            except Exception as exc:
                labels = ",".join(label for label, _motor in entries)
                message = f"controller={type(ctrl).__name__} motors={labels}: {exc}"
                initial_errors.append(message)
                for label, _motor in entries:
                    self._record_feedback_label_error(label, message)
        if initial_errors:
            self._sync_feedback_health()
            raise RuntimeError(
                "forced feedback baseline failed before request: "
                + "; ".join(initial_errors)
            )
        required_baselines = {
            label: sequence
            for label, (_state, sequence) in initial.items()
        }
        # 记住旧样本对象：即使序号相同，样本对象未变也说明没收到新帧。
        prior_samples = {
            label: self._verified_feedback_by_label.get(label)
            for label in required_baselines
        }

        def forced_sample_satisfies(label: str, baseline: int) -> bool:
            sample = self._verified_feedback_by_label.get(label)
            return (
                label not in self._feedback_error_by_label
                and sample is not None
                and sample is not prior_samples[label]
                and self._feedback_sequence_advanced(sample.sequence, baseline)
            )

        for label in required_baselines:
            self._feedback_request_baseline_by_label.pop(label, None)
            self._feedback_request_deadline_by_label.pop(label, None)

        last_error: Exception | None = None
        self._feedback_force_refresh_active = True
        try:
            for attempt in range(_FEEDBACK_REFRESH_RETRIES):
                attempt_error: Exception | None = None
                try:
                    self._refresh_feedback_batch(
                        observed_at=time.monotonic(),
                        inspect_after_poll=True,
                    )
                except Exception as exc:
                    last_error = exc
                    attempt_error = exc
                if attempt_error is None and all(
                    forced_sample_satisfies(label, baseline)
                    for label, baseline in required_baselines.items()
                ):
                    return
                if attempt + 1 < _FEEDBACK_REFRESH_RETRIES:
                    time.sleep(_FEEDBACK_RETRY_INTERVAL_SEC)
        finally:
            self._feedback_force_refresh_active = False

        missing = []
        for label, baseline in required_baselines.items():
            if not forced_sample_satisfies(label, baseline):
                reason = self._feedback_error_by_label.get(label)
                if reason is None:
                    reason = (
                        f"{label} feedback timeout: sequence did not advance "
                        f"beyond baseline={baseline}"
                    )
                self._record_feedback_label_error(label, reason)
                missing.append(reason)
            self._feedback_request_baseline_by_label.pop(label, None)
            self._feedback_request_deadline_by_label.pop(label, None)
        self._sync_feedback_health()
        if last_error is not None:
            raise last_error
        raise RuntimeError("shared feedback batch failed: " + "; ".join(missing))

    def refresh_feedback_if_due(
        self,
        *,
        force: bool = False,
        now: float | None = None,
    ) -> bool:
        """按配置频率最多执行一轮共享总线反馈批次。

        厂商命令循环运行时，只有该循环可以在总线上发起事务。ROS 定时器也会
        调用本方法，但在命令循环停止前它们被降级为"只读缓存"：``force=True``
        时直接抛出 RuntimeError，否则返回 False。
        """
        observed_at = time.monotonic() if now is None else float(now)
        control_thread = getattr(self._arm, "_ctrl_thread", None)
        if (
            self.control_loop_active
            and control_thread is not None
            and threading.current_thread() is not control_thread
        ):
            if force:
                raise RuntimeError(
                    "synchronous feedback refresh rejected while hardware loop owns bus"
                )
            return False
        with self._feedback_lock:
            due = self._feedback_next_refresh_monotonic
            # 1e-12 的容差用于抵消浮点加法误差，避免恰好到期的周期被推迟。
            if not force and due is not None and observed_at + 1e-12 < due:
                return False
            self._feedback_next_refresh_monotonic = (
                observed_at + self._hardware_feedback_period_sec
            )
            try:
                if force:
                    self._force_feedback_refresh()
                else:
                    self._refresh_feedback_batch(observed_at=observed_at)
                return True
            except Exception:
                if force:
                    raise
                return False

    @staticmethod
    def _validated_gripper_feedback_values(state) -> tuple[float, float, float, int]:
        """拆出夹爪反馈的 (位置 rad, 速度 rad/s, 力矩 N.m, 状态码)，并做有限性检查。"""
        if state is None:
            raise RuntimeError("gripper feedback unavailable")
        position = float(state.pos)
        velocity = float(state.vel)
        torque = float(state.torq)
        if not all(np.isfinite(value) for value in (position, velocity, torque)):
            raise RuntimeError("gripper feedback contains non-finite values")
        return position, velocity, torque, int(state.status_code)

    def _record_gripper_feedback(self, state, *, observed_at: float | None = None) -> None:
        position, velocity, torque, status = self._validated_gripper_feedback_values(state)
        timestamp = time.monotonic() if observed_at is None else float(observed_at)
        with self._gripper_lock:
            recovered = self._gripper_feedback_error is not None
            self._gripper_pos = position
            self._gripper_vel = velocity
            self._gripper_torque = torque
            self._gripper_status_code = status
            self._gripper_feedback_updated_monotonic = timestamp
            self._gripper_feedback_error = None
        if recovered:
            _LOG.info(
                "gripper feedback recovered pos=%.6frad vel=%.6frad/s "
                "torque=%.6fNm status=%d updated=%.6f",
                position,
                velocity,
                torque,
                status,
                timestamp,
            )

    def _record_gripper_feedback_error(self, reason: str) -> None:
        message = str(reason)
        with self._gripper_lock:
            changed = message != self._gripper_feedback_error
            self._gripper_feedback_error = message
        if changed:
            _LOG.error("gripper feedback error: %s", message)

    def _gripper_feedback_age_sec(self, *, now: float | None = None) -> float:
        with self._gripper_lock:
            updated = self._gripper_feedback_updated_monotonic
        if updated is None:
            return float("inf")
        current = time.monotonic() if now is None else float(now)
        return max(current - updated, 0.0)

    def _gripper_feedback_failure_reason_locked(self, *, now: float | None = None) -> str | None:
        """夹爪反馈失效原因（调用方需持有 ``_gripper_lock``）。

        依次检查：显式错误 -> 数据过期 -> 坐标合法性（置零错误或原始角越界）。
        """
        if self._gripper_feedback_error is not None:
            return f"gripper feedback unavailable: {self._gripper_feedback_error}"
        age = self._gripper_feedback_age_sec(now=now)
        if age > self._gripper_feedback_stale_timeout_sec:
            return (
                "gripper feedback stale: "
                f"age={age:.3f}s limit={self._gripper_feedback_stale_timeout_sec:.3f}s"
            )
        return self._gripper_coordinate_failure_reason_locked()

    def _gripper_coordinate_failure_reason_locked(self) -> str | None:
        """夹爪原始角度坐标的合法性检查（调用方需持有 ``_gripper_lock``）。

        有效区间是 [_G_ANGLE_OPEN - 量化容差, _G_CLOSED_FEEDBACK_TOL_RAD]，
        即"从全开到闭合零点附近"。越界说明零点未标定或编码器异常，必须拒绝
        使用该反馈（例如合拢方向越界会让力矩变成前馈而持续顶紧）。
        """
        if self._gripper_zero_error is not None:
            return self._gripper_zero_error
        position = self._gripper_pos
        if not np.isfinite(position) or not (
            _G_ANGLE_OPEN - _G_COORDINATE_TOL_RAD
            <= position <= _G_CLOSED_FEEDBACK_TOL_RAD
        ):
            return (
                f"gripper coordinate invalid: raw={position:.6f} rad outside "
                f"[{_G_ANGLE_OPEN:.6f}, 0.000000] rad "
                f"(open_tolerance={_G_COORDINATE_TOL_RAD:.6f}, "
                f"closed_tolerance={_G_CLOSED_FEEDBACK_TOL_RAD:.6f}); verify closed zero"
            )
        return None

    def _refresh_gripper_feedback(self):
        """同步强制刷新并返回夹爪的已验证状态对象（拿不到新帧即报错）。"""
        if self._gripper_mot is None or self._gripper_ctrl is None:
            raise RuntimeError("gripper feedback unavailable: motor/controller not initialized")
        self.refresh_feedback_if_due(force=True)
        return self._verified_feedback_sample("gripper").state

    def _validated_joint_feedback(
        self,
        *,
        expected_status: int | None = None,
        refresh: bool = True,
        check_freshness: bool = True,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, list[int]]:
        """取出全部关节的 (位置 rad, 速度 rad/s, 力矩 N.m, 状态码列表)。

        ``refresh`` 为真时先做一次同步强制刷新；``check_freshness`` 为真时要求
        整体反馈未过期；``expected_status`` 非空时要求每个电机的状态码完全一致
        （0 = 失能，1 = 使能）。任一关节缺失、非有限、越软限位或状态码不符都会
        抛错，调用方不得继续运动。
        """
        if refresh:
            self.refresh_feedback_if_due(force=True)
        if check_freshness:
            feedback_failure = self._arm_feedback_failure_reason()
            if feedback_failure is not None:
                raise RuntimeError(feedback_failure)
        joint_names = self.joint_names
        if len(joint_names) != len(set(joint_names)):
            raise RuntimeError("joint names contain duplicates")
        if len(joint_names) != len(_JOINT_POSITION_LIMITS_RAD):
            raise RuntimeError(
                f"joint count mismatch: got {len(joint_names)}, "
                f"expected {len(_JOINT_POSITION_LIMITS_RAD)}"
            )

        positions: list[float] = []
        velocities: list[float] = []
        torques: list[float] = []
        statuses: list[int] = []
        samples = self._verified_feedback_samples(joint_names)
        for name, sample in zip(joint_names, samples):
            if name not in _JOINT_POSITION_LIMITS_RAD:
                raise RuntimeError(f"no hardware soft limit configured for {name}")
            state = sample.state
            values = (float(state.pos), float(state.vel), float(state.torq))
            if not all(np.isfinite(value) for value in values):
                raise RuntimeError(f"{name} feedback contains non-finite values")
            lower, upper = _JOINT_POSITION_LIMITS_RAD[name]
            if values[0] < lower or values[0] > upper:
                raise RuntimeError(
                    f"{name} position {values[0]:.6f} rad outside "
                    f"hardware soft limit [{lower:.6f}, {upper:.6f}]"
                )
            status = int(state.status_code)
            if expected_status is not None and status != expected_status:
                raise RuntimeError(
                    f"{name} status_code={status}, expected {expected_status}"
                )
            positions.append(values[0])
            velocities.append(values[1])
            torques.append(values[2])
            statuses.append(status)

        return (
            np.array(positions, dtype=np.float64),
            np.array(velocities, dtype=np.float64),
            np.array(torques, dtype=np.float64),
            statuses,
        )

    def _validated_gripper_status(self, expected_status: int) -> None:
        if self._gripper_mot is None:
            return
        state = self._refresh_gripper_feedback()
        status = int(state.status_code)
        if status != expected_status:
            raise RuntimeError(
                f"gripper status_code={status}, expected {expected_status}"
            )

    def _disable_all_motors(self) -> None:
        """失能机械臂与夹爪；逐个收集错误后统一抛出，保证两边都尝试过。"""
        errors: list[str] = []
        try:
            self._arm.disable()
        except Exception as exc:
            errors.append(f"arm disable: {exc}")
        if self._gripper_mot is not None:
            try:
                self._gripper_mot.disable()
            except Exception as exc:
                errors.append(f"gripper disable: {exc}")
        if errors:
            raise RuntimeError("; ".join(errors))

    def connect(self) -> None:
        """连接硬件并停留在"已连接但失能"状态。

        顺序：机械臂连接 -> 初始化夹爪 -> 校验全部关节与夹爪反馈。若发现任何
        电机状态码非 0（即上电后仍处于使能态），先显式失能再重新校验，绝不带着
        未知的使能状态进入后续流程。任一步失败都会走 ``_disconnect_after_failed_connect``
        清理，避免留下半开的串口或后台线程。
        """
        if self._connected:
            return
        try:
            self._arm.connect()
            self._connected = True
            self.init_gripper(str(self._gripper_cfg_path))
            _positions, _velocities, _torques, statuses = (
                self._validated_joint_feedback()
            )
            gripper_status = None
            if self._gripper_mot is not None:
                gripper_state = self._refresh_gripper_feedback()
                gripper_status = int(gripper_state.status_code)
            if any(status != 0 for status in statuses) or gripper_status not in (None, 0):
                self._disable_all_motors()
                self._validated_joint_feedback(expected_status=0)
                self._validated_gripper_status(expected_status=0)
            self._enabled = False
            self._set_lifecycle_state("CONNECTED_DISABLED")
        except Exception:
            self._disconnect_after_failed_connect()
            raise

    def _disconnect_after_failed_connect(self) -> None:
        """连接失败后的尽力清理；每一步单独吞掉异常，只为把状态复位。"""
        try:
            self._stop_gripper_loop()
        except Exception:
            pass
        try:
            self._stop_control_loop()
        except Exception:
            pass
        try:
            self._disable_all_motors()
        except Exception:
            pass
        try:
            self._arm.disconnect()
        except Exception:
            pass
        self._connected = False
        self._enabled = False
        self._set_lifecycle_state("DISCONNECTED")

    def shutdown(self) -> None:
        """关闭流程：停循环、失能、断开，并绕开厂商的归零式收尾。

        顺序很关键——先停夹爪/重力补偿/控制循环，再失能，最后断开。
        """
        if not self._connected:
            return
        try:
            self._stop_gripper_loop()
            self.stop_gravity_compensation()
            self._stop_control_loop()
            try:
                self._disable_all_motors()
            except Exception:
                pass
            # ArmEndPos.end() 会在断开前执行厂商 safe_home()，而它指向全零折叠
            # 姿态（见 _SAFE_HOME_JOINT_POSITIONS）。此处控制循环已停止、电机已
            # 失能，因此该调用根本无法移动机械臂——它只会白白轮询满 30 s 超时，
            # 并把 _q_target 留成零，影响下一次使能。故直接把该控制器标记为已停止；
            # 有意的 safe_home 属于驱动的关闭钩子，那一步更早、且仍处于使能态。
            self._endpos_ctrl._running = False
            self._arm.disconnect()
        finally:
            self._connected = False
            self._enabled = False
            self._set_lifecycle_state("DISCONNECTED")

    def get_joint_state(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """返回 (位置, 速度, 力矩)；未连接时退回厂商原始状态读取。

        已连接时要求状态码与使能状态一致（使能 -> 1，失能 -> 0），并在控制循环
        未运行时顺带刷新反馈。
        """
        with self._motor_lifecycle_lock:
            if not self._connected:
                return self._arm.get_state()
            expected_status = 1 if self._enabled else 0
            positions, velocities, torques, _statuses = self._validated_joint_feedback(
                expected_status=expected_status,
                refresh=not self.control_loop_active,
            )
            return positions, velocities, torques

    def get_cached_joint_state(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """与 ``get_joint_state`` 相同，但不触发新的总线事务（只读缓存）。"""
        with self._motor_lifecycle_lock:
            if not self._connected:
                return self._arm.get_state()
            expected_status = 1 if self._enabled else 0
            positions, velocities, torques, _statuses = self._validated_joint_feedback(
                expected_status=expected_status,
                refresh=False,
            )
            return positions, velocities, torques

    def get_cached_joint_sample(self):
        """在一次原子读取中同时返回校验后的数值与其接收标识。"""
        with self._motor_lifecycle_lock, self._feedback_lock:
            values = self._validated_joint_feedback(
                expected_status=1 if self._enabled else 0, refresh=False,
            )
            # 序号用于上层判断两个样本是否来自同一批反馈（避免混用新旧帧）。
            sequences = tuple(
                sample.sequence for sample in self._verified_feedback_samples(self.joint_names)
            )
            return (*values, sequences)

    def hold_current_position(self) -> np.ndarray:
        """把当前关节角写成保持目标，使机械臂停在原地。"""
        q, _, _ = self.get_joint_state()
        current = np.array(q, dtype=np.float64, copy=True)
        self._endpos_ctrl._q_target[:] = current
        return current

    def stop_active_motion(self) -> None:
        """立即停止正在下发的轨迹：置停止标志、取消移动、保持当前位置。"""
        with self._motor_lifecycle_lock:
            self._endpos_ctrl._stop_send.set()
            self._endpos_ctrl._moving = False
            if self._enabled:
                self.hold_current_position()
            self.set_state_machine("IDLE")

    def safe_home_target(self) -> np.ndarray:
        return np.array(_SAFE_HOME_JOINT_POSITIONS, dtype=np.float64)

    def _validated_safe_home_target(
        self, target: Optional[Sequence[float]] = None
    ) -> np.ndarray:
        """校验 safe_home 目标：长度、有限性，以及逐关节是否在软限位内。"""
        if target is None:
            return self.safe_home_target()
        values = np.array([float(value) for value in target], dtype=np.float64)
        expected = len(_SAFE_HOME_JOINT_POSITIONS)
        if values.shape != (expected,):
            raise ValueError(
                f"safe_home target must hold {expected} joint positions, got {values.shape[0]}"
            )
        if not np.all(np.isfinite(values)):
            raise ValueError("safe_home target must be finite")
        for name, value in zip(self.joint_names, values):
            limits = _JOINT_POSITION_LIMITS_RAD.get(name)
            if limits is None:
                continue
            low, high = limits
            if not low <= value <= high:
                raise ValueError(
                    f"safe_home target for {name} is {value:.4f} rad, "
                    f"outside the allowed range [{low}, {high}]"
                )
        return values

    def safe_home(
        self,
        target: Optional[Sequence[float]] = None,
        vlim: Optional[float] = None,
        timeout_sec: Optional[float] = None,
    ) -> np.ndarray:
        """把机械臂驱到无碰撞的停放姿态。

        替代 ArmEndPos.safe_home（它把目标硬编码为全零）；零位为何不能停放见
        _SAFE_HOME_JOINT_POSITIONS 的说明。保留厂商的速度覆盖与到位轮询机制，
        但到位判据改为"相对请求目标"而不是零，且速度绝不允许超过厂商自身的
        回零速度。结束时无论成功失败都清掉速度覆盖、保持当前位置并回到 IDLE。
        """
        goal = self._validated_safe_home_target(target)
        ctrl = self._endpos_ctrl
        with self._motor_lifecycle_lock:
            self._require_enabled()
            if not ctrl._running:
                raise RuntimeError(
                    "control loop is inactive; safe_home needs position-velocity control"
                )
            speed = ctrl._home_vel if vlim is None else float(vlim)
            if not 0.0 < speed <= ctrl._home_vel:
                raise ValueError(
                    f"safe_home vlim must be within (0, {ctrl._home_vel:g}] rad/s"
                )
            ctrl._vlim_override = np.full(ctrl._n, speed, dtype=np.float64)
            ctrl._q_target[:] = goal
            ctrl._stop_send.set()
            ctrl._moving = False
            send_thread = ctrl._send_thread
            self.set_state_machine("TRAJ_RUNNING")
        try:
            # 先等在途的发送线程收尾，再开始轮询到位情况。
            if send_thread is not None:
                send_thread.join()
            deadline = time.monotonic() + (
                _SAFE_HOME_TIMEOUT_SEC if timeout_sec is None else float(timeout_sec)
            )
            while True:
                positions, _velocities, _torques = self.get_joint_state()
                reached = np.array(positions, dtype=np.float64)
                worst = float(np.max(np.abs(reached - goal)))
                if worst < _SAFE_HOME_ARRIVE_TOL_RAD:
                    return reached
                if time.monotonic() > deadline:
                    raise TimeoutError(
                        f"safe_home did not reach {goal.tolist()} within "
                        f"{_SAFE_HOME_ARRIVE_TOL_RAD} rad; worst joint error {worst:.4f} rad"
                    )
                time.sleep(ctrl._dt)
        finally:
            ctrl._vlim_override = None
            with self._motor_lifecycle_lock:
                if self._enabled:
                    self.hold_current_position()
                self.set_state_machine("IDLE")

    def enable(self) -> None:
        """显式使能：真机运动的唯一入口。

        前置条件（全部满足才继续）：已连接、反馈状态码为 0（失能）、夹爪坐标
        合法。随后把当前角度写成保持目标 -> 进入位置-速度模式 -> 使能电机 ->
        校验状态码为 1 -> 启动统一控制循环。任何一步失败都会执行
        ``_rollback_failed_enable``；回滚本身也失败时保留 _enabled=True 并追加
        ENABLE_ROLLBACK_FAILED，绝不谎报"已安全失能"。
        """
        from motorbridge import Mode

        with self._motor_lifecycle_lock:
            self._require_connected()
            if self._enabled:
                self.hold_current_position()
                self._set_lifecycle_state("ENABLED_HOLD")
                return

            self._set_lifecycle_state("ENABLING")
            try:
                positions, _velocities, _torques, _statuses = (
                    self._validated_joint_feedback(expected_status=0)
                )
                self._validated_gripper_status(expected_status=0)
                if self._gripper_mot is not None:
                    with self._gripper_lock:
                        coordinate_failure = self._gripper_coordinate_failure_reason_locked()
                    if coordinate_failure is not None:
                        raise RuntimeError(coordinate_failure)
                # 先把目标设为当前角度，使能瞬间不会产生位置阶跃。
                self._endpos_ctrl._q_target[:] = positions
                if self._arm.mode_pos_vel() is False:
                    raise RuntimeError("failed to enter position-velocity control mode")
                if self._gripper_mot is not None:
                    self._gripper_mot.ensure_mode(Mode.MIT, 1000)
                self._arm.enable()
                if self._gripper_mot is not None:
                    self._gripper_mot.enable()
                self._validated_joint_feedback(expected_status=1)
                self._validated_gripper_status(expected_status=1)
                self._enabled = True
                self._start_pos_vel_loop(target=positions)
                self.set_state_machine("IDLE")
                self._set_lifecycle_state("ENABLED_HOLD")
            except Exception as exc:
                rollback_error = self._rollback_failed_enable()
                if rollback_error is not None:
                    raise RuntimeError(
                        f"enable failed: {exc}; rollback verification failed: "
                        f"{rollback_error}"
                    ) from exc
                raise RuntimeError(f"enable failed and was rolled back: {exc}") from exc

    def _rollback_failed_enable(self) -> str | None:
        """使能失败后的回滚：停循环、失能、并验证状态码确为 0。

        返回 None 表示已确认失能；返回字符串表示回滚未完成——此时保持
        _enabled=True（宁可以为仍在使能，也不要误判为已失能），追加
        ENABLE_ROLLBACK_FAILED 并把生命周期置为 DISABLING。
        """
        rollback_errors: list[str] = []
        try:
            self._stop_gripper_loop()
        except Exception as exc:
            rollback_errors.append(f"stop gripper loop: {exc}")
        try:
            self._stop_control_loop()
        except Exception as exc:
            rollback_errors.append(f"stop arm loop: {exc}")
        try:
            self._disable_all_motors()
        except Exception as exc:
            rollback_errors.append(f"disable command: {exc}")
        if not rollback_errors:
            # 只有失能命令本身没报错才值得校验反馈；否则反馈可能已经不可信。
            try:
                self._validated_joint_feedback(expected_status=0)
                self._validated_gripper_status(expected_status=0)
            except Exception as exc:
                rollback_errors.append(f"disabled feedback: {exc}")
        self._state_machine = "IDLE"
        if rollback_errors:
            self._enabled = True
            self._error_codes.append("ENABLE_ROLLBACK_FAILED")
            self._set_lifecycle_state("DISABLING")
            return "; ".join(rollback_errors)
        self._enabled = False
        self._set_lifecycle_state("CONNECTED_DISABLED" if self._connected else "DISCONNECTED")
        return None

    def disable(self) -> None:
        """失能：先标记未使能（阻止新的运动命令），再停循环并验证状态码为 0。

        验证失败时恢复 _enabled 原值并追加 DISABLE_VERIFICATION_FAILED，让上层
        知道"可能仍在使能"，而不是假装已经安全停下。
        """
        with self._motor_lifecycle_lock:
            self._require_connected()
            self._set_lifecycle_state("DISABLING")
            was_enabled = self._enabled
            self._enabled = False
            self.stop_gravity_compensation()
            self._stop_gripper_loop()
            self._stop_control_loop()
            try:
                self._disable_all_motors()
                self._validated_joint_feedback(expected_status=0)
                self._validated_gripper_status(expected_status=0)
            except Exception:
                self._enabled = was_enabled
                self._error_codes.append("DISABLE_VERIFICATION_FAILED")
                raise
            else:
                self._enabled = False
                self._state_machine = "IDLE"
                self._set_lifecycle_state("CONNECTED_DISABLED")

    def set_mode(self, mode: str) -> bool:
        """切换电机控制模式："mit" / "pos_vel" / "vel"（大小写与空格不敏感）。

        切模式前必须先停循环并退出重力补偿；若目标模式与当前模式相同，仅在
        pos_vel 且已使能时确保控制循环在跑（或保持当前位置）。
        """
        self._require_connected()
        mode = mode.strip().lower()
        if mode not in ("mit", "pos_vel", "vel"):
            raise ValueError(f"unsupported mode: {mode}")
        self.stop_gravity_compensation()

        if mode == self.mode:
            if mode == "pos_vel" and self._enabled:
                if self.control_loop_active:
                    self.hold_current_position()
                else:
                    self._start_pos_vel_loop()
            self.set_state_machine("IDLE")
            return True

        self._stop_control_loop()
        if mode == "mit":
            ok = self._arm.mode_mit()
        elif mode == "pos_vel":
            ok = self._arm.mode_pos_vel()
            if self._enabled:
                self._start_pos_vel_loop()
        else:
            ok = self._arm.mode_vel()
        self.set_state_machine("IDLE")
        return bool(ok)

    def set_zero(self, joint_name: str = "") -> bool:
        """标定零点。空字符串表示全部关节，``"gripper"`` 表示夹爪。

        必须处于"已连接未使能"状态——标定期间电机不得带电运动。整个标定与
        使能/失能/置零共用 ``_motor_lifecycle_lock`` 串行化。
        """
        # 标定需与使能/失能串行化。即使当前坐标非法也继续采集原始反馈，
        # 以便显式置零可以修复它。
        with self._motor_lifecycle_lock:
            self._require_connected()
            if self._enabled:
                raise RuntimeError("set_zero requires CONNECTED_DISABLED state")
            self._stop_control_loop()
            joint_name = str(joint_name).strip()
            if joint_name == "gripper":
                return self._set_gripper_zero()
            if joint_name:
                ok = self._arm.set_zero_single(joint_name)
            else:
                self._arm.set_zero()
                ok = True
            self._enabled = False
            self.set_state_machine("IDLE")
            return bool(ok)

    def _set_gripper_zero(self) -> bool:
        """夹爪置零并验证：要求在超时内连续若干帧都接近零位。

        验证必须基于"置零之后新到"的帧（``_refresh_gripper_feedback`` 会重建
        接收序号基线，置零前的缓存样本无法满足）。校验失败时保留
        ``_gripper_zero_error``：仅凭"有新帧"不能证明标定成功，必须等到下一次
        显式置零验证通过才清除。
        """
        if self._gripper_mot is None:
            raise RuntimeError("gripper is not initialized")
        self._stop_gripper_loop()
        with self._feedback_lock:
            self._validated_gripper_status(expected_status=0)
            with self._gripper_lock:
                self._gripper_zero_error = "gripper zero verification pending"
            try:
                self._gripper_mot.set_zero_position()
                deadline = time.monotonic() + _G_ZERO_VERIFY_TIMEOUT_SEC
                consecutive = 0
                position = float("nan")
                while time.monotonic() < deadline:
                    # 强制刷新会重建接收序号基线；置零前缓存下来的样本无法满足
                    # 本次验证请求。
                    state = self._refresh_gripper_feedback()
                    position, _velocity, _torque, status = (
                        self._validated_gripper_feedback_values(state)
                    )
                    if status != 0:
                        raise RuntimeError(
                            f"gripper status_code={status}, expected 0 after zero"
                        )
                    if time.monotonic() >= deadline:
                        break
                    # 必须是"连续"多帧接近零位，单帧偶然命中不算通过。
                    consecutive = (
                        consecutive + 1 if abs(position) <= _G_COORDINATE_TOL_RAD else 0
                    )
                    if consecutive >= _G_ZERO_VERIFY_SAMPLES:
                        with self._gripper_lock:
                            self._gripper_zero_error = None
                        self._enabled = False
                        self.set_state_machine("IDLE")
                        return True
                    time.sleep(_FEEDBACK_RETRY_INTERVAL_SEC)
                raise RuntimeError(
                    f"zero verification timed out: raw={position:.6f} rad; "
                    f"need {_G_ZERO_VERIFY_SAMPLES} fresh disabled samples within "
                    f"+/-{_G_COORDINATE_TOL_RAD:.6f} rad"
                )
            except Exception as exc:
                message = f"gripper set_zero failed: {exc}"
                with self._gripper_lock:
                    # 仅凭新帧并不能证明标定失败已消除；在显式置零验证通过之前
                    # 一直保留该失败标记。
                    self._gripper_zero_error = message
                raise RuntimeError(message) from exc

    def ensure_pos_vel_control(self) -> None:
        """确保处于位置-速度模式且统一控制循环在运行（否则保持当前位置）。"""
        self._require_enabled()
        if self.mode != "pos_vel":
            self._stop_control_loop()
            self._arm.mode_pos_vel()
        if not self.control_loop_active:
            self._start_pos_vel_loop()
        else:
            self.hold_current_position()

    def send_joint_motor_cmd(self, joint_name: str, cmd) -> None:
        """低层单关节直通命令（JointMotorCmd）。

        未显式给出的字段回落到当前反馈值或关节默认增益，避免用零值覆盖。命令
        送入后状态机切到 LOWLEVEL_STREAMING，提示上层这不是轨迹跟踪。
        mode：0 = MIT，1 = 位置-速度，2 = 速度（需电机支持 send_vel）。
        """
        self._require_enabled()
        if joint_name not in self._arm._motor_map:
            raise KeyError(f"unknown joint: {joint_name}")

        mot = self._arm._motor_map[joint_name]
        jc = next(j for j in self._arm._joints if j.name == joint_name)
        feedback_failure = self._arm_feedback_failure_reason()
        if feedback_failure is not None:
            raise RuntimeError(feedback_failure)
        state = self._verified_feedback_sample(joint_name).state

        pos = float(cmd.pos) if cmd.use_pos else float(state.pos if state is not None else 0.0)
        vel = float(cmd.vel) if cmd.use_vel else float(state.vel if state is not None else 0.0)
        kp = float(cmd.kp) if cmd.use_kp else float(jc.kp)
        kd = float(cmd.kd) if cmd.use_kd else float(jc.kd)
        tau = float(cmd.tau) if cmd.use_tau else 0.0
        vlim = float(cmd.vlim) if cmd.use_vlim else float(jc.vlim)

        if int(cmd.mode) == 0:
            mot.send_mit(pos, vel, kp, kd, tau)
        elif int(cmd.mode) == 1:
            mot.send_pos_vel(pos, vlim)
        elif int(cmd.mode) == 2:
            if not hasattr(mot, "send_vel"):
                raise RuntimeError(f"{joint_name} does not support send_vel")
            mot.send_vel(vel)
        else:
            raise ValueError(f"unsupported JointMotorCmd mode: {cmd.mode}")
        self.set_state_machine("LOWLEVEL_STREAMING")

    def start_gravity_compensation(self) -> None:
        with self._motor_lifecycle_lock:
            self._start_gravity_compensation_locked()

    def _start_gravity_compensation_locked(self) -> None:
        """进入重力补偿：MIT 模式 + 广义重力前馈 + 目标位置 PI 锁定。

        流程：停掉位置-速度循环与发送线程 -> 刷新反馈并把当前角度设为目标 ->
        以配置的 Kp/Kd 进入 MIT 模式 -> 启动统一控制循环执行
        ``_gravity_hardware_tick``。补偿期间操作员可以推动机械臂（目标会跟随），
        停止时再回到位置-速度保持。
        """
        self._require_enabled()
        timing = LoopTiming(float(self._arm._rate))
        self.stop_gravity_compensation()
        self._stop_control_loop()
        self._endpos_ctrl._stop_send.set()
        self._endpos_ctrl._moving = False
        try:
            self._refresh_arm_feedback()
            self._gravity_comp_q_target = self._read_gravity_comp_positions()
        except Exception as exc:
            self._protective_disable_from_hardware_loop(str(exc))
            raise
        self._gravity_comp_q_last = self._gravity_comp_q_target.copy()
        # Before switching modes reject a model that cannot support this pose.
        try:
            gravity = vector(self._gc_compute_generalized_gravity(q=self._gravity_comp_q_target),
                             6, 'gravity torque')
            if (np.abs(gravity) > self._gc_torque_limits).any():
                raise RuntimeError('gravity torque exceeds model effort limits')
        except Exception:
            self._start_pos_vel_loop(target=self._gravity_comp_q_target)
            raise
        self._gravity_comp_fault = None
        self._gravity_comp_hold_feedforward = None
        self._gravity_comp_integral = np.zeros_like(self._gravity_comp_q_target)
        self._gravity_comp_lock_counter = 0
        try:
            if not self._arm.mode_mit(
                kp=np.full(6, self._gc_settings.kp), kd=np.full(6, self._gc_settings.kd)
            ):
                raise RuntimeError('MIT mode switch did not succeed on all joints')
            # Mode switching can take hundreds of ms; refresh after, not only before.
            self._refresh_arm_feedback()
            self._gravity_comp_q_target = self._read_gravity_comp_positions()
            samples = self._verified_feedback_samples(self.joint_names)
            self._gravity_comp_last_total = np.clip(
                [float(s.state.torq) for s in samples],
                -self._gc_torque_limits, self._gc_torque_limits)
            self._gravity_comp_active = True
            self._gravity_comp_tick(self._arm, 1. / float(self._arm._rate))
        except Exception:
            self._gravity_comp_active = False
            # Restore healthy hardware to verified current-position hold.
            # A failed restore is a motor/communication fault, not a task failure.
            try:
                self._refresh_arm_feedback()
                target = self._read_gravity_comp_positions()
                if not self._arm.mode_pos_vel():
                    raise GravityCommandError('POS_VEL restore failed')
                self._start_pos_vel_loop(target=target)
                self.set_state_machine('IDLE')
            except Exception as restore_error:
                self._protective_disable_from_hardware_loop(str(restore_error))
            raise
        self._gravity_timing = timing
        self._gravity_feedback_metrics = {}
        self._gravity_next_log = 0.
        self._arm.start_control_loop(self._gravity_hardware_tick, rate=self._arm._rate)
        self.set_state_machine("GRAVITY_COMP")

    def stop_gravity_compensation(self) -> None:
        with self._motor_lifecycle_lock:
            self._stop_gravity_compensation_locked()

    def _stop_gravity_compensation_locked(self) -> None:
        """退出重力补偿：用最新验证角度接管位置-速度保持。"""
        if not self._gravity_comp_active:
            return
        self._stop_control_loop()
        self._gravity_comp_active = False
        self._gravity_comp_q_target = None
        self._gravity_comp_integral = None
        self._gravity_comp_lock_counter = 0
        self._gravity_comp_q_last = None
        if self._enabled:
            try:
                self._refresh_arm_feedback()
                hold_target = self._read_gravity_comp_positions()
                if not self._arm.mode_pos_vel():
                    raise GravityCommandError('gravity POS_VEL restore failed')
                self._start_pos_vel_loop(target=hold_target)
            except Exception as exc:
                self._protective_disable_from_hardware_loop(str(exc))
                raise
        self.set_state_machine("IDLE")

    def gravity_compensation_active(self) -> bool:
        return self._gravity_comp_active

    def gravity_compensation_target(self) -> np.ndarray | None:
        if self._gravity_comp_q_target is None:
            return None
        return self._gravity_comp_q_target.copy()

    def _refresh_arm_feedback(self) -> None:
        self.refresh_feedback_if_due(force=True)

    @staticmethod
    def _angles_near_reference(values: np.ndarray, reference: np.ndarray) -> np.ndarray:
        """把角度解到与参考值最接近的等价分支上。

        差值先按模 2*pi 折到 (-pi, pi]，再加回参考值；用于抵消编码器在
        +/-pi 处的跳变，避免重力补偿把跨圈跳变误判为大幅运动。
        """
        delta = values - reference
        delta = (delta + np.pi) % (2.0 * np.pi) - np.pi
        return reference + delta

    def _read_gravity_comp_positions(
        self,
        *,
        request: bool = False,
        reference: np.ndarray | None = None,
    ) -> np.ndarray:
        # request 参数保留为兼容接口：实际采集一律通过统一反馈批次完成。
        del request
        q, _qd = self._read_gravity_comp_feedback(reference=reference)
        return q

    def _read_gravity_comp_feedback(
        self,
        *,
        reference: np.ndarray | None = None,
    ) -> tuple[np.ndarray, np.ndarray]:
        """重力补偿回路的反馈读取：(角度 rad, 角速度 rad/s)。

        要求全部电机状态码为 1（补偿期间必须使能），并做软限位/有限性校验；
        角度会解到与上一帧最接近的分支，结果同时更新 ``_gravity_comp_q_last``。
        """
        joint_names = self.joint_names
        with self._feedback_lock:
            feedback_failure = self._arm_feedback_failure_reason()
            if feedback_failure is not None:
                raise RuntimeError(feedback_failure)
            samples = self._verified_feedback_samples(joint_names)

        positions: list[float] = []
        velocities: list[float] = []
        for name, sample in zip(joint_names, samples):
            self._validate_feedback_sample(name, sample.state)
            status = int(sample.state.status_code)
            if status != 1:
                raise RuntimeError(
                    f"{name} status_code={status}, expected 1 during gravity compensation"
                )
            positions.append(float(sample.state.pos))
            velocities.append(float(sample.state.vel))

        q = np.array(positions, dtype=np.float64)
        qd = np.array(velocities, dtype=np.float64)
        ref = reference if reference is not None else self._gravity_comp_q_last
        if ref is not None:
            q = self._angles_near_reference(q, ref)
        self._gravity_comp_q_last = np.array(q, dtype=np.float64, copy=True)
        return self._gravity_comp_q_last.copy(), qd

    def _gravity_comp_tick(self, arm, dt: float) -> None:
        """重力补偿的单周期控制律（运行在统一控制循环里）。

        1) 读反馈；2) 用动力学模型算广义重力力矩并按增益缩放，作为前馈 tau；
        3) 目标角误差按实际 dt 积分，以 N.m 限幅用于抵消稳态摩擦/模型误差；
        4) 用末端雅可比把关节速度映到笛卡尔空间：一旦线速度/角速度超过阈值，
        说明人正在拖动，于是把目标角更新为当前角并衰减积分（"跟随"模式）；
        否则累加锁定计数（"锁定"模式）。
        下发时位置/速度目标固定，仅用 tau 项补偿重力，因此机械臂呈现"轻"的手感。
        """
        if not self._gravity_comp_active or self._gravity_comp_q_target is None:
            return

        q, qd = self._read_gravity_comp_feedback()
        if self._gravity_comp_fault is not None:
            self._emit_bounded_gravity_command(q, qd, self._gravity_comp_hold_feedforward, dt)
            return
        tau_g = vector(self._gc_compute_generalized_gravity(q=q), 6, 'gravity torque')
        tau_g = apply_gravity_compensation_tau_scale(tau_g)

        q_error = self._gravity_comp_q_target - q
        if self._gravity_comp_integral is None:
            self._gravity_comp_integral = np.zeros_like(q)
        self._gravity_comp_integral += q_error * self._gc_settings.integral_gain * dt
        cap = self._gc_settings.integral_limit_nm
        np.clip(self._gravity_comp_integral, -cap, cap, out=self._gravity_comp_integral)

        self._gc_pin.computeJointJacobians(self._gc_model, self._gc_data, q)
        self._gc_pin.updateFramePlacements(self._gc_model, self._gc_data)
        jacobian = self._gc_pin.getFrameJacobian(
            self._gc_model,
            self._gc_data,
            self._gc_ee_frame_id,
            self._gc_pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )
        # 空间速度前三维是线速度、后三维是角速度（世界系表达）。
        spatial_velocity = jacobian @ qd
        linear_speed = float(np.linalg.norm(spatial_velocity[:3]))
        angular_speed = float(np.linalg.norm(spatial_velocity[3:]))

        if linear_speed > _GC_VEL_THRESHOLD or angular_speed > _GC_W_VEL_THRESHOLD:
            # 检测到外部拖动：目标随之移动，并把积分项衰减 10%。
            self._gravity_comp_q_target = q.copy()
            self._gravity_comp_lock_counter = 0
            self._gravity_comp_integral *= np.exp(-dt / .2)
        else:
            self._gravity_comp_lock_counter += 1

        self._emit_bounded_gravity_command(q, qd, tau_g + self._gravity_comp_integral, dt)

    def _emit_bounded_gravity_command(self, q, qd, feedforward, dt):
        settings = self._gc_settings
        kp, kd, tau, total = bounded_mit_command(
            self._gravity_comp_q_target, q, qd, np.full(6, settings.kp),
            np.full(6, settings.kd), feedforward, self._gc_torque_limits,
            settings.torque_rate_limits_nm_s, self._gravity_comp_last_total, dt)
        self._send_gravity_mit(
            pos=self._gravity_comp_q_target,
            vel=np.zeros(6), kp=kp, kd=kd, tau=tau,
        )
        self._gravity_comp_last_total = total

    def _send_gravity_mit(self, *, pos, vel, kp, kd, tau):
        # SDK arm.mit catches CallError and continues. Send through the already
        # bus-locked motor wrappers here so partial/failed sends reach the guard.
        for index, name in enumerate(self.joint_names):
            try:
                self._arm._motor_map[name].send_mit(
                    float(pos[index]), float(vel[index]), float(kp[index]),
                    float(kd[index]), float(tau[index]))
            except Exception as exc:
                raise GravityCommandError(f'{name} MIT send failed: {exc}') from exc

    def _gravity_fault_hold(self, reason):
        q, _ = self._read_gravity_comp_feedback()
        if self._gravity_comp_fault is None:
            self._gravity_comp_fault = str(reason)
            self._gravity_comp_q_target = q.copy()
            self._gravity_comp_integral.fill(0.)
            self._gravity_comp_hold_feedforward = self._gravity_comp_last_total.copy()
            self._error_codes.append(f'GRAVITY_HOLD: {reason}')
            _LOG.error('gravity compensation stopped; enabled guarded hold: %s', reason)

    def gravity_compensation_diagnostics(self):
        timing = getattr(self, '_gravity_timing', None)
        model = getattr(self, '_gravity_model', None)
        settings = getattr(self, '_gc_settings', None)
        return dict(timing=timing.snapshot() if timing else {},
                    feedback={k: dict(v) for k, v in getattr(self, '_gravity_feedback_metrics', {}).items()},
                    configured_feedback_rate_hz=self._hardware_feedback_rate_hz,
                    fault=getattr(self, '_gravity_comp_fault', None),
                    model_path=model.path if model else None,
                    model_sha256=model.sha256 if model else None,
                    gravity_xyz_m_s2=list(settings.gravity_xyz_m_s2) if settings else None,
                    payload_mass_kg=settings.payload_mass_kg if settings else None,
                    payload_com_xyz_m=list(settings.payload_com_xyz_m) if settings else None)

    def current_pose(self):
        """用厂商正运动学把当前关节角换算成 Pose（供状态发布使用）。"""
        from reBotArm_control_py.kinematics import compute_fk

        q, _, _ = self.get_joint_state()
        position, rotation, _ = compute_fk(self._endpos_ctrl._model, q)
        return fk_to_pose(position, rotation)

    def get_joint_status_codes(self) -> list[int]:
        """返回各关节状态码；反馈整体失效时统一返回 255（未知）。"""
        if self._arm_feedback_failure_reason() is not None:
            return [255] * len(self.joint_names)
        codes: list[int] = []
        for name in self.joint_names:
            try:
                st = self._verified_feedback_sample(name).state
                codes.append(int(st.status_code) if st is not None else 255)
            except Exception:
                codes.append(255)
        return codes

    def init_gripper(self, cfg_path: str) -> None:
        """按配置初始化夹爪电机，并挂到机械臂同一控制器/总线上。

        夹爪必须与机械臂共用同一个控制器对象（同一串口），否则无法保证总线
        事务的原子性。初始化阶段保持单线程：首次新鲜反馈校验由本方法同步完成，
        位置/抓取入口再按需启动统一控制循环。
        """
        from reBotArm_control_py.actuator.gripper import load_cfg as load_gripper_cfg

        gcfg = load_gripper_cfg(cfg_path)
        gc = gcfg["gripper"]
        self._gripper_cfg = gc

        vendor = gc.vendor
        if vendor not in self._arm._ctrl_map:
            raise RuntimeError(
                f"gripper vendor={vendor!r} cannot share the arm Controller"
            )
        ctrl = self._arm._ctrl_map[vendor]

        if vendor == "damiao":
            self._gripper_mot = ctrl.add_damiao_motor(gc.motor_id, gc.feedback_id, gc.model)
        elif vendor == "myactuator":
            self._gripper_mot = ctrl.add_myactuator_motor(gc.motor_id, gc.feedback_id, gc.model)
        elif vendor == "robstride":
            self._gripper_mot = ctrl.add_robstride_motor(gc.motor_id, gc.feedback_id, gc.model)
        else:
            raise ValueError(f"unsupported gripper vendor: {vendor!r}")

        self._gripper_ctrl = ctrl

        self._patch_controller_bus(ctrl)
        self._wrap_motor_bus(self._gripper_mot, ctrl._bus_lock)
        # 机械臂与夹爪在共享串口上完成首次新鲜反馈校验期间，保持初始化单线程。
        # 位置/抓取入口会按需启动该循环。

    def set_gripper_target(self, position_m: float, max_effort: float = 0.0) -> None:
        """下发夹爪位置目标（异步）：只登记目标，实际下发由 统一硬件循环完成。

        ``position_m``：0 表示完全闭合、正值为张开距离（m），会被夹到
        [0, _G_VERIFIED_OPEN_LIMIT_M]；``max_effort`` <= 0 时使用默认力矩
        ``_G_DEFAULT_FORCE``，否则夹到 [0.05, 位置力矩上限]。
        超时按"角度行程 / 最大速度 + 余量"估算，供 ``wait_gripper_target`` 使用。
        """
        self._require_enabled()
        if not self.control_loop_active:
            raise RuntimeError(
                "gripper command requires the unified hardware control loop"
            )
        if self._gripper_mot is None:
            raise RuntimeError("gripper is not initialized")
        with self._gripper_lock:
            feedback_failure = self._gripper_feedback_failure_reason_locked()
        if feedback_failure is not None:
            raise RuntimeError(feedback_failure)
        state = self._verified_feedback_sample("gripper").state
        start_angle, _velocity, _torque, status = self._validated_gripper_feedback_values(state)
        if status != 1:
            raise RuntimeError(f"gripper status_code={status}, expected 1 before position command")
        distance = float(np.clip(position_m, 0.0, _G_VERIFIED_OPEN_LIMIT_M))
        # 线性映射：距离 -> 电机角；再取软限位，避免贴死机械端点。
        target = max((distance / _G_MAX_DIST_M) * _G_ANGLE_OPEN, _G_OPEN_SOFT_LIMIT)
        effort = _G_DEFAULT_FORCE if max_effort <= 0.0 else float(max_effort)
        now = time.monotonic()
        target_timeout_sec = (
            abs(target - start_angle) / self._gripper_position_max_speed_rad_s
            + self._gripper_position_timeout_margin_sec
        )
        with self._gripper_lock:
            self._gripper_target_angle = start_angle
            self._gripper_goal_angle = target
            self._gripper_target_effort = float(
                np.clip(
                    effort,
                    0.05,
                    getattr(
                        self,
                        "_gripper_position_torque_cap_nm",
                        _G_LARGE_MOVE_MAX_TAU_NM,
                    ),
                )
            )
            self._gripper_mode = "position"
            self._gripper_active = True
            self._gripper_position_result = "active"
            self._gripper_command_error = None
            self._gripper_last_tick_monotonic = now
            self._gripper_target_timeout_sec = target_timeout_sec
            self._gripper_target_deadline_monotonic = now + target_timeout_sec
            feedback_updated = self._gripper_feedback_updated_monotonic
            applied_effort = self._gripper_target_effort
        _LOG.info(
            "gripper position command requested=%.6fm clamped=%.6fm "
            "start=%.6frad goal=%.6frad effort=%.6fNm speed=%.6frad/s "
            "timeout=%.3fs feedback_updated=%s",
            float(position_m),
            distance,
            start_angle,
            target,
            applied_effort,
            self._gripper_position_max_speed_rad_s,
            target_timeout_sec,
            feedback_updated,
        )
        self._start_gripper_loop()

    def gripper_target_timeout_sec(self) -> float:
        with self._gripper_lock:
            return float(self._gripper_target_timeout_sec)

    @property
    def gripper_command_error(self) -> str | None:
        with self._gripper_lock:
            return self._gripper_command_error

    @property
    def gripper_feedback_error(self) -> str | None:
        with self._gripper_lock:
            return self._gripper_feedback_error

    def wait_gripper_target(self, timeout: float | None = None) -> bool:
        """阻塞等待当前位置命令完成；返回是否到位。

        只等待"自己那一次"的命令：先记住目标角，若期间出现更新的命令（目标角
        变化）立即返回 False，避免旧等待者误判或误释放新命令。到位后必须调用
        ``_release_gripper_position_target`` 解除 MIT 位置命令（完成一次定位移动
        不等于抓取保持，不能继续顶住目标）。
        超时同样结束本次命令的所有权并取消位置命令，不留残余力矩。
        """
        # 记住本次等待归属于哪个目标，这样"到位"与"超时"两条路径都不会释放
        # 期间新下发的命令。
        with self._gripper_lock:
            owned_goal = self._gripper_goal_angle
            deadline = self._gripper_target_deadline_monotonic
        now = time.monotonic()
        explicit_deadline = (
            None if timeout is None else now + max(float(timeout), 0.0)
        )
        if deadline is None:
            deadline = now if explicit_deadline is None else explicit_deadline
        elif explicit_deadline is not None:
            deadline = min(deadline, explicit_deadline)
        while time.monotonic() < deadline:
            with self._gripper_lock:
                if self._gripper_goal_angle != owned_goal:
                    return False
                if not self._gripper_active:
                    if self._gripper_position_result == "succeeded":
                        return True
                    if self._gripper_position_result == "failed":
                        return False
                feedback_failure = self._gripper_feedback_failure_reason_locked()
                if feedback_failure is not None:
                    self._fail_active_gripper_command_locked(feedback_failure)
                    return False
                arrived = bool(
                    self._gripper_active
                    and abs(self._gripper_pos - owned_goal) < _G_ARRIVE_TOL
                )
            if arrived:
                # 一次定位移动完成并不等于抓取保持。在确认成功之前先释放 MIT
                # 位置命令，使得本函数返回后电机不会继续朝目标推进。
                self._release_gripper_position_target(owned_goal)
                time.sleep(1.0 / _G_CTRL_RATE)
                continue
            time.sleep(0.02)
        # 超时同样结束本次命令对夹爪的所有权：停止下发，而不是留下一个过期的
        # 位置保持命令。
        self.cancel_gripper_position_command(
            "gripper target timeout: "
            f"goal={owned_goal:.6f}rad feedback={self._gripper_pos:.6f}rad"
        )
        return False

    def set_gripper_position(self, position_m: float, max_effort: float = 0.0) -> tuple[bool, float]:
        """同步版位置命令：下发并等待到位，返回 (是否到位, 实际位置 m)。"""
        self.set_gripper_target(position_m, max_effort)
        reached = self.wait_gripper_target()
        return reached, self.gripper_position_m()

    def grasp_gripper(
        self,
        close_force: float = _G_GRASP_CLOSE_FORCE_DEFAULT,
        hold_force: float = _G_GRASP_HOLD_FORCE_DEFAULT,
        close_timeout_sec: float = _G_GRASP_TIMEOUT,
        min_close_time_sec: float = _G_GRASP_MIN_CLOSE_TIME,
        velocity_threshold: float = _G_GRASP_VEL_THRESHOLD,
        min_closure_distance_m: float = _G_GRASP_MIN_CLOSURE_M,
        hold_timeout_sec: float | None = None,
    ) -> tuple[bool, bool, float, float, float, str]:
        """闭合抓取：检测"堵转"即认为接触，随后进入有界保持。

        判据（三者同时满足才算抓住）：已过 ``min_close_time_sec``、闭合行程
        >= ``min_closure_distance_m``、速度绝对值 <= ``velocity_threshold``。
        这里没有力传感器，因此这些条件表示"闭合推进 + 速度堵转"，即堵转检测，
        而不是实测接触力。
        返回 (是否检测到堵转, 是否进入保持, 接触位置 m, 当前位置 m, 保持力矩 N.m, 说明)。
        保持阶段有时限（``hold_timeout_sec``，默认取配置值，上限
        ``_G_GRASP_HOLD_TIMEOUT_MAX_SEC``），到时或超时路径都会中性释放，
        不会让电机长期带电顶紧。
        """
        self._require_enabled()
        if not self.control_loop_active:
            raise RuntimeError(
                "gripper grasp requires the unified hardware control loop"
            )
        if self._gripper_mot is None:
            raise RuntimeError("gripper is not initialized")
        with self._gripper_lock:
            feedback_failure = self._gripper_feedback_failure_reason_locked()
        if feedback_failure is not None:
            raise RuntimeError(feedback_failure)

        close_effort = float(np.clip(close_force, 0.05, _G_GRASP_CLOSE_FORCE_MAX))
        hold_effort = float(np.clip(hold_force, 0.05, _G_TAU_MAX))
        # 抓取保持必须有界；无界保持会让电机在控制循环中持续加载且没有退出路径。
        requested_hold = (
            self._grasp_hold_timeout_sec
            if hold_timeout_sec is None
            else float(hold_timeout_sec)
        )
        hold_timeout = float(np.clip(requested_hold, 0.1, _G_GRASP_HOLD_TIMEOUT_MAX_SEC))
        timeout = max(float(close_timeout_sec), 0.1)
        min_time = max(float(min_close_time_sec), 0.0)
        velocity_limit = max(float(velocity_threshold), 0.0)
        min_closure = max(float(min_closure_distance_m), 0.0)
        start_position_m = self.gripper_position_m()
        start = time.monotonic()

        with self._gripper_lock:
            self._gripper_close_force = close_effort
            self._gripper_hold_force = hold_effort
            self._gripper_hold_deadline = None
            self._gripper_hold_release_reason = None
            self._gripper_mode = "grasp_closing"
            self._gripper_active = True
        self._start_gripper_loop()

        while time.monotonic() - start < timeout:
            elapsed = time.monotonic() - start
            with self._gripper_lock:
                feedback_failure = self._gripper_feedback_failure_reason_locked()
                command_failure = self._gripper_command_error
                if feedback_failure is not None and self._gripper_active:
                    self._fail_active_gripper_command_locked(feedback_failure)
                    command_failure = self._gripper_command_error
            if command_failure is not None:
                reached_position_m = self.gripper_position_m()
                return (
                    False,
                    False,
                    0.0,
                    reached_position_m,
                    hold_effort,
                    command_failure,
                )
            reached_position_m = self.gripper_position_m()
            closure_m = max(start_position_m - reached_position_m, 0.0)
            if (
                elapsed >= min_time
                and closure_m >= min_closure
                and abs(float(self._gripper_vel)) <= velocity_limit
            ):
                with self._gripper_lock:
                    # 冻结当前角度作为保持目标：位置项因此把夹爪按在堵转点。
                    self._gripper_hold_angle = float(self._gripper_pos)
                    self._gripper_hold_force = hold_effort
                    self._gripper_hold_deadline = time.monotonic() + hold_timeout
                    self._gripper_mode = "grasp_holding"
                    self._gripper_active = True
                contact_position_m = self.gripper_position_m()
                return (
                    True,
                    True,
                    contact_position_m,
                    contact_position_m,
                    hold_effort,
                    # 没有力传感器：这里是"闭合行程 + 速度堵转"，即堵转检测，
                    # 不是实测接触力。
                    "closing stalled; holding "
                    f"(bounded to {hold_timeout:g} s)",
                )
            time.sleep(0.01)

        # 上游此处在切到 idle 时没有做中性化，导致最后一次闭合力矩一直被施加。
        with self._gripper_lock:
            self._release_grasp_hold_locked("close timeout before stall")
        return (
            False,
            False,
            0.0,
            self.gripper_position_m(),
            hold_effort,
            "grasp close timeout before stall detected",
        )

    def get_gripper_state(self) -> tuple[float, float, float, int]:
        """返回夹爪 (位置 rad, 速度 rad/s, 力矩 N.m, 状态码)。

        状态码 255 表示"未知/不可用"：反馈缺失、过期、坐标非法或读取异常时都会
        返回 255，调用方据此判定夹爪不可信。
        """
        with self._gripper_lock:
            position = self._gripper_pos
            velocity = self._gripper_vel
            torque = self._gripper_torque
        status = 255
        if self._gripper_mot is not None:
            try:
                st = self._verified_feedback_sample("gripper").state
                position, velocity, torque, status = self._validated_gripper_feedback_values(st)
                with self._gripper_lock:
                    feedback_error = self._gripper_feedback_error
                if feedback_error is not None:
                    status = 255
                else:
                    age = self._gripper_feedback_age_sec()
                    if age > self._gripper_feedback_stale_timeout_sec:
                        self._record_gripper_feedback_error(
                            "gripper feedback stale: "
                            f"age={age:.3f}s "
                            f"limit={self._gripper_feedback_stale_timeout_sec:.3f}s"
                        )
                        status = 255
            except Exception as exc:
                self._record_gripper_feedback_error(str(exc))
                status = 255
        with self._gripper_lock:
            if self._gripper_coordinate_failure_reason_locked() is not None:
                status = 255
        return position, velocity, torque, status

    def gripper_position_m(self) -> float:
        """把夹爪原始角度换算为开口距离（m，0 表示闭合）。"""
        with self._gripper_lock:
            if self._gripper_coordinate_failure_reason_locked() is not None:
                # 坐标非法时返回 NaN，让上层无法把无效值当距离使用。
                return float("nan")
            distance = (self._gripper_pos / _G_ANGLE_OPEN) * _G_MAX_DIST_M
        # 只有落在已验证反馈容差内的端点越界才会到这里被夹紧。
        return float(np.clip(distance, 0.0, _G_MAX_DIST_M))

    def gripper_reached_target(self) -> bool:
        """查询当前位置命令是否到位（不会推进状态机）。"""
        with self._gripper_lock:
            if self._gripper_position_result == "succeeded":
                return True
            if self._gripper_position_result == "failed" or not self._gripper_active:
                return False
            feedback_failure = self._gripper_feedback_failure_reason_locked()
            if feedback_failure is not None:
                self._fail_active_gripper_command_locked(feedback_failure)
                return False
            goal = self._gripper_goal_angle
            return abs(self._gripper_pos - goal) < _G_ARRIVE_TOL

    def send_gripper_motor_cmd(self, cmd) -> None:
        """低层夹爪直通命令（JointMotorCmd）：未给出的字段回落到反馈或配置增益。

        直通后夹爪状态机被置为 idle，避免与高层位置/抓取命令互相干扰。
        mode：0 = MIT，1 = 位置-速度，2 = 速度。
        """
        self._require_enabled()
        if self._gripper_mot is None or self._gripper_cfg is None:
            raise RuntimeError("gripper is not initialized")
        with self._gripper_lock:
            feedback_failure = self._gripper_feedback_failure_reason_locked()
        if feedback_failure is not None:
            raise RuntimeError(feedback_failure)
        state = self._verified_feedback_sample("gripper").state
        pos = float(cmd.pos) if cmd.use_pos else float(state.pos if state is not None else 0.0)
        vel = float(cmd.vel) if cmd.use_vel else float(state.vel if state is not None else 0.0)
        kp = float(cmd.kp) if cmd.use_kp else float(self._gripper_cfg.kp)
        kd = float(cmd.kd) if cmd.use_kd else float(self._gripper_cfg.kd)
        tau = float(cmd.tau) if cmd.use_tau else 0.0
        vlim = float(cmd.vlim) if cmd.use_vlim else float(self._gripper_cfg.vlim)

        if int(cmd.mode) == 0:
            self._gripper_mot.send_mit(pos, vel, kp, kd, tau)
        elif int(cmd.mode) == 1:
            self._gripper_mot.send_pos_vel(pos, vlim)
        elif int(cmd.mode) == 2:
            if not hasattr(self._gripper_mot, "send_vel"):
                raise RuntimeError("gripper does not support send_vel")
            self._gripper_mot.send_vel(vel)
        else:
            raise ValueError(f"unsupported JointMotorCmd mode: {cmd.mode}")
        with self._gripper_lock:
            self._gripper_active = False
            self._gripper_mode = "idle"

    def _patch_arm_bus_lock(self) -> None:
        """给机械臂（含夹爪）控制器与电机方法统一加上总线互斥。

        只打一次补丁：``_bus_lock_patched`` 防止重复包装导致锁嵌套。
        """
        for ctrl in self._arm._ctrl_map.values():
            self._patch_controller_bus(ctrl)

        if not hasattr(self._arm, "_bus_lock_patched"):
            for jc in self._arm._joints:
                mot = self._arm._motor_map[jc.name]
                ctrl = self._arm._ctrl_map[jc.vendor]
                self._wrap_motor_bus(mot, ctrl._bus_lock)
            self._arm._bus_lock_patched = True

    @staticmethod
    def _patch_controller_bus(ctrl) -> None:
        """给控制器的 poll_feedback_once/enable_all/disable_all 包上可重入锁。

        这几个方法内部会做多次总线读写，必须在同一把锁内完成，否则与其他线程
        的电机命令交错会串帧。重复调用是幂等的。
        """
        if not hasattr(ctrl, "_bus_lock"):
            ctrl._bus_lock = threading.RLock()
        if hasattr(ctrl, "_bus_lock_patched"):
            return
        lock = ctrl._bus_lock

        def _wrap(fn, _lock=lock):
            def _locked(*args, **kwargs):
                with _lock:
                    return fn(*args, **kwargs)

            return _locked

        for attr in ("poll_feedback_once", "enable_all", "disable_all"):
            if hasattr(ctrl, attr):
                wrapped = _wrap(getattr(ctrl, attr))
                # 标记位用于 _wrap_motor_bus 判定是否已包装，避免重复加锁。
                wrapped._rebotarm_locked = True
                setattr(ctrl, attr, wrapped)
        ctrl._bus_lock_patched = True

    @staticmethod
    def _wrap_motor_bus(mot, lock) -> None:
        """把单个电机的总线类方法包上同一把锁（幂等，已包装的跳过）。"""
        def _wrap(fn, _lock=lock):
            def _locked(*args, **kwargs):
                with _lock:
                    return fn(*args, **kwargs)

            return _locked

        for attr in (
            "send_pos_vel",
            "send_mit",
            "send_vel",
            "request_feedback",
            "enable",
            "disable",
            "ensure_mode",
            "write_register_f32",
            "set_zero_position",
        ):
            if hasattr(mot, attr) and not hasattr(getattr(mot, attr), "_rebotarm_locked"):
                wrapped = _wrap(getattr(mot, attr))
                wrapped._rebotarm_locked = True
                setattr(mot, attr, wrapped)

    def _start_pos_vel_loop(self, target: np.ndarray | None = None) -> None:
        """启动统一硬件控制循环（位置-速度跟踪），并把控制器标记为运行中。

        ``target`` 为空时以当前角度为目标（保持不动）。
        """
        if self.control_loop_active:
            return
        if target is None:
            self.hold_current_position()
        else:
            self._endpos_ctrl._q_target[:] = np.array(target, dtype=np.float64)
        self._arm.start_control_loop(self._endpos_hardware_tick)
        self._endpos_ctrl._running = True

    def _endpos_hardware_tick(self, arm, dt: float) -> None:
        self._hardware_control_tick(arm, dt, self._endpos_ctrl._loop_cb)

    def _gravity_hardware_tick(self, arm, dt: float) -> None:
        timing = getattr(self, '_gravity_timing', None)
        if timing is None:
            timing = self._gravity_timing = LoopTiming(float(arm._rate))
        started = time.monotonic()
        actual_dt = timing.begin(started)
        try:
            self._hardware_control_tick(arm, actual_dt, self._gravity_comp_tick)
        except GravityCommandError as exc:
            self._protective_disable_from_hardware_loop(str(exc), category='GRAVITY_COMMAND')
            return
        except Exception as exc:
            # Programming/model failure must not kill the sole writer thread
            # and leave a stale MIT target running on healthy motors.
            try:
                self._gravity_fault_hold(str(exc))
                q, qd = self._read_gravity_comp_feedback()
                self._emit_bounded_gravity_command(
                    q, qd, self._gravity_comp_hold_feedforward, actual_dt)
            except Exception as hold_error:
                self._protective_disable_from_hardware_loop(str(hold_error))
                return
        finally:
            timing.finish(time.monotonic())
        if not self._gravity_comp_active:
            return
        if timing.faulted:
            self._gravity_fault_hold('three consecutive control deadline misses')
        metrics = getattr(self, '_gravity_feedback_metrics', {})
        with self._feedback_lock:
            for name, sample in zip(self.joint_names, self._verified_feedback_samples(self.joint_names)):
                entry = metrics.setdefault(name, dict(sequence=sample.sequence, first=sample.observed_at,
                                                       last=sample.observed_at, updates=0))
                if sample.sequence != entry['sequence']:
                    entry.update(sequence=sample.sequence, last=sample.observed_at,
                                 updates=entry['updates'] + 1)
                elapsed = entry['last'] - entry['first']
                entry['effective_rate_hz'] = entry['updates'] / elapsed if elapsed > 0 else 0.
                entry['age_sec'] = max(0., time.monotonic() - sample.observed_at)
        self._gravity_feedback_metrics = metrics
        if started >= getattr(self, '_gravity_next_log', 0.):
            _LOG.info('gravity diagnostics: %s', self.gravity_compensation_diagnostics())
            self._gravity_next_log = started + 5.

    def _protective_disable_from_hardware_loop(self, reason: str, *, category: str = 'FEEDBACK') -> None:
        """停掉唯一写入者并请求控制器失能，且不自连接（避免线程自 join 死锁）。"""
        message = str(reason)
        with self._gripper_lock:
            if self._gripper_active:
                self._fail_active_gripper_command_locked(message)
            if self._gripper_neutral_pending is not None:
                if not self._emit_pending_gripper_neutral_locked():
                    self._error_codes.append(
                        "FEEDBACK_PROTECTIVE_NEUTRAL_FAILED"
                    )

        # RobotArm.disable()/stop_control_loop() 会 join ``_ctrl_thread``，绝不能被
        # 该线程自己调用。本回调返回后，厂商循环读到 False 自然释放线程所有权。
        self._arm._running = False
        self._endpos_ctrl._running = False
        self._endpos_ctrl._stop_send.set()
        self._endpos_ctrl._moving = False
        self._gravity_comp_active = False
        self._gravity_comp_q_target = None
        self._gravity_comp_integral = None
        self._state_machine = "IDLE"
        self._set_lifecycle_state("DISABLING")
        code = f'{category}_PROTECTIVE_DISABLE'
        if code not in self._error_codes:
            self._error_codes.append(code)

        controllers: list[object] = []
        for controller in getattr(self._arm, "_ctrl_map", {}).values():
            if all(controller is not existing for existing in controllers):
                controllers.append(controller)
        if self._gripper_ctrl is not None and all(
            self._gripper_ctrl is not controller for controller in controllers
        ):
            controllers.append(self._gripper_ctrl)
        disable_errors: list[str] = []
        for controller in controllers:
            disable_all = getattr(controller, "disable_all", None)
            if not callable(disable_all):
                continue
            try:
                disable_all()
            except Exception as exc:
                disable_errors.append(f"{type(controller).__name__}: {exc}")
        if disable_errors:
            self._error_codes.append(
                f"{category}_PROTECTIVE_DISABLE_FAILED: " + "; ".join(disable_errors)
            )
        _LOG.error(
            "hardware writer stopped and controller protective disable requested: %s",
            message,
        )

    def _hardware_control_tick(self, arm, dt: float, arm_callback) -> None:
        """机械臂命令、反馈批次与夹爪命令的唯一所有者。

        每周期顺序：刷新反馈 -> 检查反馈健康 -> 执行模式回调（位置跟踪或重力
        补偿）-> 推进夹爪状态机。若反馈刷新或回调因反馈类错误失败，则执行保护性
        失能并结束本次 tick；非反馈类异常继续向上抛出（由厂商循环处理），
        以免把编程错误当成"反馈故障"而静默失能。
        """
        try:
            self.refresh_feedback_if_due()
        except Exception:
            feedback_failure = self._arm_feedback_failure_reason()
            if feedback_failure is None:
                raise
            self._protective_disable_from_hardware_loop(feedback_failure)
            return
        feedback_failure = self._arm_feedback_failure_reason()
        if feedback_failure is not None:
            self._protective_disable_from_hardware_loop(feedback_failure)
            return
        try:
            arm_callback(arm, dt)
        except Exception:
            feedback_failure = self._arm_feedback_failure_reason()
            if feedback_failure is None:
                raise
            self._protective_disable_from_hardware_loop(feedback_failure)
            return
        self._gripper_tick()

    def _stop_control_loop(self) -> None:
        self._arm.stop_control_loop()
        thread = getattr(self._arm, '_ctrl_thread', None)
        if thread is not None and thread.is_alive():
            raise RuntimeError('hardware writer did not stop; refusing a second writer')
        self._endpos_ctrl._running = False

    def _gripper_safe_mit(
        self,
        pos: float,
        vel: float,
        kp: float,
        kd: float,
        tau_ff: float = 0.0,
        tau_limit: float = _G_TAU_MAX,
    ) -> None:
        """下发夹爪 MIT 命令，并保证总力矩不超限。

        位置命令先夹到 [_G_OPEN_SOFT_LIMIT, 0]（不允许命令超过张开软限位或越过
        闭合零点）。力矩处理是"先算位置项，再把总力矩夹到 +/-limit，最后反解出
        等效前馈"：这样即使 tau_ff 很大，实际输出也不会超过 limit。
        """
        if self._gripper_mot is None or self._gripper_ctrl is None:
            return
        pos_cmd = float(np.clip(pos, _G_OPEN_SOFT_LIMIT, 0.0))
        # 本地计算的 PD 位置项，用于后面的饱和反解。
        pos_term = kp * (pos_cmd - self._gripper_pos) + kd * (-self._gripper_vel)
        limit = float(np.clip(abs(tau_limit), 0.05, _G_TAU_MAX))
        tau_safe = float(np.clip(pos_term + tau_ff, -limit, limit)) - pos_term
        lock = getattr(self._gripper_ctrl, "_bus_lock", None)
        try:
            if lock:
                with lock:
                    self._gripper_mot.send_mit(pos_cmd, vel, kp, kd, tau_safe)
            else:
                self._gripper_mot.send_mit(pos_cmd, vel, kp, kd, tau_safe)
        except Exception as exc:
            raise RuntimeError(f"gripper MIT command failed: {exc}") from exc

    def _fail_active_gripper_command_locked(self, reason: str) -> None:
        """把当前夹爪命令标记为失败并排队一次中性释放（调用方需持锁）。"""
        message = str(reason)
        self._gripper_command_error = message
        self._gripper_position_result = "failed"
        self._gripper_active = False
        self._gripper_mode = "idle"
        self._gripper_target_deadline_monotonic = None
        _LOG.error(
            "gripper command failed reason=%s goal=%.6frad command=%.6frad "
            "feedback=%.6frad updated=%s age=%.6fs",
            message,
            self._gripper_goal_angle,
            self._gripper_target_angle,
            self._gripper_pos,
            self._gripper_feedback_updated_monotonic,
            self._gripper_feedback_age_sec(),
        )
        self._queue_gripper_neutral_locked(message, marks_success=False)

    def _queue_gripper_neutral_locked(
        self,
        reason: str,
        *,
        marks_success: bool,
    ) -> None:
        """排队一条中性 MIT 命令（零刚度/阻尼/前馈），由硬件循环在下一拍发出。

        只排队不直接发送，保证总线上只有一个写入者。``marks_success`` 决定该
        中性命令发送成功后是否把本次位置命令标记为成功。
        """
        self._gripper_neutral_pending = (
            float(self._gripper_pos),
            str(reason),
            bool(marks_success),
        )

    def _emit_pending_gripper_neutral_locked(self) -> bool:
        """发送已排队的中性命令；返回是否成功（调用方需持 ``_gripper_lock``）。"""
        pending = self._gripper_neutral_pending
        if pending is None:
            return False
        angle, reason, marks_success = pending
        self._gripper_neutral_pending = None
        try:
            self._gripper_safe_mit(
                angle,
                0.0,
                0.0,
                0.0,
                tau_limit=0.05,
            )
        except Exception as exc:
            self._gripper_position_result = "failed"
            self._gripper_command_error = (
                f"gripper neutral command failed after {reason}: {exc}"
            )
            _LOG.error("%s", self._gripper_command_error)
            return False
        if marks_success:
            self._gripper_position_result = "succeeded"
            self._gripper_command_error = None
        return True

    def cancel_gripper_position_command(self, reason: str = "position command canceled") -> bool:
        """取消进行中的位置命令；仅在确实处于 position 模式时生效。"""
        with self._gripper_lock:
            if not self._gripper_active or self._gripper_mode != "position":
                return False
            self._fail_active_gripper_command_locked(reason)
            return True

    def _release_gripper_position_target(
        self,
        expected_goal: float,
        *,
        require_arrived: bool = True,
    ) -> bool:
        """结束一次常规位置移动，并让夹爪回到空闲。

        上游 ``wait_gripper_target`` 只返回成功而不改 ``_gripper_active``/
        ``_gripper_mode``，于是统一硬件循环在服务已经应答之后仍在继续下发 MIT
        位置命令。这里改为原子地切到空闲，并在确认成功之前为唯一的硬件循环写入者
        排队一条中性 MIT 命令（零刚度、零阻尼、零前馈）。

        整个过程持有夹爪锁，避免过期的完成路径覆盖更新的目标。``grasp_closing``
        与 ``grasp_holding`` 是显式请求的力操作，绝不在此释放。
        """
        with self._gripper_lock:
            return self._release_gripper_position_target_locked(
                expected_goal,
                require_arrived=require_arrived,
            )

    def _release_gripper_position_target_locked(
        self,
        expected_goal: float,
        *,
        require_arrived: bool = True,
    ) -> bool:
        """``_release_gripper_position_target`` 的本体；调用方必须持有锁。

        拆成两部分是为了让已经持有 ``_gripper_lock`` 的调用方也能复用同一份
        所有权检查与中性化逻辑。
        """
        if not self._gripper_active or self._gripper_mode != "position":
            return False
        if self._gripper_goal_angle != expected_goal:
            return False
        arrived_angle = float(self._gripper_pos)
        if require_arrived and abs(arrived_angle - expected_goal) >= _G_ARRIVE_TOL:
            return False
        # 在排队中性命令之前先清掉所有权。统一硬件循环先判断 _gripper_active，
        # 因此它一旦为假就不会再产生任何位置命令，排队的中性命令即是最后一条。
        self._gripper_target_angle = arrived_angle
        self._gripper_active = False
        self._gripper_mode = "idle"
        self._gripper_target_deadline_monotonic = None
        if require_arrived:
            self._gripper_position_result = "releasing"
        elif self._gripper_position_result != "failed":
            self._gripper_position_result = "failed"
        self._queue_gripper_neutral_locked(
            "position target reached" if require_arrived else "position target canceled",
            marks_success=require_arrived,
        )
        return True

    def _grasp_hold_expired_locked(self) -> bool:
        """有界抓取保持是否已到时。调用方需持锁。"""
        deadline = self._gripper_hold_deadline
        if deadline is None:
            return False
        return time.monotonic() >= deadline

    def _release_grasp_hold_locked(self, reason: str) -> None:
        """对抓取闭合/保持做中性化并回到空闲。调用方需持锁。

        上游会让 ``grasp_holding`` 一直带电到下一条命令，其闭合超时路径也只是切
        到 idle 而从不中性化最后一条力矩命令。两条路径现在都汇到这里。
        """
        self._gripper_hold_deadline = None
        self._gripper_hold_release_reason = reason
        self._gripper_active = False
        self._gripper_mode = "idle"
        self._queue_gripper_neutral_locked(
            f"grasp release: {reason}",
            marks_success=False,
        )

    def release_grasp_hold(self, reason: str = "external release") -> bool:
        """释放进行中的抓取保持，且不下发新的位置命令。"""
        with self._gripper_lock:
            if not self._gripper_active:
                return False
            if self._gripper_mode not in ("grasp_closing", "grasp_holding"):
                return False
            self._release_grasp_hold_locked(reason)
        return True

    @property
    def grasp_hold_release_reason(self) -> str | None:
        with self._gripper_lock:
            return self._gripper_hold_release_reason

    def _gripper_tick(self) -> None:
        """夹爪状态机的单周期推进（由统一控制循环调用）。

        优先级：先发已排队的中性命令；无活动命令则直接返回。之后在持锁状态下
        一次性完成"读状态 -> 判定 -> 生成命令"，保证到位判定与命令下发原子，
        避免已经通过到位判定的 tick 在等待者释放后又多发一条带力矩的命令
        （真机上曾观测到服务返回后仍多闭合约 1.2-1.33 mm）。向零位闭合是最敏感
        方向，因为 abs(target) < 1e-6 时 effort 会变成前馈项。

        位置模式使用按时间步长限速的斜坡（速度上限 = 参数 rad/s），抓取闭合/
        保持模式使用固定的 kp/kd 与前馈力矩，且保持受时限约束。
        """
        with self._gripper_lock:
            if self._gripper_neutral_pending is not None:
                self._emit_pending_gripper_neutral_locked()
                return
            if not self._gripper_active or self._gripper_mot is None:
                return
        try:
            st = self._verified_feedback_sample("gripper").state
        except Exception as exc:
            self._record_gripper_feedback_error(str(exc))
            with self._gripper_lock:
                if self._gripper_active:
                    self._fail_active_gripper_command_locked(
                        f"gripper feedback read failed: {exc}"
                    )
            return

        # 到位判定与命令下发必须原子。先取状态、放开锁、再发送，会让已经通过
        # 到位判定的 tick 在 wait_gripper_target 释放之后再发出一条带力矩的命令
        # ——真机上表现为服务返回后又多闭合约 1.2-1.33 mm。向零位闭合是最暴露的
        # 方向，因为 abs(target) < 1e-6 时 effort 会变成前馈项。
        with self._gripper_lock:
            mode = self._gripper_mode
            if not self._gripper_active:
                command = None
            elif (feedback_failure := self._gripper_feedback_failure_reason_locked()) is not None:
                self._fail_active_gripper_command_locked(feedback_failure)
                command = None
            else:
                try:
                    position, velocity, torque, status = (
                        self._validated_gripper_feedback_values(st)
                    )
                except Exception as exc:
                    self._record_gripper_feedback_error(str(exc))
                    self._fail_active_gripper_command_locked(
                        f"gripper feedback read failed: {exc}"
                    )
                    return
                self._gripper_pos = position
                self._gripper_vel = velocity
                self._gripper_torque = torque
                self._gripper_status_code = status
                if status != 1:
                    self._fail_active_gripper_command_locked(
                        f"gripper status_code={status}, expected 1 during command"
                    )
                    return
                command = None

            if command is None and self._gripper_active and mode == "position":
                goal = self._gripper_goal_angle
                if abs(self._gripper_pos - goal) < _G_ARRIVE_TOL:
                    # 无同步等待者时到达，或在等待者返回之后到达。
                    self._release_gripper_position_target_locked(goal)
                    command = None
                else:
                    now = time.monotonic()
                    previous_tick = self._gripper_last_tick_monotonic
                    elapsed = 0.0 if previous_tick is None else max(now - previous_tick, 0.0)
                    # 本拍允许的最大角度增量 = 速度上限 * 实际经过时间。
                    max_step = self._gripper_position_max_speed_rad_s * elapsed
                    target = self._gripper_target_angle
                    remaining = goal - target
                    if abs(remaining) <= max_step:
                        target = goal
                    elif max_step > 0.0:
                        target += float(np.copysign(max_step, remaining))
                    self._gripper_target_angle = target
                    self._gripper_last_tick_monotonic = now
                    effort = self._gripper_target_effort
                    # ``effort`` 约束整段移动而不只是闭合终点；若不显式传
                    # tau_limit，命令力矩会回落到 _G_TAU_MAX。
                    tau_ff = effort if abs(target) < 1e-6 else 0.0
                    command = (target, 0.0, _G_KP_MOVE, _G_KD_MOVE, tau_ff, effort)
            elif command is None and self._gripper_active and mode == "grasp_closing":
                # 闭合：kp=0，仅靠阻尼 + 前馈力矩推进，撞到物体即自然堵转。
                command = (
                    0.0,
                    0.0,
                    _G_GRASP_CLOSE_KP,
                    _G_GRASP_CLOSE_KD,
                    self._gripper_close_force,
                    _G_TAU_MAX,
                )
            elif command is None and self._gripper_active and mode == "grasp_holding":
                if self._grasp_hold_expired_locked():
                    self._release_grasp_hold_locked("hold timeout")
                    command = None
                else:
                    # 保持：位置目标冻结在堵转角，位置项提供主夹持力。
                    command = (
                        self._gripper_hold_angle,
                        0.0,
                        _G_GRASP_HOLD_KP,
                        _G_GRASP_HOLD_KD,
                        self._gripper_hold_force,
                        _G_TAU_MAX,
                    )
            if command is None:
                if self._gripper_neutral_pending is not None:
                    self._emit_pending_gripper_neutral_locked()
                return

            pos, vel, kp, kd, tau_ff, tau_limit = command
            try:
                self._gripper_safe_mit(pos, vel, kp, kd, tau_ff, tau_limit=tau_limit)
            except Exception as exc:
                self._fail_active_gripper_command_locked(str(exc))

    def _start_gripper_loop(self) -> None:
        """夹爪命令的入口校验：夹爪没有独立循环，必须依赖统一硬件循环。"""
        if not self.control_loop_active:
            raise RuntimeError(
                "gripper command requires the unified hardware control loop"
            )

    def _stop_gripper_loop(self) -> None:
        # 仅复位兼容标志位；夹爪命令由统一硬件循环承载，没有独立线程需要 join。
        self._gripper_loop_running = False

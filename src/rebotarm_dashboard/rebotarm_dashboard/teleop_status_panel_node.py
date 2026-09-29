"""Web 遥操作状态面板节点：本机浏览器与机械臂之间的聚合与转发层。

职责与位置
    本文件是面板包唯一常驻的 ROS 2 节点。它做两件事：

    1. 状态聚合：订阅关节反馈、逐电机状态、整臂状态以及遥操作/示教回放状态话题，
       汇总进进程内的状态存储；前端通过 ``/events`` 的 SSE 流按固定周期取走快照。
    2. 命令转发：接收本机页面发来的 POST 请求，做统一的安全闸门检查后，把请求转交
       给对应包的能力实现——示教记录/回放编排、点对点关节运动与夹爪适配、整臂服务
       调用——并把结果与状态回写给前端。

    本节点不含任何运动规划、重定时、回放算法或硬件 SDK 代码，这些能力分别位于示教包、
    操作包与控制器包；面板只负责编排与呈现。

对外接口
    订阅：``/{ns}/joint_states``、``/{ns}/arm_status``、``/{ns}/joints/<joint>/state``、
        ``/{ns}/gripper/state``、``/{ns}/teleop/status``、``/{ns}/teleop/recording_status``、
        ``/{ns}/teleop/replay_status``；其中 ``{ns}`` 由参数 ``arm_namespace`` 决定。
    调用：动作 ``/{ns}/follow_joint_trajectory``、``/{ns}/gripper/command``；服务
        ``/{ns}/enable``、``/{ns}/disable``、``/{ns}/safe_home``、``/{ns}/trajectory_stop``、
        ``/{ns}/gravity_compensation/{start,stop}``、``/{ns}/teleop/teach_record/{start,set_path,stop}``。
    发布：仅在不接真实硬件时发布仿真夹爪状态到 ``/{ns}/gripper/state``。
    HTTP（只监听本机 UI/API）：GET ``/`` 页面、``/events`` SSE、``/api/status``、
        ``/api/config``、``/api/teach_record_info``、``/api/teach_records``、
        ``/api/teach_trajectory``、``/robot/urdf``、``/robot/meshes/<name>``；
        POST 路由表见同包的 ``status_panel_api``。

安全约束
    - 所有会影响硬件的命令都必须先过 ``execution_mode`` 闸门：只有取值为 "execute" 时
      网关才放行；其余取值一律返回 state="dry_run" 且不下发任何目标。
    - 面板下发命令还要求 ``web_execute_enabled`` 为真；默认关闭，必须由操作者在启动时显式打开。
    - 整臂命令（使能/失能/回原点）在示教回放或停止过程中被锁定，避免人工操作与回放轨迹互相打架。
    - ``use_hardware`` 为假时夹爪只做仿真：由本节点自己发布夹爪反馈，保证无硬件时界面仍可用。

线程模型
    HTTP 服务在独立线程中处理请求与 SSE，ROS 执行器线程负责订阅回调、定时器与动作回调，
    两者共享同一个加锁的状态存储；``_execute_lock`` 只保护当前 web 执行目标句柄的读写。
"""

from __future__ import annotations

from rebotarm_msgs.srv import CalibrationCommand
from .calibration_client import CalibrationClient
from .calibration_gravity import gravity_rejection, GravityRequestTracker


import json
import math
import threading
import time
from contextlib import suppress
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory, GripperCommand
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rebotarm_msgs.msg import ArmStatus, JointMotorState
from rebotarm_msgs.srv import SetTeachRecordPath
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .arm_control_client import ArmControlClient
from .arm_command_api import (
    arm_command_is_replay_locked,
    normalize_arm_command,
    should_stop_trajectory_before_arm_command,
    status_state,
)
from .parameter_helpers import build_joint_limits
from .parameter_helpers import sensor_qos_kwargs
from .status_panel_http import create_status_panel_server
from .status_panel_page import HTML_PAGE
from .status_panel_state import TeleopStatusStore
from .web_command_gateway import WebCommandGateway, WebCommandRequest
from rebotarm_teleop.teleop_core import validate_web_keyboard_command
from rebotarm_teach.teach_record_client import TeachRecordClient
from rebotarm_teach.teach_replay_workflow import TeachReplayWorkflow
from rebotarm_teach.teach_replay_trajectory_builder import set_duration
from .web_robot_assets import (
    DEFAULT_GRIPPER_LIMITS_M,
    gripper_opening_to_finger_joint_positions,
    load_gripper_limits,
    load_moveit_velocity_limits,
    load_urdf_joint_limits,
    merge_velocity_limits,
    merge_joint_limits,
)
from rebotarm_teleop.web_execute import (
    WebExecuteDecision,
    WebGripperDecision,
)
from rebotarm_teleop.web_teleop_client import WebTeleopClient, decision_response, gripper_decision_response


def _set_duration(duration_msg, seconds: float) -> None:
    """把浮点秒数写入时长消息（``sec`` + ``nanosec``）。

    这里只是转发示教包中同名的轨迹构建工具，避免面板再实现一份时间换算；纳秒部分由
    被调用方截断取整，因此写入值最多比 ``seconds`` 小 1 ns。
    """
    set_duration(duration_msg, seconds)


def _is_number_like(value) -> bool:
    """判断 ``value`` 能否被 ``float()`` 接受（字符串形式的数字也算）。

    注意：不接受 ``None``、非法字符串等无法转换的对象；本函数不判断有限性，
    NaN/Infinity 会返回 True。
    """
    try:
        float(value)
    except (TypeError, ValueError):
        return False
    return True


def _number_or_default(value, default: float) -> float:
    """把 Web 请求里的数值类字段转成 float，取不到有效值时退回 ``default``。

    Web 传来的 JSON 值可能是字符串、null 或缺失，因此先尝试转换；再排除非有限值，
    因为 NaN/Infinity 会污染后续的插值与限位比较（限位比较对 NaN 恒为假，会导致
    越界值被静默放过）。
    """
    try:
        number = float(value)
    except (TypeError, ValueError):
        return float(default)
    if not _is_number_like(number) or not math.isfinite(number):
        return float(default)
    return float(number)


def _decision_response(decision: WebExecuteDecision) -> dict:
    """把关节运动校验结果转成前端 JSON 契约（放行标志、说明、增量与其上限、时长）。"""
    return {
        "accepted": bool(decision.accepted),
        "message": decision.message,
        "max_delta": float(decision.max_delta),
        "max_delta_limit": float(decision.max_delta_limit),
        "duration": float(decision.duration),
    }


def _keyboard_decision_response(decision) -> dict:
    """把键盘单步校验结果转成前端 JSON 契约。

    比关节运动多带 ``key`` / ``joint_name`` / ``step_rad``，便于界面回显"哪一步落在哪个关节"。
    """
    return {
        "accepted": bool(decision.accepted),
        "message": decision.message,
        "key": str(decision.key),
        "joint_name": str(decision.joint_name),
        "step_rad": float(decision.step_rad),
        "duration": float(decision.duration),
        "max_joint_speed_rad_s": float(decision.max_joint_speed_rad_s),
    }




class TeleopStatusPanelNode(Node):
    """面板常驻节点：聚合机械臂状态、承载本机 HTTP/SSE 服务、转发操作者命令。

    生命周期
        构造时完成四件事：声明全部参数、构建关节/夹爪限位等派生配置、创建动作与服务客户端
        以及各能力适配器（整臂控制、示教记录、示教回放工作流、Web 点对点运动），最后拉起
        HTTP 服务线程。此后长期驻留，直到 :meth:`destroy_node` 关闭 HTTP 服务。

    状态更新的来源
        关节反馈、逐电机反馈、整臂状态、遥操作/回放状态四条订阅链路，加上两个定时器
        （仿真夹爪状态发布、重力补偿状态汇总）与回放跟踪监控定时器；全部写入同一个
        线程安全的状态存储，前端通过 ``/events`` 拉取快照。

    参数（全部通过节点参数声明，实际取值通常来自 operator_common.yaml、web_teleop.yaml、
    teach_control.yaml 与 launch 参数覆盖；单位与默认行为见下方分组注释）
        - 名称空间与 HTTP 服务：``arm_namespace``、``host``、``port``、``sse_rate_hz``。
        - 夹爪开口与力矩：``gripper_lower_limit_m``、``gripper_upper_limit_m``、
          ``web_gripper_max_effort``、``web_gripper_max_effort_limit``。
        - 示教记录/回放：``record_path`` 及 ``direct_threshold`` … ``max_prepared_jump_rad``
          一整组（含义与单位与示教回放节点保持一致的镜像参数）。
        - Web 点对点运动：``web_execute_*``、``web_keyboard_*``。
        - 运行模式与安全开关：``use_hardware``、``execution_mode``、``panel_mode``、
          ``web_execute_enabled``。
        - 关节几何：``joint_names``、``joint_lower_limits``、``joint_upper_limits``
          （仅作 URDF 读取失败时的回退值）。

    线程与并发
        HTTP 线程与执行器线程并发访问状态存储（内部加锁）；:attr:`_execute_lock` 保护当前
        web 执行目标句柄，:attr:`_web_keyboard_lock` 保护键盘步长/时长/速度这组可调参数。
    """

    def __init__(self) -> None:
        super().__init__("teleop_status_panel_node")
        # ---------------- 名称空间与 HTTP 服务参数 ----------------
        # arm_namespace：话题/服务/动作名中的机械臂命名空间段，去首尾斜杠后拼成 /{ns}/...。
        self.declare_parameter("arm_namespace", "rebotarm")
        # host / port：面板 HTTP 服务监听地址与端口，仅用于本机 UI/API，默认只绑回环地址。
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 8088)
        # sse_rate_hz：SSE 状态推送频率（Hz），内部按 >=1.0 取下限，周期 = 1/该值 秒。
        self.declare_parameter("sse_rate_hz", 10.0)
        # ---------------- 示教记录与回放参数 ----------------
        # record_path：原始示教记录 JSONL 路径；相对路径按节点工作目录解析。
        self.declare_parameter("record_path", "teleop_records/teach_record.jsonl")
        # direct_threshold（rad）：起点分类下界，最大关节误差小于该值判为 direct，直接回放。
        self.declare_parameter("direct_threshold", 0.01)
        # align_threshold（rad）：起点分类上界，误差在 [direct_threshold, align_threshold) 判为
        # align（先插值对齐）；达到或超过该值判为 reject，拒绝回放。
        self.declare_parameter("align_threshold", 0.25)
        # align_duration（s）：固定对齐时长，仅在 align_duration_auto 为假时使用。
        self.declare_parameter("align_duration", 3.0)
        # align_duration_auto（bool）：为真时对齐时长由起始误差与 align_target_speed_rad_s 自动推算。
        self.declare_parameter("align_duration_auto", True)
        # align_target_speed_rad_s（rad/s）：自动对齐时假定的关节运动速度，越小对齐越慢。
        self.declare_parameter("align_target_speed_rad_s", 0.15)
        # align_min_duration / align_max_duration（s）：自动对齐时长的下限与上限，
        # 误差很小或很大时对齐时长都被夹在该区间内。
        self.declare_parameter("align_min_duration", 3.0)
        self.declare_parameter("align_max_duration", 10.0)
        # align_steps（点数）：对齐段的插值点数，内部至少取 2 点。
        self.declare_parameter("align_steps", 30)
        # replay_speed（倍率）：回放速度倍率，无量纲；规范化时夹到 [0.1, 1.0]，只允许减速，
        # 大于 1.0 不会加速。
        self.declare_parameter("replay_speed", 1.0)
        # green_jump_rad / yellow_jump_rad（rad）：轨迹质量分级阈值——相邻样本跳变不超过
        # green 判为绿色（低风险），不超过 yellow 判为黄色（中风险），更大则判红色并阻断回放。
        self.declare_parameter("green_jump_rad", 0.03)
        self.declare_parameter("yellow_jump_rad", 0.05)
        # yellow_max_speed（倍率）：黄色风险下允许的最大有效回放速度，超出即拒绝真实回放。
        self.declare_parameter("yellow_max_speed", 0.6)
        # max_replay_velocity_rad_s（rad/s）：关节速度上限标量，仅在逐关节列表长度与记录关节数
        # 不匹配时作为回退值使用。
        self.declare_parameter("max_replay_velocity_rad_s", 3.0)
        # max_replay_velocity_rad_s_by_joint（rad/s，列表）：逐关节速度上限，顺序与记录中的关节
        # 顺序一致；默认前 3 个肩/肘大关节 3.0，后 3 个腕关节 1.8（腕部惯量小、更易抖动）。
        self.declare_parameter(
            "max_replay_velocity_rad_s_by_joint",
            [3.0, 3.0, 3.0, 1.8, 1.8, 1.8],
        )
        # max_replay_acceleration_rad_s2（rad/s²）：关节加速度上限，用于重定时。
        self.declare_parameter("max_replay_acceleration_rad_s2", 5.0)
        # max_replay_jerk_rad_s3（rad/s³）：关节加加速度（jerk）上限，限制冲击与抖动。
        self.declare_parameter("max_replay_jerk_rad_s3", 20.0)
        # large_motion_span_rad（rad）：单关节最大行程阈值，超过即判为大范围运动。
        self.declare_parameter("large_motion_span_rad", 0.8)
        # large_motion_total_rad（rad）：各关节累计行程阈值，超过即判为大范围运动。
        self.declare_parameter("large_motion_total_rad", 2.5)
        # large_motion_max_speed（倍率）：大范围运动的推荐速度上限，随预处理结果上报给界面，
        # 不在本节点内据此二次限速。
        self.declare_parameter("large_motion_max_speed", 1.0)
        # start_hold_sec（s）：轨迹开头保持"当前位形"的时长，避免启动瞬间拉扯。
        self.declare_parameter("start_hold_sec", 0.8)
        # soft_start_duration（s）/ soft_start_steps（点数）：direct 起点软启动插值的时长与点数。
        self.declare_parameter("soft_start_duration", 1.0)
        self.declare_parameter("soft_start_steps", 30)
        # first_hold_sec（s）：对齐到记录首点之后的保持时长。
        self.declare_parameter("first_hold_sec", 0.3)
        # final_hold_sec（s）：轨迹末尾保持末点位置的时长，0 表示不追加。
        self.declare_parameter("final_hold_sec", 1.0)
        # initial_replay_delay_sec（s）：录制段相对对齐段末尾的额外延时，用于等待控制器进入跟踪状态。
        self.declare_parameter("initial_replay_delay_sec", 0.2)
        # use_moveit_start_align（bool）：起点对齐是否改走规划服务；为假时退回关节空间插值。
        self.declare_parameter("use_moveit_start_align", True)
        # moveit_start_skip_threshold（rad）：当前位形与轨迹首点误差小于该值就跳过规划，只保留保持点。
        self.declare_parameter("moveit_start_skip_threshold", 0.005)
        # moveit_group_name：规划用的规划组名；collision_group_name：碰撞预检用的规划组名，
        # 取 "arm_with_gripper" 时会自动补齐夹爪指关节（见 _collision_default_joint_positions）。
        self.declare_parameter("moveit_group_name", "arm")
        self.declare_parameter("collision_group_name", "arm_with_gripper")
        # moveit_planning_service / moveit_planning_pipeline / moveit_planner_id：规划服务名、
        # 流水线 id（ompl）与规划器 id（空串表示交给 MoveIt 选默认规划器）。
        self.declare_parameter("moveit_planning_service", "/plan_kinematic_path")
        self.declare_parameter("moveit_planning_pipeline", "ompl")
        self.declare_parameter("moveit_planner_id", "")
        # moveit_planning_time（s）/ moveit_num_planning_attempts（次）：单次规划的时间上限与尝试次数。
        self.declare_parameter("moveit_planning_time", 3.0)
        self.declare_parameter("moveit_num_planning_attempts", 3)
        # moveit_joint_goal_tolerance（rad）：规划请求的关节目标容差。
        self.declare_parameter("moveit_joint_goal_tolerance", 0.005)
        # moveit_velocity_scaling / moveit_acceleration_scaling（无量纲，(0, 1]）：规划请求的速度/
        # 加速度缩放系数，越小越保守；默认 0.1 代表以额定能力的 10% 规划。
        self.declare_parameter("moveit_velocity_scaling", 0.1)
        self.declare_parameter("moveit_acceleration_scaling", 0.1)
        # collision_check_enabled（bool）：为假时碰撞预检直接返回 disabled，不再拦截真实回放。
        self.declare_parameter("collision_check_enabled", True)
        # collision_check_service：状态有效性查询服务名。
        self.declare_parameter("collision_check_service", "/check_state_validity")
        # collision_check_max_samples（点数）：预检最多抽样的轨迹点数，内部至少 1。
        self.declare_parameter("collision_check_max_samples", 80)
        # collision_check_timeout_sec（s）：预检总超时，内部下限 0.1 s。
        self.declare_parameter("collision_check_timeout_sec", 2.0)
        # ---------------- 轨迹预处理参数 ----------------
        # smoothing_enabled（bool）/ smoothing_window（点数）：是否做滑动平均平滑及窗口长度，
        # 窗口越大越平滑但越钝。
        self.declare_parameter("smoothing_enabled", True)
        self.declare_parameter("smoothing_window", 7)
        # filter_enabled（bool）/ filter_cutoff_hz（Hz）/ filter_sample_rate_hz（Hz）：是否低通滤波、
        # 截止频率与滤波采样率；截止频率必须低于采样率的一半，否则滤波无意义。
        self.declare_parameter("filter_enabled", True)
        self.declare_parameter("filter_cutoff_hz", 5.0)
        self.declare_parameter("filter_sample_rate_hz", 150.0)
        # resample_enabled（bool）/ resample_rate_hz（Hz）：是否重采样及目标采样率，默认 150 Hz。
        self.declare_parameter("resample_enabled", True)
        self.declare_parameter("resample_rate_hz", 150.0)
        # time_parameterization_method：重定时方法，取值 "auto" / "current_jerk_retime" / "ruckig"，
        # 非法值回落为 "auto"。
        self.declare_parameter("time_parameterization_method", "auto")
        # max_prepared_jump_rad（rad）：预处理后允许的最大相邻点跳变，真实回放的门控之一。
        self.declare_parameter("max_prepared_jump_rad", 0.02)
        # ---------------- 回放运行期跟踪监控参数 ----------------
        # replay_monitor_enabled（bool）：是否启用回放期间的跟踪监控（目标位形与反馈之差）。
        self.declare_parameter("replay_monitor_enabled", True)
        # replay_monitor_period_sec（s）：监控定时器周期，内部下限 0.02 s。
        self.declare_parameter("replay_monitor_period_sec", 0.05)
        # replay_monitor_start_grace_sec（s）：起步宽限期，期间不判超差。
        self.declare_parameter("replay_monitor_start_grace_sec", 1.0)
        # replay_monitor_violation_grace_sec（s）：连续超差需持续超过该时长才触发停止，用于滤掉单帧抖动误停。
        self.declare_parameter("replay_monitor_violation_grace_sec", 0.30)
        # max_tracking_error_rad（rad）：允许的关节跟踪误差；max_live_velocity_rad_s（rad/s）：
        # 允许的反馈关节速度，二者任一持续超限都会触发安全停止。
        self.declare_parameter("max_tracking_error_rad", 0.25)
        self.declare_parameter("max_live_velocity_rad_s", 3.0)
        # ---------------- 运行模式与安全开关 ----------------
        # use_hardware（bool）：是否接真实硬件；为假时夹爪走仿真、由本节点自己发布夹爪反馈。
        self.declare_parameter("use_hardware", False)
        # execution_mode：命令网关闸门，只有 "execute" 才放行真实执行，其余取值（默认 "dry_run"）只做校验。
        self.declare_parameter("execution_mode", "dry_run")
        # panel_mode：界面工作模式，"control" 可操作；"check" 为只读，仅观察状态。
        self.declare_parameter("panel_mode", "control")
        # ---------------- 关节几何（URDF 读取失败时的回退值） ----------------
        # joint_names：受控关节名，顺序即位置向量顺序，也是各类轨迹消息中的关节顺序。
        self.declare_parameter(
            "joint_names",
            ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
        )
        # joint_lower_limits / joint_upper_limits（rad）：回退关节限位，默认 ±3.14159 rad（±180°）；
        # 正常情况下会被 URDF 中的真实限位覆盖（见下方 merge_joint_limits 调用）。
        self.declare_parameter(
            "joint_lower_limits",
            [-3.14159, -3.14159, -3.14159, -3.14159, -3.14159, -3.14159],
        )
        self.declare_parameter(
            "joint_upper_limits",
            [3.14159, 3.14159, 3.14159, 3.14159, 3.14159, 3.14159],
        )
        # ---------------- Web 点对点运动与键盘遥操作参数 ----------------
        # web_execute_enabled（bool）：面板是否允许下发运动命令；默认关闭，必须显式打开。
        self.declare_parameter("web_execute_enabled", False)
        # web_execute_max_delta_rad（rad）：单次关节目标相对当前反馈允许的最大增量。
        self.declare_parameter("web_execute_max_delta_rad", 1.5)
        # web_execute_min_duration / web_execute_max_duration（s）：请求时长被夹紧到的区间。
        self.declare_parameter("web_execute_min_duration", 1.0)
        self.declare_parameter("web_execute_max_duration", 8.0)
        # web_execute_max_joint_speed_rad_s（rad/s）：Web 端关节速度总上限，也是逐关节速度
        # 限位缺失时的回退值，同时作为键盘速度的上限。
        self.declare_parameter("web_execute_max_joint_speed_rad_s", 1.5)
        # web_keyboard_default_step_rad / min / max（rad）：键盘单步默认步长及其允许区间，
        # 请求中的步长先夹紧到 [min, max] 再使用。
        self.declare_parameter("web_keyboard_default_step_rad", 0.02)
        self.declare_parameter("web_keyboard_min_step_rad", 0.005)
        self.declare_parameter("web_keyboard_max_step_rad", 0.10)
        # web_keyboard_default_duration / min / max（s）：键盘单步默认时长及其允许区间，
        # 它与步长共同决定该步的等效角速度，进而触发限速校验。
        self.declare_parameter("web_keyboard_default_duration", 0.2)
        self.declare_parameter("web_keyboard_min_duration", 0.1)
        self.declare_parameter("web_keyboard_max_duration", 2.0)
        # web_keyboard_default_speed_rad_s（rad/s）：键盘遥操作默认速度上限。
        self.declare_parameter("web_keyboard_default_speed_rad_s", 0.5)
        # ---------------- 夹爪参数 ----------------
        # gripper_lower_limit_m / gripper_upper_limit_m（m）：夹爪开口允许区间，默认 0.0 ~ 0.09 m
        # （0 = 完全闭合）；声明时会做一次上下限排序保护。
        self.declare_parameter("gripper_lower_limit_m", DEFAULT_GRIPPER_LIMITS_M[0])
        self.declare_parameter("gripper_upper_limit_m", DEFAULT_GRIPPER_LIMITS_M[1])
        # web_gripper_max_effort：夹爪默认力矩上限；web_gripper_max_effort_limit：力矩硬上限，
        # 任何请求都不会越过它（量纲为夹爪固件定义的归一化值）。
        self.declare_parameter("web_gripper_max_effort", 1.0)
        self.declare_parameter("web_gripper_max_effort_limit", 1.5)
        self._arm_namespace = str(self.get_parameter("arm_namespace").value).strip("/")
        self._joint_names = tuple(str(v) for v in self.get_parameter("joint_names").value)
        # URDF 与网格取自机械臂配置包：URDF 既用来读关节限位，也通过 HTTP 提供给前端做三维显示。
        moveit_share = Path(get_package_share_directory("rebotarm_moveit_config"))
        self._urdf_path = moveit_share / "config" / "rebotarm.urdf"
        self._mesh_dir = moveit_share / "meshes"
        lower = tuple(float(v) for v in self.get_parameter("joint_lower_limits").value)
        upper = tuple(float(v) for v in self.get_parameter("joint_upper_limits").value)
        fallback_limits = build_joint_limits(
            joint_names=self._joint_names,
            lower_limits=lower,
            upper_limits=upper,
        )
        try:
            urdf_limits = load_urdf_joint_limits(self._urdf_path, self._joint_names)
        except Exception as exc:
            # URDF 缺失或损坏不应让面板起不来：退回参数限位继续运行，但必须留下告警。
            self.get_logger().warn(f"failed to load URDF joint limits, using parameter limits: {exc}")
            urdf_limits = {}
        # 优先用 URDF 中的真实限位，缺失的关节才退回参数值（合并逻辑不改变逐关节覆盖关系）。
        self._joint_limits = merge_joint_limits(
            joint_names=self._joint_names,
            fallback_limits=fallback_limits,
            preferred_limits=urdf_limits,
        )
        # 速度限位优先取机械臂配置包 joint_limits.yaml（MoveIt 标定值），缺失时回退到 Web 速度上限。
        moveit_velocity_limits = load_moveit_velocity_limits(
            moveit_share / "config" / "joint_limits.yaml",
            self._joint_names,
        )
        self._joint_velocity_limits = merge_velocity_limits(
            joint_names=self._joint_names,
            default_limit=float(self.get_parameter("web_execute_max_joint_speed_rad_s").value),
            preferred_limits=moveit_velocity_limits,
        )
        gripper_lower = float(self.get_parameter("gripper_lower_limit_m").value)
        gripper_upper = float(self.get_parameter("gripper_upper_limit_m").value)
        # 参数可能被写反（下限大于上限），这里统一排序，后续限位比较才可以只按 (low, high) 处理。
        if gripper_upper < gripper_lower:
            gripper_lower, gripper_upper = gripper_upper, gripper_lower
        self._gripper_limits = (gripper_lower, gripper_upper)
        self._use_hardware = bool(self.get_parameter("use_hardware").value)
        # 命令网关：按 execution_mode 决定放行还是返回 dry_run，是所有 Web 命令的统一入口。
        self._web_command_gateway = WebCommandGateway()
        # 仿真夹爪的"当前位置"（开口，m）：初值为闭合端，仅在不接硬件时被更新与发布。
        self._sim_gripper_position = gripper_lower
        self._store = TeleopStatusStore()
        # 关节轨迹动作与夹爪动作：名称空间与控制器侧保持一致，缺一不可。
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            f"/{self._arm_namespace}/follow_joint_trajectory",
        )
        self._gripper_action_client = ActionClient(
            self,
            GripperCommand,
            f"/{self._arm_namespace}/gripper/command",
        )
        # 整臂与示教相关的服务客户端：全部按 arm_namespace 拼接，禁止硬编码顶层名称。
        self._gravity_start_client = self.create_client(
            Trigger,
            f"/{self._arm_namespace}/gravity_compensation/start",
        )
        self._gravity_stop_client = self.create_client(
            Trigger,
            f"/{self._arm_namespace}/gravity_compensation/stop",
        )
        self._teach_record_start_client = self.create_client(
            Trigger,
            f"/{self._arm_namespace}/teleop/teach_record/start",
        )
        self._teach_record_set_path_client = self.create_client(
            SetTeachRecordPath,
            f"/{self._arm_namespace}/teleop/teach_record/set_path",
        )
        self._teach_record_stop_client = self.create_client(
            Trigger,
            f"/{self._arm_namespace}/teleop/teach_record/stop",
        )
        # trajectory_stop 是控制器级急停通道：取消动作目标失败时的兜底，也是"停止"按钮的必走路径。
        self._trajectory_stop_client = self.create_client(
            Trigger,
            f"/{self._arm_namespace}/trajectory_stop",
        )
        self._arm_enable_client = self.create_client(
            Trigger,
            f"/{self._arm_namespace}/enable",
        )
        self._arm_disable_client = self.create_client(
            Trigger,
            f"/{self._arm_namespace}/disable",
        )
        self._arm_safe_home_client = self.create_client(
            Trigger,
            f"/{self._arm_namespace}/safe_home",
        )
        # 三个能力适配器：整臂服务、示教记录、Web 点对点运动；面板只做编排，算法都在各自包内。
        self._arm_control_client = ArmControlClient(
            enable_client=self._arm_enable_client,
            disable_client=self._arm_disable_client,
            safe_home_client=self._arm_safe_home_client,
            trajectory_stop_client=self._trajectory_stop_client,
        )
        self._teach_record_client = TeachRecordClient(
            set_path_client=self._teach_record_set_path_client,
            start_client=self._teach_record_start_client,
            stop_client=self._teach_record_stop_client,
            gravity_start_client=self._gravity_start_client,
            gravity_stop_client=self._gravity_stop_client,
            record_path_request_factory=SetTeachRecordPath.Request,
        )
        self._web_teleop_client = WebTeleopClient(
            action_client=self._action_client,
            joint_names=self._joint_names,
            joint_limits=self._joint_limits,
            joint_velocity_limits=self._joint_velocity_limits,
            trajectory_factory=JointTrajectory,
            trajectory_point_factory=JointTrajectoryPoint,
            follow_goal_factory=FollowJointTrajectory.Goal,
            gripper_action_client=self._gripper_action_client,
            gripper_goal_factory=GripperCommand.Goal,
        )
        # 当前 web 执行目标句柄（供"停止"按钮取消）与其互斥锁：动作回调与 HTTP 线程并发访问。
        self._calibration_client = CalibrationClient(
            self.create_client(CalibrationCommand, '/rebotarm_handeye_capture/command'),
            CalibrationCommand.Request,
            lambda value: self._store.update_teleop_status('calibration', value),
        )
        self._calibration_gravity_owned = False
        self._calibration_gravity_tracker = GravityRequestTracker()
        self._arm_status_received_at = None
        self._operator_request_lock = threading.RLock()
        self._execute_lock = threading.Lock()
        self._execute_goal_handle = None
        # Web 键盘会话状态：显式启用后才接受按键；步长/时长/速度可在会话内被前端调整，故加锁保护。
        self._web_keyboard_lock = threading.Lock()
        self._web_keyboard_enabled = False
        self._web_keyboard_step_rad = float(self.get_parameter("web_keyboard_default_step_rad").value)
        self._web_keyboard_duration = float(self.get_parameter("web_keyboard_default_duration").value)
        self._web_keyboard_speed = float(self.get_parameter("web_keyboard_default_speed_rad_s").value)
        # 示教回放编排器：注入状态快照、状态发布回调、碰撞默认关节与急停通道，回放算法不在本节点内。
        self._teach_workflow = TeachReplayWorkflow(
            node=self, joint_names=self._joint_names,
            snapshot=self._store.snapshot,
            publish_status=self._store.update_teleop_status,
            collision_defaults=self._collision_default_joint_positions,
            action_client=self._action_client,
            trajectory_stop_client=self._trajectory_stop_client,
            request_stop=self._request_controller_trajectory_stop,
        )
        # 传感类话题用 BEST_EFFORT + KEEP_LAST：丢帧可接受，但要尽量拿到最新帧而不是积压旧帧。
        sensor_qos_spec = sensor_qos_kwargs()
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=int(sensor_qos_spec["depth"]),
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        # 整臂状态用 RELIABLE + TRANSIENT_LOCAL：晚订阅的面板也要立刻拿到最后一帧使能/模式状态。
        arm_status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            JointState,
            f"/{self._arm_namespace}/joint_states",
            self._on_joint_state,
            sensor_qos,
        )
        self.create_subscription(ArmStatus, f"/{self._arm_namespace}/arm_status", self._on_arm_status, arm_status_qos)
        # 三条遥操作状态流：通用状态、录制状态、回放状态，内容可能是 JSON 也可能是纯文本。
        self.create_subscription(String, f"/{self._arm_namespace}/teleop/status", lambda msg: self._on_status("status", msg), 10)
        self.create_subscription(String, f"/{self._arm_namespace}/teleop/recording_status", lambda msg: self._on_status("recording", msg), 10)
        self.create_subscription(String, f"/{self._arm_namespace}/teleop/replay_status", lambda msg: self._on_status("replay", msg), 10)
        # 逐关节电机状态逐条订阅：界面需要单关节的状态码与力矩，而不只是 JointState 的汇总。
        for joint_name in self._joint_names:
            self.create_subscription(
                JointMotorState,
                f"/{self._arm_namespace}/joints/{joint_name}/state",
                self._on_motor_state,
                sensor_qos,
            )
        self.create_subscription(
            JointMotorState,
            f"/{self._arm_namespace}/gripper/state",
            self._on_gripper_state,
            sensor_qos,
        )
        self._sim_gripper_state_pub = None
        if not self._use_hardware:
            # 无硬件时由面板自己扮演夹爪反馈源，否则界面上的夹爪永远没有实时位置。
            self._sim_gripper_state_pub = self.create_publisher(
                JointMotorState,
                f"/{self._arm_namespace}/gripper/state",
                sensor_qos,
            )
            self.create_timer(0.1, self._publish_simulated_gripper_state)
        # 重力补偿状态没有专门话题，靠 1 Hz 轮询服务可用性 + 整臂状态机推断。
        self.create_timer(1.0, self._update_gravity_comp_status)
        # 回放跟踪监控定时器：周期取参数值但不小于 0.02 s，避免过密轮询挤占执行器线程。
        self.create_timer(
            max(float(self.get_parameter("replay_monitor_period_sec").value), 0.02),
            self._teach_workflow.check_tracking,
        )
        # HTTP 服务放在守护线程里：进程退出时不需要额外 join，也不会阻塞执行器。
        self._server = self._make_server()
        self._server_thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._server_thread.start()
        host = str(self.get_parameter("host").value)
        port = int(self.get_parameter("port").value)
        self.get_logger().info(f"teleop status panel available at http://{host}:{port}/")

    def _make_server(self):
        """创建本机 HTTP 服务（页面、SSE、REST、URDF/网格）。

        SSE 周期由 ``sse_rate_hz`` 换算，频率下限 1.0 Hz——再低界面就会明显"卡顿"，
        再高则只是徒增浏览器重绘，不会带来新的机械臂信息。
        """
        interval = 1.0 / max(float(self.get_parameter("sse_rate_hz").value), 1.0)
        host = str(self.get_parameter("host").value)
        port = int(self.get_parameter("port").value)
        return create_status_panel_server(
            host=host,
            port=port,
            node=self,
            store=self._store,
            html_page=HTML_PAGE,
            urdf_path=self._urdf_path,
            mesh_dir=self._mesh_dir,
            sse_interval_sec=interval,
        )

    def _handle_calibration_gravity(self, payload: dict) -> dict:
        snapshot = self._store.snapshot()
        age = (time.monotonic() - self._arm_status_received_at
               if self._arm_status_received_at is not None else float('inf'))
        busy = any(status_state(snapshot.teleop.get(key, {})) not in ('', 'idle', 'stopped', 'ready', 'completed')
                   for key in ('recording', 'replay'))
        busy = busy or self._execute_goal_handle is not None or self._web_keyboard_enabled
        command = payload.get('command')
        tracker = self._calibration_gravity_tracker
        if tracker.refresh(time.monotonic()):
            return {'accepted': False, 'message': '上一次模式请求仍未结束，保持控制占用'}
        if (command == 'stop' and payload.get('confirmed') is True and
                tracker.idle_confirmed(now=time.monotonic(), received_at=self._arm_status_received_at,
                                       arm=snapshot.arm)):
            self._calibration_gravity_owned = False
            return {'accepted': True, 'message': '请求已结束，新鲜反馈确认IDLE保持，已解除标定占用'}
        reason = gravity_rejection(command, payload.get('confirmed'), hardware=self._use_hardware,
            web_enabled=bool(self.get_parameter('web_execute_enabled').value),
            execution_mode=str(self.get_parameter('execution_mode').value),
            arm=snapshot.arm, status_age=age, busy=busy)
        if reason:
            return {'accepted': False, 'message': reason}
        # Reserve ownership before waiting: a timeout leaves the outcome unknown.
        if command == 'start':
            self._calibration_gravity_owned = True
        client = self._gravity_start_client if command == 'start' else self._gravity_stop_client
        success, message = tracker.call(client, Trigger.Request(), command, timeout=3.)
        # Keep ownership until a subsequent feedback confirms IDLE; operator can
        # use stop again to reconcile without sending another mode command.
        result = {'accepted': success, 'message': message, 'command': command}
        self._store.update_teleop_status('calibration_gravity', result)
        return result

    def _handle_calibration_command(self, payload: dict) -> dict:
        return self._calibration_client.command(payload)

    def _panel_config(self) -> dict:
        """下发前端的初始化配置（``/api/config``）。

        前端据此渲染输入框的取值范围、限位滑块与只读/可写状态；因此这里回传的是
        "面板实际生效的限位与阈值"，而不是原始参数字典——例如 ``joint_limits`` 已经
        合并过 URDF 限位，``joint_velocity_limits`` 已回退过默认速度上限。
        """
        return {
            # use_hardware 是前端功能裁剪的唯一后端类型依据：仿真模式隐藏只对真机有意义的
            # Enable / Disable / Safe Home，仍允许网页键盘和仿真轨迹 Execute。
            "use_hardware": bool(self.get_parameter("use_hardware").value),
            "joint_names": list(self._joint_names),
            "joint_limits": {
                name: [float(lower), float(upper)]
                for name, (lower, upper) in self._joint_limits.items()
            },
            "joint_velocity_limits": {
                name: float(limit)
                for name, limit in self._joint_velocity_limits.items()
            },
            "gripper_limits": [float(self._gripper_limits[0]), float(self._gripper_limits[1])],
            # web_execute / web_keyboard / web_gripper：前端输入控件的取值范围与默认值；
            # enabled 为假时页面只允许"预览"，不允许真正下发命令。
            "web_execute": {
                "enabled": bool(self.get_parameter("web_execute_enabled").value),
                "max_delta_rad": float(self.get_parameter("web_execute_max_delta_rad").value),
                "max_joint_speed_rad_s": float(self.get_parameter("web_execute_max_joint_speed_rad_s").value),
                "min_duration": float(self.get_parameter("web_execute_min_duration").value),
                "max_duration": float(self.get_parameter("web_execute_max_duration").value),
            },
            "web_keyboard": {
                "step_rad": float(self.get_parameter("web_keyboard_default_step_rad").value),
                "min_step_rad": float(self.get_parameter("web_keyboard_min_step_rad").value),
                "max_step_rad": float(self.get_parameter("web_keyboard_max_step_rad").value),
                "duration": float(self.get_parameter("web_keyboard_default_duration").value),
                "min_duration": float(self.get_parameter("web_keyboard_min_duration").value),
                "max_duration": float(self.get_parameter("web_keyboard_max_duration").value),
                "max_joint_speed_rad_s": float(self.get_parameter("web_keyboard_default_speed_rad_s").value),
            },
            "web_gripper": {
                "max_effort": float(self.get_parameter("web_gripper_max_effort").value),
                "max_effort_limit": float(self.get_parameter("web_gripper_max_effort_limit").value),
            },
            # teach：示教记录与回放的镜像参数，界面用它们做参数回显、质量分级说明与安全提示。
            "teach": {
                "record_path": str(self.get_parameter("record_path").value),
                "direct_threshold": float(self.get_parameter("direct_threshold").value),
                "align_threshold": float(self.get_parameter("align_threshold").value),
                "align_duration": float(self.get_parameter("align_duration").value),
                "align_duration_auto": bool(self.get_parameter("align_duration_auto").value),
                "align_target_speed_rad_s": float(self.get_parameter("align_target_speed_rad_s").value),
                "align_min_duration": float(self.get_parameter("align_min_duration").value),
                "align_max_duration": float(self.get_parameter("align_max_duration").value),
                "align_steps": int(self.get_parameter("align_steps").value),
                "replay_speed": float(self.get_parameter("replay_speed").value),
                "green_jump_rad": float(self.get_parameter("green_jump_rad").value),
                "yellow_jump_rad": float(self.get_parameter("yellow_jump_rad").value),
                "yellow_max_speed": float(self.get_parameter("yellow_max_speed").value),
                "max_replay_velocity_rad_s": float(self.get_parameter("max_replay_velocity_rad_s").value),
                "max_replay_velocity_rad_s_by_joint": [
                    float(value)
                    for value in self.get_parameter("max_replay_velocity_rad_s_by_joint").value
                ],
                "max_replay_acceleration_rad_s2": float(self.get_parameter("max_replay_acceleration_rad_s2").value),
                "max_replay_jerk_rad_s3": float(self.get_parameter("max_replay_jerk_rad_s3").value),
                "large_motion_span_rad": float(self.get_parameter("large_motion_span_rad").value),
                "large_motion_total_rad": float(self.get_parameter("large_motion_total_rad").value),
                "large_motion_max_speed": float(self.get_parameter("large_motion_max_speed").value),
                "start_hold_sec": float(self.get_parameter("start_hold_sec").value),
                "soft_start_duration": float(self.get_parameter("soft_start_duration").value),
                "soft_start_steps": int(self.get_parameter("soft_start_steps").value),
                "first_hold_sec": float(self.get_parameter("first_hold_sec").value),
                "final_hold_sec": float(self.get_parameter("final_hold_sec").value),
                "use_moveit_start_align": bool(self.get_parameter("use_moveit_start_align").value),
                "moveit_start_skip_threshold": float(self.get_parameter("moveit_start_skip_threshold").value),
                "collision_check_enabled": bool(self.get_parameter("collision_check_enabled").value),
                "collision_check_max_samples": int(self.get_parameter("collision_check_max_samples").value),
                "smoothing_enabled": bool(self.get_parameter("smoothing_enabled").value),
                "smoothing_window": int(self.get_parameter("smoothing_window").value),
                "filter_enabled": bool(self.get_parameter("filter_enabled").value),
                "filter_cutoff_hz": float(self.get_parameter("filter_cutoff_hz").value),
                "filter_sample_rate_hz": float(self.get_parameter("filter_sample_rate_hz").value),
                "resample_enabled": bool(self.get_parameter("resample_enabled").value),
                "resample_rate_hz": float(self.get_parameter("resample_rate_hz").value),
                "time_parameterization_method": str(self.get_parameter("time_parameterization_method").value),
                "max_prepared_jump_rad": float(self.get_parameter("max_prepared_jump_rad").value),
                "use_hardware": bool(self.get_parameter("use_hardware").value) if self.has_parameter("use_hardware") else False,
            },
            "panel_mode": str(self.get_parameter("panel_mode").value),
            "execution_mode": str(self.get_parameter("execution_mode").value),
        }

    def _teach_record_info(self, record_path: str | None = None) -> dict:
        """记录文件格式/时长/关节数等摘要；``record_path`` 为空时沿用界面当前选择的文件。"""
        return self._teach_workflow.record_info(record_path)

    def _collision_default_joint_positions(self, joint_names: tuple[str, ...]) -> tuple[tuple[str, float], ...]:
        """为碰撞预检补齐夹爪指关节位置，返回 ``((关节名, 位置), ...)``。

        计划组为 "arm_with_gripper" 时，状态有效性查询也要求给出夹爪指关节的角度；
        调用方传入的关节列表若已含手指关节就无需补齐（返回空元组）。位置按当前夹爪
        开口换算成左右手指关节角，取不到实时反馈时退回仿真开口值，避免查询因缺值失败。
        位置单位：手指关节为 m 级的平移量，正负成对，保证两指对称开合。
        """
        if str(self.get_parameter("collision_group_name").value) != "arm_with_gripper":
            return ()
        if {"left_finger_joint", "right_finger_joint"}.issubset(set(joint_names)):
            return ()
        joints = self._store.snapshot().joints
        gripper_state = joints.get("gripper", {})
        gripper_position = gripper_state.get("position", self._sim_gripper_position)
        left, right = gripper_opening_to_finger_joint_positions(
            float(gripper_position),
            self._gripper_limits,
        )
        return (("left_finger_joint", left), ("right_finger_joint", right))

    def _target_runtime(self) -> str:
        """当前目标运行环境标识："hardware" 表示真实硬件，"simulation" 表示仿真。"""
        return "hardware" if bool(self.get_parameter("use_hardware").value) else "simulation"

    def _route_web_command(self, intent: str, payload: dict | None = None) -> dict | None:
        """把 Web 命令交给统一网关裁决。

        返回值约定（调用方据此决定是否继续真实执行）：
        - 返回 dict：命令**未放行**（执行模式非 "execute" 的 dry_run，或网关判定不接受），
          调用方应把该 dict 直接当响应回写；
        - 返回 None：命令**已放行**，调用方继续走真实执行路径。

        注意 intent 会被写进请求体覆盖同名字段，避免前端伪造意图绕过网关。
        """
        request_payload = dict(payload or {})
        request_payload["intent"] = intent
        result = self._web_command_gateway.route(
            WebCommandRequest.from_payload(
                request_payload,
                execution_mode=str(self.get_parameter("execution_mode").value),
            )
        )
        if result.get("state") == "dry_run":
            return result
        if not result.get("accepted", False):
            return result
        return None

    def _legacy_blocked_response(self, intent: str, message: str, payload: dict | None = None) -> dict:
        """构造"旧执行通道被拦截"的统一响应（面板开关未打开时使用）。

        与网关的 dry_run 不同，这里是面板自身的功能开关拒绝：state="blocked" 且带
        ``blocked_legacy_execution=True``，界面据此提示需要重启并打开对应开关。
        """
        payload = dict(payload or {})
        return self._web_command_gateway.blocked_legacy_response(
            intent=intent,
            message=message,
            execution_mode=str(self.get_parameter("execution_mode").value),
            request_id=str(payload.get("request_id", "") or ""),
            blocked_legacy_execution=True,
        )

    def _teach_trajectory(self, record_path: str | None = None, max_points: int = 500) -> dict:
        """预处理后的回放预览曲线（供界面绘图）；``max_points`` 限制返回点数以防曲线过大。"""
        return self._teach_workflow.trajectory_preview(record_path, max_points)

    def _teach_records(self) -> dict:
        """列出记录目录中可用的示教记录文件及其摘要。"""
        return self._teach_workflow.records()

    def _handle_teach_dry_run(self, payload: dict) -> dict:
        """示教回放全流程演练：只做校验、规划与预检，绝不下发动作目标。"""
        return self._teach_workflow.dry_run(payload)

    def _handle_teach_replay_execute(self, payload: dict) -> dict:
        """真正执行示教回放。

        两道前置：面板的 ``web_execute_enabled`` 必须为真；命令网关必须放行。否则只回写
        拦截/演练结果，不产生任何真实运动。真正下发由回放编排器完成，它还会再校验
        dry-run 令牌与各项安全门。
        """
        if not bool(self.get_parameter("web_execute_enabled").value):
            message = "web teach replay disabled; launch with web_execute_enabled:=true"
            result = self._legacy_blocked_response("teach_replay_execute", message, payload)
            self._store.update_teleop_status("replay", result)
            return result
        gateway_result = self._route_web_command("teach_replay_execute", payload)
        if gateway_result is not None:
            self._store.update_teleop_status("replay", gateway_result)
            return gateway_result
        return self._teach_workflow.execute(payload)

    def _handle_teach_replay_stop(self) -> dict:
        """停止示教回放：命令网关放行时走编排器的停止路径，否则只回写拦截结果。"""
        gateway_result = self._route_web_command("stop_robot", {"command": "teach_replay_stop"})
        if gateway_result is not None:
            self._store.update_teleop_status("replay", gateway_result)
            return gateway_result
        return self._teach_workflow.stop()

    def _request_controller_trajectory_stop(self, *, timeout_sec: float) -> bool:
        """请求控制器级急停（非阻塞）：返回是否成功发出请求。

        等待服务最多 0.2 s（即使调用方给了更长的超时），因为停止属于紧急路径，不能在
        这里阻塞执行器线程；发出请求后不等待响应，避免与动作层取消相互等待造成死锁。
        """
        try:
            if not self._trajectory_stop_client.wait_for_service(timeout_sec=min(timeout_sec, 0.2)):
                return False
            self._trajectory_stop_client.call_async(Trigger.Request())
            return True
        except Exception:
            return False

    def _call_trigger_service(self, client, *, timeout_sec: float) -> tuple[bool, str]:
        """同步调用触发器服务的薄封装（复用整臂客户端的超时/等待实现），返回 (成功, 说明)。"""
        return ArmControlClient.call_trigger_service(client, timeout_sec=timeout_sec)

    def _handle_arm_service_command(self, command: str) -> dict:
        """处理整臂命令（safe_home / enable / disable）并回写状态。

        安全顺序：
        1. 面板开关未打开时直接拦截，绝不调用硬件服务；
        2. 示教回放进行中（含停止过渡态）时锁定整臂命令，防止人工命令与回放轨迹互相干扰；
        3. 命令名非法时拒绝；
        4. 过网关（``execution_mode``）后交给整臂客户端；对需要先停下轨迹的命令
           （safe_home / disable）先清空本地目标句柄，避免停止后仍持有已失效的句柄。
        命令受理后按操作者预期"乐观"更新本地使能状态与状态机，真实状态随后由
        ``arm_status`` 反馈覆盖，界面因此不会出现按钮与状态互相矛盾的空窗期。
        """
        command = normalize_arm_command(command) or ""
        if not self._use_hardware:
            message = "hardware arm command unavailable in simulation mode"
            result = {
                "accepted": False,
                "state": "blocked",
                "command": command,
                "message": message,
            }
            self._store.update_teleop_status("arm_command", result)
            return result
        if not bool(self.get_parameter("web_execute_enabled").value):
            message = "web arm command disabled; launch with web_execute_enabled:=true"
            # 未打开开关时仍按意图归类：回原点算 move_home，其余算 stop_robot，便于前端统计。
            gateway_intent = "move_home" if command == "safe_home" else "stop_robot"
            result = {**self._legacy_blocked_response(gateway_intent, message, {"command": command}), "command": command}
            self._store.update_teleop_status("arm_command", result)
            return result
        replay_state = status_state(self._store.snapshot().teleop.get("replay", {}))
        if arm_command_is_replay_locked(replay_state):
            message = "arm command blocked during teach replay"
            result = {"accepted": False, "state": "blocked", "command": command, "message": message}
            self._store.update_teleop_status("arm_command", result)
            return result
        if not command:
            result = {"accepted": False, "state": "rejected", "command": command, "message": "unknown arm command"}
            self._store.update_teleop_status("arm_command", result)
            return result
        gateway_intent = "move_home" if command == "safe_home" else "stop_robot"
        gateway_result = self._route_web_command(gateway_intent, {"command": command})
        if gateway_result is not None:
            gateway_result = {**gateway_result, "command": command}
            self._store.update_teleop_status("arm_command", gateway_result)
            return gateway_result
        if should_stop_trajectory_before_arm_command(command):
            # safe_home / disable 会改变控制模式，先丢弃本地句柄；客户端内部还会先发 trajectory_stop。
            with self._execute_lock:
                self._execute_goal_handle = None
        result = self._arm_control_client.execute(command)
        result.setdefault("trajectory_stop_requested", False)
        if result["accepted"]:
            # 命令已被控制器接受：按语义立刻反映到界面状态，等待臂状态反馈来最终确认。
            arm = self._store.snapshot().arm
            enabled = bool(arm.get("enabled", False))
            mode = str(arm.get("mode", ""))
            if command == "disable":
                enabled = False
            elif command in ("enable", "safe_home"):
                enabled = True
            if command == "safe_home":
                # 回原点后控制器必然处于位置+速度模式；仅在原状态为空时补一个合理默认值。
                mode = mode or "pos_vel"
            self._store.update_arm_status(
                mode=mode,
                enabled=enabled,
                state_machine="IDLE",
                error_codes=tuple(str(v) for v in arm.get("error_codes", [])),
            )
        self._store.update_teleop_status("arm_command", result)
        return result

    def _handle_teach_record_start(self, payload: dict | None = None) -> dict:
        """开始示教记录：可选切换记录文件路径并启动（必要时先开重力补偿），结果回写录制状态。"""
        result = self._teach_record_client.start(payload)
        self._store.update_teleop_status("recording", result)
        return result

    def _handle_teach_record_stop(self) -> dict:
        """停止示教记录（并结束重力补偿），结果回写录制状态。"""
        result = self._teach_record_client.stop()
        self._store.update_teleop_status("recording", result)
        return result

    def _handle_keyboard_enable(self, payload: dict) -> dict:
        """启用 Web 键盘遥操作会话，并把本次生效的步长/时长/速度夹紧后长期保存。

        夹紧规则（防止前端或手工请求绕过参数上限）：
        - 步长夹到 [web_keyboard_min_step_rad, web_keyboard_max_step_rad]（rad）；
        - 时长夹到 [web_keyboard_min_duration, web_keyboard_max_duration]（s）；
        - 速度下限固定 0.05 rad/s（再低就近乎不动，没有操作意义），上限取
          web_execute_max_joint_speed_rad_s（与整体 Web 运动共用同一安全上限）。
        只读的 check 模式下直接拒绝启用，面板不应存在任何可动的入口。
        """
        if not bool(self.get_parameter("web_execute_enabled").value):
            message = "web keyboard disabled; launch with web_execute_enabled:=true"
            result = self._legacy_blocked_response("keyboard_step", message, payload)
            self._store.update_teleop_status("status", {**result, "source": "web_keyboard"})
            return result
        if str(self.get_parameter("panel_mode").value).lower() == "check":
            message = "web keyboard blocked: check mode is read-only"
            self._store.update_teleop_status("status", {"source": "web_keyboard", "state": "blocked", "message": message})
            return {"accepted": False, "message": message}
        step = _number_or_default(payload.get("step_rad"), self._web_keyboard_step_rad)
        duration = _number_or_default(payload.get("duration"), self._web_keyboard_duration)
        speed = _number_or_default(payload.get("max_joint_speed_rad_s"), self._web_keyboard_speed)
        with self._web_keyboard_lock:
            self._web_keyboard_step_rad = min(
                max(step, float(self.get_parameter("web_keyboard_min_step_rad").value)),
                float(self.get_parameter("web_keyboard_max_step_rad").value),
            )
            self._web_keyboard_duration = min(
                max(duration, float(self.get_parameter("web_keyboard_min_duration").value)),
                float(self.get_parameter("web_keyboard_max_duration").value),
            )
            self._web_keyboard_speed = min(
                max(speed, 0.05),
                float(self.get_parameter("web_execute_max_joint_speed_rad_s").value),
            )
            self._web_keyboard_enabled = True
        result = {
            "accepted": True,
            "source": "web_keyboard",
            "state": "ready",
            "message": "web keyboard teleop enabled",
            "step_rad": self._web_keyboard_step_rad,
            "duration": self._web_keyboard_duration,
            "max_joint_speed_rad_s": self._web_keyboard_speed,
        }
        self._store.update_teleop_status("status", result)
        return result

    def _handle_keyboard_disable(self) -> dict:
        """关闭 Web 键盘遥操作会话，并请求一次控制器级急停以停住正在执行的那一步。

        先置 enabled=False 再发停止请求：即使停止服务不可用，后续按键也不会再产生新运动。
        """
        gateway_result = self._route_web_command("stop_robot", {"command": "keyboard_disable"})
        if gateway_result is not None:
            self._store.update_teleop_status("status", {**gateway_result, "source": "web_keyboard"})
            return gateway_result
        with self._web_keyboard_lock:
            self._web_keyboard_enabled = False
        stop_requested = self._request_controller_trajectory_stop(timeout_sec=0.2)
        result = {
            "accepted": True,
            "source": "web_keyboard",
            "state": "disabled",
            "message": "web keyboard teleop disabled; trajectory_stop requested" if stop_requested else "web keyboard teleop disabled",
            "trajectory_stop_requested": stop_requested,
        }
        self._store.update_teleop_status("status", result)
        return result

    def _handle_keyboard_key(self, payload: dict) -> dict:
        """处理 Web 键盘单步按键：校验 → 过网关 → 组装单点轨迹下发。

        校验全部交给操作包的核心校验函数（确认串、按键映射、实时反馈、增量限位与限速），
        本方法只负责补齐会话默认值、取实时反馈并组装 JointTrajectory。
        下发前先探测动作服务：不可用就立刻返回失败，不排队等待（0.05 s 只是"在不在"的探测）。
        轨迹只有一个点，时间戳用 duration 表示"从当前时刻起多久走到该点"。
        """
        with self._web_keyboard_lock:
            enabled = self._web_keyboard_enabled
            step_rad = self._web_keyboard_step_rad
            duration = self._web_keyboard_duration
            speed = self._web_keyboard_speed
        request_payload = dict(payload)
        # 用 setdefault 补齐会话默认值：前端显式给出的值优先，不会被会话值覆盖。
        request_payload.setdefault("step_rad", step_rad)
        request_payload.setdefault("duration", duration)
        request_payload.setdefault("max_joint_speed_rad_s", speed)
        snapshot = self._store.snapshot()
        current_positions = {
            name: float(data["position"])
            for name, data in snapshot.joints.items()
            if "position" in data
        }
        decision = validate_web_keyboard_command(
            request_payload,
            enabled=enabled,
            joint_names=self._joint_names,
            current_positions=current_positions,
            joint_limits=self._joint_limits,
            default_step_rad=float(self.get_parameter("web_keyboard_default_step_rad").value),
            min_step_rad=float(self.get_parameter("web_keyboard_min_step_rad").value),
            max_step_rad=float(self.get_parameter("web_keyboard_max_step_rad").value),
            default_duration=float(self.get_parameter("web_keyboard_default_duration").value),
            min_duration=float(self.get_parameter("web_keyboard_min_duration").value),
            max_duration=float(self.get_parameter("web_keyboard_max_duration").value),
            joint_velocity_limits=self._joint_velocity_limits,
            max_joint_speed_rad_s=float(self.get_parameter("web_execute_max_joint_speed_rad_s").value),
        )
        if not decision.accepted:
            self._store.update_teleop_status(
                "status",
                {"source": "web_keyboard", "state": "rejected", "message": decision.message, "last_key": payload.get("key")},
            )
            return _keyboard_decision_response(decision)
        gateway_result = self._route_web_command("keyboard_step", request_payload)
        if gateway_result is not None:
            self._store.update_teleop_status("status", {**gateway_result, "source": "web_keyboard", "last_key": payload.get("key")})
            return gateway_result
        if not self._action_client.wait_for_server(timeout_sec=0.05):
            message = "follow_joint_trajectory action unavailable"
            self._store.update_teleop_status("status", {"source": "web_keyboard", "state": "unavailable", "message": message, "last_key": decision.key})
            return {"accepted": False, "message": message}
        trajectory = JointTrajectory()
        trajectory.joint_names = list(decision.joint_names)
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in decision.positions]
        # 单点轨迹：时间戳即"从当前时刻起走到该目标点"的时长，速度由校验环节保证不超限。
        _set_duration(point.time_from_start, decision.duration)
        trajectory.points = [point]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(lambda fut: self._on_keyboard_goal_response(fut, decision))
        result = _keyboard_decision_response(decision)
        self._store.update_teleop_status(
            "status",
            {
                **result,
                "source": "web_keyboard",
                "state": "active",
                "last_key": decision.key,
            },
        )
        return result

    def _on_keyboard_goal_response(self, future, decision) -> None:
        """键盘单点轨迹的目标受理回调：把"收到目标/被拒绝/发送失败"写进面板状态。"""
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._store.update_teleop_status("status", {"source": "web_keyboard", "state": "failed", "message": str(exc), "last_key": decision.key})
            return
        if goal_handle is None or not goal_handle.accepted:
            self._store.update_teleop_status("status", {"source": "web_keyboard", "state": "rejected", "message": "keyboard trajectory goal rejected", "last_key": decision.key})
            return
        self._store.update_teleop_status(
            "status",
            {
                "source": "web_keyboard",
                "state": "accepted",
                "message": decision.message,
                "last_key": decision.key,
                "joint_name": decision.joint_name,
                "step_rad": decision.step_rad,
                "duration": decision.duration,
                "max_joint_speed_rad_s": decision.max_joint_speed_rad_s,
            },
        )

    def _handle_execute_preview(self, payload: dict) -> dict:
        """处理面板块上的"执行预览"（相对当前位形的关节目标）。

        面板开关与网关两道检查之后，才交给 Web 点对点适配层做校验、插值与下发；返回的
        response 是前端契约（不含动作句柄），动作结果随后由回调更新到 ``web_execute`` 状态。
        """
        if not bool(self.get_parameter("web_execute_enabled").value):
            return self._legacy_blocked_response(
                "move_relative",
                "web execute disabled; launch with web_execute_enabled:=true",
                payload,
            )
        gateway_result = self._route_web_command("move_relative", payload)
        if gateway_result is not None:
            self._store.update_teleop_status("web_execute", gateway_result)
            return gateway_result
        snapshot = self._store.snapshot()
        current_positions = {
            name: float(data["position"])
            for name, data in snapshot.joints.items()
            if "position" in data
        }
        execution = self._web_teleop_client.execute(
            payload,
            current_positions=current_positions,
            max_delta_rad=float(self.get_parameter("web_execute_max_delta_rad").value),
            min_duration=float(self.get_parameter("web_execute_min_duration").value),
            max_duration=float(self.get_parameter("web_execute_max_duration").value),
            max_joint_speed_rad_s=float(self.get_parameter("web_execute_max_joint_speed_rad_s").value),
        )
        decision = execution["decision"]
        if not execution["accepted"]:
            self._store.update_teleop_status("web_execute", execution["status"])
            return execution["response"]
        future = execution["goal_future"]
        future.add_done_callback(lambda fut: self._on_execute_goal_response(fut, decision))
        self._store.update_teleop_status("web_execute", execution["status"])
        return decision_response(decision)

    def _handle_stop_execute(self) -> dict:
        """停止 Web 触发的关节运动：总是先请求控制器级急停，再取消动作目标。

        句柄只在锁内取出，取完立刻释放，避免等待取消回调时长时间持锁。返回值里保留
        ``trajectory_stop_requested``，让前端能区分"已请求控制器停止"与"仅取消了目标"。
        """
        gateway_result = self._route_web_command("stop_robot", {"command": "stop_execute"})
        if gateway_result is not None:
            self._store.update_teleop_status("web_execute", gateway_result)
            return gateway_result
        with self._execute_lock:
            goal_handle = self._execute_goal_handle
        result = self._web_teleop_client.stop(
            goal_handle,
            trajectory_stop_client=self._trajectory_stop_client,
        )
        future = result.get("cancel_future")
        if future is not None:
            future.add_done_callback(self._on_execute_cancel_response)
        self._store.update_teleop_status("web_execute", result["status"])
        if result.get("clear_goal_handle"):
            # 客户端明确要求清空句柄：说明该目标已不再有效，继续留着会让下次停止操作瞄错对象。
            with self._execute_lock:
                self._execute_goal_handle = None
        return {
            "accepted": bool(result["accepted"]),
            "state": result.get("state", result["status"].get("state", "")),
            "message": str(result.get("message", "")),
            "trajectory_stop_requested": bool(result.get("trajectory_stop_requested", False)),
        }

    def _handle_set_gripper(self, payload: dict) -> dict:
        """处理夹爪开口命令（真实硬件或仿真）。

        仿真模式下适配层只给出模拟落点，本方法据此更新仿真开口、立刻发布一帧夹爪反馈，
        让界面马上看到变化；真实模式则下发动作目标并注册结果回调。
        """
        if not bool(self.get_parameter("web_execute_enabled").value):
            return self._legacy_blocked_response(
                "set_gripper",
                "web gripper disabled; launch with web_execute_enabled:=true",
                payload,
            )
        gateway_result = self._route_web_command("set_gripper", payload)
        if gateway_result is not None:
            self._store.update_teleop_status("web_gripper", gateway_result)
            return gateway_result
        result = self._web_teleop_client.set_gripper(
            payload,
            use_hardware=self._use_hardware,
            gripper_limits=self._gripper_limits,
            default_max_effort=float(self.get_parameter("web_gripper_max_effort").value),
            max_effort_limit=float(self.get_parameter("web_gripper_max_effort_limit").value),
        )
        decision = result["decision"]
        if not decision.accepted:
            self._store.update_teleop_status("web_gripper", result["status"])
            return result["response"]
        if result.get("simulated_position") is not None:
            # 仿真分支：没有动作目标，直接把模拟落点当作当前开口并主动推一帧反馈。
            self._sim_gripper_position = float(result["simulated_position"])
            self._publish_simulated_gripper_state()
            self._store.update_teleop_status("web_gripper", result["status"])
            return gripper_decision_response(decision)
        if not result["accepted"]:
            self._store.update_teleop_status("web_gripper", result["status"])
            return result["response"]
        future = result["goal_future"]
        future.add_done_callback(lambda fut: self._on_gripper_goal_response(fut, decision))
        self._store.update_teleop_status("web_gripper", result["status"])
        return gripper_decision_response(decision)

    def _on_gripper_goal_response(self, future, decision: WebGripperDecision) -> None:
        """夹爪目标受理回调：受理后继续注册结果回调，用于确认是否真正到位。"""
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._store.update_teleop_status("web_gripper", {"state": "failed", "message": str(exc)})
            return
        if goal_handle is None or not goal_handle.accepted:
            self._store.update_teleop_status(
                "web_gripper",
                {"state": "rejected", "message": "gripper goal rejected"},
            )
            return
        self._store.update_teleop_status(
            "web_gripper",
            {
                "state": "accepted",
                "message": "gripper goal accepted by controller",
                "position": decision.position,
                "max_effort": decision.max_effort,
            },
        )
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda fut: self._on_gripper_result(fut, decision))

    def _on_gripper_result(self, future, decision: WebGripperDecision) -> None:
        """夹爪动作最终结果回调：``reached_goal`` 为真记 done，否则记 failed。

        位置/力矩字段在部分控制器实现中可能缺失，因此取默认值兜底而不是直接抛错——
        一次缺字段不应让面板状态卡在中间态。
        """
        try:
            result_response = future.result()
            result = result_response.result
            reached = bool(getattr(result, "reached_goal", False))
            position = float(getattr(result, "position", decision.position))
            effort = float(getattr(result, "effort", 0.0))
        except Exception as exc:
            self._store.update_teleop_status("web_gripper", {"state": "failed", "message": str(exc)})
            return
        self._store.update_teleop_status(
            "web_gripper",
            {
                "state": "done" if reached else "failed",
                "message": f"gripper result reached={reached}",
                "position": position,
                "max_effort": decision.max_effort,
                "effort": effort,
            },
        )

    def _publish_simulated_gripper_state(self) -> None:
        """在不接硬件时发布一帧仿真夹爪反馈。

        ``position`` 用夹爪开口（m），速度与力矩恒为 0；``status_code=1`` 表示"使能、
        可接受位置命令"，这样界面在仿真下的显示逻辑与真实硬件一致。
        """
        if self._sim_gripper_state_pub is None:
            return
        msg = JointMotorState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_name = "gripper"
        msg.position = float(self._sim_gripper_position)
        msg.velocity = 0.0
        msg.torque = 0.0
        msg.status_code = 1
        self._sim_gripper_state_pub.publish(msg)

    def _on_execute_cancel_response(self, future) -> None:
        """关节运动取消请求的回调：区分"取消已被接受"与"目标在取消前就跑完了"。"""
        try:
            response = future.result()
            goals_canceling = len(getattr(response, "goals_canceling", []))
        except Exception as exc:
            self._store.update_teleop_status("web_execute", {"state": "failed", "message": str(exc)})
            return
        state = "cancel_requested" if goals_canceling else "done"
        message = (
            "trajectory cancel accepted"
            if goals_canceling
            else "trajectory already finished before cancel"
        )
        self._store.update_teleop_status("web_execute", {"state": state, "message": message})

    def _on_execute_goal_response(self, future, decision: WebExecuteDecision) -> None:
        """关节运动目标受理回调：保存目标句柄（供停止使用）并注册最终结果回调。"""
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._store.update_teleop_status("web_execute", {"state": "failed", "message": str(exc)})
            return
        if goal_handle is None or not goal_handle.accepted:
            self._store.update_teleop_status(
                "web_execute",
                {"state": "rejected", "message": "trajectory goal rejected"},
            )
            return
        with self._execute_lock:
            self._execute_goal_handle = goal_handle
        self._store.update_teleop_status(
            "web_execute",
            {
                "state": "accepted",
                "message": "trajectory goal accepted by controller",
                "max_delta": decision.max_delta,
                "max_delta_limit": decision.max_delta_limit,
                "duration": decision.duration,
                "max_joint_speed_rad_s": float(self.get_parameter("web_execute_max_joint_speed_rad_s").value),
            },
        )
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda fut: self._on_execute_result(fut, decision))

    def _on_execute_result(self, future, decision: WebExecuteDecision) -> None:
        """关节运动最终结果回调，并统一释放目标句柄。

        结果状态判定优先级：错误码为 SUCCESSFUL 记 ``done``；否则若动作状态码为 5
        （ROS 2 动作终态 ``CANCELED``）记 ``canceled``；其余一律记 ``failed``。
        句柄在两条路径上都会被清空，避免下一次"停止"去取消一个已经结束的目标。
        """
        try:
            result_response = future.result()
            result = result_response.result
            status = int(result_response.status)
            error_code = int(getattr(result, "error_code", 0))
            error_string = str(getattr(result, "error_string", ""))
        except Exception as exc:
            self._store.update_teleop_status("web_execute", {"state": "failed", "message": str(exc)})
            with self._execute_lock:
                self._execute_goal_handle = None
            return
        if error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            state = "done"
        elif status == 5:
            # 5 是动作终态 CANCELED：轨迹没跑完但也不是控制器报错，需与 failed 区分开。
            state = "canceled"
        else:
            state = "failed"
        with self._execute_lock:
            self._execute_goal_handle = None
        self._store.update_teleop_status(
            "web_execute",
            {
                "state": state,
                "message": f"trajectory result status={status}, error_code={error_code}: {error_string}",
                "max_delta": decision.max_delta,
                "max_delta_limit": decision.max_delta_limit,
                "duration": decision.duration,
                "max_joint_speed_rad_s": float(self.get_parameter("web_execute_max_joint_speed_rad_s").value),
            },
        )

    def _on_joint_state(self, msg: JointState) -> None:
        """关节汇总反馈回调：名/位置/速度/力矩按索引一一对应写入状态存储。"""
        self._store.update_joint_state(
            names=tuple(str(v) for v in msg.name),
            positions=tuple(float(v) for v in msg.position),
            velocities=tuple(float(v) for v in msg.velocity),
            efforts=tuple(float(v) for v in msg.effort),
        )

    def _on_motor_state(self, msg: JointMotorState) -> None:
        """单关节电机状态回调（含状态码与力矩），比 JointState 更适合界面逐电机展示。"""
        self._store.update_motor_state(
            joint_name=str(msg.joint_name),
            position=float(msg.position),
            velocity=float(msg.velocity),
            torque=float(msg.torque),
            status_code=int(msg.status_code),
        )

    def _on_gripper_state(self, msg: JointMotorState) -> None:
        """夹爪状态回调：无论发布者用什么关节名，都同时在 "gripper" 键下留一份。

        界面固定按 "gripper" 读取夹爪数据，而不同固件可能用 joint7 之类的名字发布，
        因此这里做一次别名写入，保证显示不依赖具体发布方的命名。
        """
        joint_name = str(msg.joint_name).strip() or "gripper"
        self._store.update_motor_state(
            joint_name=joint_name,
            position=float(msg.position),
            velocity=float(msg.velocity),
            torque=float(msg.torque),
            status_code=int(msg.status_code),
        )
        if joint_name != "gripper":
            self._store.update_motor_state(
                joint_name="gripper",
                position=float(msg.position),
                velocity=float(msg.velocity),
                torque=float(msg.torque),
                status_code=int(msg.status_code),
            )

    def _on_arm_status(self, msg: ArmStatus) -> None:
        """整臂状态回调：刷新模式/使能/状态机/错误码，并立即重算重力补偿可用性。"""
        now = time.monotonic()
        source_ns = int(msg.header.stamp.sec) * 10**9 + int(msg.header.stamp.nanosec)
        source_age = (self.get_clock().now().nanoseconds - source_ns) / 1e9
        self._arm_status_received_at = (now - max(0., source_age)
            if source_ns > 0 and -.05 <= source_age <= .5 else None)
        # GRAVITY_COMP is also the normal teach-recording mode.  It does not by
        # itself prove that calibration requested ownership; ownership is set
        # only by _handle_calibration_gravity(start).  Otherwise teach recording
        # would permanently block its own stop and dry-run HTTP routes.
        self._store.update_arm_status(
            mode=str(msg.mode),
            enabled=bool(msg.enabled),
            state_machine=str(msg.state_machine),
            error_codes=tuple(str(v) for v in msg.error_codes),
        )
        self._update_gravity_comp_status()

    def _on_status(self, key: str, msg: String) -> None:
        """遥操作状态流回调：能解析成 JSON 就存对象，否则按纯文本存字符串。

        ``key`` 决定写进存储的哪个槽位（status / recording / replay），与订阅的三条话题一一对应。
        """
        try:
            value = json.loads(msg.data)
        except Exception:
            value = msg.data
        self._store.update_teleop_status(key, value)

    def _update_gravity_comp_status(self) -> None:
        """汇总重力补偿可用性并写入 ``gravity_comp`` 状态（1 Hz 定时 + 整臂状态回调触发）。

        没有专门的重力补偿话题，因此状态由两部分推断：
        - 整臂状态机是否为 "GRAVITY_COMP"，决定 ``active``；
        - start/stop 服务是否可见，决定 ``state``：active > ready（start 可用）> unavailable。
        探测用 ``service_is_ready()`` 为主、0 超时的 ``wait_for_service`` 为辅，两者都在
        try 里保护——探测异常只应体现为"不可用"，不能让定时器抛错。
        ``ready_for_teach_recording`` 是界面"能否开始录制"的判据：录制流程要求重力补偿
        生效（``recording.require_gravity_comp``）时必须 active；若录制方声明不需要，则恒为真。
        """
        snapshot = self._store.snapshot()
        arm_state = str(snapshot.arm.get("state_machine", ""))
        received = getattr(self, '_arm_status_received_at', None)
        self._store.update_teleop_status('arm_feedback', {
            'fresh': received is not None and time.monotonic() - received <= .5,
            'age_sec': time.monotonic() - received if received is not None else None,
        })
        try:
            start_available = bool(self._gravity_start_client.service_is_ready())
            if not start_available:
                start_available = bool(self._gravity_start_client.wait_for_service(timeout_sec=0.0))
        except Exception:
            start_available = False
        try:
            stop_available = bool(self._gravity_stop_client.service_is_ready())
            if not stop_available:
                stop_available = bool(self._gravity_stop_client.wait_for_service(timeout_sec=0.0))
        except Exception:
            stop_available = False
        active = arm_state == "GRAVITY_COMP"
        if active:
            state = "active"
            message = "gravity compensation is active"
        elif start_available:
            state = "ready"
            message = "gravity compensation start service is available"
        else:
            state = "unavailable"
            message = "gravity compensation services unavailable"
        recording = snapshot.teleop.get("recording")
        # 录制状态缺失时按"需要重力补偿"处理（默认 True）：宁可多提示一步，也不要放过不安全录制。
        require_gravity = bool(recording.get("require_gravity_comp")) if isinstance(recording, dict) else True
        self._store.update_teleop_status(
            "gravity_comp",
            {
                "state": state,
                "message": message,
                "arm_state": arm_state,
                "start_service_available": start_available,
                "stop_service_available": stop_available,
                "active": active,
                "ready_for_teach_recording": (not require_gravity) or active,
                "recording_requires_gravity_comp": require_gravity,
            },
        )

    def destroy_node(self) -> bool:
        """先关 HTTP 服务再释放 ROS 资源。

        HTTP 服务线程是守护线程且可能正阻塞在 SSE 写循环里，必须显式 shutdown/server_close
        才能断开在连的浏览器；放在 finally 中保证即使关闭过程抛异常，节点资源也照常释放。
        """
        try:
            self._server.shutdown()
            self._server.server_close()
        finally:
            return super().destroy_node()


def main(args=None) -> None:
    """节点入口：初始化 → 常驻自旋 → 退出时安全销毁。

    退出路径同时兼容 Ctrl-C 与外部关闭（如 launch 关闭时的 ExternalShutdownException），
    并在 finally 里兜底销毁节点；``rclpy.ok()`` 判断避免对已关闭的上下文重复 shutdown。
    """
    rclpy.init(args=args)
    node = TeleopStatusPanelNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        with suppress(KeyboardInterrupt):
            node.destroy_node()
        if rclpy.ok():
            with suppress(KeyboardInterrupt):
                rclpy.shutdown()


if __name__ == "__main__":
    main()

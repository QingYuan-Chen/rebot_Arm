"""Immutable data models used by teach recording and replay workflows.

This module contains no ROS, filesystem, or motion-planning dependencies.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


@dataclass(frozen=True)
class TeachSample:
    """一帧重力补偿示教采样（不可变；关节顺序即 ``joint_names`` 的顺序）。

    ``stamp``        采样时间（秒），单调递增，用于差分出速度/加速度/加加速度；
    ``joint_names``  本帧包含的关节名，整条记录内应保持一致，否则判为结构性异常；
    ``positions``    关节位置（rad，仅旋转关节），与 ``joint_names`` 一一对应且等长；
    ``velocities``   关节速度（rad/s），可能为空元组（重采样后会清空）；
    ``efforts``      关节力矩/电流读数（原始采样单位，来自硬件反馈），可能为空；
    ``motor_status`` 电机状态码字典，键为电机标识、值为整型状态码（硬件定义），可能为空；
    ``arm_state``    采集当时的机械臂状态字符串（由采集侧写入，回放侧只透传不解释）。

    注意：``velocities`` / ``efforts`` / ``motor_status`` 允许为空，凡是要用到它们的逻辑都必须
    先判空，不能假定一定存在。
    """

    stamp: float
    joint_names: tuple[str, ...]
    positions: tuple[float, ...]
    velocities: tuple[float, ...]
    efforts: tuple[float, ...]
    motor_status: dict[str, int]
    arm_state: str


@dataclass(frozen=True)
class TeachTrajectoryEvent:
    """轨迹质量评估中的一条异常事件（一条事件只对应某一帧的某一个关节）。

    ``sample``            发生异常的样本下标（从 0 开始；首帧不参与差分，故不会是 0）；
    ``joint_name``        出问题的关节名；
    ``level``             事件等级字符串：``red``（跳变超黄线，禁止回放）或 ``yellow``
                          （跳变超绿线 / 速度 / 加速度 / 加加速度越限，需降速或重定时）；
    ``message``           人类可读的事件描述（同时被收集进 anomalies 列表）；
    ``delta_rad``         该帧该关节的位置增量绝对值（rad）；
    ``velocity_rad_s``    差分速度绝对值（rad/s）；时间差为 0 等无法差分时为 None；
    ``acceleration_rad_s2`` 差分加速度绝对值（rad/s^2）；无法差分时为 None；
    ``jerk_rad_s3``       差分加加速度绝对值（rad/s^3）；无法差分时为 None。
    """

    sample: int
    joint_name: str
    level: str
    message: str
    delta_rad: float
    velocity_rad_s: float | None
    acceleration_rad_s2: float | None = None
    jerk_rad_s3: float | None = None


@dataclass(frozen=True)
class TeachTrajectoryQuality:
    """一条示教轨迹的质量评估结果（由 :func:`analyze_teach_trajectory` 产生）。

    ``risk_level``       总风险等级：``green``（正常回放）/ ``yellow``（需安全重定时）/
                         ``red``（禁止真实回放）；
    ``replay_policy``    与等级对应的策略说明文本（英文，供上层 UI 直接展示）；
    ``allow_real_replay`` 是否允许真实回放，等价于 ``risk_level != "red"``；
    ``requires_safe_retiming`` 是否必须先做安全重定时，等价于 ``risk_level == "yellow"``；
    ``max_jump_rad``     全轨迹最大单帧关节增量（rad）；
    ``max_velocity_rad_s`` / ``max_acceleration_rad_s2`` / ``max_jerk_rad_s3``
                         差分得到的最大速度 / 加速度 / 加加速度（rad/s、rad/s^2、rad/s^3）；
    ``worst_joint`` / ``worst_sample`` 最大跳变所在关节与样本下标（无跳变时分别
                         为空串和 -1）；
    ``anomalies``        全部异常描述（去重前），含结构性异常与越限事件；
    ``events``           结构化的异常事件元组；
    ``green_jump_rad`` / ``yellow_jump_rad`` 判定时使用的绿/黄跳变阈值（rad），随结果一起
                         回传，便于上层复现判定条件；
    ``velocity_limit_rad_s`` 生效的速度上限（rad/s，多关节时取各关节上限中的最大值）；
    ``acceleration_limit_rad_s2`` / ``jerk_limit_rad_s3`` 生效的加速度、加加速度上限；
    ``max_jerk_rad_s3``  实测最大 jerk（rad/s^3），与 ``jerk_limit_rad_s3``（阈值）区分。
    """

    risk_level: str
    replay_policy: str
    allow_real_replay: bool
    requires_safe_retiming: bool
    max_jump_rad: float
    max_velocity_rad_s: float
    max_acceleration_rad_s2: float
    worst_joint: str
    worst_sample: int
    anomalies: tuple[str, ...]
    events: tuple[TeachTrajectoryEvent, ...]
    green_jump_rad: float
    yellow_jump_rad: float
    velocity_limit_rad_s: float
    acceleration_limit_rad_s2: float
    max_jerk_rad_s3: float = 0.0
    jerk_limit_rad_s3: float = 999.0


@dataclass(frozen=True)
class RetimedTeachPoint:
    """重定时后的一段轨迹点（时间轴已重排，位置仍取自原始示教样本）。

    ``time_from_start`` 相对本段轨迹起点的时间（秒），严格递增；
    ``positions``       关节位置（rad），顺序与来源记录的 ``joint_names`` 一致；
    ``source_sample``   对应的原始样本下标；由软启动/对齐等步骤插入的合成点为 -1；
    ``velocities``      重定时算出的关节速度（rad/s，位置对时间的差分），可为空元组。
    """

    time_from_start: float
    positions: tuple[float, ...]
    source_sample: int
    velocities: tuple[float, ...] = ()


@dataclass(frozen=True)
class PreparedTeachReplay:
    """一条示教记录的完整预处理结果（原始质量 → 滤波后质量 → 重定时后质量）。

    中间数据：
    ``samples``            预处理流水线最终得到的样本序列（平滑 + 滤波 + 重采样之后）；
    ``raw_quality``        原始记录的质量评估（回放前的"before"）；
    ``filtered_quality``   平滑 + 滤波之后、重采样/重定时之前的质量评估；
    ``retimed_quality``    重定时之后（若未重定时则为 ``samples``）的质量评估，也就是
                           真实回放实际执行的那条轨迹的"after"；
    各步骤是否真正生效（供 UI 展示实际生效的处理链，注意 ``*_applied`` 表示"这一步跑过"，
    不等同于"数据被改变了"）：
    ``smoothing_applied`` / ``filter_applied`` / ``resample_applied`` / ``retime_applied``；
    生效参数（都是被夹到安全下限之后的实际值）：
    ``smoothing_window``   平滑窗口长度（奇数，样本数）；
    ``filter_cutoff_hz``   低通滤波截止频率（Hz）；
    ``filter_sample_rate_hz`` 滤波假定的采样率（Hz）；
    ``resample_rate_hz``   重采样目标频率（Hz）；
    ``retimed_points``     重定时轨迹点列表；为空表示未做重定时；
    大幅度运动与降速信息：
    ``large_motion``       轨迹行程是否达到"大幅度"判据；
    ``max_joint_span_rad`` 单关节最大行程（rad，各关节位置极差的最大值）；
    ``total_joint_motion_rad`` 全轨迹各关节增量绝对值之和（rad），反映总运动量；
    ``requested_replay_speed`` 请求的回放倍速（已夹到 (0, 1]）；
    ``effective_replay_speed`` 实际生效倍速（当前实现等于请求值，保留字段以便后续限速）；
    ``large_motion_max_speed`` 大幅度运动允许的最大倍速（越小越慢越安全）；
    时间参数化后端信息：
    ``time_parameterization_requested_method`` 请求的方法名（``auto`` / ``ruckig`` /
                          ``current_jerk_retime`` 等）；
    ``time_parameterization_used_method`` 实际使用的方法名，未重定时时为 ``none``；
    ``time_parameterization_message`` 后端给出的说明（英文，含回退原因）。

    属性 ``before_quality`` / ``after_quality`` 是门控代码使用的简写：分别指向原始质量与
    重定时后质量。
    """

    samples: list[TeachSample]
    raw_quality: TeachTrajectoryQuality
    filtered_quality: TeachTrajectoryQuality
    retimed_quality: TeachTrajectoryQuality
    smoothing_applied: bool
    filter_applied: bool
    resample_applied: bool
    retime_applied: bool
    smoothing_window: int
    filter_cutoff_hz: float
    filter_sample_rate_hz: float
    resample_rate_hz: float
    retimed_points: list[RetimedTeachPoint]
    large_motion: bool = False
    max_joint_span_rad: float = 0.0
    total_joint_motion_rad: float = 0.0
    requested_replay_speed: float = 1.0
    effective_replay_speed: float = 1.0
    large_motion_max_speed: float = 1.0
    time_parameterization_requested_method: str = "auto"
    time_parameterization_used_method: str = "current_jerk_retime"
    time_parameterization_message: str = ""

    @property
    def before_quality(self) -> TeachTrajectoryQuality:
        return self.raw_quality

    @property
    def after_quality(self) -> TeachTrajectoryQuality:
        return self.retimed_quality


class ReplayStartBand(str, Enum):
    """回放起点误差分档（也是真实回放的第一道门）。

    ``direct``       当前位姿已足够接近记录起点，可直接回放，无需对齐；
    ``align``        误差较小，允许自动对齐（软启动/对齐段）后再回放；
    ``moveit_align`` 需要走 MoveIt 规划对齐（由上层工作流实现），本模块只把它当作合法分档；
    ``reject``       误差过大，禁止回放，必须人工把机械臂拖到记录起点附近。
    """

    DIRECT = "direct"
    ALIGN = "align"
    MOVEIT_ALIGN = "moveit_align"
    REJECT = "reject"


@dataclass(frozen=True)
class ReplayStartDecision:
    """起点误差分档结果。

    ``band``            分档（见 :class:`ReplayStartBand`）；
    ``max_error``       各关节误差绝对值的最大值（rad）；关节数不匹配时为正无穷；
    ``per_joint_error`` 逐关节误差绝对值（rad），顺序与传入的关节向量一致；分档为
                        ``reject`` 且因长度不匹配时为空元组；
    ``allow_replay``    是否允许（含自动对齐后）回放；
    ``allow_auto_align`` 是否允许自动对齐（只有 ``align`` 档为 True）；
    ``message``         说明文本（英文，供 UI 直接展示）。
    """

    band: ReplayStartBand
    max_error: float
    per_joint_error: tuple[float, ...]
    allow_replay: bool
    allow_auto_align: bool
    message: str


@dataclass(frozen=True)
class TeachRecordInfo:
    """一条示教记录文件的巡检结果（文件是否存在、能否解析、以及质量与起点误差）。

    ``path``            记录文件路径（字符串形式）；
    ``exists``          文件是否存在；
    ``samples``         解析出的样本数（文件缺失/损坏/为空时为 0）；
    ``duration_sec``    记录时长（秒，末帧时间戳减首帧时间戳，负数被夹到 0）；
    ``joint_names``     关节顺序（取自首帧）；
    ``start_positions`` / ``end_positions`` 首帧与末帧的关节位置（rad）；
    ``start_band``      起点分档字符串：``direct``/``align``/``moveit_align``/``reject``，
                        以及异常取值 ``missing``（文件不存在）、``invalid``（JSONL 解析
                        失败）、``empty``（没有样本）、``unknown``（没有当前关节状态或缺少
                        记录里的关节，无法比较）；
    ``max_error``       起点最大关节误差（rad）；无法比较时为 None；
    ``worst_joint``     误差最大的关节名（无误差信息时为空串）；
    ``per_joint_error`` 逐关节误差（rad），键为关节名；
    ``anomalies``       异常描述元组（结构巡检与质量评估结果合并去重后）；
    ``message``         说明文本（英文）；
    ``quality``         轨迹质量评估；文件缺失或损坏时为 None。
    """

    path: str
    exists: bool
    samples: int
    duration_sec: float
    joint_names: tuple[str, ...]
    start_positions: tuple[float, ...]
    end_positions: tuple[float, ...]
    start_band: str
    max_error: float | None
    worst_joint: str
    per_joint_error: dict[str, float]
    anomalies: tuple[str, ...]
    message: str
    quality: TeachTrajectoryQuality | None = None


@dataclass(frozen=True)
class TeachDryRunDecision:
    """dry-run / 真实回放 / 停止请求的统一门控结果。

    ``accepted`` 是否接受该请求；
    ``state``    请求被接受后应进入的状态字符串（如 ``dry_run``、``replaying``、
                 ``cancel_requested``），被拒绝时为 ``blocked``（无活动目标时停止请求返回
                 ``idle``）；
    ``message``  拒绝或接受的原因（英文，供 UI 直接展示）。
    """

    accepted: bool
    state: str
    message: str


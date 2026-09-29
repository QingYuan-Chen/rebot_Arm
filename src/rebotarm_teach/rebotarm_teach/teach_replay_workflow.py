"""示教回放工作流：从示教记录文件到实际回放的完整生命周期。

职责与位置
    本模块只做"回放编排"，不做 HTTP、不做界面、不做底层电机访问：它从示教记录文件读取原始
    样本，调用本包的预处理能力生成"预处理轨迹"（滤波/重采样/重定时/校验），在真正下发
    Action 之前依次经过三道安全门（文件检查+质量分级、MoveIt 起始对齐评估、碰撞预检），
    最后通过 FollowJointTrajectory 动作把轨迹交给轨迹控制器，并在回放过程中做跟踪误差
    监控与安全停止。上层界面节点只负责把命令 payload 传进来、把状态 payload 广播出去。

对外接口（全部由调用方注入，本模块不直接创建节点/网络资源）
    ``record_info`` / ``records``      记录文件检查与目录列举；
    ``trajectory_preview``             预处理后的预览曲线与碰撞预检结果；
    ``dry_run``                        全流程演练：只做检查与规划，绝不下发目标；
    ``execute``                        真正回放：dry-run 令牌 + 三道门全部通过后才下发；
    ``stop``                           请求取消回放（必要时回退到控制器急停服务）；
    ``check_tracking``                 由上层周期调用，做运行时跟踪监控。

回放状态（``publish_status`` 推送的 ``state`` 字段，取值由回放协调器与运行时监控决定）
    ``dry_run`` / ``replaying`` / ``done`` / ``canceled`` / ``safety_stop`` / ``failed``
    ``blocked``（被安全门拦住）/ ``unavailable``（动作服务不可用）。

安全约束（顺序不可颠倒）
    1. ``execute`` 必须携带一次成功且参数一致的 ``dry_run`` 令牌，否则一律拒绝下发；
    2. 预处理质量分级为 red、或预处理后跳变/加速度/急动度超限时直接阻断；
    3. MoveIt 起始对齐状态为 failed/unavailable/unknown 时按"不可放行"处理；
    4. 碰撞预检只有 ``pass`` 才放行，``collision`` 与 ``unknown`` 都阻断；
    5. 回放中跟踪误差或实时速度超限并持续超过宽限时间时，主动请求控制器停止并取消目标。

    参数来源
    全部通过节点参数读取（``node.get_parameter``），参数名、单位与取值范围见上层启动组合包
    ``config/teach_control.yaml``、``config/operator_common.yaml`` 与对应的回放 launch 文件；
    本模块不修改任何参数。

线程模型
    持有状态（目标句柄、活动轨迹、监控器）用一把互斥锁保护；Action 结果回调运行在
    executor 线程，与上层周期调用的 ``check_tracking`` 并发，所有读写都在锁内完成。
"""

from __future__ import annotations

import threading
import time
from contextlib import suppress
from pathlib import Path
from typing import Callable, Protocol, Mapping

from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetStateValidity
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rebotarm_motion.collision_precheck import CollisionPrechecker
from rebotarm_motion.replay_runtime_monitor import ReplayRuntimeMonitor, ReplayRuntimeMonitorConfig
from .teach_replay_client import TeachReplayClient
from .teach_replay_coordinator import TeachReplayCoordinator, TeachReplayLimits
from rebotarm_teach.teach_recording import (
    ReplayStartBand,
    inspect_teach_record,
    list_teach_record_files,
    load_teach_samples,
    prepare_teach_replay_samples,
    prepared_teach_replay_to_dict,
    teach_record_info_to_dict,
    teach_trajectory_preview_to_dict,
    validate_teach_dry_run_request,
    write_prepared_teach_record,
)
from rebotarm_teach.teach_replay_settings import TeachReplaySettingsProvider
from rebotarm_motion.teach_replay_start_align_precheck import MoveItStartAlignPrechecker
from rebotarm_motion.teach_replay_start_alignment import MoveItStartAligner
from rebotarm_teach.teach_replay_trajectory_builder import (
    TeachReplayTrajectoryBuilder,
    TeachReplayTrajectoryConfig,
)
from rebotarm_motion.moveit_planner import MoveItMotionPlanner

from .replay_action_lifecycle import ReplayActionLifecycleMixin
from .replay_moveit_precheck import ReplayMoveItPrecheckMixin


class ReplaySnapshot(Protocol):
    """工作流所需的节点状态快照协议（结构化类型，无需继承）。

    ``joints``  关节名 -> 反馈字典，至少含 ``position`` 键（单位 rad），
                用于计算与记录起点的误差、构造轨迹首点与运行时跟踪监控；
    ``teleop``  操作状态字典，其中 ``recording`` / ``replay`` 子字典提供当前记录文件
                路径与最近一次回放状态，实现"未显式指定记录文件时沿用当前选择"。
    """

    joints: Mapping[str, dict]
    teleop: Mapping[str, dict]


def _is_number_like(value) -> bool:
    try:
        float(value)
    except (TypeError, ValueError):
        return False
    return True


class TeachReplayWorkflow(ReplayMoveItPrecheckMixin, ReplayActionLifecycleMixin):
    """示教回放全流程编排器。

    生命周期
        由上层界面节点在 ``__init__`` 中构造一次，随后长期驻留；每次回放（dry-run 或
        execute）都是本对象上的无状态入口调用，跨调用保留的状态只有 dry-run 令牌
        （``_last_teach_dry_run``）和当前活动回放（目标句柄/轨迹/起始时刻/运行时监控器）。

    与其他组件的关系
        记录的读写与预处理算法在本包的示教记录模块，规划、起始对齐、碰撞预检、运行时监控在
        运动包；本类只负责按正确顺序调用它们、组装状态 payload，并把动作客户端与状态发布
        交给注入的回调。它不导入界面模块，也不做 HTTP 命令鉴权。

    依赖注入（全部由构造参数提供，便于脱离 ROS 与界面做单元测试）
        ``node``              提供 ``get_parameter`` / ``create_client`` 的节点对象；
        ``joint_names``       受控关节名（顺序即位置向量顺序，单位 rad）；
        ``snapshot``          返回当前关节/操作状态快照的可调用对象；
        ``publish_status``    ``(channel, payload)`` 状态发布回调，本类只用 ``"replay"``；
        ``collision_defaults`` 给定关节名序列，返回碰撞检查需要补齐的默认关节位置
                              （例如夹爪指关节），否则状态有效性查询结果不可信；
        ``action_client``     轨迹控制器动作客户端（发送示教回放轨迹）；
        ``trajectory_stop_client`` 控制器急停服务客户端，取消目标时的兜底通道；
        ``request_stop``      直接请求控制器停止轨迹的可调用对象（带 ``timeout_sec``）。

    线程/回调模型
        状态发布与 ROS 回调可能来自不同线程，活动回放状态统一用 ``_teach_replay_lock``
        保护；本类不创建 timer，状态推送即"当前快照"，不保证按序送达。
    """

    def __init__(
        self, *, node, joint_names: tuple[str, ...],
        snapshot: Callable[[], ReplaySnapshot],
        publish_status: Callable[[str, dict], None],
        collision_defaults: Callable[[tuple[str, ...]], tuple[tuple[str, float], ...]],
        action_client, trajectory_stop_client, request_stop: Callable[..., bool],
    ):
        self.get_parameter = node.get_parameter
        self.create_client = node.create_client
        self._joint_names = joint_names
        self._snapshot = snapshot
        self._publish_status = publish_status
        self._collision_default_joint_positions = collision_defaults
        self._action_client = action_client
        self._trajectory_stop_client = trajectory_stop_client
        self._request_controller_trajectory_stop = request_stop
        # 关节空间规划器：与规划组名、规划服务、规划管线与求解器 ID 一同来自节点参数；
        # 末端执行器坐标系 end_link、参考坐标系 base_link 与回放动作的坐标约定一致。
        self._moveit_planner = MoveItMotionPlanner(
            node,
            group_name=str(self.get_parameter("moveit_group_name").value),
            ee_frame_id="end_link",
            frame_id="base_link",
            planning_service=str(self.get_parameter("moveit_planning_service").value),
            planning_pipeline=str(self.get_parameter("moveit_planning_pipeline").value),
            planner_id=str(self.get_parameter("moveit_planner_id").value),
            planning_time=float(self.get_parameter("moveit_planning_time").value),
            num_attempts=int(self.get_parameter("moveit_num_planning_attempts").value),
            # 这里的两个容差是"无参数可调"的硬编码值：位置 0.005 m、姿态 0.02 rad，
            # 仅用于起始对齐这一段关节空间规划，不参与示教轨迹本身的重定时。
            goal_position_tolerance=0.005,
            goal_orientation_tolerance=0.02,
        )
        # 状态有效性查询服务客户端：用于下发前的碰撞预检（服务名由参数给出）。
        self._state_validity_client = self.create_client(
            GetStateValidity,
            str(self.get_parameter("collision_check_service").value),
        )
        self._collision_prechecker = CollisionPrechecker(
            client=self._state_validity_client,
            request_factory=GetStateValidity.Request,
        )
        # 以下四个协作对象都是无状态工具：回放客户端负责取消防抖与急停兜底，
        # 协调器集中所有放行门，轨迹构建器拼接"起始对齐段 + 录制段 + 末尾保持"。
        self._teach_replay_client = TeachReplayClient()
        self._teach_replay_coordinator = TeachReplayCoordinator()
        self._teach_replay_trajectory_builder = TeachReplayTrajectoryBuilder(
            trajectory_factory=JointTrajectory,
            trajectory_point_factory=JointTrajectoryPoint,
        )
        self._moveit_start_aligner = MoveItStartAligner(
            planner=self._moveit_planner,
            trajectory_point_factory=JointTrajectoryPoint,
        )
        self._moveit_start_align_prechecker = MoveItStartAlignPrechecker(
            planner=self._moveit_planner,
        )
        # 回放设置提供者：把"速度/对齐时长/对齐步数"等参数与本次请求 payload 合并成
        # 规范化设置；align_duration_auto 打开时对齐时长由起始误差自动推算。
        self._teach_replay_settings_provider = TeachReplaySettingsProvider(
            replay_speed=float(self.get_parameter("replay_speed").value),
            align_duration=float(self.get_parameter("align_duration").value),
            align_duration_auto=bool(self.get_parameter("align_duration_auto").value),
            align_target_speed_rad_s=float(self.get_parameter("align_target_speed_rad_s").value),
            align_min_duration=float(self.get_parameter("align_min_duration").value),
            align_max_duration=float(self.get_parameter("align_max_duration").value),
            align_steps=int(self.get_parameter("align_steps").value),
        )
        # 活动回放状态：目标句柄/活动轨迹/起始时刻三个字段必须一起读写，
        # 用互斥锁保证与 ROS 回调线程、周期跟踪检查之间的一致性。
        self._teach_replay_lock = threading.Lock()
        self._teach_replay_goal_handle = None
        self._active_teach_replay_trajectory: JointTrajectory | None = None
        self._active_teach_replay_started_at: float | None = None
        self._replay_runtime_monitor = ReplayRuntimeMonitor()
        # dry-run 令牌：只有成功且参数一致的 dry-run 才会写入，execute 时逐字段比对；
        # 回放被拒绝或失败时必须清空（见 dry_run 末尾），防止旧令牌放行新请求。
        self._last_teach_dry_run: dict | None = None

    def _target_runtime(self) -> str:
        # 仅用于状态 payload 标注本次回放面向真实硬件还是仿真，不参与任何放行判定。
        return "hardware" if bool(self.get_parameter("use_hardware").value) else "simulation"

    def record_info(self, record_path: str | None = None) -> dict:
        """检查示教记录文件并返回紧凑状态 payload（记录检查是回放的第一道门）。

        ``record_path`` 为 None 时按"当前操作状态里选中的记录文件"解析，其次退回节点参数
        ``record_path``；起始误差用当前关节反馈与记录首样本比较，按 ``direct_threshold`` /
        ``align_threshold`` 分级为 direct / align / reject；reject 且开启 MoveIt 起始对齐
        且误差可用时，分级改写为 moveit_align（由规划代偿过大的起始偏差）。
        本方法只读取与统计，不写文件、不下发动作。
        """
        snapshot = self._snapshot()
        path = record_path or str(self.get_parameter("record_path").value)
        # 显式传入的路径优先；未指定时才沿用"当前正在录制/上一次回放"的记忆路径。
        for key in ("recording", "replay"):
            value = snapshot.teleop.get(key)
            if record_path is None and isinstance(value, dict) and value.get("record_path"):
                path = str(value["record_path"])
                break
        # 只取本机受控关节的当前位置（rad）；快照缺失关节位置时整体传 None，
        # 记录检查会退化为"当前关节状态不可用"，而不是用 0 冒充真实反馈。
        current_positions = {
            name: float(data["position"])
            for name, data in snapshot.joints.items()
            if name in self._joint_names and "position" in data
        }
        info = inspect_teach_record(
            path,
            current_positions=current_positions if current_positions else None,
            direct_threshold=float(self.get_parameter("direct_threshold").value),
            align_threshold=float(self.get_parameter("align_threshold").value),
        )
        payload = teach_record_info_to_dict(info)
        # 起始误差被判为 reject（超出 align_threshold）但允许 MoveIt 规划代偿时，把分级
        # 提升为 moveit_align：此时回放仍可执行，只是起始段由规划器接管（单位 rad）。
        if (
            str(payload.get("start_band", "")).lower() == ReplayStartBand.REJECT.value
            and bool(self.get_parameter("use_moveit_start_align").value)
            and _is_number_like(payload.get("max_error"))
        ):
            payload["start_band"] = ReplayStartBand.MOVEIT_ALIGN.value
            payload["message"] = "start error requires MoveIt start alignment"
        # 阈值一并回传，便于界面显示"当前误差相对哪条界线"。
        payload["direct_threshold"] = float(self.get_parameter("direct_threshold").value)
        payload["align_threshold"] = float(self.get_parameter("align_threshold").value)
        return self._compact_replay_payload(payload)

    @staticmethod
    def _compact_list(items, *, limit: int = 12) -> list:
        # 统一截断入口：非序列输入按空列表处理，limit 负数被夹到 0。
        values = list(items) if isinstance(items, (list, tuple)) else []
        return values[: max(int(limit), 0)]

    @classmethod
    def _compact_quality_payload(cls, quality: dict, *, limit: int = 12) -> dict:
        """压缩一份质量字典：事件与异常各保留前 ``limit`` 条，并补总数与截断标记。

        质量明细会随状态 payload 周期性广播，条目数可能上百，必须限长；截断后仍保留
        ``*_total`` 与 ``*_truncated`` 字段，界面据此提示"还有更多未展示"。
        """
        compact = dict(quality)
        if isinstance(compact.get("events"), list):
            compact["events_total"] = len(compact["events"])
            compact["events"] = cls._compact_list(compact["events"], limit=limit)
            compact["events_truncated"] = compact["events_total"] > len(compact["events"])
        if isinstance(compact.get("anomalies"), list):
            compact["anomalies_total"] = len(compact["anomalies"])
            compact["anomalies"] = cls._compact_list(compact["anomalies"], limit=limit)
            compact["anomalies_truncated"] = compact["anomalies_total"] > len(compact["anomalies"])
        return compact

    @classmethod
    def _compact_replay_payload(cls, payload: dict, *, limit: int = 12) -> dict:
        """压缩完整回放状态 payload：六个质量块逐块限长，顶层异常限长，嵌套 payload 递归。

        递归只沿 ``prepared_replay`` 一层层深入，不会重复展开已压缩过的质量块，因此
        整份 payload 的体积与嵌套深度都受控。
        """
        compact = dict(payload)
        for key in (
            "quality",
            "before_quality",
            "after_quality",
            "raw_quality",
            "filtered_quality",
            "retimed_quality",
        ):
            if isinstance(compact.get(key), dict):
                compact[key] = cls._compact_quality_payload(compact[key], limit=limit)
        if isinstance(compact.get("anomalies"), list):
            compact["anomalies_total"] = len(compact["anomalies"])
            compact["anomalies"] = cls._compact_list(compact["anomalies"], limit=limit)
            compact["anomalies_truncated"] = compact["anomalies_total"] > len(compact["anomalies"])
        if isinstance(compact.get("prepared_replay"), dict):
            compact["prepared_replay"] = cls._compact_replay_payload(compact["prepared_replay"], limit=limit)
        return compact

    def _max_replay_velocity_limits(self, joint_names: tuple[str, ...]):
        """解析回放速度上限：按关节的列表优先，长度对不上时退回标量上限。

        逐关节列表（单位 rad/s）允许大关节与小关节使用不同上限；长度与关节序列不匹配
        说明配置与机型不符，此时退回参数 ``max_replay_velocity_rad_s`` 的标量值，
        不猜测"哪个值对应哪个关节"的映射关系（该标量值是否偏保守取决于配置）。
        """
        scalar_limit = float(self.get_parameter("max_replay_velocity_rad_s").value)
        values = self.get_parameter("max_replay_velocity_rad_s_by_joint").value
        if isinstance(values, (list, tuple)) and len(values) == len(joint_names):
            return tuple(float(value) for value in values)
        return scalar_limit

    def _prepare_teach_replay_samples(self, samples, settings: dict[str, float | int] | None = None):
        """把原始示教样本预处理成可用于回放的样本序列（滤波/重采样/重定时）。

        处理链固定为"平滑 -> 低通滤波 -> 重采样 -> 重定时（时间参数化）"，开关与参数
        全部来自节点参数；``settings`` 提供本次请求的规范化设置（速度、对齐参数），
        缺省时用参数里的 ``replay_speed``。``retime_enabled`` 恒为 True：回放一律使用
        预处理轨迹，原始录制样本只作为输入数据。返回值同时携带处理前后质量报告。
        """
        replay_speed = float(settings["replay_speed"]) if settings else float(self.get_parameter("replay_speed").value)
        return prepare_teach_replay_samples(
            samples,
            smoothing_enabled=bool(self.get_parameter("smoothing_enabled").value),
            # 平滑窗口为采样点个数（奇数更对称），越大曲线越平滑但细节丢失越多。
            smoothing_window=int(self.get_parameter("smoothing_window").value),
            filter_enabled=bool(self.get_parameter("filter_enabled").value),
            # 低通截止频率 Hz；必须明显低于采样率，否则滤波无意义（一般取采样率的 1/10 量级）。
            filter_cutoff_hz=float(self.get_parameter("filter_cutoff_hz").value),
            filter_sample_rate_hz=float(self.get_parameter("filter_sample_rate_hz").value),
            resample_enabled=bool(self.get_parameter("resample_enabled").value),
            # 统一重采样到固定频率 Hz，使后续重定时与动作点的 dt 均匀。
            resample_rate_hz=float(self.get_parameter("resample_rate_hz").value),
            retime_enabled=True,
            replay_speed=replay_speed,
            max_velocity_rad_s=self._max_replay_velocity_limits(tuple(samples[0].joint_names) if samples else ()),
            # 重定时的加速度/急动度上限（rad/s^2、rad/s^3），超限会被拉长轨迹时间。
            max_acceleration_rad_s2=float(self.get_parameter("max_replay_acceleration_rad_s2").value),
            max_jerk_rad_s3=float(self.get_parameter("max_replay_jerk_rad_s3").value),
            # 时间参数化算法选择（auto 时由实现按轨迹特征挑选）。
            time_parameterization_method=str(self.get_parameter("time_parameterization_method").value),
            # 大运动判据：单关节跨度 rad、全关节累计运动 rad、以及触发后限制到的速度倍率。
            large_motion_span_rad=float(self.get_parameter("large_motion_span_rad").value),
            large_motion_total_rad=float(self.get_parameter("large_motion_total_rad").value),
            large_motion_max_speed=float(self.get_parameter("large_motion_max_speed").value),
        )





    def _teach_replay_limits(self) -> TeachReplayLimits:
        """本次回放的硬性上限（预处理跳变 rad、加速度 rad/s^2、急动度 rad/s^3）。

        这三个值与回放协调器里的质量分级共同决定 execute 是否放行。
        """
        return TeachReplayLimits(
            max_prepared_jump_rad=float(self.get_parameter("max_prepared_jump_rad").value),
            max_replay_acceleration_rad_s2=float(self.get_parameter("max_replay_acceleration_rad_s2").value),
            max_replay_jerk_rad_s3=float(self.get_parameter("max_replay_jerk_rad_s3").value),
        )

    def trajectory_preview(self, record_path: str | None = None, max_points: int = 500) -> dict:
        """生成预处理轨迹的可视化预览（曲线数据 + 预处理报告 + 碰撞预检）。

        预览曲线取自"预处理后的记录文件"而非原始记录：界面看到的就是回放真正会走的轨迹。
        预处理结果会写入 ``<记录名>.prepared.jsonl``（与原始记录同目录、扩展名不同），
        并排除在记录列举之外；``max_points`` 限制返回给界面的采样点数（默认为 500）。
        读取失败不抛异常，返回 ``accepted=False`` 与失败原因。
        """
        path = record_path or str(self.record_info(None).get("path", self.get_parameter("record_path").value))
        try:
            samples = load_teach_samples(path)
        except Exception as exc:
            return {
                "accepted": False,
                "message": f"failed to load teach trajectory: {exc}",
                "path": str(path),
                "points": [],
            }
        prepared = self._prepare_teach_replay_samples(samples)
        prepared_path = write_prepared_teach_record(path, prepared)
        preview_samples = load_teach_samples(prepared_path)
        payload = teach_trajectory_preview_to_dict(preview_samples, max_points=max_points)
        payload["prepared_replay"] = prepared_teach_replay_to_dict(prepared)
        payload["collision_precheck"] = self._collision_precheck(preview_samples)
        payload["accepted"] = True
        # 明确标注曲线来源是预处理轨迹，界面可据此提示"原始数据仅作输入"。
        payload["curve_source"] = "prepared"
        payload["path"] = str(prepared_path)
        payload["raw_record_path"] = str(path)
        payload["prepared_record_path"] = str(prepared_path)
        payload["info"] = self.record_info(str(path))
        return payload

    def records(self) -> dict:
        """列举记录目录下的示教记录文件（用于界面选择）。

        目录取参数 ``record_path`` 的父目录；父目录为空时退回相对目录 ``teleop_records``。
        返回目录、默认记录路径与每个记录的概况（样本数、时长、起始分级、异常）。
        """
        record_path = Path(str(self.get_parameter("record_path").value))
        directory = record_path.parent if str(record_path.parent) else Path("teleop_records")
        records = list_teach_record_files(directory)
        return {
            "directory": str(directory),
            "default_record_path": str(record_path),
            "records": records,
        }

    def dry_run(self, payload: dict) -> dict:
        """全流程演练：检查记录、预处理、评估起始对齐与碰撞，但绝不下发任何目标。

        ``payload`` 可含 ``record_path`` 与 ``settings``（回放速度、对齐时长/步数）。
        只有本次演练被接受时才写入 dry-run 令牌（``_last_teach_dry_run``），execute 必须
        凭该令牌才允许下发；令牌绑定记录路径、预处理风险等级与设置，防止"用 A 记录演练、
        下发 B 记录"或"演练后偷改速度"。本方法只发布状态，不发送 Action。
        """
        record_path = payload.get("record_path")
        info_payload = self.record_info(str(record_path) if record_path else None)
        settings = self._teach_replay_settings_from_payload(
            payload,
            max_error=info_payload.get("max_error"),
        )
        decision = validate_teach_dry_run_request(str(info_payload.get("start_band", "")))
        prepared_payload = {}
        prepared_record_path = ""
        # 预检默认状态是 unknown（"尚未检查"），未经检查绝不能被当成通过。
        collision_precheck = {"state": "unknown", "message": "collision precheck not run"}
        moveit_align = self._moveit_align_summary(info_payload)
        samples_for_precheck = []
        trajectory_points = 0
        try:
            samples_for_precheck = load_teach_samples(str(info_payload.get("path", "")))
            prepared = self._prepare_teach_replay_samples(samples_for_precheck, settings)
            prepared_record_path = str(write_prepared_teach_record(str(info_payload.get("path", "")), prepared))
            prepared_payload = prepared_teach_replay_to_dict(prepared)
            # 只有 dry-run 本身被接受时才真的调用规划服务（plan=True），避免为必然
            # 被拒的请求浪费规划时间；对齐状态是 failed/unavailable/unknown 时不再
            # 构造轨迹，直接让协调器按"门未通过"给出 blocked 结论。
            moveit_align = self._moveit_align_summary(info_payload, samples_for_precheck, plan=decision.accepted)
            if decision.accepted and str(moveit_align.get("state", "")).lower() not in ("failed", "unavailable", "unknown"):
                trajectory = self._build_teach_replay_trajectory(
                    samples_for_precheck,
                    str(info_payload.get("start_band", "")),
                    settings,
                )
                trajectory_points = len(trajectory.points)
                collision_precheck = self._collision_precheck_trajectory(trajectory)
        except Exception as exc:
            # 预处理/构造失败一律降级为 unknown 预检，绝不放行，也不中断状态上报。
            prepared_payload = {"error": str(exc)}
            collision_precheck = {"state": "unknown", "message": f"collision precheck failed: {exc}"}
        result = self._teach_replay_coordinator.build_dry_run_result(
            info_payload=info_payload,
            settings=settings,
            decision=decision,
            prepared_payload=prepared_payload,
            prepared_record_path=prepared_record_path,
            moveit_align=moveit_align,
            collision_precheck=collision_precheck,
            trajectory_points=trajectory_points,
            limits=self._teach_replay_limits(),
            target_runtime=self._target_runtime(),
            compact_payload=self._compact_replay_payload,
        )
        # 令牌只在成功时生效，并被清空于失败；这是 execute 的唯一凭据。
        self._last_teach_dry_run = result if result["accepted"] else None
        self._publish_status("replay", result)
        return result

    def execute(self, payload: dict) -> dict:
        """真实回放：dry-run 令牌 + 质量门 + 起始对齐门 + 碰撞门全部通过后下发轨迹。

        ``payload`` 与 dry-run 同构（``record_path`` / ``settings``）。被任一门拦住时只
        发布状态并返回，不发送 Action；通过后在同一调用内发送 FollowJointTrajectory 目标，
        后续状态（接受/结果/取消）由异步回调推送。回放一律使用本次重新生成的预处理轨迹。
        """
        record_path = payload.get("record_path")
        info_payload = self.record_info(str(record_path) if record_path else None)
        settings = self._teach_replay_settings_from_payload(
            payload,
            max_error=info_payload.get("max_error"),
        )
        # quality 是原始记录的质量报告，prepared_quality 是预处理后的报告（后续覆盖）。
        quality = info_payload.get("quality") if isinstance(info_payload.get("quality"), dict) else {}
        prepared_payload = {}
        prepared_quality = {}
        prepared_record_path = ""
        collision_precheck = {"state": "unknown", "message": "collision precheck not run"}
        moveit_align = self._moveit_align_summary(info_payload)
        trajectory = None
        try:
            source_samples = load_teach_samples(str(info_payload.get("path", "")))
            prepared = self._prepare_teach_replay_samples(source_samples, settings)
            prepared_record_path = str(write_prepared_teach_record(str(info_payload.get("path", "")), prepared))
            prepared_payload = prepared_teach_replay_to_dict(prepared)
            # 放行判定使用"预处理后"的质量等级：原始记录的质量只作参考。
            prepared_quality = prepared_payload.get("after_quality") if isinstance(prepared_payload.get("after_quality"), dict) else {}
            # execute 前只做轻量对齐评估（plan=False），真正规划留给轨迹构建阶段。
            moveit_align = self._moveit_align_summary(info_payload, source_samples, plan=False)
        except Exception as exc:
            prepared_payload = {"error": str(exc)}
            collision_precheck = {"state": "unknown", "message": f"collision precheck failed: {exc}"}
        # 第一道门：dry-run 令牌一致性 + 质量分级 + 跳变/加速度/急动度上限 + 黄色速度上限。
        decision = self._teach_replay_coordinator.evaluate_execute_request(
            info_payload=info_payload,
            settings=settings,
            prepared_quality=prepared_quality,
            dry_run_token=self._last_teach_dry_run or {},
            limits=self._teach_replay_limits(),
            yellow_max_speed=float(self.get_parameter("yellow_max_speed").value),
        )
        # 第二道门：起始对齐不可用或结论不明时按"不可放行"处理。
        moveit_state = str(moveit_align.get("state", "")).lower()
        if decision.accepted and moveit_state in ("failed", "unavailable", "unknown"):
            decision = type(decision)(
                accepted=False,
                state="blocked",
                message=f"MoveIt start alignment not ready: {moveit_align.get('message', moveit_state)}",
            )
        if decision.accepted:
            try:
                samples = load_teach_samples(str(info_payload["path"]))
                if not samples:
                    raise ValueError("record contains no samples")
                trajectory = self._build_teach_replay_trajectory(samples, str(info_payload.get("start_band", "")), settings)
                collision_precheck = self._collision_precheck_trajectory(trajectory)
            except Exception as exc:
                collision_precheck = {"state": "unknown", "message": f"collision precheck failed: {exc}"}
        # 第三道门：碰撞预检必须明确 pass；检测到碰撞或结论不明（服务不可用/超时/异常）
        # 都要阻断，绝不允许把"没查成功"当成"没有碰撞"。
        precheck_state = str(collision_precheck.get("state", "")).lower()
        if decision.accepted and precheck_state in ("collision", "unknown"):
            decision = type(decision)(
                accepted=False,
                state="blocked",
                message=f"collision precheck blocked replay: {collision_precheck.get('message', precheck_state)}",
            )
        if not decision.accepted:
            result = self._teach_replay_coordinator.build_execute_result(
                info_payload=info_payload,
                settings=settings,
                decision=decision,
                prepared_payload=prepared_payload,
                prepared_record_path=prepared_record_path,
                moveit_align=moveit_align,
                collision_precheck=collision_precheck,
                trajectory_points=0,
                limits=self._teach_replay_limits(),
                target_runtime=self._target_runtime(),
                compact_payload=self._compact_replay_payload,
            )
            self._publish_status("replay", result)
            return result
        # 动作服务不可用属于"环境问题"而非"安全拒绝"，单独报 unavailable；
        # 0.1 s 只是探测，不阻塞调用线程。
        if not self._action_client.wait_for_server(timeout_sec=0.1):
            message = "follow_joint_trajectory action unavailable"
            self._publish_status("replay", {"state": "unavailable", "message": message})
            return {"accepted": False, "message": message}
        # 门都通过了却没有轨迹（构造阶段异常）：仍然拒绝，不能下发空轨迹。
        if trajectory is None:
            result = {
                "accepted": False,
                "state": "blocked",
                "message": "failed to build replay trajectory",
                "record_path": str(info_payload.get("path", "")),
                "prepared_record_path": prepared_record_path,
                "start_band": str(info_payload.get("start_band", "")),
                "moveit_start_align": moveit_align,
                "collision_precheck": collision_precheck,
                "prepared_replay": prepared_payload,
                "dry_run": False,
            }
            result = self._compact_replay_payload(result)
            self._publish_status("replay", result)
            return result
        # 轨迹构建时缓存的预处理报告优先（它对应最终下发的轨迹），没有才用本次预处理结果。
        prepared_payload = getattr(self, "_last_teach_prepared_payload", prepared_payload)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        future = self._action_client.send_goal_async(goal)
        # 目标被接受/拒绝后进入回调：只有 accepted 才会登记活动回放并开启运行时监控。
        future.add_done_callback(lambda fut: self._on_teach_replay_goal_response(fut, info_payload, len(trajectory.points), trajectory))
        result = self._teach_replay_coordinator.build_execute_result(
            info_payload=info_payload,
            settings=settings,
            decision=decision,
            prepared_payload=prepared_payload,
            prepared_record_path=prepared_record_path,
            moveit_align=moveit_align,
            collision_precheck=collision_precheck,
            trajectory_points=len(trajectory.points),
            limits=self._teach_replay_limits(),
            target_runtime=self._target_runtime(),
            compact_payload=self._compact_replay_payload,
        )
        self._publish_status("replay", result)
        return result

    def stop(self) -> dict:
        """请求停止当前回放：有活动目标时取消目标，同时用急停服务兜底。

        没有活动目标时不做取消，但仍请求控制器 ``trajectory_stop``（清除可能的残留运动）；
        取消结果的回调由 ``_on_teach_replay_cancel_response`` 处理，本方法只返回即时结论。
        """
        with self._teach_replay_lock:
            goal_handle = self._teach_replay_goal_handle
        result = self._teach_replay_client.stop(
            goal_handle,
            trajectory_stop_client=self._trajectory_stop_client,
        )
        # cancel_future 是内部字段，不能出现在状态 payload 中，取出后单独挂回调。
        future = result.pop("cancel_future", None)
        if future is not None:
            future.add_done_callback(self._on_teach_replay_cancel_response)
        self._publish_status("replay", result)
        return result

    def _auto_align_duration_from_error(self, max_error: float | None) -> float:
        # 起始误差越大，对齐段越长；具体换算由设置提供者按目标速度与时长上下限计算。
        return float(self._teach_replay_settings_provider.auto_align_duration(max_error))

    def _teach_replay_settings_from_payload(
        self,
        payload: dict,
        *,
        max_error: float | None = None,
    ) -> dict[str, float | int]:
        """把请求 payload 中的 ``settings`` 与节点参数合并成规范化回放设置。

        返回键固定为 ``replay_speed`` / ``align_duration`` / ``align_steps`` /
        ``final_hold_sec``，取值已按实现内的上下限夹紧；``max_error`` 仅在自动对齐时长
        模式下参与计算。
        """
        return self._teach_replay_settings_provider.from_payload(payload, max_error=max_error)

    def _build_teach_replay_trajectory(self, samples, start_band: str, settings: dict[str, float | int]) -> JointTrajectory:
        """由原始样本构建最终下发的关节轨迹（预处理 + 起始段 + 录制段 + 末尾保持）。

        起始段有两种策略：开启 MoveIt 起始对齐时由规划器生成"当前位置 -> 记录首点"的
        过渡（失败即抛异常，不会静默降级）；关闭时用插值软启动。构建前会缓存预处理报告，
        供状态 payload 使用。当前位置取快照中的关节反馈（rad），与记录首点同序对齐。
        """
        prepared = self._prepare_teach_replay_samples(samples, settings)
        self._last_teach_prepared_payload = prepared_teach_replay_to_dict(prepared)
        first = prepared.samples[0]
        snapshot = self._snapshot()
        current_map = {
            name: float(data["position"])
            for name, data in snapshot.joints.items()
            if "position" in data
        }
        result = self._teach_replay_trajectory_builder.build(
            prepared=prepared,
            current_positions=current_map,
            start_band=start_band,
            settings=settings,
            config=TeachReplayTrajectoryConfig(
                use_moveit_start_align=bool(self.get_parameter("use_moveit_start_align").value),
                # 起始保持秒数（先原地停留再运动，避免启动瞬间拉扯）。
                start_hold_sec=float(self.get_parameter("start_hold_sec").value),
                # 软启动时长与插值步数（仅在未启用 MoveIt 起始对齐时生效）。
                soft_start_duration=float(self.get_parameter("soft_start_duration").value),
                soft_start_steps=int(self.get_parameter("soft_start_steps").value),
                # 到达记录首点后的保持秒数。
                first_hold_sec=float(self.get_parameter("first_hold_sec").value),
                # 质量等级为 yellow 时回放速度的上限（倍率）。
                yellow_max_speed=float(self.get_parameter("yellow_max_speed").value),
                # 录制段开始前的额外延迟（秒）。
                initial_replay_delay_sec=float(self.get_parameter("initial_replay_delay_sec").value),
                max_velocity_rad_s=self._max_replay_velocity_limits(tuple(first.joint_names)),
                max_acceleration_rad_s2=float(self.get_parameter("max_replay_acceleration_rad_s2").value),
                max_jerk_rad_s3=float(self.get_parameter("max_replay_jerk_rad_s3").value),
            ),
            moveit_start_alignment=self._append_moveit_start_alignment,
        )
        return result.trajectory

    def _append_final_hold(self, trajectory: JointTrajectory, *, final_hold_sec: float) -> None:
        # 在轨迹末尾追加一个"保持当前位置"的点，保证回放结束后动作正常收尾。
        self._teach_replay_trajectory_builder.append_final_hold(
            trajectory,
            final_hold_sec=final_hold_sec,
        )





    def check_tracking(self) -> None:
        """运行时跟踪监控（由上层按 ``replay_monitor_period_sec`` 周期调用）。

        取快照中的关节位置/速度与活动轨迹的期望值比较，超过 ``max_tracking_error_rad``
        或 ``max_live_velocity_rad_s`` 且持续超过 ``replay_monitor_violation_grace_sec``
        时，先请求控制器停止轨迹（0.2 s 超时），再取消动作目标（取消失败不抛出），
        最后发布 ``safety_stop`` 状态。没有活动目标时直接返回，不做任何判定。
        """
        snapshot = self._snapshot()
        with self._teach_replay_lock:
            goal_handle = self._teach_replay_goal_handle
            trajectory = self._active_teach_replay_trajectory
            started_at = self._active_teach_replay_started_at
        if goal_handle is None:
            return
        decision = self._replay_runtime_monitor.check(
            trajectory=trajectory,
            started_at=started_at,
            joints=snapshot.joints,
            now=time.monotonic(),
            config=ReplayRuntimeMonitorConfig(
                enabled=bool(self.get_parameter("replay_monitor_enabled").value),
                # 起步宽限：动作刚开始时反馈与期望天然有偏差，此期间不判定。
                start_grace_sec=float(self.get_parameter("replay_monitor_start_grace_sec").value),
                # 违规宽限：超限需持续这么久才真的刹停，避免单个抖动采样触发急停。
                violation_grace_sec=float(self.get_parameter("replay_monitor_violation_grace_sec").value),
                max_tracking_error_rad=float(self.get_parameter("max_tracking_error_rad").value),
                max_live_velocity_rad_s=float(self.get_parameter("max_live_velocity_rad_s").value),
            ),
        )
        if not decision.should_stop:
            return
        # 先走控制器急停通道（更直接），再取消动作目标，最后对外发布安全停止状态。
        self._request_controller_trajectory_stop(timeout_sec=0.2)
        with suppress(Exception):
            goal_handle.cancel_goal_async()
        self._publish_status("replay", decision.status)

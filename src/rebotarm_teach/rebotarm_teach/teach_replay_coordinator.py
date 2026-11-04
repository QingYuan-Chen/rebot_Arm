from __future__ import annotations

# 示教回放门控与结果负载构造（纯逻辑，无 ROS 依赖）。
#
# 职责
# ----
# 把“能否真的回放一段示教”的判定条件集中到一处：调用方（回放工作流与上层界面）
# 只负责采集记录信息、预处理结果和各类预检结论，由本模块决定放行或阻断，并按统一
# 结构返回结果负载，避免同一套阈值在多处各写一遍而产生分歧。
#
# 模块边界
# --------
# 本模块不订阅话题、不调用服务、不读参数、不发运动指令，输入输出都是普通字典，
# 因此可以脱离运行时单独测试。真正的阈值来源（参数服务器）由调用方装配成
# TeachReplayLimits，避免此处再依赖参数读取。
#
# 安全语义（三道门，按顺序）
# --------------------------
# 1. 起点门：记录文件的起点偏差档位必须是 direct / align / moveit_align，
#    reject、missing 等档位一律拒绝。
# 2. 质量门：预处理后轨迹的风险等级为 red，或跳变/加速度/加加速度超限即拒绝；
#    yellow 风险还要求回放速度不超过 yellow_max_speed。
# 3. dry-run 门：真实回放必须先有一次“同一记录、同一预处理结果、同一组设置”的
#    dry-run 通过记录，否则拒绝；dry-run 本身额外要求起点对齐与碰撞预检没有失败。

from dataclasses import dataclass
from typing import Any, Callable

from .teach_recording import estimate_teach_replay, validate_teach_replay_execute_request


@dataclass(frozen=True)
class TeachReplayLimits:
    """回放放行阈值（由上层参数装配，均为冻结字段，运行中不可变）。

    max_prepared_jump_rad：预处理后轨迹允许的单步最大关节跳变，单位 rad，超限阻断。
    max_replay_acceleration_rad_s2：重定时轨迹允许的最大关节加速度，单位 rad/s^2。
    max_replay_jerk_rad_s3：重定时轨迹允许的最大关节加加速度，单位 rad/s^3。
    """

    max_prepared_jump_rad: float
    max_replay_acceleration_rad_s2: float
    max_replay_jerk_rad_s3: float


class TeachReplayCoordinator:
    """集中安置示教回放的门控判定与 API 结果负载构造。

    无状态、可复用：既不保存上一次判定结果，也不访问硬件；dry-run 的“凭证”由
    调用方自己保存并在执行时回传（见 ``evaluate_execute_request``），因此本类
    实例可以长期驻留、线程安全地只读使用。
    """

    def evaluate_execute_request(
        self,
        *,
        info_payload: dict,
        settings: dict[str, float | int],
        prepared_quality: dict,
        dry_run_token: dict,
        limits: TeachReplayLimits,
        yellow_max_speed: float,
        align_duration_auto: bool = False,
    ):
        """校验一次真实回放请求，返回决策对象（含 accepted/state/message）。

        关键点是 dry-run 凭证必须“三同”才算有效：同一记录路径、同一预处理后风险
        等级、同一组回放设置。任何一项不一致都说明条件已经变化（例如换了文件或改了
        速度），必须重新 dry-run，防止用旧的通过结果放行新的轨迹。

        参数
        ----
        info_payload：记录文件信息（``start_band`` 决定起点门，``quality`` 为原始质量）。
        settings：本次回放设置（含 ``replay_speed`` 等），与凭证中的设置做整体比较。
        prepared_quality：预处理后轨迹的质量字典，其 ``risk_level`` 参与质量门。
        dry_run_token：上一次 dry-run 的结果负载；为空字典表示没有通过记录。
        limits：放行阈值集合。
        yellow_max_speed：yellow 风险下允许的最大回放速度倍率，取值 <= 1。
        """
        quality = info_payload.get("quality") if isinstance(info_payload.get("quality"), dict) else {}
        # 自动时长来自当前反馈，不是用户设置。执行仍重新规划并检查完整曲线；
        # 不能因正常停稳后的反馈变化要求用户反复 dry-run。手动时长及其他设置仍严格绑定。
        def bound_settings(values):
            return {k:v for k,v in values.items() if not (align_duration_auto and k == 'align_duration')}
        # 凭证有效性 = 已通过 + 记录路径一致 + 预处理风险等级一致 + 设置完全一致。
        dry_run_passed = (
            bool(dry_run_token.get("accepted"))
            and str(dry_run_token.get("record_path", "")) == str(info_payload.get("path", ""))
            and str(dry_run_token.get("prepared_risk_level", "")) == str(prepared_quality.get("risk_level", ""))
            and bound_settings(dry_run_token.get("settings") or {}) == bound_settings(settings)
        )
        return validate_teach_replay_execute_request(
            str(info_payload.get("start_band", "")),
            dry_run_passed=dry_run_passed,
            risk_level=str(quality.get("risk_level", "unknown")),
            prepared_risk_level=str(prepared_quality.get("risk_level", "")) or None,
            prepared_max_jump_rad=prepared_quality.get("max_jump_rad"),
            max_prepared_jump_rad=float(limits.max_prepared_jump_rad),
            retimed_max_acceleration_rad_s2=prepared_quality.get("max_acceleration_rad_s2"),
            max_replay_acceleration_rad_s2=float(limits.max_replay_acceleration_rad_s2),
            retimed_max_jerk_rad_s3=prepared_quality.get("max_jerk_rad_s3"),
            max_replay_jerk_rad_s3=float(limits.max_replay_jerk_rad_s3),
            replay_speed=float(settings["replay_speed"]),
            yellow_max_speed=float(yellow_max_speed),
        )

    def build_dry_run_result(
        self,
        *,
        info_payload: dict,
        settings: dict[str, float | int],
        decision: Any,
        prepared_payload: dict,
        prepared_record_path: str,
        moveit_align: dict,
        collision_precheck: dict,
        trajectory_points: int,
        limits: TeachReplayLimits,
        target_runtime: str,
        compact_payload: Callable[[dict], dict] | None = None,
    ) -> dict:
        """构造 dry-run 结果负载。

        除记录与预处理信息外，dry-run 还要叠加一道额外的预检门：起点对齐处于
        failed/unavailable，或碰撞预检处于 collision/unknown 时，即使基础判定通过也
        改为阻断。原因是 dry-run 的意义就在于提前暴露这两类真实回放才有的风险；
        预检本身没跑通（unknown）不能当作通过处理。

        ``target_runtime`` 只是标注本次判定面向的运行时（真实硬件或仿真），
        不改变判定逻辑。
        """
        # 预检状态统一转小写比较，容忍各来源的大小写差异。
        gate_blocked = (
            str(moveit_align.get("state", "")).lower() in ("failed", "unavailable", "unknown")
            or str(collision_precheck.get("state", "")).lower() != 'pass'
        )
        return self._build_replay_result(
            info_payload=info_payload,
            settings=settings,
            accepted=bool(decision.accepted) and not gate_blocked,
            state="blocked" if decision.accepted and gate_blocked else decision.state,
            message=(
                (prepared_payload.get('trajectory_preparation',{}).get('message')
                 if prepared_payload.get('trajectory_preparation',{}).get('state') == 'failed'
                 else f"{decision.message}; MoveIt/collision precheck blocked real replay")
                if decision.accepted and gate_blocked
                else decision.message
            ),
            prepared_payload=prepared_payload,
            prepared_record_path=prepared_record_path,
            moveit_align=moveit_align,
            collision_precheck=collision_precheck,
            trajectory_points=trajectory_points,
            limits=limits,
            target_runtime=target_runtime,
            dry_run=True,
            compact_payload=compact_payload,
        )

    def build_execute_result(
        self,
        *,
        info_payload: dict,
        settings: dict[str, float | int],
        decision: Any,
        prepared_payload: dict,
        prepared_record_path: str,
        moveit_align: dict,
        collision_precheck: dict,
        trajectory_points: int,
        limits: TeachReplayLimits,
        target_runtime: str,
        compact_payload: Callable[[dict], dict] | None = None,
    ) -> dict:
        """构造真实回放（execute）结果负载。

        与 dry-run 的区别：这里只做“转述”，不再叠加预检门，因为调用方在调用前已经
        用预检结论修正过 ``decision``（起点的对齐与碰撞检查各自决定是否放行），
        重复判定会让两处结论互相掩盖。``accepted`` 与 ``state`` 直接取 ``decision``，
        保证对外返回的状态与真实动作完全一致。
        """
        return self._build_replay_result(
            info_payload=info_payload,
            settings=settings,
            accepted=bool(decision.accepted),
            state=str(decision.state),
            message=str(decision.message),
            prepared_payload=prepared_payload,
            prepared_record_path=prepared_record_path,
            moveit_align=moveit_align,
            collision_precheck=collision_precheck,
            trajectory_points=trajectory_points,
            limits=limits,
            target_runtime=target_runtime,
            dry_run=False,
            compact_payload=compact_payload,
        )

    def _build_replay_result(
        self,
        *,
        info_payload: dict,
        settings: dict[str, float | int],
        accepted: bool,
        state: str,
        message: str,
        prepared_payload: dict,
        prepared_record_path: str,
        moveit_align: dict,
        collision_precheck: dict,
        trajectory_points: int,
        limits: TeachReplayLimits,
        target_runtime: str,
        dry_run: bool,
        compact_payload: Callable[[dict], dict] | None,
    ) -> dict:
        """拼装 dry-run 与 execute 共用的结果字典（两者的差异只在于传入的判定与 ``dry_run``）。

        结果同时携带原始记录质量与预处理后质量：``risk_level`` 是原始档位，
        ``prepared_risk_level`` 是预处理后档位，``effective_risk_level`` 取预处理后
        档位、缺失时回退原始档位——门控实际依据的是后者，所以对外以它为“生效”档位。
        阈值字段（``max_*``）一并回传，便于界面直接展示“实测值 vs 限值”。

        ``compact_payload`` 非空时用于裁剪结果（例如截断过长的异常事件列表），
        裁剪函数由调用方提供，本模块不关心裁剪策略。
        """
        quality = info_payload.get("quality") if isinstance(info_payload.get("quality"), dict) else {}
        # 预处理质量以 after_quality（重定时后的结果）为准。
        prepared_quality = (
            prepared_payload.get("after_quality")
            if isinstance(prepared_payload.get("after_quality"), dict)
            else {}
        )
        estimate = estimate_teach_replay(
            samples=int(info_payload.get("samples") or 0),
            record_duration_sec=float(info_payload.get("duration_sec") or 0.0),
            start_band=str(info_payload.get("start_band", "")),
            replay_speed=float(settings["replay_speed"]),
            align_duration=float(settings["align_duration"]),
            align_steps=int(settings["align_steps"]),
            final_hold_sec=float(settings["final_hold_sec"]),
        )
        result = {
            "accepted": bool(accepted),
            "state": str(state),
            "message": str(message),
            "record_path": str(info_payload.get("path", "")),
            "prepared_record_path": prepared_record_path,
            "start_band": str(info_payload.get("start_band", "")),
            "max_error": info_payload.get("max_error"),
            "worst_joint": str(info_payload.get("worst_joint", "")),
            "samples": int(info_payload.get("samples") or 0),
            # 轨迹点数按优先级取值：本次实际点数 -> 预处理样本数 -> 估算值，
            # 这样被阻断（没有真实轨迹）时也能给出可预期的工作量。
            "trajectory_points": int(trajectory_points or prepared_payload.get("prepared_samples") or estimate["trajectory_points"]),
            "estimated_duration_sec": float(prepared_payload.get('final_curve',{}).get(
                'duration',estimate["estimated_duration_sec"])),
            "settings": settings,
            "quality": quality,
            "risk_level": str(quality.get("risk_level", "unknown")),
            "prepared_risk_level": str(prepared_quality.get("risk_level", "unknown")),
            "effective_risk_level": str(prepared_quality.get("risk_level", quality.get("risk_level", "unknown"))),
            "prepared_max_jump_rad": prepared_quality.get("max_jump_rad"),
            "retimed_max_acceleration_rad_s2": prepared_quality.get("max_acceleration_rad_s2"),
            "retimed_max_jerk_rad_s3": prepared_quality.get("max_jerk_rad_s3"),
            "max_prepared_jump_rad": float(limits.max_prepared_jump_rad),
            "max_replay_acceleration_rad_s2": float(limits.max_replay_acceleration_rad_s2),
            "max_replay_jerk_rad_s3": float(limits.max_replay_jerk_rad_s3),
            "prepared_replay": prepared_payload,
            "moveit_start_align": moveit_align,
            "collision_precheck": collision_precheck,
            "trajectory_preparation": prepared_payload.get('trajectory_preparation',{}),
            "target_runtime": target_runtime,
            "dry_run": bool(dry_run),
        }
        return compact_payload(result) if compact_payload is not None else result

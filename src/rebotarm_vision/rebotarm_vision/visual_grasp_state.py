"""视觉抓取运行时状态与诊断快照（不依赖 ROS 节点）。"""

from __future__ import annotations

from dataclasses import dataclass, field
from threading import RLock
from typing import Any


@dataclass(frozen=True)
class PoseSnapshot:
    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]


@dataclass(frozen=True)
class PlanSnapshot:
    valid: bool
    source: str
    reason: str
    frame_id: str
    stamp_ns: int
    confidence: float
    class_name: str
    jaw_width_m: float
    pregrasp: PoseSnapshot
    grasp: PoseSnapshot


@dataclass(frozen=True)
class FailureSnapshot:
    stage: str
    message: str
    plan: PlanSnapshot | None


@dataclass
class VisualGraspState:
    """执行器跨回调状态；消息对象只由节点线程安全地替换。"""

    latest_plan: Any = None
    latest_candidates: Any = None
    last_plan_rejection: str = ""
    plan_revision: int = 0
    last_gripper_reached_position: float | None = None
    last_grasp_contact_detected: bool = False
    last_grasp_closure_distance_m: float = 0.0
    retry_retreat_stage: Any = None
    run_counter: int = 0
    current_run_id: int = 0
    current_attempt_index: int = 0
    current_candidate_index: int = -1
    current_attempt_plan: Any = None
    preview_start_joint_state: Any = None
    preview_trajectories: list[Any] | None = None
    preview_gripper_events: list[tuple[int, float]] = field(default_factory=list)
    running: bool = False
    # stop 将 running 置为假后，execute 服务回调仍可能处于收尾阶段。
    # 恢复执行前必须等待该回调释放下游动作和服务调用，
    # 然后才能启动下一条轨迹。
    run_callback_active: bool = False
    phase: str = "IDLE"
    active_request_id: int = 0
    stop_requested: bool = False
    plans_blocked: bool = False
    abort_reason: str = ""
    lock: RLock = field(default_factory=RLock, repr=False, compare=False)

    def __post_init__(self) -> None:
        if self.preview_trajectories is None:
            self.preview_trajectories = []

    def begin_run(self) -> bool:
        with self.lock:
            if self.running or self.phase in ("RUNNING", "STOP_REQUESTED", "ABORTING") or self.plans_blocked:
                return False
            self.run_counter += 1
            self.current_run_id = self.run_counter
            self.active_request_id = self.run_counter
            self.running = True
            self.run_callback_active = True
            self.phase = "RUNNING"
            self.stop_requested = False
            self.abort_reason = ""
            return True

    def request_stop(self, reason: str = "stop requested") -> None:
        with self.lock:
            self.stop_requested = True
            self.plans_blocked = True
            self.abort_reason = reason
            self.phase = "STOP_REQUESTED"
            self.running = False

    def mark_aborting(self, reason: str) -> None:
        with self.lock:
            self.abort_reason = reason
            self.phase = "ABORTING"
            self.stop_requested = True
            self.plans_blocked = True
            self.running = False

    def finish_run(self, *, preserve_abort: bool = False) -> None:
        with self.lock:
            self.running = False
            self.run_callback_active = False
            if self.plans_blocked or (preserve_abort and self.phase in ("STOP_REQUESTED", "ABORTING")):
                return
            self.phase = "IDLE"
            self.active_request_id = 0

    def confirm_stop(self) -> bool:
        """内部接口约束：调用方必须已取得下游停止完成的证据。"""
        with self.lock:
            if self.run_callback_active or self.running or not self.plans_blocked:
                return False
            self.latest_plan = None
            self.latest_candidates = None
            self.last_plan_rejection = "waiting for a new plan after confirmed stop"
            self.plans_blocked = False
            self.stop_requested = False
            self.phase = "IDLE"
            self.abort_reason = ""
            self.active_request_id = 0
            return True

    def is_running(self) -> bool:
        with self.lock:
            return self.running


def _pose_snapshot(pose) -> PoseSnapshot:
    return PoseSnapshot(
        position=(float(pose.position.x), float(pose.position.y), float(pose.position.z)),
        orientation=(
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        ),
    )


def plan_snapshot(plan) -> PlanSnapshot:
    """把计划消息提取成可序列化、可独立测试的诊断快照。"""

    stamp = getattr(plan.header, "stamp", None)
    stamp_ns = int(getattr(stamp, "sec", 0)) * 1_000_000_000 + int(
        getattr(stamp, "nanosec", 0)
    )
    candidate = getattr(plan, "candidate", None)
    return PlanSnapshot(
        valid=bool(getattr(plan, "valid", False)),
        source=str(getattr(plan, "source", "")),
        reason=str(getattr(plan, "reason", "") or ""),
        frame_id=str(getattr(plan.header, "frame_id", "")),
        stamp_ns=stamp_ns,
        confidence=float(getattr(candidate, "confidence", 0.0) or 0.0),
        class_name=str(getattr(candidate, "class_name", "") or ""),
        jaw_width_m=float(
            getattr(plan, "jaw_width", 0.0)
            or getattr(candidate, "jaw_width", 0.0)
            or 0.0
        ),
        pregrasp=_pose_snapshot(plan.pregrasp_pose),
        grasp=_pose_snapshot(plan.grasp_pose),
    )


def _state_for(node) -> VisualGraspState:
    """取得集中状态，并兼容未调用构造函数的旧私有方法测试替身。"""
    state = getattr(node, "_state", None)
    if state is not None:
        return state
    state = VisualGraspState()
    for state_field in (
        "latest_plan", "last_plan_rejection", "latest_candidates", "plan_revision",
        "last_gripper_reached_position", "last_grasp_contact_detected",
        "last_grasp_closure_distance_m", "retry_retreat_stage", "run_counter",
        "current_run_id", "current_attempt_index", "current_candidate_index",
        "current_attempt_plan", "preview_start_joint_state", "preview_trajectories", "running",
        "run_callback_active", "phase", "active_request_id", "stop_requested", "abort_reason",
        "plans_blocked",
    ):
        legacy_name = "_" + state_field
        if hasattr(node, legacy_name):
            setattr(state, state_field, getattr(node, legacy_name))
    node._state = state
    return state

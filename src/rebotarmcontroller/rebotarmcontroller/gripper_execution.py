"""夹爪命令执行状态及周期推进。

``GripperExecution`` 独立持有夹爪目标、反馈镜像、释放状态和锁；周期线程
调用 ``_gripper_tick``，经注入的窄回调获取已验证反馈及发送命令。调用方
持有反馈锁时才可进入夹爪锁；等待者只登记释放，实际写入由周期线程执行。
中性命令成功前位置命令不能成功，取消后不能发送旧的带力矩命令。
"""
import logging
import threading
import time
import numpy as np
from .gripper_control import (GripperControl, _G_ARRIVE_TOL, _G_TAU_MAX,
    _G_KP_MOVE, _G_KD_MOVE, _G_GRASP_CLOSE_KP, _G_GRASP_CLOSE_KD,
    _G_GRASP_HOLD_KP, _G_GRASP_HOLD_KD, _G_MAX_DIST_M, _G_ANGLE_OPEN)

_LOG = logging.getLogger('rebotarmcontroller.hardware_manager')

class GripperExecution:
    """一套夹爪执行状态；设备和反馈依赖由调用方注入。"""
    def __init__(self, *, motor, verified, feedback_failure, record_feedback_error,
                 validate, feedback_age, feedback_updated, speed, send, clock=None):
        self._motor = motor
        self._verified = verified
        self._gripper_feedback_failure_reason_locked = feedback_failure
        self._record_gripper_feedback_error = record_feedback_error
        self._validated_gripper_feedback_values = validate
        self._gripper_feedback_age_sec = feedback_age
        self._feedback_updated = feedback_updated
        self._speed = speed
        self._gripper_safe_mit = send
        self._clock = clock if clock is not None else lambda: time.monotonic()
        self._gripper_lock = threading.RLock()
        self._gripper_target_angle = self._gripper_goal_angle = 0.0
        self._gripper_target_effort = self._gripper_close_force = self._gripper_hold_force = 0.4
        self._gripper_hold_angle = 0.0
        self._gripper_hold_deadline = self._gripper_hold_release_reason = None
        self._gripper_mode, self._gripper_active = 'idle', False
        self._gripper_pos = self._gripper_vel = self._gripper_torque = 0.0
        self._gripper_command_error = None
        self._gripper_position_result = 'idle'
        self._gripper_neutral_pending = None
        self._gripper_target_timeout_sec = 0.0
        self._gripper_target_deadline_monotonic = None
        self._gripper_last_tick_monotonic = None
        self._grasp_request = None
        self._generation = 0

    def _position_m_locked(self):
        return float(np.clip(self._gripper_pos / _G_ANGLE_OPEN * _G_MAX_DIST_M,
                             0.0, _G_MAX_DIST_M))

    def _finish_grasp_locked(self, request, result):
        if request is not None and request['result'] is None:
            request['result'] = result
            request['event'].set()

    def supersede_locked(self):
        """使旧抓取等待者失效，且不让旧中性命令确认新位置目标。"""
        self._generation += 1
        request = self._grasp_request
        if request is not None:
            self._finish_grasp_locked(request, (False, False, 0.0,
                self._position_m_locked(), request['hold_force'], 'grasp superseded'))
        self._grasp_request = None
        if self._gripper_neutral_pending is not None:
            angle, reason, _success = self._gripper_neutral_pending
            self._gripper_neutral_pending = (angle, reason, False)

    def start_grasp(self, *, close_force, hold_force, timeout, min_time,
                    velocity_limit, min_closure, hold_timeout, start_position, now):
        """接收抓取参数，返回专属事件记录；堵转只由周期推进判定。"""
        with self._gripper_lock:
            self.supersede_locked()
            request = dict(event=threading.Event(), result=None, started=now,
                deadline=now + timeout, min_time=min_time, velocity_limit=velocity_limit,
                min_closure=min_closure, start_position=start_position,
                hold_timeout=hold_timeout, hold_force=hold_force)
            self._grasp_request = request
            self._gripper_close_force = close_force
            self._gripper_hold_force = hold_force
            self._gripper_hold_deadline = None
            self._gripper_hold_release_reason = None
            self._gripper_command_error = None
            self._gripper_mode = 'grasp_closing'
            self._gripper_active = True
            return request

    def expire_grasp(self, request):
        """循环停止或等待超时时撤销本次请求；旧等待者不能撤销新请求。"""
        with self._gripper_lock:
            if self._grasp_request is request and request['result'] is None:
                self._release_grasp_hold_locked('close timeout before stall')
            return request['result']

    @property
    def _gripper_mot(self):
        return self._motor()

    @property
    def _gripper_feedback_updated_monotonic(self):
        return self._feedback_updated()

    @property
    def _gripper_position_max_speed_rad_s(self):
        return self._speed()

    def _fail_active_gripper_command_locked(self, reason: str) -> None:
        """把当前夹爪命令标记为失败并排队一次中性释放（调用方需持锁）。"""
        message = str(reason)
        request = self._grasp_request
        if self._gripper_mode == 'grasp_closing' and request is not None:
            self._finish_grasp_locked(request, (False, False, 0.0,
                self._position_m_locked(), request['hold_force'], message))
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
            # 接替目标必须等旧命令完成中性化；释放失败时不能继续执行新目标。
            if self._gripper_active:
                request = self._grasp_request
                if self._gripper_mode == 'grasp_closing' and request is not None:
                    self._finish_grasp_locked(request, (False, False, 0.0,
                        self._position_m_locked(), request['hold_force'],
                        self._gripper_command_error))
                self._gripper_active = False
                self._gripper_mode = 'idle'
                self._gripper_target_deadline_monotonic = None
            _LOG.error("%s", self._gripper_command_error)
            return False
        if marks_success:
            self._gripper_position_result = "succeeded"
            self._gripper_command_error = None
        return True

    def cancel_gripper_position_command(self, reason: str = "position command canceled",
                                        *, expected_generation=None) -> bool:
        """取消进行中的位置命令；仅在确实处于 position 模式时生效。"""
        with self._gripper_lock:
            if expected_generation is not None and expected_generation != self._generation:
                return False
            if not self._gripper_active or self._gripper_mode != "position":
                return False
            self._fail_active_gripper_command_locked(reason)
            return True

    def _release_gripper_position_target(
        self,
        expected_goal: float,
        *,
        require_arrived: bool = True,
        expected_generation=None,
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
                expected_generation=expected_generation,
            )

    def _release_gripper_position_target_locked(
        self,
        expected_goal: float,
        *,
        require_arrived: bool = True,
        expected_generation=None,
    ) -> bool:
        """``_release_gripper_position_target`` 的本体；调用方必须持有锁。

        拆成两部分是为了让已经持有 ``_gripper_lock`` 的调用方也能复用同一份
        所有权检查与中性化逻辑。
        """
        if expected_generation is not None and expected_generation != self._generation:
            return False
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
        return self._clock() >= deadline

    def _release_grasp_hold_locked(self, reason: str) -> None:
        """对抓取闭合/保持做中性化并回到空闲。调用方需持锁。

        上游会让 ``grasp_holding`` 一直带电到下一条命令，其闭合超时路径也只是切
        到 idle 而从不中性化最后一条力矩命令。两条路径现在都汇到这里。
        """
        request = self._grasp_request
        if self._gripper_mode == 'grasp_closing' and request is not None:
            detail = ('grasp close timeout before stall detected'
                      if reason == 'close timeout before stall' else reason)
            self._finish_grasp_locked(request, (False, False, 0.0,
                self._position_m_locked(), request['hold_force'], detail))
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
            generation = self._generation
        try:
            st = self._verified()
        except Exception as exc:
            self._record_gripper_feedback_error(str(exc))
            with self._gripper_lock:
                if self._gripper_active and generation == self._generation:
                    self._fail_active_gripper_command_locked(
                        f"gripper feedback read failed: {exc}"
                    )
            return

        # 到位判定与命令下发必须原子。先取状态、放开锁、再发送，会让已经通过
        # 到位判定的 tick 在 wait_gripper_target 释放之后再发出一条带力矩的命令
        # ——真机上表现为服务返回后又多闭合约 1.2-1.33 mm。向零位闭合是最暴露的
        # 方向，因为 abs(target) < 1e-6 时 effort 会变成前馈项。
        with self._gripper_lock:
            # 读取反馈时锁已释放；若目标被接替，本拍不能用旧反馈推进新目标。
            if generation != self._generation:
                return
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
                if status != 1:
                    self._fail_active_gripper_command_locked(
                        f"gripper status_code={status}, expected 1 during command"
                    )
                    return
                command = None

            if command is None and self._gripper_active and mode == "position":
                goal = self._gripper_goal_angle
                deadline = self._gripper_target_deadline_monotonic
                if deadline is not None and self._clock() >= deadline:
                    self._fail_active_gripper_command_locked(
                        f"gripper target timeout: goal={goal:.6f}rad feedback={self._gripper_pos:.6f}rad"
                    )
                elif abs(self._gripper_pos - goal) < _G_ARRIVE_TOL:
                    # 无同步等待者时到达，或在等待者返回之后到达。
                    self._release_gripper_position_target_locked(goal)
                    command = None
                else:
                    now = self._clock()
                    previous_tick = self._gripper_last_tick_monotonic
                    elapsed = 0.0 if previous_tick is None else max(now - previous_tick, 0.0)
                    # 本拍允许的最大角度增量 = 速度上限 * 实际经过时间。
                    target = GripperControl.ramp(
                        self._gripper_target_angle, goal,
                        self._gripper_position_max_speed_rad_s, elapsed,
                    )
                    self._gripper_target_angle = target
                    self._gripper_last_tick_monotonic = now
                    effort = self._gripper_target_effort
                    # ``effort`` 约束整段移动而不只是闭合终点；若不显式传
                    # tau_limit，命令力矩会回落到 _G_TAU_MAX。
                    tau_ff = effort if abs(target) < 1e-6 else 0.0
                    command = (target, 0.0, _G_KP_MOVE, _G_KD_MOVE, tau_ff, effort)
            elif command is None and self._gripper_active and mode == "grasp_closing":
                request = self._grasp_request
                now = self._clock()
                position_m = self._position_m_locked()
                if request is not None and now >= request['deadline']:
                    self._release_grasp_hold_locked('close timeout before stall')
                elif (request is not None and now - request['started'] >= request['min_time']
                      and max(request['start_position'] - position_m, 0.0) >= request['min_closure']
                      and abs(self._gripper_vel) <= request['velocity_limit']):
                    # 冻结堵转点；保持命令发送成功后才通知等待者。
                    self._gripper_hold_angle = self._gripper_pos
                    self._gripper_hold_deadline = now + request['hold_timeout']
                    self._gripper_mode = 'grasp_holding'
                    command = (self._gripper_hold_angle, 0.0, _G_GRASP_HOLD_KP,
                               _G_GRASP_HOLD_KD, self._gripper_hold_force, _G_TAU_MAX)
                else:
                    # 闭合仅靠阻尼和前馈力矩推进，堵转不等于测得接触力。
                    command = (0.0, 0.0, _G_GRASP_CLOSE_KP, _G_GRASP_CLOSE_KD,
                               self._gripper_close_force, _G_TAU_MAX)
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
                if mode == 'grasp_closing' and self._gripper_mode == 'grasp_holding':
                    request = self._grasp_request
                    if request is not None:
                        position_m = self._position_m_locked()
                        self._finish_grasp_locked(request, (True, True, position_m,
                            position_m, request['hold_force'],
                            f"closing stalled; holding (bounded to {request['hold_timeout']:g} s)"))
            except Exception as exc:
                if mode == 'grasp_closing' and self._grasp_request is not None:
                    request = self._grasp_request
                    self._finish_grasp_locked(request, (False, False, 0.0,
                        self._position_m_locked(), request['hold_force'], str(exc)))
                self._fail_active_gripper_command_locked(str(exc))

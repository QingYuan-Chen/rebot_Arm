"""夹爪命令执行状态及周期推进.

职责与位置
    夹爪执行状态所有者: 目标代次, 斜坡, 堵转, 保持期限及待发中性释放.

对外接口
    accept_position/start_grasp 接纳已校验参数, snapshot 查询本代结果;
    _gripper_tick 仅由设备周期调用, 取消只作用于匹配的目标.

安全与并发约束
    读取反馈时释放执行锁, 回来后复核代次; 反馈事务锁序为反馈 -> 夹爪.
    到位后中性发送成功才报告位置成功; 堵转仅为接触推断, 抓取保持必须有期限.

文件结构(按定义顺序)
    GripperExecution: 一套夹爪执行状态; 设备和反馈依赖由调用方注入.
"""

import logging
import threading
import time
import numpy as np
from .gripper_control import (GripperControl, _G_ARRIVE_TOL, _G_TAU_MAX,
    _G_KP_MOVE, _G_KD_MOVE, _G_GRASP_CLOSE_KP, _G_GRASP_CLOSE_KD,
    _G_GRASP_HOLD_KP, _G_GRASP_HOLD_KD, _G_MAX_DIST_M, _G_ANGLE_OPEN)
from .controller_values import GripperTarget, GripperResult

_LOG = logging.getLogger('rebotarmcontroller.hardware_manager')


class GripperExecution:
    """一套夹爪执行状态; 设备和反馈依赖由调用方注入."""

    def __init__(self, *, motor, verified, feedback_failure, record_feedback_error,
                 validate, feedback_age, feedback_updated, speed, send, clock=None):
        """注入反馈与发送回调, 初始化独立执行锁, 目标代次, 抓取状态和中性释放队列."""
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

    def accept_position(self, target: GripperTarget, now):
        """单次接纳已校验的数值, 代次, 目标和超时在同一锁内更新."""
        with self._gripper_lock:
            self.supersede_locked()
            self._gripper_target_angle = target.start_angle
            self._gripper_goal_angle = target.goal_angle
            self._gripper_target_effort = target.effort
            self._gripper_mode, self._gripper_active = "position", True
            self._gripper_position_result = "active"
            self._gripper_command_error = None
            self._gripper_last_tick_monotonic = now
            self._gripper_target_timeout_sec = target.timeout
            self._gripper_target_deadline_monotonic = now + target.timeout
            return self._generation

    def snapshot(self, generation):
        """查询旧代次只返回 superseded, 不能读成接替目标的成功."""
        with self._gripper_lock:
            if generation != self._generation:
                return GripperResult(generation, 'superseded', 'gripper target superseded')
            state = ('releasing' if self._gripper_neutral_pending is not None
                     else self._gripper_position_result)
            return GripperResult(generation, state, self._gripper_command_error)

    def _position_m_locked(self):
        """在执行锁内把原始电机角度折算为标称开口 m, 调用前已完成坐标校验."""
        return float(np.clip(self._gripper_pos / _G_ANGLE_OPEN * _G_MAX_DIST_M,
                             0.0, _G_MAX_DIST_M))

    def _finish_grasp_locked(self, request, result):
        """只完成一次本请求结果并唤醒专属等待者; 调用方持执行锁."""
        if request is not None and request['result'] is None:
            request['result'] = result
            request['event'].set()

    def supersede_locked(self):
        """使旧抓取等待者失效, 且不让旧中性命令确认新位置目标."""
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
        """接收抓取参数, 返回专属事件记录; 堵转只由周期推进判定."""
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
        """循环停止或等待超时时撤销本次请求; 旧等待者不能撤销新请求."""
        with self._gripper_lock:
            if self._grasp_request is request and request['result'] is None:
                self._release_grasp_hold_locked('close timeout before stall')
            return request['result']

    @property
    def _gripper_mot(self):
        """通过装配回调读取当前设备引用, 不在执行器中保存第二份绑定."""
        return self._motor()

    @property
    def _gripper_feedback_updated_monotonic(self):
        """读取反馈所有者保存的单调观测时刻, 不生成新时间."""
        return self._feedback_updated()

    @property
    def _gripper_position_max_speed_rad_s(self):
        """读取配置斜坡速度上限 rad/s, 按实际周期生成目标增量."""
        return self._speed()

    def _fail_active_gripper_command_locked(self, reason: str) -> None:
        """把当前夹爪命令标记为失败并排队一次中性释放(调用方需持锁)."""
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
        """排队一条中性 MIT 命令(零刚度/阻尼/前馈), 由硬件循环在下一拍发出.

        只排队不直接发送, 保证总线上只有一个写入者. ``marks_success`` 决定该
        中性命令发送成功后是否把本次位置命令标记为成功.
        """
        self._gripper_neutral_pending = (
            float(self._gripper_pos),
            str(reason),
            bool(marks_success),
        )

    def _emit_pending_gripper_neutral_locked(self) -> bool:
        """发送已排队的中性命令; 返回是否成功(调用方需持 ``_gripper_lock``)."""
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
            # 接替目标必须等旧命令完成中性化; 释放失败时不能继续执行新目标.
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
        # 进入成功必须晚于中性命令发送; 仅几何到位还不能释放 Action 等待者.
        if marks_success:
            self._gripper_position_result = "succeeded"
            self._gripper_command_error = None
        return True

    def cancel_gripper_position_command(self, reason: str = "position command canceled",
                                        *, expected_generation=None) -> bool:
        """取消进行中的位置命令; 仅在确实处于 position 模式时生效."""
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
        """结束一次常规位置移动, 并让夹爪回到空闲.

        上游 ``wait_gripper_target`` 只返回成功而不改 ``_gripper_active``/
        ``_gripper_mode``, 于是统一硬件循环在服务已经应答之后仍在继续下发 MIT
        位置命令. 这里改为原子地切到空闲, 并在确认成功之前为唯一的硬件循环写入者
        排队一条中性 MIT 命令(零刚度, 零阻尼, 零前馈).

        整个过程持有夹爪锁, 避免过期的完成路径覆盖更新的目标. ``grasp_closing``
        与 ``grasp_holding`` 是显式请求的力操作, 绝不在此释放.
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
        """``_release_gripper_position_target`` 的本体; 调用方必须持有锁.

        拆成两部分是为了让已经持有 ``_gripper_lock`` 的调用方也能复用同一份
        所有权检查与中性化逻辑.
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
        # 在排队中性命令之前先清掉所有权. 统一硬件循环先判断 _gripper_active,
        # 因此它一旦为假就不会再产生任何位置命令, 排队的中性命令即是最后一条.
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
        """有界抓取保持是否已到时. 调用方需持锁."""
        deadline = self._gripper_hold_deadline
        if deadline is None:
            return False
        return self._clock() >= deadline

    def _release_grasp_hold_locked(self, reason: str) -> None:
        """对抓取闭合/保持做中性化并回到空闲. 调用方需持锁.

        上游会让 ``grasp_holding`` 一直带电到下一条命令, 其闭合超时路径也只是切
        到 idle 而从不中性化最后一条力矩命令. 两条路径现在都汇到这里.
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
        """释放进行中的抓取保持, 且不下发新的位置命令."""
        with self._gripper_lock:
            if not self._gripper_active:
                return False
            if self._gripper_mode not in ("grasp_closing", "grasp_holding"):
                return False
            self._release_grasp_hold_locked(reason)
        return True

    @property
    def grasp_hold_release_reason(self) -> str | None:
        """在执行锁内读取最近一次抓取保持释放原因."""
        with self._gripper_lock:
            return self._gripper_hold_release_reason

    def _gripper_tick(self) -> None:
        """由唯一设备周期推进一拍夹爪状态, 不创建独立发送线程.

        先处理待发中性命令, 再检查活动目标. 读取反馈时释放执行锁, 重新持锁后复核代次,
        旧反馈不能推进接替目标. 到位判断, 状态更新和本拍发送在同一执行锁内完成,
        防止等待者已收到完成结果后又发出该目标的带力矩命令.

        位置模式使用按实际时间推进的限速斜坡, 抓取模式使用配置的 kp/kd 与前馈.
        位置到位后中性发送成功才报告成功; 抓取堵转只是接触推断, 保持仍受时限约束.
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

        # 反馈已在执行锁外获取; 锁内复核代次, 连续完成到位判定与本拍发送.
        # 完成确认不能与该目标的后续力矩输出交错, 避免等待者返回后继续闭合.
        with self._gripper_lock:
            # 读取反馈时锁已释放; 若目标被接替, 本拍不能用旧反馈推进新目标.
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
                    # 无同步等待者时到达, 或在等待者返回之后到达.
                    self._release_gripper_position_target_locked(goal)
                    command = None
                else:
                    now = self._clock()
                    previous_tick = self._gripper_last_tick_monotonic
                    elapsed = 0.0 if previous_tick is None else max(now - previous_tick, 0.0)
                    # 本拍允许的最大角度增量 = 速度上限 * 实际经过时间.
                    target = GripperControl.ramp(
                        self._gripper_target_angle, goal,
                        self._gripper_position_max_speed_rad_s, elapsed,
                    )
                    self._gripper_target_angle = target
                    self._gripper_last_tick_monotonic = now
                    effort = self._gripper_target_effort
                    # ``effort`` 约束整段移动而不只是闭合终点; 若不显式传
                    # tau_limit, 命令力矩会回落到 _G_TAU_MAX.
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
                    # 冻结堵转点; 保持命令发送成功后才通知等待者.
                    self._gripper_hold_angle = self._gripper_pos
                    self._gripper_hold_deadline = now + request['hold_timeout']
                    self._gripper_mode = 'grasp_holding'
                    command = (self._gripper_hold_angle, 0.0, _G_GRASP_HOLD_KP,
                               _G_GRASP_HOLD_KD, self._gripper_hold_force, _G_TAU_MAX)
                else:
                    # 闭合仅靠阻尼和前馈力矩推进, 堵转不等于测得接触力.
                    command = (0.0, 0.0, _G_GRASP_CLOSE_KP, _G_GRASP_CLOSE_KD,
                               self._gripper_close_force, _G_TAU_MAX)
            elif command is None and self._gripper_active and mode == "grasp_holding":
                if self._grasp_hold_expired_locked():
                    self._release_grasp_hold_locked("hold timeout")
                    command = None
                else:
                    # 保持: 位置目标冻结在堵转角, 位置项提供主夹持力.
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

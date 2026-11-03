"""六轴目标准入, 轨迹任务, 保持与透传交接.

职责与位置
    六轴业务组件拥有轨迹, 透传租约, 保持证据和停止事务; 运行时拥有数值目标.

对外接口
    submit_arm_trajectory 返回独立任务; stop_active_motion 等待确认;
    set_joint_target, send_joint_motor_cmd 和 safe_home 提供按职责组织的命令.

安全与并发约束
    命令由 device_request 入队; 周期发送和停止确认共享目标代次.
    健康失败保持使能; 反馈, 通信或设备错误交保护组件. safe_home 不是现场安全基线证明.

文件结构(按定义顺序)
    ArmController: 六轴任务与停止保持的业务所有者; 数值目标由运行时执行器持有, 端口只交接能力.
"""

from __future__ import annotations
from .component_ports import CyclePort, ArmFeedbackPort, GravityArmPort, ProtectionPort, ArmGripperPort
from .arm_stop import ArmStop, CycleResult
from .arm_trajectory import ArmTrajectory
from .arm_execution import ArmExecution
from .stationary_evidence import StationaryEvidence, POSITION_SPAN
import logging
from .sdk_driver import CommandTransportError
from typing import Optional
from typing import Sequence
from .feedback_acquisition import _JOINT_POSITION_LIMITS_RAD
from .device_requests import device_request
from .conversions import fk_to_pose
import numpy as np
from .command_guard import passthrough_decision
import time
from .controller_parameters import (
    _SAFE_HOME_JOINT_POSITIONS, _SAFE_HOME_TIMEOUT_SEC
)


class ArmController:
    """六轴任务与停止保持的业务所有者; 数值目标由运行时执行器持有, 端口只交接能力."""

    def __init__(self, runtime, lifecycle, settings):
        """创建六轴任务, 透传租约和停止/保持证据, 运行时目标仍由数值执行器拥有."""
        self._backend = runtime
        self._lifecycle = lifecycle
        self.settings = settings
        self._bound = False
        self._arm_trajectory = ArmTrajectory(runtime.execution)
        self._arm_passthrough_until = None
        self._stop = ArmStop()
        self._hold_evidence = StationaryEvidence()
        self._hold_generation = None
        self._hold_confirmed_at = None
        self._stop_wait = None
        self._scene_revision = ''
        self.gripper = None

    def reset_session(self):
        """失能/重连不能复用上一会话的静止证据或停止续体."""
        self._hold_evidence.reset()
        self._hold_generation = None
        self._hold_confirmed_at = None
        self._stop.abort('controller session reset')
        self._backend.execution.reservation = None
        self._backend.execution.last_sent_target = None

    def bind(self, *, cycle: CyclePort, feedback: ArmFeedbackPort, gravity: GravityArmPort,
             protection: ProtectionPort, gripper: ArmGripperPort=None):
        """装配阶段一次注入明确组件接口; 运行后不替换依赖."""
        if self._bound:
            raise RuntimeError("component already bound")
        self.cycle = cycle
        self.feedback = feedback
        self.gravity = gravity
        self.protection = protection
        self.gripper = gripper
        self._bound = True

    def clear_passthrough(self):
        """清除六轴透传租约标记; 本方法不发送保持或切换模式."""
        self._arm_passthrough_until = None

    def expire_passthrough(self, now):
        """租约到期切回当前反馈保持, 恢复统一周期输出."""
        if self._arm_passthrough_until is not None and now >= self._arm_passthrough_until:
            self.clear_passthrough()
            self.ensure_pos_vel_control()
            self.hold_current_position()
            self._lifecycle.set_execution_state('IDLE')

    def abort_task(self, message):
        """生命周期或保护终止任务, 不再等待旧停止确认."""
        self._stop.abort(message)
        task = self._arm_trajectory.task
        if task is not None:
            self._arm_trajectory.finish(task, 'aborted', message)

    def passthrough_active(self):
        """查询是否存在透传租约, 由周期负责到期交接."""
        return self._arm_passthrough_until is not None

    def check_task_generation(self):
        """空闲周期结束过期任务; 停止中的任务交给停止状态机收尾."""
        task = self._arm_trajectory.task
        if (task is not None and task.result.state != 'stopping'
                and task.generation != self._backend.target_generation):
            self._arm_trajectory.finish(task, 'aborted', 'execution canceled or superseded')

    def check_prepared_context(self):
        """固定夹爪变化或保护性释放先停止六轴，不能继续使用已检查的碰撞曲线。"""
        token = self._backend.execution.reservation
        if token is None or self._stop.pending:
            return
        try:
            state = self.gripper.snapshot()
            valid = (state['generation'] == token.gripper_generation
                and abs(state['raw_position']-token.gripper_raw_position) <= POSITION_SPAN)
        except RuntimeError:
            valid = False
        if valid:
            return
        stopping = self.stop_active_motion()
        if isinstance(stopping, CycleResult):
            # 保持已立即接管；确认交回请求运行时，周期不能等待或保护性失能健康机械臂。
            try:
                self._backend.requests.submit(lambda: stopping, resource='device', priority=0)
            except RuntimeError as exc:
                self._lifecycle.errors.append(f'PREPARED_STOP_UNCONFIRMED: {exc}')

    def update_task(self, sample):
        """发送后以新鲜快照推进任务, 超差保持目标仍由六轴所有者设置."""
        hold = self._arm_trajectory.update(sample.positions, sample.observed_at)
        if hold is not None:
            self._backend.set_target(hold)
        task = self._arm_trajectory.task
        if task is not None and task.event.is_set() and self._lifecycle.execution == 'TRAJ_RUNNING':
            self._lifecycle.set_execution_state('IDLE')
            if task.result.state == 'succeeded':
                self._backend.execution.reservation = None
        stable_mode = (self._lifecycle.enabled and self._lifecycle.execution in ('IDLE', 'TRAJ_PREPARING')
                       and all(code == 1 for code in sample.statuses)
                       and self._backend.mode == 'pos_vel' and not self._backend.trajectory_moving
                       and not self.passthrough_active() and not self.gravity.gravity_compensation_active()
                       and (task is None or task.event.is_set()))
        # 保持证据只适用于空闲, 使能, 位置速度输出的同一目标; 其他资源交接不能借用它.
        if not stable_mode:
            self._hold_generation = None
            self._hold_confirmed_at = None
            self._hold_evidence.reset()
        else:
            if self._hold_generation != self._backend.target_generation:
                self._hold_evidence.reset(self._backend.target())
                self._hold_generation = self._backend.target_generation
                self._hold_confirmed_at = None
            reason = self._hold_evidence.observe(sample.positions, sample.velocities, sample.observed_at,
                                                 time.monotonic(), sequences=sample.sequences)
            if reason in ('invalid_or_stale', 'source_regressed', 'source_gap', 'anchor_displacement'):
                self._hold_confirmed_at = None
            elif self._hold_evidence.confirmed():
                self._hold_confirmed_at = sample.observed_at

    @device_request
    def hold_current_position(self) -> np.ndarray:
        """复制当前已验证六轴角度并设置新的数值保持目标, 返回该角度副本.

        不在这里发送或确认停稳; 调用方仍须管理使能, 模式和后续静止证据.
        需要取消与停止确认的流程应调用 stop_active_motion, 避免重设已有保持目标.
        """
        q, _, _ = self.feedback.get_joint_state()
        current = np.array(q, dtype=np.float64, copy=True)
        self._backend.set_target(current)
        return current

    @device_request
    def stop_active_motion(self, task=None) -> None:
        """撤销当前任务并确认保持; 旧任务的取消不能停止接替任务.

        只有保持命令成功发送且后续新帧持续静止 0.2 s 才返回成功. 超时报告
        未确认, 保留使能和保持目标; 这里绝不把普通取消变成自动失能.
        """
        owner = self._arm_trajectory
        active = None if owner is None else owner.task
        if task is not None and (active is not task or task.event.is_set()):
            return
        if self._stop.pending:
            # 加入同一确认, 不递增资源代次, 不重设目标, 也不共享可变的续体.
            poll, _ = self._stop_wait
            return CycleResult(poll)
        with self._lifecycle.lock:
            previous = self._backend.target()
            evidence = self._hold_evidence
            known_hold = self._hold_generation == self._backend.target_generation
            # 孤立速度峰只使新的停止确认延后; 只要上一份已确认保持仍在
            # 150 ms 内且当前位置仍接近原目标, 不因该峰重新锚定量化位置.
            recently_confirmed = (self._hold_confirmed_at is not None
                                  and time.monotonic()-self._hold_confirmed_at <= .15)
            if (self._stop.result.state == 'confirmed'
                    and self._stop.result.generation == self._backend.target_generation):
                evidence, known_hold = self._stop.evidence, True
            preserve = (known_hold and (evidence.confirmed() or recently_confirmed)
                        and evidence.last_stamp is not None
                        and time.monotonic()-evidence.last_stamp <= .15
                        and self._lifecycle.execution == 'IDLE' and self._backend.mode == 'pos_vel'
                        and self._backend.control_loop_active and not self._backend.trajectory_moving
                        and (active is None or active.event.is_set())
                        and not self.passthrough_active() and not self.gravity.gravity_compensation_active()
                        and self.feedback.arm_failure() is None
                        and np.max(np.abs(previous-self.feedback.joint_snapshot().positions)) <= POSITION_SPAN)
            self._backend.requests.invalidate('arm')
            if active is not None:
                owner.stopping(active)
            self._backend.cancel_trajectory()
            self._arm_passthrough_until = None
            if self._lifecycle.enabled:
                self.gravity.stop_gravity_compensation()
                if preserve:
                    current = previous
                else:
                    # 活动轨迹, 重力和透传交接必须重新取当前值, 不能保留远处目标.
                    self.ensure_pos_vel_control()
                    current = self.hold_current_position()
                # 续体捕获本次状态机; 旧会话清理不得终止后继停止事务.
                stop = ArmStop()
                self._stop = stop
                stop.begin(current, self._backend.target_generation)
                logging.getLogger('rebotarm.audit').info('stop_target', extra={'audit': dict(
                    previous=previous.tolist(), target=list(current), preserved=bool(preserve),
                    generation=self._backend.target_generation)})

                def cancel(reason):
                    """代次失效或运行时关闭时终结本次停止, 旧续体不能改变后继目标."""
                    stop.abort(reason)
                    if (self._lifecycle.enabled
                            and self._backend.target_generation == stop.result.generation):
                        self._lifecycle.set_execution_state('IDLE')
                    if active is not None:
                        owner.finish(active, 'aborted', f'stop/hold unconfirmed: {reason}')

                def poll():
                    """只读已验证缓存以推进本次停止; 未确认时返回未完成, 不阻塞设备线程."""
                    # 所有读取均来自已验证缓存; 这里不发送, 不睡眠, 不持锁等新帧.
                    if (not self._lifecycle.enabled
                            or self._backend.target_generation != stop.result.generation):
                        stop.abort('stop superseded by lifecycle or target generation')
                    if stop.pending:
                        sample = self.feedback.joint_snapshot()
                        if any(code != 1 for code in sample.statuses):
                            stop.abort('enabled holding status not confirmed')
                        else:
                            stop.update(sample.positions, sample.velocities, sample.observed_at,
                                              generation=self._backend.target_generation,
                                              enabled=self._lifecycle.enabled, sequences=sample.sequences)
                        logging.getLogger('rebotarm.audit').info('stop_evidence', extra={'audit': dict(
                            sequences=sample.sequences, observed_at=sample.observed_at,
                            state=stop.result.state, reason=stop.evidence.reason,
                            quiet_since=stop.result.quiet_since)})
                    result = stop.result
                    if result.state == 'stopping':
                        return False, None
                    if result.state != 'confirmed':
                        raise RuntimeError(result.message)
                    self._lifecycle.set_execution_state('IDLE')
                    self._backend.execution.reservation = None
                    if active is not None:
                        owner.finish(active, 'canceled', result.message)
                    return True, None

                self._stop_wait = (poll, cancel)
                return CycleResult(poll, cancel=cancel)
            self._lifecycle.set_execution_state('IDLE')
            self._backend.execution.reservation = None
            if active is not None:
                owner.finish(active, 'canceled', 'stop/hold confirmed')

    def require_available(self):
        """停止确认完成前禁止接替输出; 显式失能仍由独立高优先级入口处理."""
        if self._stop.pending:
            raise RuntimeError('arm stop/hold confirmation pending')

    def _prepared_snapshot(self):
        """设备线程读取原静止证据和已发送保持值，不借单帧制造静止。"""
        self._lifecycle.require_enabled()
        execution = self._backend.execution
        now = time.monotonic()
        sample = self.feedback.joint_snapshot()
        hold = execution.last_sent_target
        if (hold is None or self._backend.mode != 'pos_vel'
                or self.passthrough_active() or self.gravity.gravity_compensation_active()
                or self._hold_generation != execution.generation
                or not self._hold_evidence.confirmed(now)
                or any(code != 1 for code in sample.statuses)
                or not -.005 <= now-sample.observed_at <= .15
                or np.max(np.abs(np.asarray(hold)-sample.positions)) > POSITION_SPAN):
            raise RuntimeError('fresh stationary acknowledged hold required')
        if self.gripper is None:
            raise RuntimeError('fixed gripper snapshot port unavailable')
        return sample, tuple(hold), self.gripper.snapshot()

    @device_request
    def reserve_arm_preparation(self, deadline):
        """先预留资源，再让 ROS 调用方离开设备线程执行耗时校验。"""
        from .controller_values import ArmPreparation
        self.require_available()
        execution = self._backend.execution
        task = self._arm_trajectory.task
        if execution.reservation is not None or (task is not None and not task.event.is_set()):
            raise RuntimeError('arm resource busy')
        if not np.isfinite(deadline) or time.monotonic() >= deadline or not self._scene_revision:
            raise RuntimeError('preparation deadline expired or scene unavailable')
        sample, hold, gripper = self._prepared_snapshot()
        token = ArmPreparation(self._backend.requests.generations['device'], execution.generation,
            hold, sample.positions, sample.observed_at, tuple(gripper['positions']),
            gripper['generation'], gripper['raw_position'], self._scene_revision, deadline)
        execution.reservation = token
        self._lifecycle.set_execution_state('TRAJ_PREPARING')
        return token

    @staticmethod
    def decode_prepared_arm_trajectory(payload):
        """在调用线程完成线性规模的数值解码，禁止占用设备周期。"""
        from rebotarm_trajectory.polynomial import curve_from_payload
        from .controller_values import PreparedArmHandoff
        curve = curve_from_payload(payload['curve'])
        for point in curve.points:
            ArmExecution.validate_target(point.positions)
        context = payload['context']
        return PreparedArmHandoff(curve, context['session'], context['generation'],
            tuple(context['hold']), tuple(payload['gripper_positions']),
            payload['gripper_generation'], payload['scene_revision'])

    @device_request
    def admit_prepared_arm_trajectory(self, token, handoff):
        """同一设备事务内复核身份与反馈并接纳原系数，不再改点或时间。"""
        execution = self._backend.execution
        if (execution.reservation is not token or self._stop.pending
                or time.monotonic() >= token.deadline
                or self._backend.requests.generations['device'] != token.session
                or execution.generation != token.generation
                or self._scene_revision != token.scene_revision):
            raise RuntimeError('prepared trajectory superseded or expired')
        sample, hold, gripper = self._prepared_snapshot()
        if (hold != token.hold or handoff.hold != token.hold
                or handoff.session != token.session or handoff.generation != token.generation
                or handoff.scene_revision != token.scene_revision
                or handoff.gripper_generation != token.gripper_generation
                or handoff.gripper_positions != token.gripper_positions
                or gripper['generation'] != token.gripper_generation
                or abs(gripper['raw_position']-token.gripper_raw_position) > POSITION_SPAN):
            raise RuntimeError('prepared trajectory context changed')
        curve = handoff.curve
        if curve.points[0].positions != token.hold:
            raise ValueError('prepared initial hold mismatch')
        task = self._arm_trajectory.submit_prepared(curve, token.generation)
        self._lifecycle.set_execution_state('TRAJ_RUNNING')
        return task

    @device_request
    def cancel_arm_preparation(self, token):
        """仅撤销本令牌，停止确认之前继续占用固定夹爪。"""
        if self._backend.execution.reservation is token:
            return self.stop_active_motion()

    @device_request
    def update_trajectory_scene(self, revision):
        """已观察到的环境变化使准备/活动曲线失效，健康设备继续使能保持。"""
        self._scene_revision = str(revision)
        token = self._backend.execution.reservation
        if token is not None and token.scene_revision != self._scene_revision:
            return self.stop_active_motion()

    @device_request
    def submit_arm_trajectory(self, joint_names, points, times, *, home=False,
                              velocity=None, timeout=2.):
        """原子检查六轴资源并提交轨迹; 返回只属于本请求的任务句柄."""
        self._lifecycle.require_enabled()
        self.require_available()
        if self._backend.arm_resource_reserved:
            raise RuntimeError('arm resource reserved by trajectory preparation')
        if len(joint_names) != 6 or set(joint_names) != set(self._backend.joint_names):
            raise ValueError('trajectory joints must match hardware exactly')
        if not points or len(points) != len(times):
            raise ValueError('trajectory must contain matching points and times')
        if any(len(point) != 6 for point in points):
            raise ValueError('each trajectory point must contain six positions')
        if not np.isfinite(timeout) or timeout <= 0:
            raise ValueError('trajectory settling timeout must be positive and finite')
        order = [joint_names.index(name) for name in self._backend.joint_names]
        ordered = [self._backend.execution.validate_target([point[i] for i in order])
                   for point in points]
        stamps = [float(value) for value in times]
        if (not np.isfinite(stamps).all() or min(stamps) < 0
                or any(b < a for a, b in zip(stamps, stamps[1:]))):
            raise ValueError('trajectory times must be finite, nonnegative and nondecreasing')
        active = self._arm_trajectory.task
        if active is not None and not active.event.is_set():
            raise RuntimeError('arm resource busy')
        if self._arm_passthrough_until is not None:
            raise RuntimeError('arm resource owned by passthrough')
        current, _, _ = self.feedback.get_joint_state()
        if np.max(np.abs(np.asarray(ordered[0])-current)) > .10:
            raise ValueError('first trajectory point is too far from current joint state')
        self.ensure_pos_vel_control()
        # 同时刻的点合并为最后一个; t=0 首点按既有接口仅作为对齐占位.
        samples = {0.: tuple(current)}
        for index, (stamp, point) in enumerate(zip(stamps, ordered)):
            if index == 0 and stamp <= 1e-9:
                continue
            samples[stamp] = point
        task = self._arm_trajectory.submit(list(samples.values()), list(samples), current,
            tolerance=.02 if home else .03, settle=timeout,
            velocity=velocity, hold_on_success=home)
        self._lifecycle.set_execution_state('TRAJ_RUNNING')
        return task

    def arm_execution_status(self, task):
        """返回指定任务结果快照, 状态不能与接替任务混用."""
        return self._arm_trajectory.snapshot(task)

    @device_request
    def set_joint_target(self, positions) -> None:
        """显式使能且无停止确认占用时设置六轴 rad 目标, 清除透传租约."""
        self._lifecycle.require_enabled()
        self.require_available()
        if self._backend.arm_resource_reserved:
            raise RuntimeError('arm resource reserved by trajectory preparation')
        self._arm_passthrough_until = None
        self._backend.set_target(positions)

    def joint_target(self):
        """返回运行时保持或轨迹目标副本, 不作为实测位置."""
        return self._backend.target()

    @device_request
    def cancel_arm_trajectory(self) -> None:
        """兼容取消入口, 复用带反馈确认的停止保持事务, 不执行失能."""
        return self.stop_active_motion()

    @property
    def trajectory_moving(self):
        """查询执行器是否仍有未发送轨迹点, 不表示设备是否真实静止."""
        return self._backend.trajectory_moving

    @property
    def trajectory_progress(self):
        """查询已发送轨迹点比例, 成功到位需独立任务反馈确认."""
        return self._backend.trajectory_progress

    def safe_home_target(self) -> np.ndarray:
        """返回配置基线的停靠姿态副本, 不能代替本轮现场安全确认."""
        return np.array(_SAFE_HOME_JOINT_POSITIONS, dtype=np.float64)

    def _validated_safe_home_target(
        self, target: Optional[Sequence[float]] = None
    ) -> np.ndarray:
        """校验 safe_home 目标: 长度, 有限性, 以及逐关节是否在软限位内."""
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
        for name, value in zip(self._backend.joint_names, values):
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

    def safe_home(self, target=None, vlim=None, timeout_sec=None) -> np.ndarray:
        """提交带速度覆盖的两点停靠任务, 阻塞调用方直至取得任务终态.

        target 为六轴 rad 目标, vlim 为不高于配置停靠上限的 rad/s, timeout_sec 为任务预算.
        只校验数值和关节限位, 不生成避障路径或执行碰撞检查; 调用方须先确认现场路径.
        成功返回实测角, 其他终态抛错; 健康任务失败由周期接管保持, 不自动失能.
        """
        goal = self._validated_safe_home_target(target)
        speed = self._backend.home_velocity if vlim is None else float(vlim)
        timeout = _SAFE_HOME_TIMEOUT_SEC if timeout_sec is None else float(timeout_sec)
        if not np.isfinite(timeout) or timeout <= 0:
            raise ValueError('safe_home timeout must be positive and finite')
        if not 0 < speed <= self._backend.home_velocity:
            raise ValueError('safe_home velocity exceeds reviewed limit')
        current, _, _ = self.feedback.get_joint_state()
        task = self.submit_arm_trajectory(self._backend.joint_names, [current, goal], [0., .001],
            home=True, velocity=[speed]*6, timeout=timeout)
        task.event.wait()
        status = self.arm_execution_status(task)
        if status.state != 'succeeded':
            raise RuntimeError(status.message)
        return self.feedback.get_joint_state()[0]

    @device_request
    def ensure_pos_vel_control(self) -> None:
        """确保处于位置-速度模式且统一控制循环在运行(否则保持当前位置)."""
        with self._lifecycle.lock:
            self._lifecycle.require_enabled()
            self.require_available()
            if self._backend.mode != "pos_vel":
                self.stop_control_loop()
                self._lifecycle.set_mode("pos_vel")
            if not self._backend.control_loop_active:
                self.start_position_loop()

    @staticmethod
    def _validate_passthrough_values(pos, vel, kp, kd, tau, vlim, bounds, max_velocity, max_effort, mode):
        """复用无状态准入规则检查补齐后的单电机数值; 拒绝不修改原任务."""
        passthrough_decision(pos, vel, kp, kd, tau, vlim, bounds,
                             max_velocity, max_effort, mode).require_allowed()

    @device_request
    def send_joint_motor_cmd(self, joint_name: str, cmd, arbitration="reject") -> None:
        """低层单关节直通命令(JointMotorCmd).

        未显式给出的字段回落到当前反馈值或关节默认增益, 避免用零值覆盖. 命令
        送入后状态机切到 LOWLEVEL_STREAMING, 提示上层这不是轨迹跟踪.
        mode: 0 = MIT, 1 = 位置-速度, 2 = 速度(需电机支持 send_vel).
        """
        with self._lifecycle.lock:
            self._lifecycle.require_enabled()
            self.require_available()
            if self._backend.execution.reservation is not None:
                raise RuntimeError('arm resource reserved by trajectory preparation')
            if joint_name not in self._backend.joint_names:
                raise KeyError(f"unknown joint: {joint_name}")

            task = self._arm_trajectory.task
            busy = ((task is not None and not task.event.is_set())
                    or self.gravity.gravity_compensation_active())
            if busy and arbitration != 'preempt':
                raise RuntimeError('arm resource busy')
            mot = self._backend.joint_device(joint_name)
            jc = self._backend.joint_configuration(joint_name)
            feedback_failure = self.feedback.arm_failure()
            if feedback_failure is not None:
                raise RuntimeError(feedback_failure)
            state = self.feedback.verified_sample(joint_name).state

            def values_from(sample):
                """按 use_* 标记组合指令, 当前反馈与配置默认值; 模式交接后须重新调用."""
                return (float(cmd.pos) if cmd.use_pos else float(sample.pos),
                        float(cmd.vel) if cmd.use_vel else float(sample.vel),
                        float(cmd.kp) if cmd.use_kp else float(jc.kp),
                        float(cmd.kd) if cmd.use_kd else float(jc.kd),
                        float(cmd.tau) if cmd.use_tau else 0.,
                        float(cmd.vlim) if cmd.use_vlim else float(jc.vlim))

            values = values_from(state)
            passthrough_decision(*values,
                _JOINT_POSITION_LIMITS_RAD[joint_name], jc.vlim,
                self.gravity.effort_limit(joint_name),
                int(cmd.mode), device_mode='pos_vel' if busy else self._backend.mode).require_allowed()
            if busy:
                # 无效命令不得取消原任务; 完成准入后才交接资源并确认保持.
                stopping = self.stop_active_motion(task)
                if isinstance(stopping, CycleResult):
                    # 完成静止确认后重新走完整准入; 失能/旧代次由运行时取消续体.
                    stopping.continuation = lambda: self.send_joint_motor_cmd(
                        joint_name, cmd, arbitration="reject")
                    return stopping
                # 停止可能耗时且切回 POS_VEL; 默认反馈值须取停止后的新快照.
                values = values_from(self.feedback.verified_sample(joint_name).state)
                passthrough_decision(*values,
                    _JOINT_POSITION_LIMITS_RAD[joint_name], jc.vlim,
                    self.gravity.effort_limit(joint_name),
                    int(cmd.mode), device_mode=self._backend.mode).require_allowed()
            pos, vel, kp, kd, tau, vlim = values
            try:
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
            except CommandTransportError as exc:
                self.protection.protect(str(exc),category="COMMAND")
                raise
            # 一次透传租约最长 0.25 s; 后续周期不覆盖它, 断流后在当前位置保持.
            self._arm_passthrough_until = time.monotonic() + .25
            self._lifecycle.set_execution_state("LOWLEVEL_STREAMING")

    def current_pose(self):
        """用规范模型的纯数值 FK 换算成 ROS Pose(供状态发布使用)."""
        q, _, _ = self.feedback.get_joint_state()
        position, rotation = self._backend.current_pose_values(q)
        return fk_to_pose(position, rotation)

    def start_position_loop(self, target: np.ndarray | None = None) -> None:
        """启动统一硬件控制循环(位置-速度跟踪), 并把控制器标记为运行中.

        ``target`` 为空时以当前角度为目标(保持不动).
        """
        if self._backend.control_loop_active:
            return
        if target is None:
            self.hold_current_position()
        else:
            self._backend.set_target(target)
        self._backend.set_execution_running(True)
        try:
            self._backend.start_control_loop(self._endpos_hardware_tick)
        except Exception:
            self._backend.set_execution_running(False)
            raise

    def _endpos_hardware_tick(self, arm, dt: float) -> None:
        """向周期协调器提供六轴位置发送回调, 成功发送后才标记本代保持已送出."""
        def send(runtime, period):
            """先执行六轴发送, 只有返回成功才记录保持发送时间."""
            self._backend.position_tick(runtime, period)
            self._stop.sent(self._backend.target_generation)
        self.cycle.control_tick(arm, dt, send)

    def stop_control_loop(self) -> None:
        """撤下周期并作废数值运行状态, 终结旧轨迹任务; 不直接失能."""
        self._backend.stop_control_loop()
        self._backend.set_execution_running(False)
        self.reset_session()
        owner = self._arm_trajectory
        if owner is not None and owner.task is not None:
            owner.finish(owner.task, 'aborted', 'control mode or lifecycle changed')

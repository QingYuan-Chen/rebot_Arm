"""六轴目标准入、轨迹任务、保持与透传交接。

对外接口：submit_arm_trajectory/set_joint_target 接纳六轴数值目标；stop_active_motion
返回由运行时推进的停止事务；周期回调只在实际发送成功后记录保持发送。
约束：状态由本组件及 ArmTrajectory/ArmStop 拥有；停止确认期间拒绝新运动。
健康任务失败交回当前位置保持，设备错误交给保护组件。IK 与路径生成归 motion。
"""
from __future__ import annotations
from .component_ports import CyclePort, ArmFeedbackPort, GravityArmPort, ProtectionPort
from .arm_stop import ArmStop, CycleResult
from .arm_trajectory import ArmTrajectory
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
    """拥有本职责的状态和事务；ROS 门面仅提交请求或读取结果。"""

    def __init__(self, runtime, lifecycle, settings):
        self._backend = runtime
        self._lifecycle = lifecycle
        self.settings = settings
        self._bound = False
        self._arm_trajectory = ArmTrajectory(runtime.execution)
        self._arm_passthrough_until = None
        self._stop = ArmStop()

    def bind(self, *, cycle: CyclePort, feedback: ArmFeedbackPort, gravity: GravityArmPort, protection: ProtectionPort):
        """装配阶段一次注入明确组件接口；运行后不替换依赖。"""
        if self._bound:
            raise RuntimeError("component already bound")
        self.cycle = cycle
        self.feedback = feedback
        self.gravity = gravity
        self.protection = protection
        self._bound = True

    def clear_passthrough(self):
        self._arm_passthrough_until = None

    def expire_passthrough(self, now):
        """租约到期切回当前反馈保持，恢复统一周期输出。"""
        if self._arm_passthrough_until is not None and now >= self._arm_passthrough_until:
            self.clear_passthrough()
            self.ensure_pos_vel_control()
            self.hold_current_position()
            self._lifecycle.set_execution_state('IDLE')

    def abort_task(self, message):
        """生命周期或保护终止任务，不再等待旧停止确认。"""
        self._stop.abort(message)
        task = self._arm_trajectory.task
        if task is not None:
            self._arm_trajectory.finish(task, 'aborted', message)

    def passthrough_active(self):
        return self._arm_passthrough_until is not None

    def check_task_generation(self):
        """空闲周期结束过期任务；停止中的任务交给停止状态机收尾。"""
        task = self._arm_trajectory.task
        if (task is not None and task.result.state != 'stopping'
                and task.generation != self._backend.target_generation):
            self._arm_trajectory.finish(task, 'aborted', 'execution canceled or superseded')

    def update_task(self, sample):
        """发送后以新鲜快照推进任务，超差保持目标仍由六轴所有者设置。"""
        hold = self._arm_trajectory.update(sample.positions, sample.observed_at)
        if hold is not None:
            self._backend.set_target(hold)
        task = self._arm_trajectory.task
        if task is not None and task.event.is_set() and self._lifecycle.execution == 'TRAJ_RUNNING':
            self._lifecycle.set_execution_state('IDLE')

    @device_request(resource='arm', priority=0, invalidate=('arm',))
    def hold_current_position(self) -> np.ndarray:
        """把当前关节角写成保持目标，使机械臂停在原地。"""
        q, _, _ = self.feedback.get_joint_state()
        current = np.array(q, dtype=np.float64, copy=True)
        self._backend.set_target(current)
        return current


    @device_request(resource='arm', priority=0, ttl=5.)
    def stop_active_motion(self, task=None) -> None:
        """撤销当前任务并确认保持；旧任务的取消不能停止接替任务。

        只有保持命令成功发送且后续新帧持续静止 0.1 s 才返回成功。超时报告
        未确认，保留使能和保持目标；这里绝不把普通取消变成自动失能。
        """
        owner = self._arm_trajectory
        active = None if owner is None else owner.task
        if task is not None and (active is not task or task.event.is_set()):
            return
        if self._stop.pending:
            raise RuntimeError('stop/hold confirmation already pending')
        with self._lifecycle.lock:
            self._backend.requests.invalidate('arm')
            if active is not None:
                owner.stopping(active)
            self._backend.cancel_trajectory()
            self._arm_passthrough_until = None
            if self._lifecycle.enabled:
                self.gravity.stop_gravity_compensation()
                self.ensure_pos_vel_control()
                current = self.hold_current_position()
                self._stop.begin(current, self._backend.target_generation)

                def cancel(reason):
                    self._stop.abort(reason)
                    if (self._lifecycle.enabled
                            and self._backend.target_generation == self._stop.result.generation):
                        self._lifecycle.set_execution_state('IDLE')
                    if active is not None:
                        owner.finish(active, 'aborted', f'stop/hold unconfirmed: {reason}')

                def poll():
                    # 所有读取均来自已验证缓存；这里不发送、不睡眠、不持锁等新帧。
                    if (not self._lifecycle.enabled
                            or self._backend.target_generation != self._stop.result.generation):
                        self._stop.abort('stop superseded by lifecycle or target generation')
                    if self._stop.pending:
                        sample = self.feedback.joint_snapshot()
                        self._stop.update(sample.positions, sample.velocities, sample.observed_at,
                                          generation=self._backend.target_generation,
                                          enabled=self._lifecycle.enabled)
                    result = self._stop.result
                    if result.state == 'stopping':
                        return False, None
                    if result.state != 'confirmed':
                        raise RuntimeError(result.message)
                    self._lifecycle.set_execution_state('IDLE')
                    if active is not None:
                        owner.finish(active, 'canceled', result.message)
                    return True, None

                return CycleResult(poll, cancel=cancel)
            self._lifecycle.set_execution_state('IDLE')
            if active is not None:
                owner.finish(active, 'canceled', 'stop/hold confirmed')


    def require_available(self):
        """停止确认完成前禁止接替输出；显式失能仍由独立高优先级入口处理。"""
        if self._stop.pending:
            raise RuntimeError('arm stop/hold confirmation pending')


    @device_request(resource='arm')
    def submit_arm_trajectory(self, joint_names, points, times, *, home=False,
                              velocity=None, timeout=2.):
        """原子检查六轴资源并提交轨迹；返回只属于本请求的任务句柄。"""
        self._lifecycle.require_enabled()
        self.require_available()
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
        # 同时刻的点合并为最后一个；t=0 首点按既有接口仅作为对齐占位。
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
        return self._arm_trajectory.snapshot(task)


    @device_request(resource='arm')
    def set_joint_target(self, positions) -> None:
        self._lifecycle.require_enabled()
        self.require_available()
        self._arm_passthrough_until = None
        self._backend.set_target(positions)


    def joint_target(self):
        return self._backend.target()


    @device_request(resource='arm', priority=0, invalidate=('arm',))
    def cancel_arm_trajectory(self) -> None:
        return self.stop_active_motion()


    @property
    def trajectory_moving(self):
        return self._backend.trajectory_moving


    @property
    def trajectory_progress(self):
        return self._backend.trajectory_progress


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
        """提交有速度覆盖的停靠任务并等待；超时/取消由周期执行器处理。"""
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


    @device_request(resource='arm', priority=5)
    def ensure_pos_vel_control(self) -> None:
        """确保处于位置-速度模式且统一控制循环在运行（否则保持当前位置）。"""
        with self._lifecycle.lock:
            self._lifecycle.require_enabled()
            self.require_available()
            if self._backend.mode != "pos_vel":
                self.stop_control_loop()
                self._lifecycle.set_mode("pos_vel")
            if not self._backend.control_loop_active:
                self.start_position_loop()
            else:
                self.hold_current_position()


    @staticmethod
    def _validate_passthrough_values(pos, vel, kp, kd, tau, vlim, bounds, max_velocity, max_effort, mode):
        passthrough_decision(pos, vel, kp, kd, tau, vlim, bounds,
                             max_velocity, max_effort, mode).require_allowed()


    @device_request(resource='arm', ttl=.25)
    def send_joint_motor_cmd(self, joint_name: str, cmd, arbitration="reject") -> None:
        """低层单关节直通命令（JointMotorCmd）。

        未显式给出的字段回落到当前反馈值或关节默认增益，避免用零值覆盖。命令
        送入后状态机切到 LOWLEVEL_STREAMING，提示上层这不是轨迹跟踪。
        mode：0 = MIT，1 = 位置-速度，2 = 速度（需电机支持 send_vel）。
        """
        with self._lifecycle.lock:
            self._lifecycle.require_enabled()
            self.require_available()
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
                # 无效命令不得取消原任务；完成准入后才交接资源并确认保持。
                stopping = self.stop_active_motion(task)
                if isinstance(stopping, CycleResult):
                    # 完成静止确认后重新走完整准入；失能/旧代次由运行时取消续体。
                    stopping.continuation = lambda: self.send_joint_motor_cmd(
                        joint_name, cmd, arbitration="reject")
                    return stopping
                # 停止可能耗时且切回 POS_VEL；默认反馈值须取停止后的新快照。
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
            # 一次透传租约最长 0.25 s；后续周期不覆盖它，断流后在当前位置保持。
            self._arm_passthrough_until = time.monotonic() + .25
            self._lifecycle.set_execution_state("LOWLEVEL_STREAMING")


    def current_pose(self):
        """用规范模型的纯数值 FK 换算成 ROS Pose(供状态发布使用)。"""
        q, _, _ = self.feedback.get_joint_state()
        position, rotation = self._backend.current_pose_values(q)
        return fk_to_pose(position, rotation)


    def start_position_loop(self, target: np.ndarray | None = None) -> None:
        """启动统一硬件控制循环（位置-速度跟踪），并把控制器标记为运行中。

        ``target`` 为空时以当前角度为目标（保持不动）。
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
        def send(runtime, period):
            self._backend.position_tick(runtime, period)
            self._stop.sent(self._backend.target_generation)
        self.cycle.control_tick(arm, dt, send)


    def stop_control_loop(self) -> None:
        self._backend.stop_control_loop()
        self._backend.set_execution_running(False)
        owner = self._arm_trajectory
        if owner is not None and owner.task is not None:
            owner.finish(owner.task, 'aborted', 'control mode or lifecycle changed')

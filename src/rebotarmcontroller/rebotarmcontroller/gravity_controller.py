"""重力补偿启停、模式交接、周期计算与诊断。

对外接口：start/stop_gravity_compensation 切换执行；周期回调调用已有模型与
有界力矩计算；diagnostics 提供时序、反馈和故障保持结果。
约束：GravityExecutionState 唯一持有补偿状态；健康模型/计算失败保持使能，
通信、发送或反馈故障才进入保护失能；不创建送点线程，不生成路径。
"""
from __future__ import annotations
from .component_ports import ArmGravityPort, CyclePort, GravityFeedbackPort, ProtectionPort
from .command_guard import failure_decision
from .sdk_driver import CommandTransportError
from .gravity_control import GravityExecutionState
from .gravity_control import LoopTiming
from .gravity_control import bounded_mit_command
from .device_requests import device_request
import numpy as np
import time
from .gravity_control import vector
from .controller_parameters import (
    GravityCommandError, _LOG, apply_gravity_compensation_tau_scale
)


class GravityController:
    """拥有本职责的状态和事务；ROS 门面仅提交请求或读取结果。"""

    def __init__(self, runtime, lifecycle, settings, model):
        self._backend = runtime
        self._lifecycle = lifecycle
        self.settings = settings
        self._bound = False
        self._gravity_model = model
        self._gc_settings = model.settings
        self._gc_torque_limits = model.effort
        self._gc_model = model.model
        self._gc_data = self._gc_model.createData()
        self._gc_ee_frame_id = model.frame_id
        self._gc_compute_generalized_gravity = model.gravity
        self._gc_pin = model.pin
        self._gravity_execution = GravityExecutionState()
        self._gravity_timing = None
        self._gravity_feedback_metrics = {}
        self._gravity_next_log = 0.
        self._hold_decision = None

    def bind(self, *, arm: ArmGravityPort, cycle: CyclePort, feedback: GravityFeedbackPort, protection: ProtectionPort):
        """装配阶段一次注入明确组件接口；运行后不替换依赖。"""
        if self._bound:
            raise RuntimeError("component already bound")
        self.arm = arm
        self.cycle = cycle
        self.feedback = feedback
        self.protection = protection
        self._bound = True

    def clear_execution(self):
        """保护路径仅清状态，不启动任何替代输出或模式切换。"""
        self._gravity_execution.active = False
        self._gravity_execution.target = None
        self._gravity_execution.integral = None

    def effort_limit(self, name):
        """只返回指定关节的 N·m 标量，调用方不能修改模型限位数组。"""
        return float(self._gc_torque_limits[self._backend.joint_names.index(name)])

    @device_request(resource='arm', priority=5, invalidate=('arm',))
    def start_gravity_compensation(self) -> None:
        with self._lifecycle.lock:
            self.arm.require_available()
            self._start_gravity_compensation_locked()


    def _start_gravity_compensation_locked(self) -> None:
        """进入重力补偿：MIT 模式 + 广义重力前馈 + 目标位置 PI 锁定。

        流程：停掉位置-速度循环与发送线程 -> 刷新反馈并把当前角度设为目标 ->
        以配置的 Kp/Kd 进入 MIT 模式 -> 启动统一控制循环执行
        ``_gravity_hardware_tick``。补偿期间操作员可以推动机械臂（目标会跟随），
        停止时再回到位置-速度保持。
        """
        self._lifecycle.require_enabled()
        self.arm.clear_passthrough()
        timing = LoopTiming(float(self._backend.rate))
        self.stop_gravity_compensation()
        self.arm.stop_control_loop()
        self._backend.cancel_trajectory()
        try:
            self._refresh_arm_feedback()
            self._gravity_execution.target = self._read_gravity_comp_positions()
        except Exception as exc:
            self.protection.protect(str(exc))
            raise
        self._gravity_execution.last_position = self._gravity_execution.target.copy()
        # 切模式前先拒绝无法支撑当前姿态的模型计算结果。
        try:
            gravity = vector(self._gc_compute_generalized_gravity(q=self._gravity_execution.target),
                             6, 'gravity torque')
            if (np.abs(gravity) > self._gc_torque_limits).any():
                raise RuntimeError('gravity torque exceeds model effort limits')
        except Exception:
            self.arm.start_position_loop(target=self._gravity_execution.target)
            raise
        self._gravity_execution.fault = None
        self._hold_decision = None
        self._gravity_execution.hold_feedforward = None
        self._gravity_execution.integral = np.zeros_like(self._gravity_execution.target)
        self._gravity_execution.lock_counter = 0
        try:
            if not self._backend.mode_mit(
                kp=np.full(6, self._gc_settings.kp), kd=np.full(6, self._gc_settings.kd)
            ):
                raise RuntimeError('MIT mode switch did not succeed on all joints')
            # 模式交接可能持续数百毫秒，交接后必须重新取得新帧。
            self._refresh_arm_feedback()
            self._gravity_execution.target = self._read_gravity_comp_positions()
            samples = self.feedback.verified_samples(self._backend.joint_names)
            self._gravity_execution.last_total = np.clip(
                [float(s.state.torq) for s in samples],
                -self._gc_torque_limits, self._gc_torque_limits)
            self._gravity_execution.active = True
            self._gravity_comp_tick(self._backend, 1. / float(self._backend.rate))
        except (GravityCommandError, CommandTransportError) as exc:
            self._gravity_execution.active = False
            self.protection.protect(str(exc),category="GRAVITY_COMMAND")
            raise
        except Exception:
            self._gravity_execution.active = False
            # 健康任务失败恢复已验证的当前位置保持；恢复失败按设备故障处理。
            try:
                self._refresh_arm_feedback()
                target = self._read_gravity_comp_positions()
                if not self._backend.mode_pos_vel():
                    raise GravityCommandError('POS_VEL restore failed')
                self.arm.start_position_loop(target=target)
                self._lifecycle.set_execution_state('IDLE')
            except Exception as restore_error:
                self.protection.protect(str(restore_error))
            raise
        self._gravity_timing = timing
        self._gravity_feedback_metrics = {}
        self._gravity_next_log = 0.
        self._lifecycle.set_execution_state("GRAVITY_COMP")
        self._backend.start_control_loop(self._gravity_hardware_tick, rate=self._backend.rate)


    @device_request(resource='arm', priority=5, invalidate=('arm',))
    def stop_gravity_compensation(self) -> None:
        with self._lifecycle.lock:
            self._stop_gravity_compensation_locked()


    def _stop_gravity_compensation_locked(self) -> None:
        """退出重力补偿：用最新验证角度接管位置-速度保持。"""
        if not self._gravity_execution.active:
            return
        self.arm.stop_control_loop()
        self._gravity_execution.active = False
        self._gravity_execution.target = None
        self._gravity_execution.integral = None
        self._gravity_execution.lock_counter = 0
        self._gravity_execution.last_position = None
        if self._lifecycle.enabled:
            try:
                self._refresh_arm_feedback()
                hold_target = self._read_gravity_comp_positions()
                if not self._backend.mode_pos_vel():
                    raise GravityCommandError('gravity POS_VEL restore failed')
                self.arm.start_position_loop(target=hold_target)
            except Exception as exc:
                self.protection.protect(str(exc))
                raise
        self._lifecycle.set_execution_state("IDLE")


    def gravity_compensation_active(self) -> bool:
        return self._gravity_execution.active


    def gravity_compensation_target(self) -> np.ndarray | None:
        if self._gravity_execution.target is None:
            return None
        return self._gravity_execution.target.copy()


    def _refresh_arm_feedback(self) -> None:
        self.feedback.refresh_feedback_if_due(force=True)


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
        joint_names = self._backend.joint_names
        with self.feedback.transaction():
            feedback_failure = self.feedback.arm_failure()
            if feedback_failure is not None:
                raise RuntimeError(feedback_failure)
            samples = self.feedback.verified_samples(joint_names)

        positions: list[float] = []
        velocities: list[float] = []
        for name, sample in zip(joint_names, samples):
            self.feedback.validate_sample(name, sample.state)
            status = int(sample.state.status_code)
            if status != 1:
                raise RuntimeError(
                    f"{name} status_code={status}, expected 1 during gravity compensation"
                )
            positions.append(float(sample.state.pos))
            velocities.append(float(sample.state.vel))

        q = np.array(positions, dtype=np.float64)
        qd = np.array(velocities, dtype=np.float64)
        ref = reference if reference is not None else self._gravity_execution.last_position
        if ref is not None:
            q = self._angles_near_reference(q, ref)
        self._gravity_execution.last_position = np.array(q, dtype=np.float64, copy=True)
        return self._gravity_execution.last_position.copy(), qd


    def _gravity_comp_tick(self, arm, dt: float) -> None:
        """重力补偿的单周期控制律（运行在统一控制循环里）。

        1) 读反馈；2) 用动力学模型算广义重力力矩并按增益缩放，作为前馈 tau；
        3) 目标角误差按实际 dt 积分，以 N.m 限幅用于抵消稳态摩擦/模型误差；
        4) 用末端雅可比把关节速度映到笛卡尔空间：一旦线速度/角速度超过阈值，
        说明人正在拖动，于是把目标角更新为当前角并衰减积分（"跟随"模式）；
        否则累加锁定计数（"锁定"模式）。
        下发时位置/速度目标固定，仅用 tau 项补偿重力，因此机械臂呈现"轻"的手感。
        """
        if not self._gravity_execution.active or self._gravity_execution.target is None:
            return

        q, qd = self._read_gravity_comp_feedback()
        if self._gravity_execution.fault is not None:
            self._emit_bounded_gravity_command(q, qd, self._gravity_execution.hold_feedforward, dt)
            return
        tau_g = vector(self._gc_compute_generalized_gravity(q=q), 6, 'gravity torque')
        tau_g = apply_gravity_compensation_tau_scale(tau_g)

        self._gc_pin.computeJointJacobians(self._gc_model, self._gc_data, q)
        self._gc_pin.updateFramePlacements(self._gc_model, self._gc_data)
        jacobian = self._gc_pin.getFrameJacobian(
            self._gc_model, self._gc_data, self._gc_ee_frame_id,
            self._gc_pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )
        # 计算状态不直接写设备；发送成功后才更新上一周期总力矩。
        feedforward = self._gravity_execution.step(
            q, dt, tau_g, jacobian @ qd, self._gc_settings,
        )
        self._emit_bounded_gravity_command(q, qd, feedforward, dt)


    def _emit_bounded_gravity_command(self, q, qd, feedforward, dt):
        settings = self._gc_settings
        kp, kd, tau, total = bounded_mit_command(
            self._gravity_execution.target, q, qd, np.full(6, settings.kp),
            np.full(6, settings.kd), feedforward, self._gc_torque_limits,
            settings.torque_rate_limits_nm_s, self._gravity_execution.last_total, dt)
        self._send_gravity_mit(
            pos=self._gravity_execution.target,
            vel=np.zeros(6), kp=kp, kd=kd, tau=tau,
        )
        self._gravity_execution.last_total = total


    def _send_gravity_mit(self, *, pos, vel, kp, kd, tau):
        try:
            self._backend.send_mit_batch(pos=pos,vel=vel,kp=kp,kd=kd,tau=tau)
        except Exception as exc:
            raise GravityCommandError(f'MIT batch failed: {exc}') from exc


    def _gravity_fault_hold(self, reason):
        q, _ = self._read_gravity_comp_feedback()
        if self._gravity_execution.fault is None:
            self._hold_decision = failure_decision(reason, 'GRAVITY')
            self._gravity_execution.fault = self._hold_decision.reason
            self._gravity_execution.target = q.copy()
            self._gravity_execution.integral.fill(0.)
            self._gravity_execution.hold_feedforward = self._gravity_execution.last_total.copy()
            self._lifecycle.errors.append(f'GRAVITY_HOLD: {reason}')
            _LOG.error('gravity compensation stopped; enabled guarded hold: %s', reason)


    def gravity_compensation_diagnostics(self):
        timing = self._gravity_timing
        model = self._gravity_model
        settings = self._gc_settings
        return dict(timing=timing.snapshot() if timing else {},
                    safety_decision=self._hold_decision,
                    feedback={k: dict(v) for k, v in self._gravity_feedback_metrics.items()},
                    configured_feedback_rate_hz=self.settings.hardware_feedback_rate_hz,
                    fault=self._gravity_execution.fault,
                    model_path=model.path if model else None,
                    model_sha256=model.sha256 if model else None,
                    gravity_xyz_m_s2=list(settings.gravity_xyz_m_s2) if settings else None,
                    payload_mass_kg=settings.payload_mass_kg if settings else None,
                    payload_com_xyz_m=list(settings.payload_com_xyz_m) if settings else None)


    def _gravity_hardware_tick(self, arm, dt: float) -> None:
        timing = self._gravity_timing
        if timing is None:
            timing = self._gravity_timing = LoopTiming(float(self._backend.rate))
        started = time.monotonic()
        actual_dt = timing.begin(started)
        try:
            self.cycle.control_tick(arm, actual_dt, self._gravity_comp_tick)
        except GravityCommandError as exc:
            self.protection.protect(str(exc), category='GRAVITY_COMMAND')
            return
        except Exception as exc:
            # 模型或计算异常不能直接终止唯一线程，让健康电机持续旧 MIT 目标。
            try:
                self._gravity_fault_hold(str(exc))
                q, qd = self._read_gravity_comp_feedback()
                self._emit_bounded_gravity_command(
                    q, qd, self._gravity_execution.hold_feedforward, actual_dt)
            except Exception as hold_error:
                self.protection.protect(str(hold_error))
                return
        finally:
            timing.finish(time.monotonic())
        if not self._gravity_execution.active:
            return
        if timing.faulted:
            self._gravity_fault_hold('three consecutive control deadline misses')
        metrics = self._gravity_feedback_metrics
        with self.feedback.transaction():
            for name, sample in zip(self._backend.joint_names, self.feedback.verified_samples(self._backend.joint_names)):
                entry = metrics.setdefault(name, dict(sequence=sample.sequence, first=sample.observed_at,
                                                       last=sample.observed_at, updates=0))
                if sample.sequence != entry['sequence']:
                    entry.update(sequence=sample.sequence, last=sample.observed_at,
                                 updates=entry['updates'] + 1)
                elapsed = entry['last'] - entry['first']
                entry['effective_rate_hz'] = entry['updates'] / elapsed if elapsed > 0 else 0.
                entry['age_sec'] = max(0., time.monotonic() - sample.observed_at)
        self._gravity_feedback_metrics = metrics
        if started >= self._gravity_next_log:
            _LOG.info('gravity diagnostics: %s', self.gravity_compensation_diagnostics())
            self._gravity_next_log = started + 5.

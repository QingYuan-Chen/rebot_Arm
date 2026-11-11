"""重力补偿硬件适配: 受门控启停、模式交接、周期发送与诊断.

职责与位置
    设备侧适配所有者, 调用 rebotarm_force_control 的模型和控制状态,
    持有设备事务门控、反馈准入、唯一写入线程调度和发送确认.

对外接口
    start/stop_gravity_compensation 显式交接; target 和 diagnostics 提供副本与诊断.
    clear_hold_fault 供成功保持交接或已验证失能后的故障确认使用。

安全与并发约束
    交接前后重新检查新反馈; 模型/计算失败优先有界保持, 无法确认设备输出时进入保护.
    重力模型有可变 Data 缓存, 只由统一设备周期串行调用.
    故障保持需显式退出或确认失能后解除；交接失败继续保留锁存。

文件结构(按定义顺序)
    GravityController: 装配共享算法状态并适配设备交接、回执与硬件诊断.
"""

from __future__ import annotations
from ..contracts.component_ports import ArmGravityPort, CyclePort, GravityFeedbackPort, ProtectionPort
from ..runtime.command_guard import failure_decision
from ..hardware.sdk_driver import CommandTransportError
from rebotarm_force_control.gravity_control import (
    GravityExecutionState, LoopTiming, bounded_mit_command,
    vector, apply_gravity_compensation_tau_scale,
)
from ..runtime.device_requests import device_request
import numpy as np
import time
from ..controller_parameters import GravityCommandError, _LOG


class GravityController:
    """共享重力算法的硬件适配器, 统一设备门控、模式交接和命令确认."""

    def __init__(self, runtime, lifecycle, settings, model):
        """绑定共享模型计算并创建独立执行状态; 模型缓存由唯一设备周期串行使用."""
        self._backend = runtime
        self._lifecycle = lifecycle
        self.settings = settings
        self._bound = False
        self._gravity_model = model
        self._gc_settings = model.settings
        self._gc_torque_limits = model.effort
        self._gc_compute_generalized_gravity = model.gravity
        self._gc_jacobian = model.jacobian
        self._gravity_execution = GravityExecutionState()
        self._gravity_timing = None
        self._gravity_feedback_metrics = {}
        self._gravity_next_log = 0.
        self._hold_decision = None

    def bind(self, *, arm: ArmGravityPort, cycle: CyclePort, feedback: GravityFeedbackPort, protection: ProtectionPort):
        """装配阶段一次注入明确组件接口; 运行后不替换依赖."""
        if self._bound:
            raise RuntimeError("component already bound")
        self.arm = arm
        self.cycle = cycle
        self.feedback = feedback
        self.protection = protection
        self._bound = True

    def clear_execution(self):
        """保护路径仅清状态, 不启动任何替代输出或模式切换."""
        self._gravity_execution.active = False
        self._gravity_execution.target = None
        self._gravity_execution.integral = None

    def clear_hold_fault(self):
        """设备线程在成功交接保持或验证失能后清除本组件故障，保留其他错误。"""
        self._backend.assert_writer()
        if self._gravity_execution.active:
            raise RuntimeError('stop gravity compensation before clearing its fault')
        self._gravity_execution.fault = None
        self._gravity_execution.hold_feedforward = None
        self._hold_decision = None
        self._lifecycle.errors[:] = [code for code in self._lifecycle.errors
                                    if not code.startswith('GRAVITY_HOLD:')]

    def effort_limit(self, name):
        """只返回指定关节的 N·m 标量, 调用方不能修改模型限位数组."""
        return float(self._gc_torque_limits[self._backend.joint_names.index(name)])

    @device_request
    def start_gravity_compensation(self) -> None:
        """在生命周期锁内检查六轴可用性并切入补偿; 外部调用先由请求装饰器排队."""
        with self._lifecycle.lock:
            self.arm.require_available()
            self._start_gravity_compensation_locked()

    def _start_gravity_compensation_locked(self) -> None:
        """进入重力补偿: MIT 模式 + 广义重力前馈 + 目标位置 PI 锁定.

        流程: 撤下位置-速度周期回调(唯一设备线程继续运行) -> 刷新反馈并把当前角度设为目标 ->
        以配置的 Kp/Kd 进入 MIT 模式 -> 启动统一控制循环执行
        ``_gravity_hardware_tick``. 补偿期间操作员可以推动机械臂(目标会跟随),
        停止时再回到位置-速度保持.
        """
        if self._gravity_execution.fault is not None:
            raise RuntimeError('gravity fault latched; explicitly stop gravity compensation or verify disable first')
        self._lifecycle.require_enabled()
        # 准备期间也在保持输出，但不能据此切入拖拽；模式变化会破坏已绑定的曲线起点。
        if self._backend.arm_resource_reserved:
            raise RuntimeError('arm resource reserved by prepared trajectory')
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
        # 切模式前先拒绝无法支撑当前姿态的模型计算结果.
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
            # 模式交接可能持续数百毫秒, 交接后必须重新取得新帧.
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
            # 健康任务失败恢复已验证的当前位置保持; 恢复失败按设备故障处理.
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

    @device_request
    def stop_gravity_compensation(self) -> None:
        """在生命周期锁内退出补偿并交接当前位置保持; 未活动时不重设目标."""
        with self._lifecycle.lock:
            self._stop_gravity_compensation_locked()

    def _stop_gravity_compensation_locked(self) -> None:
        """退出重力补偿: 用最新验证角度接管位置-速度保持."""
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
        if self._lifecycle.enabled:
            # 失能事务在新反馈确认后另行清错；保持交接成功才可在此解除故障。
            self.clear_hold_fault()

    def gravity_compensation_active(self) -> bool:
        """查询补偿执行状态, 不以模式名推断补偿是否活动."""
        return self._gravity_execution.active

    def gravity_compensation_target(self) -> np.ndarray | None:
        """返回当前补偿目标副本; 未建目标时为 None."""
        if self._gravity_execution.target is None:
            return None
        return self._gravity_execution.target.copy()

    def _refresh_arm_feedback(self) -> None:
        """要求统一采集器取得新的六轴反馈, 交接后的旧缓存不能用于接管."""
        self.feedback.refresh_feedback_if_due(force=True)

    @staticmethod
    def _angles_near_reference(values: np.ndarray, reference: np.ndarray) -> np.ndarray:
        """把角度解到与参考值最接近的等价分支上.

        差值先按模 2*pi 折到 (-pi, pi], 再加回参考值; 用于抵消编码器在
        +/-pi 处的跳变, 避免重力补偿把跨圈跳变误判为大幅运动.
        """
        delta = values - reference
        delta = (delta + np.pi) % (2.0 * np.pi) - np.pi
        return reference + delta

    def _read_gravity_comp_positions(self, *, reference: np.ndarray | None = None) -> np.ndarray:
        """取得已验证的六轴角度副本, 可相对参考角解分支."""
        q, _qd = self._read_gravity_comp_feedback(reference=reference)
        return q

    def _read_gravity_comp_feedback(
        self,
        *,
        reference: np.ndarray | None = None,
    ) -> tuple[np.ndarray, np.ndarray]:
        """重力补偿回路的反馈读取: (角度 rad, 角速度 rad/s).

        要求全部电机状态码为 1(补偿期间必须使能), 并做软限位/有限性校验;
        角度会解到与上一帧最接近的分支, 结果同时更新 ``GravityExecutionState.last_position``.
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
        """重力补偿的单周期控制律(运行在统一控制循环里).

        1) 读反馈; 2) 用动力学模型算广义重力力矩并按增益缩放, 作为前馈 tau;
        3) 目标角误差按实际 dt 积分, 以 N.m 限幅用于抵消稳态摩擦/模型误差;
        4) 用末端雅可比把关节速度映到笛卡尔空间: 一旦线速度/角速度超过阈值,
        说明人正在拖动, 于是把目标角更新为当前角并衰减积分("跟随"模式);
        否则累加锁定计数("锁定"模式).
        每拍发送目标角, 零目标速度, 受限 PD 增益和重力/积分前馈; 拖动时目标角会跟随.
        """
        if not self._gravity_execution.active or self._gravity_execution.target is None:
            return

        q, qd = self._read_gravity_comp_feedback()
        if self._gravity_execution.fault is not None:
            self._emit_bounded_gravity_command(q, qd, self._gravity_execution.hold_feedforward, dt)
            return
        tau_g = vector(self._gc_compute_generalized_gravity(q=q), 6, 'gravity torque')
        tau_g = apply_gravity_compensation_tau_scale(tau_g)

        jacobian = self._gc_jacobian(q)
        # 计算状态不直接写设备; 发送成功后才更新上一周期总力矩.
        feedforward = self._gravity_execution.step(
            q, dt, tau_g, jacobian @ qd, self._gc_settings,
        )
        self._emit_bounded_gravity_command(q, qd, feedforward, dt)

    def _emit_bounded_gravity_command(self, q, qd, feedforward, dt):
        """结合当前反馈限制总力矩幅值与变化率, 整批发送成功后才保存 last_total."""
        settings = self._gc_settings
        kp, kd, tau, total = bounded_mit_command(
            self._gravity_execution.target, q, qd, np.full(6, settings.kp),
            np.full(6, settings.kd), feedforward, self._gc_torque_limits,
            settings.torque_rate_limits_nm_s, self._gravity_execution.last_total, dt)
        self._send_gravity_mit(
            pos=self._gravity_execution.target,
            vel=np.zeros(6), kp=kp, kd=kd, tau=tau,
        )
        # 上一输出用于下一拍变化率约束, 只记录整批成功发送的估算总力矩.
        self._gravity_execution.last_total = total

    def _send_gravity_mit(self, *, pos, vel, kp, kd, tau):
        """通过唯一运行时发送六轴 MIT 批次, 将发送失败归为 GravityCommandError."""
        try:
            self._backend.send_mit_batch(pos=pos,vel=vel,kp=kp,kd=kd,tau=tau)
        except Exception as exc:
            raise GravityCommandError(f'MIT batch failed: {exc}') from exc

    def _gravity_fault_hold(self, reason):
        """健康计算失败时固定当前目标, 清积分并锁存有界前馈保持; 读反馈失败仍向上抛出."""
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
        """复制周期, 反馈频率, 模型来源和故障保持信息; 查询不访问设备."""
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
        """统一周期中的重力补偿调度, 故障分类和诊断更新.

        使用实测周期推进积分与力矩变化率; 发送失败直接保护, 计算失败先尝试有界保持.
        保持也无法确认时进入保护; 连续周期超时锁存保持, 不退出唯一写入线程.
        反馈频率按真实序号推进统计, 重复缓存不算新样本.
        """
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
            # 模型或计算异常不能直接终止唯一线程, 让健康电机持续旧 MIT 目标.
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

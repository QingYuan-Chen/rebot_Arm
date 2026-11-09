"""MuJoCo 轨迹的预约、原子接纳、采样和停止生命周期。

文件职责：拥有规划代次、准备上下文及已接纳曲线；复用活动目标、命令闸门和
    仿真访问锁，协调六轴与独立夹爪的整体终态。
对外接口：planning_snapshot/reserve/admit；observe/complete/step；stop/poll_stop/cleanup；
    status_snapshot 供 ROS 诊断读取。输入输出均为普通 Python 数据。
安全约束：锁序固定为目标锁→仿真锁。组件不接收 Node、GoalHandle、ROS 消息，
    不等待服务或自主睡眠；准备/停止期限用单调时间，参考曲线用仿真时间。
    终态通知先于释放预约，迟到操作只作用于原 token；停止未确认保留执行权。
文件结构：具名上下文与快照 → 规划/准备 → 原子接纳 → 周期及终态 → 停止收尾。
"""

from collections import deque
from dataclasses import dataclass
import time

from rebotarm_trajectory.planning import planning_state, decode_plan, require_plan_context
from rebotarm_trajectory.polynomial import curve_from_payload
from .trajectory_state import ActiveTrajectory, TrajectoryCommandGate, ExecutionLifecycle, GateOutcome
from .trajectory_sampler import ARM_JOINT_NAMES, PreparedTrajectorySampler
from .gripper_execution import GripperExecution, GripperTask


@dataclass(frozen=True)
class Preparation:
    """只读预约快照；服务协议由节点据此编码，身份只在后端解释。"""

    token: object
    binding: str
    session: int
    generation: int
    hold_positions: tuple
    feedback_positions: tuple
    feedback_observed_at: float
    gripper_positions: tuple
    gripper_generation: int
    gripper_collision_policy: int
    scene_revision: str
    deadline_monotonic: float
    gripper_command: tuple | None


@dataclass
class AcceptedTrajectory:
    """已接纳曲线的唯一运行状态，最后发送时刻由物理周期写入。"""

    curve: object
    start: float
    revision: str
    gripper: GripperTask | None = None
    sent_at: float | None = None


@dataclass(frozen=True)
class ExecutionObservation:
    state: object
    elapsed: float
    desired: tuple
    settling: str
    arm_state: str
    gripper_state: str


@dataclass(frozen=True)
class StopWait:
    token: object
    after: float
    deadline: float


@dataclass(frozen=True)
class ExecutionStatus:
    phase: str
    scene_revision: str
    stop_unconfirmed: bool


class TrajectoryExecution:
    """执行状态唯一所有者；节点只持有此组件及 ROS 传输对象。"""

    def __init__(self, access, control, capability, mode, settling_policy, *, clock=time.monotonic):
        self.access, self.control, self.capability = access, control, capability
        self.mode, self.settling_policy, self.clock = mode, settling_policy, clock
        self.active = ActiveTrajectory()
        self.lock = self.active._lock
        self.gate = TrajectoryCommandGate(self.active)
        self.lifecycle = ExecutionLifecycle(self.active, self.gate)
        self.gripper = access.run(lambda sim: GripperExecution(sim, self.active, capability))
        self.planning_generation = 0
        self.scene_revision = ''
        self.preparation: Preparation | None = None
        self.pending_sampler = None
        self._sample_history = deque(maxlen=512)
        self._last_physics_wall = self.clock()
        self.stop_unconfirmed = False
        self._retained_stop_token = None

    def hold(self):
        """目标锁由闸门或调用方持有；保持、作废规划及夹爪停止属于同一事务。"""
        def apply(_sim):
            self.planning_generation += 1
            self.control.hold_current_position()
            if self.gripper.task is not None:
                self.gripper.cancel(self.gripper.task, 'combination stopped')
        self.access.run(apply)

    def _admission_snapshot(self, sim):
        state = sim.get_state()
        cutoff = state.simulation_time-.2
        samples = [item for item in self._sample_history if item.simulation_time >= cutoff-.0021]
        if (self.clock()-self._last_physics_wall > .15 or len(samples) < 3
                or samples[-1].simulation_time-samples[0].simulation_time < .2-1e-9):
            raise RuntimeError('fresh complete gripper stationary window required')
        if any(abs(item.joint_velocities[index]) > .00045 for item in samples for index in (6, 7)):
            raise RuntimeError('gripper stationary evidence unavailable')
        return state, tuple(sim.get_control_status().joint_targets)

    def _stationary_snapshot(self, sim, *, after=None):
        """停止确认使用原六轴及双指速度窗口；准入仍沿用夹爪条件。"""
        state = sim.get_state()
        cutoff = state.simulation_time-.2
        samples = [item for item in self._sample_history if item.simulation_time >= cutoff-.0021]
        if (self.clock()-self._last_physics_wall > .15 or len(samples) < 3
                or samples[-1].simulation_time-samples[0].simulation_time < .2-1e-9
                or (after is not None and samples[0].simulation_time <= after)):
            raise RuntimeError('fresh complete stationary window required')
        for index, speed in enumerate((.05,)*6+(.00045,)*2):
            if any(abs(item.joint_velocities[index]) > speed for item in samples):
                raise RuntimeError(f'{state.joint_names[index]} stationary evidence:'
                    f' speed={max(abs(item.joint_velocities[index]) for item in samples):.9f} > {speed:.9f}')
        return state, tuple(sim.get_control_status().joint_targets)

    def planning_snapshot(self):
        with self.lock:
            if self.active.busy or self.gripper.busy or self.stop_unconfirmed:
                raise RuntimeError('arm/gripper resources reserved or stop unconfirmed')
            def snapshot(sim):
                _state, hold = self._admission_snapshot(sim)
                if sim.control_mode not in ('position', 'hold'):
                    raise RuntimeError('planning requires position/hold mode')
                return planning_state(self.capability['revision'], self.capability['session'],
                    self.planning_generation, hold, executable=True)
            return self.access.run(snapshot)

    def admission_reasons(self):
        """诊断只读快照，不进行设备访问或重新接纳。"""
        with self.lock:
            reasons = []
            if self.active.busy or self.stop_unconfirmed:
                reasons.append('arm/gripper resources reserved or stop unconfirmed')
            def inspect(sim):
                try:
                    self._admission_snapshot(sim)
                except RuntimeError as exc:
                    reasons.append(str(exc))
                if sim.control_mode not in ('position', 'hold'):
                    reasons.append(f'mode={sim.control_mode}, required=position/hold')
            self.access.run(inspect)
            return reasons

    def reserve(self, token, names, points, binding, *, web, gripper_command, received):
        """结构和规模已由协议层校验；在同一锁序内绑定规划与执行资源。"""
        with self.lock:
            if not self.scene_revision:
                raise RuntimeError('scene revision unavailable')
            if self.active.busy or self.gripper.busy:
                raise RuntimeError('arm/gripper resources reserved')
            def reserve_in_sim(sim):
                state, hold = self._admission_snapshot(sim)
                if sim.control_mode not in ('position', 'hold'):
                    raise RuntimeError(f'mode={sim.control_mode}, required=position/hold')
                plan = decode_plan(binding, names, points,
                    capability_revision=self.capability['revision'], require_execution=True)
                require_plan_context(plan, capability_revision=self.capability['revision'],
                    session=self.capability['session'], generation=self.planning_generation, hold=hold)
                preparation = Preparation(token, binding, self.capability['session'],
                    self.planning_generation, hold, tuple(state.joint_positions[:6]),
                    self._last_physics_wall, tuple(state.joint_positions[-2:]), 0,
                    1 if web else 0, self.scene_revision, received+min(2., .2*points[-1].time+.25),
                    gripper_command)
                if not self.active.try_start(token):
                    raise RuntimeError('arm/gripper resources reserved')
                self.preparation = preparation
                self.pending_sampler = None
                self.stop_unconfirmed = False
                return preparation
            return self.access.run(reserve_in_sim)

    def admit_legacy(self, token, sampler):
        with self.lock:
            if self.gripper.busy or not self.active.try_start(token):
                raise RuntimeError('arm/gripper resources reserved')
            self.pending_sampler = sampler
            self.planning_generation += 1

    def take_sampler(self):
        with self.lock:
            sampler, self.pending_sampler = self.pending_sampler, None
            return sampler

    def accepted_context(self, token):
        """返回本次 Action 的只读时间起点与夹爪句柄，后续写入仍由组件负责。"""
        with self.lock:
            if self.active.token is not token or self.active.prepared is None:
                raise RuntimeError('accepted trajectory ownership lost')
            return self.active.prepared.start, self.active.prepared.gripper

    def fail_legacy(self, token, notify):
        """保留旧异常清理的通知顺序；迟到异常仅通知原 Action。"""
        with self.lock:
            if self.active.token is token:
                self.lifecycle.fail(token, self.hold, notify)
            else:
                notify()

    def preparation_for(self, token):
        with self.lock:
            if self.preparation is None or self.preparation.token is not token:
                raise RuntimeError('preparation ownership lost')
            return self.preparation

    def scene_changed(self, revision):
        with self.lock:
            self.scene_revision = revision
            expected = (self.preparation.scene_revision if self.preparation is not None else
                self.active.prepared.revision if self.active.prepared else None)
            if expected is not None and expected != revision:
                self.gate.stop_and_hold(self.hold)

    def admit(self, token, payload, response_revision, cancel_requested):
        """服务返回后的最终原子复核；输入曲线保持已检查系数及绑定。"""
        request = self.preparation_for(token)
        curve = curve_from_payload(payload['curve'])
        decode_plan(request.binding, curve.joint_names, curve.points,
            capability_revision=self.capability['revision'], require_execution=True)

        def apply():
            def in_sim(sim):
                state, hold = self._admission_snapshot(sim)
                context = payload['context']
                if (sim.control_mode not in ('position', 'hold')
                        or self.clock() >= request.deadline_monotonic
                        or self.scene_revision != response_revision
                        or payload['scene_revision'] != request.scene_revision
                        or context['session'] != request.session or context['generation'] != request.generation
                        or self.planning_generation != request.generation
                        or tuple(context['hold']) != request.hold_positions
                        or hold != request.hold_positions or curve.points[0].positions[:6] != hold
                        or payload['gripper_generation'] != request.gripper_generation
                        or tuple(payload['gripper_positions']) != request.gripper_positions
                        or payload['gripper_collision_policy'] != request.gripper_collision_policy
                        or payload['capability_revision'] != self.capability['revision']
                        or tuple(curve.joint_names) != tuple(ARM_JOINT_NAMES)):
                    raise RuntimeError('prepared simulation context changed')
                accepted = AcceptedTrajectory(curve, state.simulation_time, response_revision)
                # 同周期登记；登记阶段不推进任一目标或物理时间。
                if request.gripper_command is not None:
                    accepted.gripper = self.gripper.start(token, *request.gripper_command)
                self.active.prepared = accepted
                self.planning_generation += 1
                self.preparation = None
            self.access.run(in_sim)

        if self.gate.apply_with_reason(token, cancel_requested, apply, self.hold) is not GateOutcome.APPLIED:
            raise RuntimeError('prepared trajectory canceled before atomic admission')
        return PreparedTrajectorySampler(curve)

    def observe(self, token, sampler, start_time, cancel_requested):
        """闸门内读取反馈；legacy 下发目标，prepared 的采样写入只在物理周期发生。"""
        observation = None
        def apply():
            nonlocal observation
            def inspect(sim):
                state = sim.get_state()
                elapsed = max(0., state.simulation_time-start_time)
                desired = sampler.sample(min(elapsed, sampler.duration))
                if self.mode == 'legacy':
                    self.control.command_joint_positions(desired)
                settling = self.settling_policy.evaluate(sampler.sample(sampler.duration)[:6],
                    tuple(state.joint_positions[:6]), tuple(state.joint_velocities[:6]),
                    max(0., elapsed-sampler.duration)) if elapsed >= sampler.duration else 'tracking'
                arm_state = 'succeeded' if settling == 'succeeded' else 'running'
                active = self.active.prepared
                grip = active.gripper if active else None
                if grip and grip.state in ('failed', 'releasing'):
                    raise RuntimeError(grip.reason or 'independent gripper failed')
                if settling == 'succeeded' and grip and grip.state != 'succeeded':
                    settling = 'settling'
                if self.mode == 'continuous' and settling == 'succeeded':
                    if active is None or active.sent_at is None or state.simulation_time <= active.sent_at:
                        settling = 'settling'
                return ExecutionObservation(state, elapsed, desired, settling, arm_state,
                    grip.state if grip else 'not_requested')
            observation = self.access.run(inspect)
        outcome = self.gate.apply_with_reason(token, cancel_requested, apply, self.hold)
        return outcome, observation

    def complete(self, token, sampler, cancel_requested, notify):
        """终态通知与释放保持原闸门事务顺序，通知回调在仿真锁外运行。"""
        def succeed():
            if self.mode == 'continuous':
                self.access.run(lambda _sim: self.control.command_joint_positions(sampler.sample(sampler.duration)[:6]))
            else:
                self.hold()
            notify()
        return self.gate.complete_with_reason(token, cancel_requested, self.hold, succeed)

    def step(self, count, record):
        """物理调度调用；延迟后按当前仿真时间采样，不补发过期目标。"""
        with self.lock:
            def advance(sim):
                for _ in range(count):
                    active = self.active.prepared
                    if self.mode == 'continuous' and active is not None and not self.active.cancel_requested:
                        current = sim.get_state()
                        elapsed = current.simulation_time-active.start
                        self.control.command_joint_positions(active.curve.sample(elapsed)[0])
                        if elapsed >= active.curve.duration and active.sent_at is None:
                            active.sent_at = current.simulation_time
                    self.gripper.advance()
                    state = sim.step()
                    if self.mode == 'continuous':
                        # 记录器只读本步快照，任务写权限留在执行组件。
                        record(state, sim.get_control_status(),
                            dict(curve=active.curve, start=active.start) if active else None)
                        self._sample_history.append(state)
                if self.mode == 'continuous':
                    self._last_physics_wall = self.clock()
                return state
            return self.access.run(advance)

    def stop(self, token=None):
        with self.lock:
            if token is not None and self.active.token is not token:
                return False
            stopped = self.gate.stop_and_hold(self.hold)
            if token is None and not stopped:
                self.hold()
            return stopped

    def begin_stop_wait(self, token):
        with self.lock:
            return StopWait(token, self.access.run(lambda sim: sim.get_state().simulation_time), self.clock()+5.)

    def poll_stop(self, wait):
        """单次观察后立即返回；等待循环由 ROS 工作线程在锁外执行。"""
        with self.lock:
            if self.active.token is not wait.token:
                return True
            try:
                self.access.run(lambda sim: self._stationary_snapshot(sim, after=wait.after))
            except RuntimeError:
                if self.clock() < wait.deadline:
                    return False
                self.stop_unconfirmed = True
                raise RuntimeError('simulation stop/hold unconfirmed')
            self.stop_unconfirmed = False
            if self._retained_stop_token is wait.token:
                self.cleanup(wait.token)
                self._retained_stop_token = None
            return True

    def cleanup(self, token):
        with self.lock:
            if self.active.token is not token:
                return
            if self.stop_unconfirmed:
                self._retained_stop_token = token
                return
            if self.preparation is not None and self.preparation.token is token:
                self.preparation = None
            self.active.finish(token)

    def change_mode(self, mode):
        with self.lock:
            if self.active.busy or self.gripper.busy:
                raise RuntimeError('trajectory active; stop or cancel it before changing mode')
            result = self.access.run(lambda _sim: self.control.set_mode(mode))
            self.planning_generation += 1
            return result

    def status_snapshot(self):
        with self.lock:
            return ExecutionStatus('TRAJ_PREPARING' if self.preparation else
                'TRAJ_RUNNING' if self.active.busy else 'IDLE', self.scene_revision, self.stop_unconfirmed)

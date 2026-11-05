"""MuJoCo 独立夹爪的任务、限速和停止确认。

文件职责：拥有单个夹爪任务及预约身份，推进原有位置控制和中性释放。
对外接口：reserve/start/cancel/advance/release_reservation；task 为具名任务句柄。
安全约束：调用方按“活动目标锁→仿真访问锁”串行调用；本组件不创建线程或计时器，
    不接收 ROS 消息。最终目标发送后要求新物理帧，到位/超时保持原预算。
文件结构：任务数据 → 预约与登记 → 取消 → 物理周期推进。
"""

from dataclasses import dataclass
import math


@dataclass
class GripperTask:
    """每个字段由夹爪执行器写入；跨回调传递原句柄身份。"""

    owner: object
    start: float
    initial: float
    target: float
    speed: float
    deadline: float
    effort: float
    state: str = 'active'
    reason: str = ''
    released: float | None = None
    sent_at: float | None = None


class GripperExecution:
    """复用现有仿真对象和活动目标；锁由上层协调器统一持有。"""

    def __init__(self, sim, active, capability):
        self._sim, self._active = sim, active
        self._capability = capability
        self.task: GripperTask | None = None
        self.reservation = None

    @property
    def busy(self):
        return self.reservation is not None or self.task is not None and self.task.state in ('active', 'releasing')

    def reserve(self, owner):
        if self._active.busy or self.busy:
            raise RuntimeError('gripper resources reserved')
        self.reservation = owner

    def release_reservation(self, owner):
        if self.reservation is owner:
            self.reservation = None

    def start(self, owner, width, effort):
        """输入已由协议适配层校验；内部目标仍受启动配置限速、限力矩。"""
        if self._active.busy and self._active.token is not owner:
            raise RuntimeError('gripper reserved by arm task')
        if self.reservation is not None and self.reservation is not owner:
            raise RuntimeError('gripper reserved by another task')
        if self.task is not None and self.task.state in ('active', 'releasing'):
            raise RuntimeError('gripper busy')
        state = self._sim.get_state()
        speed = self._capability['gripper_motor_speed_rad_s']*.018
        self.task = GripperTask(owner, state.simulation_time, state.gripper_width, width,
            speed, state.simulation_time+abs(width-state.gripper_width)/speed+1.5,
            min(effort if effort > 0 else 1., self._capability['gripper_torque_cap_nm']))
        return self.task

    def cancel(self, task, reason):
        """本任务中性释放后等待静止；旧句柄对新任务无写权限。"""
        if task is not self.task or task.state != 'active':
            return
        self._sim.release_gripper_position()
        task.state, task.reason = 'releasing', reason
        task.released = self._sim.get_state().simulation_time

    def advance(self):
        """每物理步推进一次；重复读取同一物理帧不会提前到位。"""
        task = self.task
        if task is None or task.state not in ('active', 'releasing'):
            return
        state = self._sim.get_state()
        if task.state == 'releasing':
            if state.simulation_time-task.released >= .2 and max(abs(v) for v in state.joint_velocities[-2:]) <= .00045:
                task.state = 'failed'
            return
        elapsed = state.simulation_time-task.start
        distance = min(abs(task.target-task.initial), task.speed*elapsed)
        target = task.initial+math.copysign(distance, task.target-task.initial)
        self._sim.command_gripper_position(target, task.effort)
        if distance >= abs(task.target-task.initial):
            if task.sent_at is None:
                task.sent_at = state.simulation_time
            if state.simulation_time > task.sent_at and abs(state.gripper_width-task.target) <= .12*.018:
                self._sim.release_gripper_position()
                task.state = 'succeeded'
        if task.state == 'active' and state.simulation_time >= task.deadline:
            self.cancel(task, 'gripper goal timeout')

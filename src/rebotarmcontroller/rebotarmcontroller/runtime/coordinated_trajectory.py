"""唯一设备周期中的机械臂与夹爪协同任务所有者。

文件职责：持有一条七自由度曲线的起时、任务与整体终态，协调原组件发送。
对外接口：submit 接纳原系数；tick 一次采样并发送两类命令；finish/stopping/snapshot
    沿用任务值契约，供六轴业务组件统一处理取消及 ROS 结果。
安全约束：仅设备线程调用 submit/tick；无独立线程。逐轴发送不原子，部分成功只记录
    实际回执，整体不能成功；夹爪不独立追赶，健康失败交回停止保持事务。
文件结构：任务创建 → 同周期发送与终点确认。
"""

from dataclasses import replace
import logging
import time
from ..arm.arm_trajectory import ArmTrajectory, ArmTask
from ..arm.arm_execution import ArmTarget
from rebotarm_trajectory.joint_contract import ArmCurveView
from rebotarm_trajectory.sampling import step_index


class CoordinatedTrajectory(ArmTrajectory):
    """复用任务锁和结果契约，七自由度任务状态只由本对象持有。"""

    def __init__(self, execution, clock=time.monotonic):
        super().__init__(execution, clock)
        self.curve = None
        self.started = None
        self.gripper_generation = None

    @property
    def active(self):
        return self.task is not None and not self.task.event.is_set() and self.task.result.state != 'stopping'

    def submit(self, curve, generation, gripper):
        """已完成原子快照核验后调用；夹爪接管不发送，首拍才开始曲线。"""
        with self.lock:
            if self.active:
                raise RuntimeError('coordinated resource busy')
            if curve.duration <= 0 or len(curve.joint_names) != 7:
                raise ValueError('positive-duration seven-joint trajectory required')
            started = self.clock()
            gripper_generation = gripper.begin_coordinated()
            try:
                if not self.execution.submit_prepared(ArmCurveView(curve), generation, now=started):
                    raise RuntimeError('coordinated execution superseded')
            except Exception:
                gripper.end_coordinated(gripper_generation)
                raise
            self.curve, self.started = curve, started
            self.gripper_generation = gripper_generation
            self.task = ArmTask(self.execution.generation, curve.points[-1].positions,
                started+curve.duration+2., .03)
            return self.task

    def tick(self, runtime, gripper, arm_sample):
        """一次取时、一次求值；两侧发送后才检查整体结果，不提前发布到位。"""
        with self.lock:
            if not self.active:
                return False
            task, curve = self.task, self.curve
            if task.generation != self.execution.generation:
                raise RuntimeError('coordinated generation changed')
            now = self.clock()
            state = gripper.coordinated_state(self.gripper_generation)
            q, v, a, jerk = curve.sample(now-self.started)
            target = ArmTarget(q[:6], None, task.generation, step_index(curve.times, now-self.started))
            arm_confirmed = False
            try:
                runtime.send_position_target(target)
                arm_confirmed = True
                # 六轴回执可先成功，但只有夹爪也成功后才推进本任务的整批确认。
                gripper.send_coordinated(self.gripper_generation, q[-1])
            except Exception as exc:
                logging.getLogger('rebotarm.audit').error('coordinated_receipt', extra={'audit': dict(
                    generation=task.generation, sampled_at=now, completed_at=self.clock(),
                    reference=q, arm_batch_confirmed=arm_confirmed, gripper_sent=False,
                    success=False, error=str(exc))})
                raise
            finished_send = self.clock()
            logging.getLogger('rebotarm.audit').info('coordinated_receipt', extra={'audit': dict(
                generation=task.generation, sampled_at=now, completed_at=finished_send,
                reference=q, velocity=v, acceleration=a, jerk=jerk, success=True)})
            task.result = replace(task.result, progress=min(1., max(0., (now-self.started)/curve.duration)), desired=q)
            if now-self.started >= curve.duration and task.sent_at is None:
                task.sent_at = finished_send
            if (task.sent_at is not None and min(arm_sample.observed_at, state['observed_at']) > task.sent_at
                    and max(abs(q-r) for q,r in zip(arm_sample.positions, task.target[:6])) <= .03
                    and abs(state['raw_position']+task.target[-1]/.009) <= .12):
                if not gripper.end_coordinated(self.gripper_generation):
                    raise RuntimeError('coordinated gripper neutral not confirmed')
                self.finish(task, 'succeeded', 'coordinated trajectory goal reached; gripper neutral confirmed')
            elif now >= task.deadline:
                raise ValueError('coordinated trajectory goal did not settle')
            return True

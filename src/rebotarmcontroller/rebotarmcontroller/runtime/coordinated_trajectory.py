"""网页六轴轨迹与独立夹爪的组合任务所有者。

文件职责：保存组合任务身份并汇总原六轴、夹爪执行器的结果，不生成夹爪参考。
对外接口：submit 在设备事务中登记两个目标；tick 查询两侧结果；cancel_gripper
    只取消本任务的夹爪代次；snapshot 沿用六轴结果契约供 ROS 查询。
安全约束：仅设备线程推进，无自主线程。准备通过前不接纳夹爪；失败由六轴所有者
    执行整体停止，停止确认前不释放预约。两侧输出仍由原周期各自发送并确认。
文件结构：整体任务创建 → 两侧状态汇总 → 指定代次取消。
"""

from dataclasses import replace
from ..arm.arm_trajectory import ArmTrajectory, ArmTask


class CoordinatedTrajectory(ArmTrajectory):
    """保留原模块入口，职责改为六轴与独立开合的整体终态管理。"""

    def __init__(self, execution, arm_trajectory):
        super().__init__(execution, arm_trajectory.clock)
        self.arm = arm_trajectory
        self.arm_task = None
        self.gripper_generation = None
        self.gripper_state = 'not_requested'

    @property
    def active(self):
        return self.task is not None and not self.task.event.is_set() and self.task.result.state != 'stopping'

    def submit(self, curve, token, gripper):
        """同一设备事务登记两侧；登记本身不发总线命令，随后同一周期各自推进。"""
        with self.lock:
            if self.active or len(curve.joint_names) != 6:
                raise ValueError('idle combination and six-joint curve required')
            generation = None
            if token.gripper_command:
                generation = gripper.start_position(token, *token.gripper_command)
            try:
                arm_task = self.arm.submit_prepared(curve, token.generation)
            except Exception:
                if generation is not None:
                    gripper.cancel_position('combination admission failed', expected_generation=generation)
                raise
            self.arm_task = arm_task
            self.gripper_generation = generation
            self.gripper_state = 'running' if generation is not None else 'not_requested'
            self.task = ArmTask(arm_task.generation, arm_task.target, arm_task.deadline, arm_task.tolerance)
            return self.task

    def tick(self, gripper, sample):
        """只观察原执行器；夹爪使用自身限速、到位、超时和中性释放，不随 FJT 采样。"""
        with self.lock:
            if not self.active:
                return
            if self.task.generation != self.execution.generation:
                raise RuntimeError('combination execution generation changed')
            self.arm.update(sample.positions, sample.observed_at)
            arm = self.arm.snapshot(self.arm_task)
            self.task.result = replace(self.task.result, progress=arm.progress, desired=arm.desired)
            if arm.state not in ('running', 'succeeded'):
                raise RuntimeError(arm.message or 'arm execution failed')
            if self.gripper_generation is not None:
                state, reason = gripper.position_state(self.gripper_generation)
                self.gripper_state = state
                if state not in ('active', 'releasing', 'succeeded'):
                    raise RuntimeError(reason or 'independent gripper execution failed')
            if arm.state == 'succeeded' and self.gripper_state in ('succeeded', 'not_requested'):
                self.finish(self.task, 'succeeded', 'arm and independent gripper completed')

    def cancel_gripper(self, gripper):
        """取消只作用于本组合的夹爪代次；实际中性发送和静止仍由原周期确认。"""
        if self.gripper_generation is not None:
            gripper.cancel_position('combination stopped', expected_generation=self.gripper_generation)

    def stopping(self, task):
        """停止期间同时冻结本组合的六轴子任务，避免旧到位事件提前完成。"""
        with self.lock:
            if task is self.task and self.arm_task is not None:
                self.arm.stopping(self.arm_task)
            super().stopping(task)

    def finish(self, task, state, message):
        """原预约停止确认后终结子任务；迟到收尾不能覆盖后继任务。"""
        with self.lock:
            if task is self.task and self.arm_task is not None:
                self.arm.finish(self.arm_task, state, message)
            super().finish(task, state, message)

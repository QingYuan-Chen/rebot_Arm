"""控制器 ROS Action 协议适配.

职责与位置
    FJT 和夹爪位置 Action 的协议层; 不推进设备周期或计算轨迹.

对外接口
    ArmActions 注册回调, 提交数值请求, 发布缓存反馈并映射任务终态.

安全与并发约束
    准入检查不能替代设备线程中的再次检查; 取消携带任务身份或代次.
    六轴取消等待保持确认; 夹爪失败/释放状态按执行器查询映射, 不能把取消回执等同到位.

文件结构(按定义顺序)
    ArmActions: 将 ROS Action 协议映射到原组件的数值任务与查询端口, 不拥有执行状态.
"""

from __future__ import annotations
import time
from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from trajectory_msgs.msg import JointTrajectoryPoint

from .ros_ports import RuntimeStatePort, LifecycleStatePort, JointFeedbackPort, ArmCommandPort, GripperCommandPort


class ArmActions:
    """将 ROS Action 协议映射到原组件的数值任务与查询端口, 不拥有执行状态."""

    def __init__(self, node, namespace, *, runtime: RuntimeStatePort, lifecycle: LifecycleStatePort,
                 feedback: JointFeedbackPort, arm: ArmCommandPort, gripper: GripperCommandPort):
        """注入各职责组件并注册可重入 FJT 与夹爪 Action, 设备串行化由运行时负责."""
        self._node = node
        self._runtime, self._lifecycle = runtime, lifecycle
        self._feedback, self._arm, self._gripper = feedback, arm, gripper
        self._follow_joint_trajectory_server = ActionServer(
            node, FollowJointTrajectory, f'/{namespace}/follow_joint_trajectory',
            execute_callback=self.execute_follow_joint_trajectory,
            goal_callback=self.goal_callback, cancel_callback=self.cancel_follow_joint_trajectory,
            callback_group=node.reentrant_group)
        self._gripper_command_server = ActionServer(
            node, GripperCommand, f'/{namespace}/gripper/command',
            execute_callback=self.execute_gripper_command,
            goal_callback=self.goal_callback, cancel_callback=self.cancel_gripper_command,
            callback_group=node.reentrant_group)

    def goal_callback(self, _request):
        """只按当前 ready_for_motion 做第一层准入; 实际提交仍要在设备线程重新校验."""
        return GoalResponse.ACCEPT if self._lifecycle.ready_for_motion else GoalResponse.REJECT

    def cancel_follow_joint_trajectory(self, _goal):
        """接纳取消意图; 执行回调用自己的任务身份提交停止, 不在此停止全局任务."""
        # 只接纳取消意图; execute 回调使用自己的句柄提交, 避免误停其他任务.
        return CancelResponse.ACCEPT

    def cancel_gripper_command(self, _goal):
        """接纳取消意图; 执行回调随后携带本目标代次取消."""
        return CancelResponse.ACCEPT

    def execute_follow_joint_trajectory(self, goal_handle):
        """复制 ROS 轨迹为数值请求, 等待专属任务并发布缓存反馈.

        事件完成后按任务终态映射 succeeded / canceled / aborted; 末点已发送不等于到位.
        取消与异常仅撤销本句柄, 不能覆盖接替目标; 停止未确认时保留失败原因.
        本回调允许等待, 因此需要可重入组与仍在运行的多线程执行器.
        """
        result = FollowJointTrajectory.Result()
        task = None
        try:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.error_code = result.INVALID_GOAL
                result.error_string = 'canceled before submission'
                return result
            trajectory = goal_handle.request.trajectory
            names = list(trajectory.joint_names)
            points = [tuple(point.positions) for point in trajectory.points]
            times = [float(p.time_from_start.sec)+p.time_from_start.nanosec*1e-9
                     for p in trajectory.points]
            task = self._arm.submit_arm_trajectory(names, points, times)
            self._node.publish_arm_status()
            while not task.event.is_set():
                if goal_handle.is_cancel_requested:
                    self._arm.stop_active_motion(task)
                    break
                status = self._arm.arm_execution_status(task)
                pos, vel, _ = self._feedback.get_cached_joint_state()
                order = [self._runtime.joint_names.index(name) for name in names]
                feedback = FollowJointTrajectory.Feedback()
                feedback.joint_names = names
                feedback.actual = JointTrajectoryPoint()
                feedback.actual.positions = [float(pos[i]) for i in order]
                feedback.actual.velocities = [float(vel[i]) for i in order]
                if status.desired:
                    feedback.desired.positions = [status.desired[i] for i in order]
                    feedback.error.positions = [a-d for a,d in zip(
                        feedback.actual.positions, feedback.desired.positions)]
                goal_handle.publish_feedback(feedback)
                task.event.wait(.05)
            status = self._arm.arm_execution_status(task)
            result.error_string = status.message
            result.error_code = (result.SUCCESSFUL if status.state == 'succeeded' else
                result.GOAL_TOLERANCE_VIOLATED if status.state == 'tolerance' else result.INVALID_GOAL)
            if status.state == 'succeeded':
                goal_handle.succeed()
            elif status.state == 'canceled' and goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()
        except Exception as exc:
            # 观察/传输失败同样只撤销自己的目标; 失败不能无条件把全局状态复位.
            if task is not None and not task.event.is_set():
                try:
                    self._arm.stop_active_motion(task)
                except Exception as stop_error:
                    exc = RuntimeError(f'{exc}; stop unconfirmed: {stop_error}')
            result.error_code, result.error_string = result.INVALID_GOAL, str(exc)
            goal_handle.abort()
        self._node.publish_arm_status()
        return result

    def execute_gripper_command(self, goal_handle):
        """把米制开口与力矩交给夹爪组件, 以返回代次轮询本目标终态.

        取消携带 expected_generation, 旧回调不能释放接替目标. 位置成功由执行器在中性发送后给出.
        Action 的 stalled 固定为 False; 普通位置到位不能作为堵转抓取证据.
        """
        result = GripperCommand.Result()
        generation = None
        canceled = False
        try:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return result
            goal = goal_handle.request.command
            generation = self._gripper.set_gripper_target(goal.position, goal.max_effort)
            while True:
                if goal_handle.is_cancel_requested and not canceled:
                    self._gripper.cancel_gripper_position_command(
                        'gripper action canceled', expected_generation=generation)
                    canceled = True
                state, reason = self._gripper.gripper_execution_status(generation)
                result.position = self._gripper.gripper_position_m()
                result.effort = self._gripper.get_gripper_state()[2]
                result.reached_goal = state == 'succeeded'
                # 堵转不代表到位; 位置命令的到位/超时只由周期执行器判定.
                result.stalled = False
                feedback = GripperCommand.Feedback()
                feedback.position, feedback.effort = result.position, result.effort
                feedback.reached_goal = result.reached_goal
                goal_handle.publish_feedback(feedback)
                if state in ('succeeded', 'failed', 'superseded'):
                    if result.reached_goal:
                        goal_handle.succeed()
                    elif canceled and state == 'failed' and reason == 'gripper action canceled':
                        goal_handle.canceled()
                    else:
                        goal_handle.abort()
                    return result
                time.sleep(.05)
        except Exception as exc:
            if generation is not None:
                try:
                    self._gripper.cancel_gripper_position_command(str(exc), expected_generation=generation)
                except Exception:
                    pass
            self._node.get_logger().error(f'gripper action failed: {exc}')
            goal_handle.abort()
            return result

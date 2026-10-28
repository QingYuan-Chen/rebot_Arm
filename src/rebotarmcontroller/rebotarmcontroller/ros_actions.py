"""控制器 ROS Action 协议适配。

职责：注册唯一 FJT 与夹爪位置服务端，把 ROS 数值复制成运行时请求，发布
只读反馈并映射各任务终态。位姿 Action 由 motion 包提供，不在此计算 IK。

对外接口：``/<namespace>/follow_joint_trajectory``、``gripper/command``。
约束：默认失能；准入及起点/终点检查仍由硬件执行器完成。回调不推进轨迹、
不改生命周期、不写电机。取消携带自己的任务句柄/代次，确认保持或中性释放
后才报告 canceled；旧回调不能停止新目标。反馈轮询只影响 ROS 发布频率。
"""
import time
from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from trajectory_msgs.msg import JointTrajectoryPoint


class ArmActions:
    def __init__(self, node, hardware, namespace):
        self._node, self._hardware = node, hardware
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
        return GoalResponse.ACCEPT if self._hardware.ready_for_motion else GoalResponse.REJECT

    def cancel_follow_joint_trajectory(self, _goal):
        # 只接纳取消意图；execute 回调使用自己的句柄提交，避免误停其他任务。
        return CancelResponse.ACCEPT

    def cancel_gripper_command(self, _goal):
        return CancelResponse.ACCEPT

    def execute_follow_joint_trajectory(self, goal_handle):
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
            task = self._hardware.submit_arm_trajectory(names, points, times)
            self._node.publish_arm_status()
            while not task.event.is_set():
                if goal_handle.is_cancel_requested:
                    self._hardware.stop_active_motion(task)
                    break
                status = self._hardware.arm_execution_status(task)
                pos, vel, _ = self._hardware.get_cached_joint_state()
                order = [self._hardware.joint_names.index(name) for name in names]
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
            status = self._hardware.arm_execution_status(task)
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
            # 观察/传输失败同样只撤销自己的目标；失败不能无条件把全局状态复位。
            if task is not None and not task.event.is_set():
                try:
                    self._hardware.stop_active_motion(task)
                except Exception as stop_error:
                    exc = RuntimeError(f'{exc}; stop unconfirmed: {stop_error}')
            result.error_code, result.error_string = result.INVALID_GOAL, str(exc)
            goal_handle.abort()
        self._node.publish_arm_status()
        return result

    def execute_gripper_command(self, goal_handle):
        result = GripperCommand.Result()
        generation = None
        canceled = False
        try:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return result
            goal = goal_handle.request.command
            generation = self._hardware.set_gripper_target(goal.position, goal.max_effort)
            while True:
                if goal_handle.is_cancel_requested and not canceled:
                    self._hardware.cancel_gripper_position_command(
                        'gripper action canceled', expected_generation=generation)
                    canceled = True
                state, reason = self._hardware.gripper_execution_status(generation)
                result.position = self._hardware.gripper_position_m()
                result.effort = self._hardware.get_gripper_state()[2]
                result.reached_goal = state == 'succeeded'
                # 堵转不代表到位；位置命令的到位/超时只由周期执行器判定。
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
                    self._hardware.cancel_gripper_position_command(str(exc), expected_generation=generation)
                except Exception:
                    pass
            self._node.get_logger().error(f'gripper action failed: {exc}')
            goal_handle.abort()
            return result

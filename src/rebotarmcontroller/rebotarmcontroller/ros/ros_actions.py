"""控制器 ROS Action 协议适配.

职责与位置
    FJT 和夹爪位置 Action 的协议层; 不推进设备周期或计算轨迹.

对外接口
    ArmActions 注册回调, 提交数值请求, 发布缓存反馈并映射任务终态.
    同一公共 FJT 按配置选择 legacy 或 prepared_v1；后者异步调用内部准备服务，
    经原所有者复核后提交，准备失败不回退到 legacy。

安全与并发约束
    准入检查不能替代设备线程中的再次检查; 取消携带任务身份或代次.
    六轴取消回调先撤销许可并回接纳，任务终态仍等待保持确认；不能把取消回执等同停稳。

文件结构(按定义顺序)
    ArmActions: 将 ROS Action 协议映射到原组件的数值任务与查询端口, 不拥有执行状态.
"""

from __future__ import annotations
import json
import math
import time
from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from trajectory_msgs.msg import JointTrajectoryPoint
from rebotarm_msgs.action import ExecuteTeleopMotion

from ..contracts.ros_ports import RuntimeStatePort, LifecycleStatePort, JointFeedbackPort, ArmCommandPort, GripperCommandPort


class ArmActions:
    """将 ROS Action 协议映射到原组件的数值任务与查询端口, 不拥有执行状态."""

    def __init__(self, node, namespace, *, runtime: RuntimeStatePort, lifecycle: LifecycleStatePort,
                 feedback: JointFeedbackPort, arm: ArmCommandPort, gripper: GripperCommandPort,
                 trajectory_execution_mode='legacy'):
        """注入各职责组件并注册可重入 FJT 与夹爪 Action, 设备串行化由运行时负责."""
        self._node = node
        self._runtime, self._lifecycle = runtime, lifecycle
        self._feedback, self._arm, self._gripper = feedback, arm, gripper
        if trajectory_execution_mode not in ('legacy', 'prepared_v1'):
            raise ValueError('trajectory_execution_mode must be legacy or prepared_v1')
        self._trajectory_mode = trajectory_execution_mode
        # 保留请求对象本身，避免只用 id 时对象被回收重用而把迟到回调配到另一 Goal。
        self._pending_preparations = {}
        from std_msgs.msg import String
        from rclpy.qos import QoSProfile, DurabilityPolicy
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._capability_publisher = node.create_publisher(
            String, f'/{namespace}/trajectory_execution_mode', qos)
        self._capability_publisher.publish(String(data=trajectory_execution_mode))
        self._versioned_capabilities = node.create_publisher(
            String, f'/{namespace}/trajectory_capabilities', qos)
        from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
        self._diagnostics = node.create_publisher(DiagnosticArray, '/diagnostics', 10)

        def publish_admission():
            """只读数值缓存；标准诊断不替代 Action 因果信息，也不改变电机故障码。"""
            capability = self._arm.trajectory_capability(self._trajectory_mode)
            self._versioned_capabilities.publish(String(data=json.dumps(capability, allow_nan=False)))
            snapshot = self._arm.admission_snapshot()
            status = DiagnosticStatus(name=f'{namespace}/trajectory_admission', hardware_id='trajectory',
                level=DiagnosticStatus.OK if snapshot['ready'] else DiagnosticStatus.WARN,
                message='ready' if snapshot['ready'] else '; '.join(snapshot['reasons']))
            status.values = [KeyValue(key=key, value=json.dumps(value, ensure_ascii=False))
                for key,value in snapshot.items()]
            message = DiagnosticArray()
            message.header.stamp = node.get_clock().now().to_msg()
            message.status = [status]
            self._diagnostics.publish(message)
        self._diagnostic_timer = node.create_timer(.1, publish_admission)
        from rebotarm_msgs.srv import GetArmPlanningState
        def get_planning_state(_request, response):
            """只读服务走所有者设备队列；服务成功不授予执行资源。"""
            try:
                state = self._arm.planning_snapshot(self._trajectory_mode)
                response.state_json = json.dumps(state, allow_nan=False)
                response.success, response.message = True, 'ready' if state['executable'] else 'preview only: disabled'
            except (ValueError, RuntimeError) as exc:
                response.success, response.message = False, str(exc)
            return response
        self._planning_state_service = node.create_service(GetArmPlanningState,
            f'/{namespace}/get_arm_planning_state', get_planning_state,
            callback_group=node.reentrant_group)
        if trajectory_execution_mode == 'prepared_v1':
            from rebotarm_msgs.srv import PrepareArmTrajectory
            self._prepare_type = PrepareArmTrajectory
            self._prepare_client = node.create_client(PrepareArmTrajectory,
                f'/{namespace}/prepare_arm_trajectory', callback_group=node.reentrant_group)
            node.create_subscription(String, f'/{namespace}/trajectory_scene_revision',
                lambda message: self._arm.update_trajectory_scene(message.data), qos,
                callback_group=node.reentrant_group)
        self._follow_joint_trajectory_server = ActionServer(
            node, FollowJointTrajectory, f'/{namespace}/follow_joint_trajectory',
            execute_callback=self.execute_follow_joint_trajectory,
            goal_callback=self.trajectory_goal_callback, cancel_callback=self.cancel_follow_joint_trajectory,
            callback_group=node.reentrant_group)
        self._gripper_command_server = ActionServer(
            node, GripperCommand, f'/{namespace}/gripper/command',
            execute_callback=self.execute_gripper_command,
            goal_callback=self.goal_callback, cancel_callback=self.cancel_gripper_command,
            callback_group=node.reentrant_group)
        self._moveit_gripper_server = ActionServer(
            node, GripperCommand, f'/{namespace}/gripper/moveit_command',
            execute_callback=lambda goal: self.execute_gripper_command(goal, moveit=True),
            goal_callback=self.goal_callback, cancel_callback=self.cancel_gripper_command,
            callback_group=node.reentrant_group)
        if trajectory_execution_mode == 'prepared_v1':
            self._teleop_server = ActionServer(
                node, ExecuteTeleopMotion, f'/{namespace}/execute_teleop_motion',
                execute_callback=lambda goal: self.execute_follow_joint_trajectory(goal, web=True),
                goal_callback=lambda request: self.trajectory_goal_callback(request, web=True),
                cancel_callback=self.cancel_follow_joint_trajectory, callback_group=node.reentrant_group)

    def goal_callback(self, _request):
        """只按当前 ready_for_motion 做第一层准入; 实际提交仍要在设备线程重新校验."""
        return (GoalResponse.ACCEPT if self._lifecycle.ready_for_motion
                and not (self._trajectory_mode == 'prepared_v1' and self._runtime.arm_resource_reserved)
                else GoalResponse.REJECT)

    def trajectory_goal_callback(self, request, *, web=False):
        """准备路径先占用资源；忙时直接拒绝，不能积累尚未执行的 Goal。"""
        received = time.monotonic()
        try:
            if (len(request.trajectory.joint_names) != 6
                    or set(request.trajectory.joint_names) != set(self._runtime.joint_names)):
                raise ValueError('FJT requires exactly joint1..joint6; use independent gripper command')
            if self._trajectory_mode == 'legacy':
                return self.goal_callback(request)
            if not self._lifecycle.ready_for_motion:
                snapshot = self._arm.admission_snapshot()
                raise RuntimeError('controller not ready: '+ '; '.join(snapshot['reasons']))
            if not self._prepare_client.service_is_ready():
                raise RuntimeError('trajectory preparation service unavailable')
            last = request.trajectory.points[-1].time_from_start
            duration = last.sec+last.nanosec*1e-9
            if not math.isfinite(duration) or not 0 <= duration <= 300.:
                raise ValueError(f'trajectory duration={duration}, required range=[0,300] s')
            # 从收到 Goal 而非服务调用开始计时；准备消耗也必须留在 MoveIt 的总预算内。
            deadline = received+min(2., .2*duration+.25)
            command = ((request.gripper_position, request.gripper_max_effort)
                       if web and request.move_gripper else ())
            from rebotarm_trajectory.planning import decode_plan
            from rebotarm_trajectory.polynomial import TrajectoryPoint
            points = tuple(TrajectoryPoint(p.time_from_start.sec+p.time_from_start.nanosec*1e-9,
                p.positions, p.velocities, p.accelerations) for p in request.trajectory.points)
            planning = decode_plan(request.trajectory.header.frame_id, request.trajectory.joint_names,
                points, capability_revision=self._arm.trajectory_capability(self._trajectory_mode)['revision'],
                require_execution=True)
            options = dict(web=web, gripper_command=command)
            if planning is not None:
                options['planning'] = planning
            token = self._arm.reserve_arm_preparation(deadline, **options)
            # Jazzy 将同一个请求交给 GoalHandle；按请求绑定，停止后新 Goal 不能串用旧令牌。
            self._pending_preparations[id(request)] = (request, token)
            return GoalResponse.ACCEPT
        except (ValueError, RuntimeError, IndexError, AttributeError, TypeError) as exc:
            if hasattr(self._node, 'get_logger'):
                self._node.get_logger().warning(f'FJT goal rejected: {exc}')
            return GoalResponse.REJECT

    def _prepare_follow_goal(self, goal_handle, token):
        """服务等待和数值解码均在 ROS 调用线程；设备周期继续发送原保持目标。"""
        request = self._prepare_type.Request()
        request.trajectory = goal_handle.request.trajectory
        request.session, request.generation = token.session, token.generation
        request.hold_positions, request.feedback_positions = list(token.hold), list(token.feedback)
        request.feedback_observed_at = token.observed_at
        request.stationary = False  # 兼容字段；机械臂准入不再声称已有静止证据。
        request.enabled = True
        request.gripper_positions = list(token.gripper_positions)
        request.gripper_generation = token.gripper_generation
        request.gripper_collision_policy = token.collision_policy
        request.scene_revision, request.deadline_monotonic = token.scene_revision, token.deadline
        request.capability_json = json.dumps(self._arm.trajectory_capability(self._trajectory_mode), allow_nan=False)
        future = self._prepare_client.call_async(request)
        while not future.done():
            if goal_handle.is_cancel_requested or time.monotonic() >= token.deadline:
                future.cancel()
                # 取消 Future 不保证远端计算立即结束；令牌与期限复核才阻止迟到结果发送。
                raise RuntimeError('trajectory preparation canceled or timed out')
            time.sleep(.002)
        response = future.result()
        if not response.success:
            raise ValueError(response.message)
        handoff = self._arm.decode_prepared_arm_trajectory(json.loads(response.prepared_json))
        from rebotarm_trajectory.planning import decode_plan
        decode_plan(request.trajectory.header.frame_id, handoff.curve.joint_names, handoff.curve.points,
            capability_revision=token.capability, require_execution=True)
        if goal_handle.is_cancel_requested:
            raise RuntimeError('trajectory canceled before atomic admission')
        # 最终复核在唯一设备线程中完成，不能用回调线程刚刚看到的状态代替它。
        return self._arm.admit_prepared_arm_trajectory(token, handoff)

    def cancel_follow_joint_trajectory(self, goal):
        """准备路径先撤销本 Goal 令牌；旧路径由执行回调按任务身份停止。"""
        if self._trajectory_mode == 'prepared_v1':
            pending = self._pending_preparations.get(id(goal.request))
            if pending is not None and pending[0] is goal.request:
                try:
                    # 撤销所有者令牌先于取消回执，已排队的接纳也不能越过取消。
                    self._arm.cancel_arm_preparation(pending[1], wait_for_stop=False)
                except RuntimeError as exc:
                    self._node.get_logger().error(f'trajectory cancel stop unconfirmed: {exc}')
        return CancelResponse.ACCEPT

    def cancel_gripper_command(self, _goal):
        """接纳取消意图; 执行回调随后携带本目标代次取消."""
        return CancelResponse.ACCEPT

    def execute_follow_joint_trajectory(self, goal_handle, *, web=False):
        """复制 ROS 轨迹为数值请求, 等待专属任务并发布缓存反馈.

        事件完成后按任务终态映射 succeeded / canceled / aborted; 末点已发送不等于到位.
        取消与异常仅撤销本句柄, 不能覆盖接替目标; 停止未确认时保留失败原因.
        本回调允许等待, 因此需要可重入组与仍在运行的多线程执行器.
        """
        result = ExecuteTeleopMotion.Result() if web else FollowJointTrajectory.Result()
        task = None
        pending = (self._pending_preparations.get(id(goal_handle.request))
            if self._trajectory_mode == 'prepared_v1' else None)
        token = pending[1] if pending is not None and pending[0] is goal_handle.request else None
        try:
            if goal_handle.is_cancel_requested:
                if token is not None:
                    self._arm.cancel_arm_preparation(token)
                goal_handle.canceled()
                result.error_code = result.INVALID_GOAL
                result.error_string = 'canceled before submission'
                return result
            trajectory = goal_handle.request.trajectory
            names = list(trajectory.joint_names)
            points = [tuple(point.positions) for point in trajectory.points]
            times = [float(p.time_from_start.sec)+p.time_from_start.nanosec*1e-9
                     for p in trajectory.points]
            if self._trajectory_mode == 'prepared_v1':
                if token is None:
                    raise RuntimeError('trajectory was not reserved')
                if web:
                    goal_handle.publish_feedback(ExecuteTeleopMotion.Feedback(
                        stage='preparing',gripper_state='not_started',
                        gripper_position=self._gripper.gripper_position_m()))
                task = self._prepare_follow_goal(goal_handle, token)
            else:
                task = self._arm.submit_arm_trajectory(names, points, times)
            self._node.publish_arm_status()
            while not task.event.is_set():
                if goal_handle.is_cancel_requested:
                    self._arm.stop_active_motion(task)
                    break
                status = self._arm.arm_execution_status(task)
                pos, vel, _ = self._feedback.get_cached_joint_state()
                actual = dict(zip(self._runtime.joint_names, zip(pos, vel)))
                desired = dict(zip(self._runtime.joint_names, status.desired))
                feedback = ExecuteTeleopMotion.Feedback() if web else FollowJointTrajectory.Feedback()
                if web:
                    feedback.stage = 'stopping' if status.state == 'stopping' else 'executing'
                    feedback.arm_progress = status.progress
                    _, feedback.gripper_state = self._arm.teleop_execution_status(task)
                    feedback.gripper_position = self._gripper.gripper_position_m()
                else:
                    feedback.joint_names = names
                feedback.actual = JointTrajectoryPoint()
                feedback.actual.positions = [float(actual[name][0]) for name in names]
                feedback.actual.velocities = [float(actual[name][1]) for name in names]
                if status.desired:
                    feedback.desired.positions = [desired[name] for name in names]
                    last = goal_handle.request.trajectory.points[-1].time_from_start
                    elapsed_ns = round(status.progress*(last.sec*1_000_000_000+last.nanosec))
                    feedback.desired.time_from_start.sec, feedback.desired.time_from_start.nanosec = divmod(elapsed_ns,1_000_000_000)
                    if not web:
                        feedback.error.positions = [a-d for a,d in zip(
                            feedback.actual.positions, feedback.desired.positions)]
                goal_handle.publish_feedback(feedback)
                task.event.wait(.05)
            status = self._arm.arm_execution_status(task)
            if web:
                result.arm_state, result.gripper_state = self._arm.teleop_execution_status(task)
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
            stopped = True
            if token is not None:
                try:
                    self._arm.cancel_arm_preparation(token)
                except Exception as stop_error:
                    stopped = False
                    exc = RuntimeError(f'{exc}; stop unconfirmed: {stop_error}')
            result.error_code, result.error_string = result.INVALID_GOAL, str(exc)
            if web:
                result.arm_state, result.gripper_state = ('failed', 'not_started') if task is None else self._arm.teleop_execution_status(task)
            if token is not None and task is None and goal_handle.is_cancel_requested and stopped:
                goal_handle.canceled()
            else:
                goal_handle.abort()
        finally:
            # 清理 ROS 请求登记与释放设备预约是两件事；后者必须由原所有者确认停止。
            if pending is not None:
                self._pending_preparations.pop(id(goal_handle.request), None)
            if token is not None and task is not None and task.result.state != 'succeeded':
                try:
                    self._arm.cancel_arm_preparation(token)
                except Exception as stop_error:
                    # 未确认停止仍由所有者保留资源，清理异常不能吞掉原 Action 结果。
                    result.error_string += f'; stop unconfirmed: {stop_error}'
        self._node.publish_arm_status()
        return result

    def execute_gripper_command(self, goal_handle, *, moveit=False):
        """把毫米制开口与力矩交给夹爪组件, 以返回代次轮询本目标终态.

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
            generation = self._gripper.set_gripper_target(goal.position * (2. if moveit else 1.), goal.max_effort)
            while True:
                if goal_handle.is_cancel_requested and not canceled:
                    self._gripper.cancel_gripper_position_command(
                        'gripper action canceled', expected_generation=generation)
                    canceled = True
                state, reason = self._gripper.gripper_execution_status(generation)
                result.position = self._gripper.gripper_position_m() / (2. if moveit else 1.)
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

"""旧位姿 ROS 接口的运动层所有者。

职责：保留 move_to_pose Action 和 move_to_pose_ik Service，执行规范模型
IK/笛卡尔采样、动态限位重定时和 MoveIt 碰撞检查，再提交唯一 FJT 服务端。
服务 success 仅表示 FJT 已接纳；Action 等待 FJT 到位终态。这里不自动使能，
不访问硬件实现。缺新鲜状态、碰撞结论未知、重复服务端均拒绝执行。

并发：一个兼容位姿请求占用本节点入口；规划、接纳、执行、停止均持有独立
记录。超时的 send future 不能遗忘，迟到接纳会被取消，停止确认前继续阻断
新请求。取消要求 FJT 终态及新鲜静止反馈，不能把已发送取消当作已经停住。
"""
import math
import logging
import threading
import time
from dataclasses import dataclass, field, replace
import numpy as np
import pinocchio as pin
import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetStateValidity
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse, get_action_server_names_and_types_by_node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rebotarm_msgs.action import MoveToPose
from rebotarm_msgs.srv import MoveToPoseIK
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from .pose_kinematics import JOINT_NAMES, PoseKinematics, model_resource
from .cartesian_trajectory import plan_cartesian, retime_path
from .collision_precheck import CollisionPrechecker, CollisionPrecheckConfig
from .feedback_receiver import MotionFeedbackReceiver
from .paired_trajectory_protocol import sample_command
from .trajectory_runtime_limits import TrajectoryRuntimeLimitGuard, load_joint_runtime_limits


@dataclass
class PoseRun:
    epoch: int
    deadline: float
    cancel: bool = False
    dispatched: float = 0.
    dispatched_ns: int = 0
    command: object = None
    send: object = None
    handle: object = None
    result: object = None
    cancel_future: object = None
    error: str = ''
    terminal: object = None
    guard: object = None
    observed_stamp: int = 0
    stopping_since: float | None = None
    stop_epoch: int | None = None
    done: threading.Event = field(default_factory=threading.Event)


class PoseCompatNode(Node):
    def __init__(self):
        super().__init__('pose_compat')
        self.declare_parameter('arm_namespace', 'rebotarm')
        self.declare_parameter('moveit_group_name', 'arm')
        self.declare_parameter('state_validity_service', '/check_state_validity')
        self.namespace = str(self.get_parameter('arm_namespace').value).strip('/')
        self.group = ReentrantCallbackGroup()
        self.lock = threading.RLock()
        self.kinematics = PoseKinematics()
        limits = load_joint_runtime_limits(model_resource('config/joint_limits.yaml'), JOINT_NAMES)
        self.runtime_limits = {name: replace(limits[name], max_effort=float(self.kinematics.model.effortLimit[i]))
                               for i, name in enumerate(JOINT_NAMES)}
        self.feedback = None
        self.gripper = None
        self.status = None
        self.status_at = 0.
        self.epoch = 0
        self.active = None
        self.action_name = f'/{self.namespace}/follow_joint_trajectory'
        self.action = ActionClient(self, FollowJointTrajectory, self.action_name, callback_group=self.group)
        self.validity = self.create_client(GetStateValidity,
            str(self.get_parameter('state_validity_service').value), callback_group=self.group)
        self.checker = CollisionPrechecker(client=self.validity, request_factory=GetStateValidity.Request)
        self.server = ActionServer(self, MoveToPose, f'/{self.namespace}/move_to_pose',
            execute_callback=self._execute, goal_callback=self._goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=self.group)
        self.create_service(MoveToPoseIK, f'/{self.namespace}/move_to_pose_ik', self._ik,
                            callback_group=self.group)
        self.create_timer(.02, self._monitor, callback_group=self.group)
        # 接收线程不获取 self.lock；同步规划和动作等待不能阻塞反馈缓存。
        self._feedback_receiver = MotionFeedbackReceiver(self, self.namespace, gripper=True)
        self.stop_feedback = self._feedback_receiver.stop_feedback

    def _refresh_feedback(self):
        snapshot = self._feedback_receiver.snapshot()
        self.feedback, self.gripper = snapshot.joints, snapshot.gripper
        self.status, self.status_at, self.epoch = snapshot.status, snapshot.status_at, snapshot.epoch

    def destroy_node(self):
        self._feedback_receiver.close()
        return super().destroy_node()

    def _fresh(self, msg, age=.15):
        if msg is None:
            return False
        stamp = msg.header.stamp.sec*1_000_000_000 + msg.header.stamp.nanosec
        return stamp > 0 and -.05 <= (self.get_clock().now().nanoseconds-stamp)/1e9 <= age

    def _positions(self):
        if not self._fresh(self.feedback):
            raise RuntimeError('fresh joint feedback required')
        indices = [list(self.feedback.name).index(n) for n in JOINT_NAMES]
        return self.kinematics.joints([self.feedback.position[i] for i in indices])

    def _ready(self):
        self._refresh_feedback()
        status = self.status
        if (status is None or time.monotonic()-self.status_at > .6 or not status.enabled
                or not status.control_loop_active or status.error_codes
                or len(status.per_joint_status_code) != 6
                or any(value != 1 for value in status.per_joint_status_code)):
            raise RuntimeError('explicit enable and fresh healthy controller state required')
        return self._positions()

    def _goal(self, _request):
        with self.lock:
            try:
                self._ready()
                return GoalResponse.ACCEPT if self.active is None else GoalResponse.REJECT
            except Exception:
                return GoalResponse.REJECT

    def _begin(self):
        with self.lock:
            self._ready()
            if self.active is not None:
                raise RuntimeError('pose request busy or stop unconfirmed')
            if self.status.state_machine != 'IDLE':
                raise RuntimeError('arm resource is not idle')
            run = PoseRun(self.epoch, time.monotonic()+30.)
            run.guard = TrajectoryRuntimeLimitGuard(self.runtime_limits, derivative_policy='diagnostic')
            self.active = run
            return run

    def _check_run(self, run, canceled=lambda: False):
        with self.lock:
            current = self._ready()
            if (run.cancel or canceled() or run.epoch != self.epoch
                    or time.monotonic() > run.deadline or self.active is not run):
                run.cancel = True
                raise RuntimeError('pose request canceled, expired or controller state changed')
            return current

    @staticmethod
    def _target(msg):
        quaternion = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        position = np.array([msg.position.x, msg.position.y, msg.position.z])
        if not np.isfinite(quaternion).all() or not np.isfinite(position).all() or abs(np.linalg.norm(quaternion)-1.) > 1e-3:
            raise ValueError('finite position and unit quaternion required')
        return pin.SE3(pin.Quaternion(quaternion/np.linalg.norm(quaternion)).matrix(), position)

    def _prepare(self, run, target_pose, duration, cartesian, canceled=lambda: False):
        seed = self._check_run(run, canceled)
        target = self._target(target_pose)
        if cartesian:
            path, duration = plan_cartesian(self.kinematics, seed, target, duration)
        else:
            solution = self.kinematics.solve(target, seed)
            if not solution.success:
                raise ValueError(f'IK failed: residual {solution.error:.4e}')
            path, duration = [seed, solution.q], 1.
        self._check_run(run, canceled)
        command = retime_path(self.kinematics, path, duration)
        # 检查全部输出点，并对稀疏关节段补齐不超过 0.02 rad 的碰撞采样。
        samples = [tuple(command['points'][0]['positions'])]
        for point in command['points'][1:]:
            previous, current = np.asarray(samples[-1]), np.asarray(point['positions'])
            count = max(1, math.ceil(float(np.max(np.abs(current-previous)))/.02))
            samples.extend(tuple(previous+(current-previous)*i/count) for i in range(1, count+1))
        with self.lock:
            self._refresh_feedback()
            if not self._fresh(self.gripper):
                raise RuntimeError('fresh gripper feedback required for collision state')
            opening = float(self.gripper.position)
            if not math.isfinite(opening) or not 0. <= opening <= .09 or self.gripper.status_code != 1:
                raise RuntimeError('invalid gripper opening/status for collision state')
        config = CollisionPrecheckConfig(True, self.validity.srv_name,
            str(self.get_parameter('moveit_group_name').value), len(samples),
            max(.1, run.deadline-time.monotonic()),
            (('left_finger_joint', opening/2), ('right_finger_joint', -opening/2)))
        checked = self.checker.check_positions(joint_names=JOINT_NAMES,
            positions_list=samples, config=config)
        if checked['state'] != 'pass':
            raise RuntimeError(f"collision check {checked['state']}: {checked['message']}")
        current = self._check_run(run, canceled)
        if np.max(np.abs(current-seed)) > .10:
            raise RuntimeError('joint state changed during planning')
        if not self.action.wait_for_server(timeout_sec=.5):
            raise RuntimeError('FJT server unavailable')
        owners = sum(self.action_name == name for node, namespace in self.get_node_names_and_namespaces()
            for name, _ in get_action_server_names_and_types_by_node(self, node, namespace))
        if owners != 1:
            raise RuntimeError(f'expected one FJT server, found {owners}')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINT_NAMES)
        for value in command['points']:
            point = JointTrajectoryPoint()
            point.positions = list(value['positions'])
            stamp = round(value['elapsed_sec']*1e9)
            point.time_from_start = Duration(sec=stamp//1_000_000_000, nanosec=stamp%1_000_000_000)
            goal.trajectory.points.append(point)
        goal.trajectory.points[-1].velocities = [0.]*6
        with self.lock:
            self._check_run(run, canceled)
            run.command, run.dispatched = command, time.monotonic()
            run.dispatched_ns = self.get_clock().now().nanoseconds
            run.send = self.action.send_goal_async(goal)
        return path[-1]

    def _monitor(self):
        """处理接纳/结果及迟到取消；只有完成或确认停止才释放入口。"""
        with self.lock:
            run = self.active
            if run is None or run.send is None:
                return
            now = time.monotonic()
            try:
                if run.handle is None and run.send.done():
                    run.handle = run.send.result()
                    if run.handle is None or not run.handle.accepted:
                        self._finish(run, 'FJT rejected')
                        return
                    run.result = run.handle.get_result_async()
                if run.result is not None and run.result.done():
                    run.terminal = run.result.result()
                    if not run.cancel:
                        # 终态送达前的反馈空档仍使旧请求失效，不能被恢复后的新帧覆盖。
                        self._ready()
                        if run.epoch != self.epoch:
                            raise RuntimeError('controller state or feedback continuity changed')
                        self._finish(run)
                        return
                if run.handle is None and now-run.dispatched > 3.:
                    run.cancel, run.error = True, 'FJT acceptance timeout; draining late goal'
                feedback_ready = False
                try:
                    current = self._ready()
                    feedback_ready = True
                    stamp = self.feedback.header.stamp.sec*1_000_000_000 + self.feedback.header.stamp.nanosec
                    if run.handle is not None and stamp > run.observed_stamp and not run.cancel:
                        # 源时间与指令起点同属 ROS 时钟，不能用到达时刻比较旧反馈。
                        elapsed = max(0., (stamp-run.dispatched_ns)/1e9)
                        desired = sample_command(run.command, elapsed)
                        tracking_error = float(np.max(np.abs(current-desired)))
                        indices = [list(self.feedback.name).index(n) for n in JOINT_NAMES]
                        violation = run.guard.observe(joint_names=JOINT_NAMES,
                            velocities=[self.feedback.velocity[i] for i in indices],
                            efforts=[self.feedback.effort[i] for i in indices], elapsed=stamp/1e9)
                        run.observed_stamp = stamp
                        logging.getLogger('rebotarm.audit').info('pose_guard_sample', extra={'audit': dict(
                            epoch=run.epoch, stamp_ns=stamp, dispatched_ns=run.dispatched_ns,
                            elapsed=elapsed, positions=current.tolist(), desired=list(desired),
                            velocities=[float(self.feedback.velocity[i]) for i in indices],
                            efforts=[float(self.feedback.effort[i]) for i in indices],
                            tracking_error=tracking_error, acceleration=run.guard.accelerations,
                            jerk=run.guard.jerks, derivative_policy=run.guard.derivative_policy,
                            derivative_diagnostics=[vars(value) for value in run.guard.derivative_diagnostics],
                            violation=None if violation is None else vars(violation))})
                        if tracking_error > .2:
                            raise RuntimeError('pose execution tracking error exceeds 0.20 rad')
                        if violation is not None:
                            raise RuntimeError(f'pose execution dynamic limit: {violation}')
                    if run.epoch != self.epoch:
                        raise RuntimeError('controller state changed')
                    if now-run.dispatched > run.command['duration_sec']+5.:
                        raise RuntimeError('FJT result timeout')
                except Exception as exc:
                    run.cancel = True
                    run.error = run.error or str(exc)
                if run.cancel:
                    if run.handle is not None and run.cancel_future is None:
                        run.cancel_future = run.handle.cancel_goal_async()
                    # 已知动作终态后才建立停止证据。若反馈代次再变化，旧动作
                    # 错误保持锁存，另建保持确认；不能把旧动作恢复为成功。
                    if run.terminal is not None and feedback_ready and run.stop_epoch != self.epoch:
                        logging.getLogger('rebotarm.audit').info('pose_stop_observation', extra={'audit': dict(
                            run_epoch=run.epoch, stop_epoch=self.epoch, previous_stop_epoch=run.stop_epoch,
                            since=now, error=run.error)})
                        run.stopping_since, run.stop_epoch = now, self.epoch
                    if (run.terminal is not None and run.stopping_since is not None
                            and self.stop_feedback.stationary(
                            run.stopping_since, now, self.get_clock().now().nanoseconds)):
                        self._finish(run, run.error or 'canceled; stationary feedback confirmed')
            except Exception as exc:
                # 不丢弃未知状态的目标；下一轮继续排空，入口保持阻断。
                run.cancel = True
                run.error = run.error or str(exc)

    def _finish(self, run, error=''):
        run.error = error
        run.done.set()
        if self.active is run:
            self.active = None

    def _abandon(self, run, error):
        with self.lock:
            if run.send is None:
                self._finish(run, str(error))
            else:
                run.cancel, run.error = True, str(error)

    def _ik(self, request, response):
        run = None
        try:
            run = self._begin()
            solution = self._prepare(run, request.target_pose, 0., False)
            deadline = time.monotonic()+3.
            while run.handle is None and not run.done.is_set() and time.monotonic() < deadline:
                time.sleep(.01)
            if run.handle is None or not run.handle.accepted or run.cancel or run.error:
                raise RuntimeError(run.error or 'FJT acceptance unconfirmed')
            response.success, response.message = True, 'FJT 已接纳；不表示已到位'
            response.q_solution = [float(v) for v in solution]
        except Exception as exc:
            if run is not None:
                self._abandon(run, exc)
            response.success, response.message, response.q_solution = False, str(exc), []
        return response

    def _pose_message(self):
        self._refresh_feedback()
        transform = self.kinematics.pose(self._positions())
        msg = Pose()
        msg.position.x, msg.position.y, msg.position.z = map(float, transform.translation)
        q = pin.Quaternion(transform.rotation).coeffs()
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = map(float, q)
        return msg

    def _execute(self, goal):
        result, run = MoveToPose.Result(), None
        try:
            run = self._begin()
            self._prepare(run, goal.request.target_pose, goal.request.duration, True,
                          lambda: goal.is_cancel_requested)
            while not run.done.wait(.05):
                if goal.is_cancel_requested:
                    with self.lock:
                        run.cancel = True
                feedback = MoveToPose.Feedback()
                with self.lock:
                    self._refresh_feedback()
                    if not self._fresh(self.feedback):
                        continue
                    feedback.current_pose = self._pose_message()
                feedback.time_elapsed = time.monotonic()-run.dispatched
                feedback.progress = min(1., feedback.time_elapsed/run.command['duration_sec'])
                goal.publish_feedback(feedback)
            terminal = run.terminal
            result.success = bool(not run.error and terminal is not None
                and terminal.status == GoalStatus.STATUS_SUCCEEDED
                and terminal.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL)
            result.message = run.error or (terminal.result.error_string if terminal else 'no execution result')
            with self.lock:
                result.final_pose = self._pose_message()
            if result.success:
                goal.succeed()
            elif goal.is_cancel_requested and run.done.is_set():
                goal.canceled()
            else:
                goal.abort()
        except Exception as exc:
            if run is not None:
                self._abandon(run, exc)
            result.success, result.message = False, str(exc)
            if goal.is_cancel_requested and run is not None and run.send is None:
                goal.canceled()
            else:
                goal.abort()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = PoseCompatNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()

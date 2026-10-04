"""RViz 轨迹插值预演控制器（不加载物理引擎）。

职责与位置：
    本节点属于 rebotarm_preview 的轻量预演后端。真实系统上 ``follow_joint_trajectory`` 由运动规划执行层
    驱动真实电机；在仿真/演示场景下本节点顶替这一角色，直接按目标轨迹在内存里做线性插值，
    以固定频率发布关节状态，让 RViz 中的模型动起来。

    本节点不加载任何物理引擎、不做动力学与力矩计算、不检查碰撞，也绝不接触真实电机驱动，
    因此它只能用于可视化与上层流程联调，不能作为物理正确性的证据。

对外接口（``arm_namespace`` 默认 ``rebotarm``，下列话题/服务均自动加前缀）：
    动作服务端 ``/<ns>/follow_joint_trajectory``：接收关节轨迹目标，执行期间周期性发反馈；
    发布 ``/<ns>/joint_states``：8 个关节（6 个臂关节 + 2 个手指关节）的位置/速度，effort 恒为 0；
    发布 ``/<ns>/gripper/state``：夹爪开口宽度（米），用关节状态消息承载；
    服务 ``/<ns>/trajectory_stop``：请求停止当前轨迹（等价于取消）；
    服务 ``/<ns>/safe_home``：从任意停止姿态平滑回到统一 safe_home；
    服务 ``/<ns>/gripper/set``：直接设置夹爪开口宽度并立即刷新状态。

关键流程与状态：
    ``_goal_callback`` 只拒绝空轨迹；``_execute_goal`` 用单调时钟推进插值，直到轨迹末端或被取消；
    订阅端（上层流程）可随时通过取消或停止服务打断。轨迹点时间戳非法时会按默认总时长做均匀回退。

线程模型：
    ``main`` 使用多线程执行器（3 线程）：动作执行回调内部会 sleep 等待轨迹推进，
    若单线程执行器会阻塞定时器与服务回调，因此必须多线程。
    共享的关节位置/速度字典由 ``self._lock`` 保护，取消标志使用 ``threading.Event``。
"""

from __future__ import annotations

import bisect
import threading
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from rebotarm_msgs.msg import JointMotorState
from rebotarm_msgs.srv import SetGripper
from trajectory_msgs.msg import JointTrajectory

from .preview_config import SAFE_HOME_JOINT_POSITIONS
from .sim_gripper import gripper_joint_positions_for_width


def _duration_to_sec(duration: Duration) -> float:
    """把 ROS 时间消息（秒 + 纳秒）换算为浮点秒。

    纳秒字段占 1e-9 秒权重；轨迹点的 ``time_from_start`` 即用此函数转为相对起点的秒数。
    """
    return float(duration.sec) + float(duration.nanosec) * 1e-9


class SimTrajectoryControllerNode(Node):
    """仅用于 RViz 的 FollowJointTrajectory 服务端，负责把完整关节状态“演”出来。

    生命周期：节点构造时声明参数、建立发布器/动作服务端/服务与定时器；``main`` 中自旋，
    收到关闭信号后销毁节点。整个生命周期内不进行物理仿真，只维护一份“当前关节位置”内存状态。
    """

    def __init__(self) -> None:
        super().__init__("rebotarm_sim_trajectory_controller")
        # joint_names：状态发布与轨迹匹配使用的关节名列表，顺序即发布顺序；
        # 默认 6 个臂关节 + 2 个手指关节，改名需与模型/上层接口保持一致。
        self.declare_parameter(
            "joint_names",
            [
                "joint1",
                "joint2",
                "joint3",
                "joint4",
                "joint5",
                "joint6",
                "left_finger_joint",
                "right_finger_joint",
            ],
        )
        # arm_namespace：所有话题/服务/动作的命名空间前缀（会去掉首尾斜杠）。
        self.declare_parameter("arm_namespace", "rebotarm")
        # publish_rate_hz：关节状态发布频率（Hz），同时决定轨迹执行时的插值步长。
        self.declare_parameter("publish_rate_hz", 30.0)
        # default_duration_sec：轨迹点缺少合法时间戳时的默认总时长（秒）。
        self.declare_parameter("default_duration_sec", 2.0)
        # gripper_max_width_m / gripper_min_width_m：夹爪开口宽度的物理上下限（米）。
        self.declare_parameter("gripper_max_width_m", 0.09)
        self.declare_parameter("gripper_min_width_m", 0.0)
        # initial_joint_positions：启动时的臂关节初始位置（弧度），按 joint1..joint6 顺序，
        # 只覆盖前 6 个；手指关节位置默认保持 0（即完全闭合）。
        self.declare_parameter("initial_joint_positions", list(SAFE_HOME_JOINT_POSITIONS))

        self._arm_namespace = str(self.get_parameter("arm_namespace").value).strip("/")
        self._joint_names = [str(v) for v in list(self.get_parameter("joint_names").value)]
        # 频率与时长做下限裁剪，避免参数误设导致忙循环或零时长轨迹
        self._publish_rate_hz = max(float(self.get_parameter("publish_rate_hz").value), 1.0)
        self._default_duration_sec = max(float(self.get_parameter("default_duration_sec").value), 0.1)
        self._gripper_max_width_m = max(float(self.get_parameter("gripper_max_width_m").value), 0.0)
        self._gripper_min_width_m = max(float(self.get_parameter("gripper_min_width_m").value), 0.0)
        self._positions_by_name = {name: 0.0 for name in self._joint_names}
        for name, position in zip(self._joint_names, list(self.get_parameter("initial_joint_positions").value)):
            self._positions_by_name[str(name)] = float(position)
        self._velocities_by_name = {name: 0.0 for name in self._joint_names}
        # 最近一次设置的夹爪开口宽度（米），定时器每周期都会把它作为夹爪状态重发
        self._last_gripper_width_m = self._gripper_min_width_m
        # 位置/速度字典会被动作执行线程、服务回调与定时器并发访问，必须加锁
        self._lock = threading.Lock()
        # 置位表示“请求停止当前轨迹”：取消回调与停止服务都会置位，执行回调每周期检查并清除
        self._stop_requested = threading.Event()
        self._recovery_lock = threading.Lock()
        self._motion_lock = threading.Lock()
        self._callback_group = ReentrantCallbackGroup()

        self._joint_state_pub = self.create_publisher(JointState, f"/{self._arm_namespace}/joint_states", 10)
        self._gripper_state_pub = self.create_publisher(
            JointMotorState,
            f"/{self._arm_namespace}/gripper/state",
            10,
        )
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            f"/{self._arm_namespace}/follow_joint_trajectory",
            execute_callback=self._execute_goal,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )
        self.create_service(Trigger, f"/{self._arm_namespace}/trajectory_stop", self._stop_service, callback_group=self._callback_group)
        self.create_service(Trigger, f"/{self._arm_namespace}/safe_home", self._safe_home_service, callback_group=self._callback_group)
        self.create_service(SetGripper, f"/{self._arm_namespace}/gripper/set", self._set_gripper_service, callback_group=self._callback_group)
        self.create_timer(1.0 / self._publish_rate_hz, self._publish_joint_state)
        self.get_logger().info(
            f"RViz sim trajectory controller ready: /{self._arm_namespace}/follow_joint_trajectory, "
            f"/{self._arm_namespace}/joint_states"
        )

    def _goal_callback(self, goal_request) -> GoalResponse:
        """目标准入检查：只有关节名为空的轨迹会被拒绝，其余一律接受。

        保守起见这里不做范围校验——本节点只做可视化，位置越界由上层规划与安全监控负责。
        """
        if self._recovery_lock.locked() or self._motion_lock.locked() or not goal_request.trajectory.joint_names:
            self.get_logger().warn("rejecting empty trajectory or busy simulation controller")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        """取消请求：置位停止标志并立即接受，实际停止发生在执行循环的下一个周期。"""
        self._stop_requested.set()
        return CancelResponse.ACCEPT

    def _stop_service(self, _request, response):
        """``/<ns>/trajectory_stop`` 服务：与取消等价的外部停止入口。"""
        self._stop_requested.set()
        response.success = True
        response.message = "sim trajectory stop requested"
        return response

    def _safe_home_service(self, _request, response):
        """仿真等价的安全回位：从任意停止姿态平滑回到统一 safe_home。"""
        if not self._recovery_lock.acquire(blocking=False):
            response.success = False
            response.message = "sim safe_home already running"
            return response
        motion_owned = False
        try:
            # 先让当前 FollowJointTrajectory 走取消分支，再独占更新关节状态。
            self._stop_requested.set()
            if not self._motion_lock.acquire(timeout=3.0):
                response.success = False
                response.message = "sim safe_home refused: active trajectory did not stop"
                return response
            motion_owned = True
            with self._lock:
                start = [self._positions_by_name.get(name, 0.0) for name in self._joint_names[:6]]
            duration = 2.0
            steps = max(1, int(duration * self._publish_rate_hz))
            self._stop_requested.clear()
            for index in range(1, steps + 1):
                if not rclpy.ok() or self._stop_requested.is_set():
                    response.success = False
                    response.message = "sim safe_home stopped"
                    return response
                ratio = index / steps
                smooth = 10.0 * ratio**3 - 15.0 * ratio**4 + 6.0 * ratio**5
                with self._lock:
                    for name, initial, target in zip(
                        self._joint_names[:6], start, SAFE_HOME_JOINT_POSITIONS
                    ):
                        self._positions_by_name[name] = initial + (target - initial) * smooth
                        self._velocities_by_name[name] = 0.0
                time.sleep(1.0 / self._publish_rate_hz)
            response.success = True
            response.message = "sim safe_home reached"
            return response
        finally:
            if motion_owned:
                self._motion_lock.release()
            self._recovery_lock.release()

    def _set_gripper_service(self, request, response):
        """``/<ns>/gripper/set`` 服务：把请求的开口宽度换算成两个手指关节位置并立即发布状态。

        请求字段 ``position`` 为目标开口宽度（米），会先被裁剪到配置的上下限；
        响应中 ``reached_position`` 回填裁剪后实际生效的宽度，便于调用方判断是否被限位。
        """
        left_position, right_position, width = gripper_joint_positions_for_width(
            float(request.position),
            min_width=self._gripper_min_width_m,
            max_width=self._gripper_max_width_m,
        )
        with self._lock:
            if "left_finger_joint" in self._positions_by_name:
                self._positions_by_name["left_finger_joint"] = left_position
            if "right_finger_joint" in self._positions_by_name:
                self._positions_by_name["right_finger_joint"] = right_position
            # 夹爪是位置型执行器，直接跳变到位，速度按 0 上报
            if "left_finger_joint" in self._velocities_by_name:
                self._velocities_by_name["left_finger_joint"] = 0.0
            if "right_finger_joint" in self._velocities_by_name:
                self._velocities_by_name["right_finger_joint"] = 0.0
        response.success = True
        response.reached_position = width
        self._last_gripper_width_m = width
        self._publish_gripper_state(width)
        self.get_logger().info(f"sim gripper set: width={width:.4f} m")
        return response

    def _publish_gripper_state(self, width_m: float) -> None:
        """把夹爪开口宽度（米）包装成关节状态消息发布。

        约定：``joint_name`` 固定为 ``gripper``，``position`` 承载宽度而非角度，
        ``torque`` 恒为 0（本节点不产生力），``status_code`` 0 表示正常。
        """
        msg = JointMotorState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_name = "gripper"
        msg.position = float(width_m)
        msg.velocity = 0.0
        msg.torque = 0.0
        msg.status_code = 0
        self._gripper_state_pub.publish(msg)

    def _execute_goal(self, goal_handle):
        if self._recovery_lock.locked() or not self._motion_lock.acquire(blocking=False):
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "another motion owns the simulation controller"
            goal_handle.abort()
            return result
        try:
            return self._execute_owned_goal(goal_handle)
        finally:
            self._motion_lock.release()

    def _execute_owned_goal(self, goal_handle):
        """执行一条关节轨迹目标：按单调时钟线性插值，周期发布反馈。

        时间基准用 ``time.monotonic()``（不受系统时间调整影响），相对轨迹起点计 elapsed；
        ``elapsed`` 上限截断到轨迹末点时间，保证不会越过最后一个轨迹点。
        取消或停止请求都会走 ``canceled()`` 分支，并把 ``error_code`` 记为 SUCCESSFUL
        （按该动作协议，取消成功不算错误）。
        """
        self._stop_requested.clear()
        trajectory = self._normalize_trajectory(goal_handle.request.trajectory)
        result = FollowJointTrajectory.Result()
        if trajectory is None:
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "trajectory contains no usable points"
            goal_handle.abort()
            return result

        names, points, times = trajectory
        start_time = time.monotonic()
        end_time = times[-1]
        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = names

        while rclpy.ok():
            if goal_handle.is_cancel_requested or self._stop_requested.is_set():
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                else:
                    goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                result.error_string = "sim trajectory canceled"
                return result
            elapsed = min(time.monotonic() - start_time, end_time)
            positions = self._interpolate(points, times, elapsed)
            self._set_positions(names, positions)
            # 本节点没有真实动力学，“期望位置”与“实际位置”都由插值结果同时充当
            feedback.actual.positions = positions
            feedback.desired.positions = positions
            goal_handle.publish_feedback(feedback)
            if elapsed >= end_time:
                break
            # 用固定周期 sleep 控制插值步长；执行回调阻塞在这里，因此执行器必须多线程
            time.sleep(1.0 / self._publish_rate_hz)

        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = "sim trajectory finished"
        return result

    def _set_positions(self, names: list[str], positions: list[float]) -> None:
        """把插值结果写回内存状态（加锁），速度置 0（位置直驱，无速度前馈）。"""
        with self._lock:
            for name, position in zip(names, positions):
                self._positions_by_name[name] = float(position)
                self._velocities_by_name[name] = 0.0

    def _normalize_trajectory(self, trajectory: JointTrajectory):
        """把外部轨迹整理成本节点可直接插值的 ``(names, points, times)``。

        处理内容：
            1. 只保留本节点认识的关节名（按轨迹给出的顺序），全都不认识或点为空则返回 ``None``；
            2. 轨迹点位置数量不足时，用该关节的当前位置补齐（避免索引越界）；
            3. 时间戳非正（缺省或非法）时，按点序号在 ``default_duration_sec`` 上均匀分摊；
            4. 强制时间严格递增（相邻至少间隔 1 ms），杜绝重复或回退时间导致的插值除零。

        返回：``names`` 为关节名列表，``points`` 为 ``[点][关节]`` 的位置矩阵，``times`` 为相对秒数。
        """
        source_names = list(trajectory.joint_names)
        if not source_names or not trajectory.points:
            return None

        names = [name for name in source_names if name in self._positions_by_name]
        if not names:
            return None
        name_to_source_index = {name: index for index, name in enumerate(source_names)}

        points: list[list[float]] = []
        times: list[float] = []
        last_time = 0.0
        for index, point in enumerate(trajectory.points):
            positions = []
            with self._lock:
                current = dict(self._positions_by_name)
            for name in names:
                source_index = name_to_source_index[name]
                if source_index < len(point.positions):
                    positions.append(float(point.positions[source_index]))
                else:
                    positions.append(float(current.get(name, 0.0)))
            point_time = _duration_to_sec(point.time_from_start)
            if point_time <= 0.0:
                # 时间戳缺失/非法：按点序号在默认总时长内均匀分摊
                point_time = self._default_duration_sec * float(index + 1) / float(len(trajectory.points))
            # 至少 1 ms 间隔，保证后续线性插值不会出现零长度区间
            point_time = max(point_time, last_time + 1e-3)
            last_time = point_time
            points.append(positions)
            times.append(point_time)
        return names, points, times

    def _interpolate(self, points: list[list[float]], times: list[float], elapsed: float) -> list[float]:
        """在相邻轨迹点之间做线性插值，返回 elapsed 时刻各关节的位置。

        边界：``elapsed`` 早于首个点则返回首点，晚于末点则返回末点（不外推）。
        区间定位用二分查找；若区间时长为 0（理论上已被规范化排除）则比例取 0 以防除零。
        """
        if elapsed <= times[0]:
            return list(points[0])
        if elapsed >= times[-1]:
            return list(points[-1])
        right = bisect.bisect_left(times, elapsed)
        left = max(0, right - 1)
        t0 = times[left]
        t1 = times[right]
        ratio = 0.0 if t1 <= t0 else (elapsed - t0) / (t1 - t0)
        return [float(p0) + (float(p1) - float(p0)) * ratio for p0, p1 in zip(points[left], points[right])]

    def _publish_joint_state(self) -> None:
        """定时器回调：按 ``joint_names`` 顺序发布一次完整关节状态，并顺带刷新夹爪状态。

        ``effort`` 恒为 0——本节点不做力矩仿真，该字段仅为消息结构完整而填充。
        """
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._joint_names)
        with self._lock:
            msg.position = [float(self._positions_by_name[name]) for name in self._joint_names]
            msg.velocity = [float(self._velocities_by_name[name]) for name in self._joint_names]
        msg.effort = [0.0 for _ in self._joint_names]
        self._joint_state_pub.publish(msg)
        self._publish_gripper_state(self._last_gripper_width_m)


def main(args=None) -> None:
    """节点入口：初始化 rclpy，用 3 线程执行器自旋，退出时按序释放资源。

    必须使用多线程执行器：动作执行回调内部会 sleep 推进轨迹，单线程会饿死定时器与服务回调。
    """
    rclpy.init(args=args)
    node = SimTrajectoryControllerNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        # 正常退出路径：Ctrl-C 或外部（如 launch）请求关闭，不视为错误
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

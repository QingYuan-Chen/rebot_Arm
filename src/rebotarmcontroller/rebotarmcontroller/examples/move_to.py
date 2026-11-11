#!/usr/bin/env python3
"""关节空间点到点演示: 把一组绝对关节角下发给控制器执行.

职责与位置
    通过控制器 FJT 提交关节目标的手动演示客户端.

对外接口
    DemoMoveTo 从首帧反馈建立起点, main 解析参数; 不会自行使能.

安全与并发约束
    本例只等首帧, 不证明持续新鲜或无碰撞; 现场条件和规划检查由操作者另行保证.

文件结构(按定义顺序)
    _duration_msg: 把秒(float)转成 ROS 的 Duration(整秒 + 纳秒两部分).
    _smoothstep: 五次光滑插值: 把 [0, 1] 的归一化进度映射成 [0, 1] 的缓入缓出系数.
    DemoMoveTo: 把关节目标组织成一条轨迹, 交给控制器动作服务器执行的演示节点.
    _parse_args: 解析整臂或单关节目标与时长参数, 互斥和维度检查在生成目标时完成.
    main: 控制台入口: 解析参数, 运行演示, 并按结果设置退出码(成功 0 / 失败 1).
"""

from __future__ import annotations

import argparse
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# 默认命名空间, 与控制器参数 arm_namespace 的默认值保持一致.
_NAMESPACE = "rebotarm"


def _duration_msg(seconds: float) -> Duration:
    """把秒(float)转成 ROS 的 Duration(整秒 + 纳秒两部分)."""
    sec = int(seconds)
    # 1e9 为每秒纳秒数; 小数部分转成整数纳秒.
    nanosec = int((float(seconds) - sec) * 1e9)
    return Duration(sec=sec, nanosec=nanosec)


def _smoothstep(ratio: float) -> float:
    """五次光滑插值: 把 [0, 1] 的归一化进度映射成 [0, 1] 的缓入缓出系数.

    多项式 10r³-15r⁴+6r⁵ 在 r=0 与 r=1 处的一阶, 二阶导数都为 0, 因此起停平滑,
    无速度与加速度突变. 输入先夹到 [0, 1]: 超界时饱和而不是外推, 避免速度反向.
    """
    ratio = max(0.0, min(1.0, ratio))
    return 10.0 * ratio**3 - 15.0 * ratio**4 + 6.0 * ratio**5


class DemoMoveTo(Node):
    """把关节目标组织成一条轨迹, 交给控制器动作服务器执行的演示节点.

    生命周期: 构造时订阅关节状态并创建动作客户端; ``run`` 内用单线程
    ``spin_until_future_complete`` 顺序等待"动作服务器就绪 -> 目标被接受 -> 结果返回",
    因此不需要执行器线程. 节点自身不使能硬件, 也不做安全预检或碰撞检查.
    """

    def __init__(
        self,
        target_positions: list[float],
        joint_name: str | None,
        joint_position: float | None,
        duration: float,
    ) -> None:
        """保存目标参数, 创建关节订阅和 FJT 客户端; 构造不发送运动."""
        super().__init__("move_to")
        self._target_positions = target_positions
        self._joint_name = joint_name
        self._joint_position = joint_position
        # 时长下限 0.1 s: 避免 0 或负时长导致轨迹退化(单点或时间倒退).
        self._duration = max(float(duration), 0.1)
        self._latest_joint_state: JointState | None = None
        self._last_feedback_log = 0.0

        # 只缓存最新一帧关节状态: 用它取轨迹起点和关节名/顺序.
        self.create_subscription(
            JointState,
            f"/{_NAMESPACE}/joint_states",
            self._joint_state_cb,
            qos_profile_sensor_data,
        )
        # 控制器的 FollowJointTrajectory 动作服务器; 硬件未显式使能时目标会被拒绝.
        self._follow_joint_trajectory = ActionClient(
            self,
            FollowJointTrajectory,
            f"/{_NAMESPACE}/follow_joint_trajectory",
        )

    def _joint_state_cb(self, msg: JointState) -> None:
        """保存最新关节消息供本例取起点; 不独立验证源年龄或反馈连续性."""
        self._latest_joint_state = msg

    def run(self) -> bool:
        """等待反馈与动作服务器, 发送目标并等待结果; 成功返回 True.

        返回 False 的四种情况: 拿不到 joint_states, 动作服务器 5 s 内不可用,
        目标被拒绝, 控制器返回非 SUCCESSFUL 的错误码.
        """
        if not self._wait_for_joint_state():
            self.get_logger().error("joint_states not available")
            return False

        if not self._follow_joint_trajectory.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("follow_joint_trajectory action not available")
            return False

        goal = FollowJointTrajectory.Goal()
        # 整条轨迹作为一次目标下发; 执行与终点收敛检查都在控制器侧完成.
        goal.trajectory = self._make_trajectory()

        send_future = self._follow_joint_trajectory.send_goal_async(
            goal,
            feedback_callback=self._feedback_cb,
        )
        # 阻塞式自旋等待: 本演示是单线程脚本, 不需要额外的执行器.
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(
            f"error_code={result.error_code} message={result.error_string}"
        )
        # SUCCESSFUL 为 0; 其它错误码表示被拒绝, 超差或中途中止.
        return result.error_code == FollowJointTrajectory.Result.SUCCESSFUL

    def _wait_for_joint_state(self, timeout_sec: float = 5.0) -> bool:
        """在 timeout_sec 内轮询等待第一帧关节状态; 超时返回 False."""
        # 用单调时钟做死线, 避免系统时间跳变影响超时判定.
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and self._latest_joint_state is None:
            if time.monotonic() > deadline:
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._latest_joint_state is not None

    def _make_trajectory(self) -> JointTrajectory:
        """生成从当前反馈位置到目标的关节空间轨迹(不带速度/加速度前馈).

        采样周期固定 0.05 s(20 Hz), 点数 = max(2, duration/0.05) + 1; 每个点的
        ``time_from_start`` 与位置插值共用同一个归一化进度, 保证时间与位置同步;
        位置用 smoothstep 系数在当前值与目标值之间插值, 因此起点恒等于当前反馈.
        """
        assert self._latest_joint_state is not None

        # 关节名与顺序以反馈为准, 目标数组必须按同一顺序排列.
        joint_names = list(self._latest_joint_state.name)
        current = np.array(self._latest_joint_state.position, dtype=np.float64)
        target = self._resolve_target(current)

        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names
        trajectory.points = []

        # 每 0.05 s 一个轨迹点(20 Hz); 至少 2 段, 保证首尾点都存在.
        steps = max(2, int(self._duration / 0.05))
        for step in range(steps + 1):
            ratio = step / float(steps)
            # 缓入缓出的归一化系数: 0 -> 1, 两端速度与加速度为 0.
            blend = _smoothstep(ratio)
            point = JointTrajectoryPoint()
            point.positions = [float(v) for v in current + (target - current) * blend]
            # 该点的绝对时间 = 总时长 × 归一化进度(单位 s).
            point.time_from_start = _duration_msg(self._duration * ratio)
            trajectory.points.append(point)

        self.get_logger().info(
            "moving joints to "
            + ", ".join(f"{name}={value:+.3f}" for name, value in zip(joint_names, target))
        )
        return trajectory

    def _resolve_target(self, current: np.ndarray) -> np.ndarray:
        """把命令行参数解析成 6 维绝对目标角(rad).

        ``--joint``/``--position`` 与 6 个位置参数互斥: 混用, 只给一半, 关节名不在
        反馈里, 或整臂目标维度与反馈不一致时, 都抛 ValueError(由 ``main`` 捕获并
        记录日志, 进程以失败退出).
        """
        assert self._latest_joint_state is not None

        if self._joint_name is not None:
            if self._target_positions:
                raise ValueError("use either 6 joint positions or --joint/--position")
            if self._joint_position is None:
                raise ValueError("--joint requires --position")
            if self._joint_name not in self._latest_joint_state.name:
                raise ValueError(f"unknown joint: {self._joint_name}")
            # 单关节模式: 以当前反馈为底, 只覆盖指定关节.
            target = current.copy()
            joint_index = self._latest_joint_state.name.index(self._joint_name)
            target[joint_index] = float(self._joint_position)
            return target

        if self._joint_position is not None:
            raise ValueError("--position requires --joint")
        if len(self._target_positions) != len(current):
            raise ValueError(
                f"expected {len(current)} absolute joint positions, "
                f"got {len(self._target_positions)}"
            )
        target = np.array(self._target_positions, dtype=np.float64)
        return target

    def _feedback_cb(self, feedback_msg) -> None:
        """动作反馈回调: 最多每 0.5 s 打印一次实测关节角, 避免日志刷屏."""
        now = time.monotonic()
        # 节流 0.5 s: 反馈频率远高于人工观察需要, 限频保持日志可读.
        if now - self._last_feedback_log < 0.5:
            return
        self._last_feedback_log = now
        actual = feedback_msg.feedback.actual.positions
        if actual:
            self.get_logger().info(
                "actual="
                + ", ".join(f"{value:+.3f}" for value in actual)
            )


def _parse_args() -> argparse.Namespace:
    """解析整臂或单关节目标与时长参数, 互斥和维度检查在生成目标时完成."""
    parser = argparse.ArgumentParser(description=__doc__)
    # 位置参数: 6 个绝对关节角(rad), 顺序与 joint_states 中的关节顺序一致.
    parser.add_argument(
        "positions",
        nargs="*",
        type=float,
        help="absolute 6-axis joint target positions in radians",
    )
    # 轨迹总时长(s), 内部下限 0.1 s; 调大更平缓, 调小冲击更大.
    parser.add_argument("--duration", type=float, default=2.0)
    # 单关节模式: 关节名, 例如 joint3.
    parser.add_argument(
        "--joint",
        help="single joint name, for example joint3",
    )
    # 单关节模式: 该关节的绝对目标角(rad).
    parser.add_argument(
        "--position",
        type=float,
        help="absolute target position for --joint, in radians",
    )
    return parser.parse_args()


def main() -> None:
    """控制台入口: 解析参数, 运行演示, 并按结果设置退出码(成功 0 / 失败 1)."""
    args = _parse_args()
    rclpy.init()
    node = DemoMoveTo(
        target_positions=args.positions,
        joint_name=args.joint,
        joint_position=args.position,
        duration=args.duration,
    )
    try:
        ok = node.run()
    except Exception as exc:
        # 参数互斥冲突等异常一律按失败处理, 但节点与 rclpy 仍要在 finally 里关闭.
        node.get_logger().error(str(exc))
        ok = False
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()

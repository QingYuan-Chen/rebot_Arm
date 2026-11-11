#!/usr/bin/env python3
"""末端位姿点到点演示: 让末端执行器走到给定位姿.

职责与位置
    通过运动层兼容位姿 Action 提交目标的手动客户端.

对外接口
    DemoMoveToPose 发送 MoveToPose, 等待结果并打印进度; 不直接访问硬件.

安全与并发约束
    本例不自动使能, 也不独立验证反馈连续性; IK, 路径和碰撞能力由运动层服务提供.

文件结构(按定义顺序)
    DemoMoveToPose: 把命令行给的位姿组装成动作目标并等待结果的动作客户端节点.
    _parse_args: 解析基座中的米制位置, 单位四元数与期望时长; 目标合法性由运动层继续检查.
    main: 控制台入口: 解析参数, 运行演示, 并按结果设置退出码(成功 0 / 失败 1).
"""

from __future__ import annotations

import argparse
import time

import rclpy
from geometry_msgs.msg import Pose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rebotarm_msgs.action import MoveToPose
from sensor_msgs.msg import JointState


class DemoMoveToPose(Node):
    """把命令行给的位姿组装成动作目标并等待结果的动作客户端节点.

    生命周期: 构造时订阅关节状态(仅用于等待控制器反馈就绪)并创建动作客户端;
    ``run`` 内用单线程 ``spin_until_future_complete`` 顺序等待"服务器就绪 -> 目标
    被接受 -> 结果返回", 因此不需要执行器线程. 节点自身不使能硬件, 也不做碰撞检查.
    """

    def __init__(self, args: argparse.Namespace) -> None:
        """保存命名空间与目标, 创建反馈订阅和运动层位姿 Action 客户端."""
        super().__init__("move_to_pose")
        # 命名空间统一去掉首尾斜杠, 避免拼出双斜杠话题名.
        self._namespace = args.namespace.strip("/")
        # 直接持有 argparse 结果: 目标字段在 run() 里逐项取出.
        self._target = args
        self._latest_joint_state: JointState | None = None
        # 订阅关节状态只是为了让演示脚本等到控制器已经在发反馈再下发目标.
        self.create_subscription(
            JointState,
            f"/{self._namespace}/joint_states",
            self._joint_state_cb,
            qos_profile_sensor_data,
        )
        # 运动层提供的位姿兼容 Action; 其下游控制器仍要求显式使能.
        self._move_to_pose = ActionClient(
            self,
            MoveToPose,
            f"/{self._namespace}/move_to_pose",
        )

    def _joint_state_cb(self, msg: JointState) -> None:
        """只保存反馈供演示等待首帧, 不作为独立运动安全守卫."""
        self._latest_joint_state = msg

    def run(self) -> bool:
        """等待反馈与动作服务器, 发送位姿目标并等待结果; 成功返回 True.

        返回 False 的四种情况: 拿不到 joint_states, 动作服务器 5 s 内不可用,
        目标被拒绝(通常是硬件未使能), 或结果里 ``success`` 为假
        (规划失败, 取消或被抢占).
        """
        if not self._wait_for_joint_state():
            self.get_logger().error("joint_states not available")
            return False

        if not self._move_to_pose.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("move_to_pose action not available")
            return False

        goal = MoveToPose.Goal()
        goal.target_pose = Pose()
        # 位置单位为米, 姿态为单位四元数; 默认 0, 0, 0, 1 表示相对基座不旋转, 并非沿用当前姿态.
        goal.target_pose.position.x = float(self._target.x)
        goal.target_pose.position.y = float(self._target.y)
        goal.target_pose.position.z = float(self._target.z)
        goal.target_pose.orientation.x = float(self._target.qx)
        goal.target_pose.orientation.y = float(self._target.qy)
        goal.target_pose.orientation.z = float(self._target.qz)
        goal.target_pose.orientation.w = float(self._target.qw)
        # 期望运动时长(s); <= 0 表示交给底层按末端移动距离估算.
        goal.duration = float(self._target.duration)

        send_future = self._move_to_pose.send_goal_async(
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
        self.get_logger().info(f"success={result.success} message={result.message}")
        return bool(result.success)

    def _feedback_cb(self, feedback_msg) -> None:
        """动作反馈回调: 打印进度与已用时长.

        ``progress`` 为 0~1 的估算值(给了期望时长就按时长比例, 否则按已发送的
        轨迹点占比); ``time_elapsed`` 为自收到目标起的秒数. 这里不节流, 因为
        位姿兼容节点按约 20 Hz 的等待节拍发布反馈.
        """
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"progress={feedback.progress:.2f} elapsed={feedback.time_elapsed:.2f}s"
        )

    def _wait_for_joint_state(self, timeout_sec: float = 5.0) -> bool:
        """在 timeout_sec 内轮询等待第一帧关节状态; 超时返回 False."""
        # 用单调时钟做死线, 避免系统时间跳变影响超时判定.
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and self._latest_joint_state is None:
            if time.monotonic() > deadline:
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._latest_joint_state is not None


def _parse_args() -> argparse.Namespace:
    """解析基座中的米制位置, 单位四元数与期望时长; 目标合法性由运动层继续检查."""
    parser = argparse.ArgumentParser(description=__doc__)
    # 控制器命名空间, 与 arm_namespace 参数一致; 统一去掉首尾斜杠.
    parser.add_argument("--namespace", default="rebotarm")
    # 目标位置(m), 参考系为基座 base_link: 默认机械臂前方 0.30 m, 高 0.30 m.
    parser.add_argument("--x", type=float, default=0.30)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--z", type=float, default=0.30)
    # 目标姿态四元数 (x, y, z, w), 默认单位四元数表示保持基座姿态(不旋转).
    parser.add_argument("--qx", type=float, default=0.0)
    parser.add_argument("--qy", type=float, default=0.0)
    parser.add_argument("--qz", type=float, default=0.0)
    parser.add_argument("--qw", type=float, default=1.0)
    # 期望运动时长(s), 默认 2.0; <= 0 时由底层按末端移动距离估算.
    parser.add_argument("--duration", type=float, default=2.0)
    return parser.parse_args()


def main() -> None:
    """控制台入口: 解析参数, 运行演示, 并按结果设置退出码(成功 0 / 失败 1)."""
    args = _parse_args()
    rclpy.init()
    node = DemoMoveToPose(args)
    try:
        ok = node.run()
    except Exception as exc:
        # 任何异常都按失败处理, 但节点与 rclpy 仍要在 finally 里关闭.
        node.get_logger().error(str(exc))
        ok = False
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()

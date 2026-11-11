#!/usr/bin/env python3
"""夹爪开合交互演示：可直接运行的现场调试客户端。

职责与位置
    通过 ROS 服务操作夹爪的单线程交互示例，不直接连接串口。

对外接口
    DemoGripperControl.run 使能后接收键盘命令；cleanup 尝试闭合并请求失能。

安全与并发约束
    现有 cleanup 不验证六轴安全基线；_enabled_by_demo 只说明 enable 回执成功，不能证明设备原先失能。
    这是示例当前行为的说明，不能作为健康任务失败后自动失能的生产恢复策略。

文件结构（按定义顺序）
    DemoGripperControl：演示节点：持有使能/失能与夹爪设置三类服务的客户端。
    main：运行前台交互示例，finally 调用现有 cleanup；该清理不包含六轴安全基线确认。
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rebotarm_msgs.srv import SetGripper
from std_srvs.srv import Trigger

_NAMESPACE = "rebotarm"  # 机械臂命名空间：服务名统一为 /<namespace>/...
_OPEN_POSITION_M = 0.09  # 标称全开请求（米）；当前控制器按 90 mm 映射，实际指间距须另行标定
_CLOSE_POSITION_M = 0.0  # “闭合”请求开度（米），0 表示完全闭合
_MAX_EFFORT = 0.0  # 最大夹持力矩（N·m）；<=0 表示由硬件层取默认夹持力


class DemoGripperControl(Node):
    """演示节点：持有使能/失能与夹爪设置三类服务的客户端。

    生命周期很短，全部动作在 run() 中同步完成，因此成员只在单线程中被读写。
    ``_enabled_by_demo`` 记录“使能是本演示打开的”：只有它成立时 cleanup 才会去失能，
    该标志不检查此前是否已使能，不能证明对其他使用者的状态隔离。
    """

    def __init__(self) -> None:
        """创建使能，失能和夹爪服务客户端，尚未发出设备操作。"""
        super().__init__("gripper_control")
        self._enabled_by_demo = False
        self._enable = self.create_client(Trigger, f"/{_NAMESPACE}/enable")
        self._disable = self.create_client(Trigger, f"/{_NAMESPACE}/disable")
        self._client = self.create_client(SetGripper, f"/{_NAMESPACE}/gripper/set")

    def run(self) -> bool:
        """执行一次交互式演示。

        返回 True 表示用户正常退出，False 表示使能失败或夹爪服务不可用；调用方据此
        决定进程退出码。 期间会阻塞在标准输入上，属于前台交互程序。
        """
        # 真机默认失能：必须显式使能后才允许下发夹爪命令
        if not self._call_trigger(self._enable, "enable"):
            return False
        self._enabled_by_demo = True

        if not self._client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("gripper set service not available")
            return False

        self.get_logger().info("commands: o/open, c/close, q/quit")
        while rclpy.ok():
            try:
                command = input("gripper> ").strip().lower()
            except (EOFError, KeyboardInterrupt):
                # 非交互输入（管道/重定向）或 Ctrl+C：按退出处理，先换行避免日志粘在提示符后
                print()
                break

            if command in ("q", "quit", "exit"):
                break
            if command in ("o", "open"):
                self._set_gripper(_OPEN_POSITION_M, "open")
                continue
            if command in ("c", "close"):
                self._set_gripper(_CLOSE_POSITION_M, "close")
                continue
            self.get_logger().info("commands: o/open, c/close, q/quit")
        return True

    def cleanup(self) -> None:
        """收尾：仅在本次演示使能过时才动硬件，先合上夹爪再失能。

        夹爪先闭合是为了不把夹具留在张开姿态；失能后电机不再输出力矩，姿态由重力与
        机械结构决定。
        """
        if self._enabled_by_demo:
            self._set_gripper(_CLOSE_POSITION_M, "close")
            self._call_trigger(self._disable, "disable")

    def _call_trigger(self, client, label: str, timeout_sec: float = 5.0) -> bool:
        """同步调用一个标准触发器服务。

        返回 True 仅当服务返回 success；服务不可用，等待响应超时，success 为假都返回
        False 并记录错误日志。 注意：服务可用性探测固定等待 5.0 秒，形参 timeout_sec
        只作用于等待响应（默认 5.0 秒）。
        """
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"{label} service not available")
            return False
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done():
            self.get_logger().error(f"{label} timed out")
            return False
        result = future.result()
        if result is None or not result.success:
            message = result.message if result is not None else "no response"
            self.get_logger().error(f"{label} failed: {message}")
            return False
        self.get_logger().info(message if (message := result.message) else f"{label} OK")
        return True

    def _set_gripper(self, position: float, label: str) -> bool:
        """请求夹爪运动到指定开度。

        position 单位为米（0 = 闭合），max_effort 单位为 N·m，0 表示用硬件层默认夹持力，
        硬件层会先把开度裁剪到当前标称行程 [0, 0.09] m。 返回 True 表示服务报告在超时内
        到达目标；失败时也会打印实际到达开度，便于区分“夹到物体停住”和“完全没动作”。
        等待响应未设超时：服务端长时间不回复时本调用会一直阻塞。
        """
        request = SetGripper.Request()
        request.position = float(position)
        request.max_effort = _MAX_EFFORT

        future = self._client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if result is None:
            self.get_logger().error(f"{label} failed: no response")
            return False
        if not result.success:
            self.get_logger().warn(
                f"{label} not reached, current={result.reached_position:.3f}m"
            )
            return False
        self.get_logger().info(
            f"{label} reached, current={result.reached_position:.3f}m"
        )
        return True


def main() -> None:
    """运行前台交互示例，finally 调用现有 cleanup；该清理不包含六轴安全基线确认。"""
    rclpy.init()
    node = DemoGripperControl()
    try:
        ok = node.run()
    except Exception as exc:
        # 交互过程中的任何异常都折算成失败退出，日志保留原始信息
        node.get_logger().error(str(exc))
        ok = False
    finally:
        # 现有演示清理会请求闭合和失能；不包含生产要求的六轴安全基线验证。
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()

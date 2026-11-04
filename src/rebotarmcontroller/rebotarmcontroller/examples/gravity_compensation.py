#!/usr/bin/env python3
"""重力补偿(零重力手动拖拽)演示: 进入补偿模式, 等待操作员 Ctrl+C 后尝试清理.

职责与位置
    通过 ROS 服务进入重力补偿并等待退出的手动演示.

对外接口
    _call_trigger 同步等待回执; main 依次显式使能, 启动补偿并在 finally 尝试停靠和失能.

安全与并发约束
    现有 finally 在停靠失败后仍尝试失能, 且没有现场基线验证; 不能把该行为当作生产恢复方案.
    本例没有独立安全监测, 脚本退出或服务调用完成不证明设备已安全收尾.

文件结构(按定义顺序)
    _call_trigger: 同步调用一个标准触发器服务.
    main: 运行补偿演示并接管退出信号; finally 按现有顺序尝试停靠和失能, 不证明安全基线.
"""

from __future__ import annotations

import signal

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import Trigger

_NAMESPACE = "rebotarm"  # 机械臂命名空间: 服务名统一为 /<namespace>/...


def _call_trigger(
    node: Node,
    client,
    label: str,
    timeout_sec: float = 5.0,
) -> bool:
    """同步调用一个标准触发器服务.

    返回 True 仅当服务返回 success; 服务不可用, 等待响应超时, success 为假都返回
    False 并记录错误日志. 注意: 服务可用性探测固定等待 5.0 秒, 形参 timeout_sec 只
    作用于等待响应(safe_home 传 35.0 s, 其余默认 5.0 s).
    """
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error(f"{label} service not available")
        return False
    future = client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    if not future.done():
        node.get_logger().error(f"{label} timed out")
        return False
    result = future.result()
    if result is None or not result.success:
        message = result.message if result is not None else "no response"
        node.get_logger().error(f"{label} failed: {message}")
        return False
    node.get_logger().info(message if (message := result.message) else f"{label} OK")
    return True


def main() -> None:
    """运行补偿演示并接管退出信号; finally 按现有顺序尝试停靠和失能, 不证明安全基线."""
    # 关闭 rclpy 自带的信号处理: 默认行为会在收到信号时直接抛出/终止, 导致 finally 里的
    # 回零与失能来不及执行; 这里改由脚本自己置停止标志, 让主循环退出后走统一清理路径.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node("gravity_compensation")
    stop_requested = False

    def request_stop(_signum, _frame) -> None:
        """信号处理器: 只置停止标志并打印一次日志, 真正的退出动作在 finally 中执行."""
        nonlocal stop_requested
        if not stop_requested:
            node.get_logger().info("stop requested, shutting down gravity compensation")
        stop_requested = True

    old_sigint = signal.getsignal(signal.SIGINT)
    old_sigterm = signal.getsignal(signal.SIGTERM)
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    enable_client = node.create_client(
        Trigger,
        f"/{_NAMESPACE}/enable",
    )
    start_client = node.create_client(
        Trigger,
        f"/{_NAMESPACE}/gravity_compensation/start",
    )
    safe_home_client = node.create_client(
        Trigger,
        f"/{_NAMESPACE}/safe_home",
    )
    disable_client = node.create_client(
        Trigger,
        f"/{_NAMESPACE}/disable",
    )

    try:
        # 真机默认失能: 不先使能, 启动重力补偿会被硬件层拒绝
        if not _call_trigger(node, enable_client, "enable"):
            raise SystemExit(1)
        if not _call_trigger(
            node,
            start_client,
            "start gravity compensation",
        ):
            raise SystemExit(1)
        node.get_logger().info("press Ctrl+C to stop gravity compensation")
        while rclpy.ok() and not stop_requested:
            # 0.2 s 轮询一次: 既能及时响应信号, 又不会空转占满 CPU
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        try:
            # 现有顺序先尝试停靠; 即使失败, 下方仍会请求失能, 不能视作安全基线确认.
            _call_trigger(node, safe_home_client, "safe_home", timeout_sec=35.0)
        except Exception as exc:
            node.get_logger().warn(f"safe_home cleanup failed: {exc}")
        try:
            _call_trigger(node, disable_client, "disable")
        except Exception as exc:
            node.get_logger().warn(f"disable cleanup failed: {exc}")
        # 恢复原有信号处理器, 避免在被其它程序调用时留下副作用
        signal.signal(signal.SIGINT, old_sigint)
        signal.signal(signal.SIGTERM, old_sigterm)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

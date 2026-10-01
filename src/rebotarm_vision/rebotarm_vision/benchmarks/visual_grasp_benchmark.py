"""视觉抓取稳定性 benchmark 命令行工具（离线统计，不直接操作硬件）。

用途与位置
----------
按固定次数反复调用视觉准备位服务与视觉抓取执行服务，统计成功率，并把失败按发生
阶段（检测、候选过滤、忙、执行器、未知）分桶汇总，用于软件/仿真链路的稳定性回归。

运行方式与接口
--------------
命令行入口为 ``ros2 run <本包> rebotarm_visual_grasp_benchmark``，节点名固定为
``rebotarm_visual_grasp_benchmark``。它只作为客户端调用两个由上层启动的服务
（``/<namespace>/visual_ready/move`` 与 ``/<namespace>/visual_grasp/execute``，
两者签名均为 Trigger），自己既订阅话题也不发布话题。

安全边界
--------
本工具不启动控制器、不使能真机；执行是否真的动作取决于被抓取服务所在后端的
``execution_mode``。因此 benchmark 通过只代表候选/规划/仿真或软件执行链路稳定，
不能作为真实抓取成功的验收证据。
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from typing import Iterable

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


def classify_failure_stage(message: str) -> str:
    """从执行器返回的消息推断失败阶段，返回用于汇总的短标签。

    匹配的是消息文本字面量，属于与执行器之间的隐式约定，改动这些前缀会破坏
    统计分类：优先取 " failed:" 之前的前缀（例如 "visual_ready failed: ..."
    归类为 visual_ready），否则按已知前缀判定，无法识别时归为 "unknown"。
    """
    text = str(message or "").strip()
    if not text:
        return "unknown"
    if " failed:" in text:
        return text.split(" failed:", 1)[0].strip() or "unknown"
    if text.startswith("no valid grasp plan"):
        return "detect"
    if text.startswith("no candidate attempts"):
        return "filter"
    if text.startswith("visual grasp already running"):
        return "busy"
    if text.startswith("visual grasp failed"):
        return "executor"
    return "unknown"


@dataclass
class BenchmarkStats:
    """多次尝试的累计统计。

    total 为尝试总次数，success 为成功次数；failures_by_stage 按失败阶段计数；
    messages 按时间顺序保留每次尝试的原始消息，便于事后核对失败原因。
    """

    total: int = 0
    success: int = 0
    failures_by_stage: dict[str, int] = field(default_factory=dict)
    messages: list[str] = field(default_factory=list)

    def record(self, *, success: bool, message: str) -> None:
        self.total += 1
        self.messages.append(str(message))
        if success:
            self.success += 1
            return
        stage = classify_failure_stage(message)
        self.failures_by_stage[stage] = self.failures_by_stage.get(stage, 0) + 1

    @property
    def success_rate(self) -> float:
        """成功率百分比（0~100）；总次数为 0 时返回 0.0，避免除零。"""
        if self.total <= 0:
            return 0.0
        return 100.0 * float(self.success) / float(self.total)

    def summary_lines(self) -> list[str]:
        """生成汇总文本行，供日志逐行输出（首行为总数与成功率，其后为失败分桶）。"""
        lines = [
            (
                f"total={self.total} success={self.success} failed={self.total - self.success} "
                f"success_rate={self.success_rate:.1f}%"
            )
        ]
        if self.failures_by_stage:
            lines.append("failed_stage:")
            for stage, count in sorted(self.failures_by_stage.items()):
                lines.append(f"  {stage}: {count}")
        return lines


class VisualGraspBenchmark(Node):
    """持有两个 Trigger 客户端并驱动多次尝试的节点。

    生命周期：由 ``main`` 创建、跑完所有尝试后销毁。所有服务调用都在同一个线程里
    同步等待（signal 阻塞式 spin），所以不涉及并发回调；一旦调用超时或服务不可用，
    本次尝试记为失败并继续下一次，不会中断整轮 benchmark。
    """

    def __init__(self, *, namespace: str, service_timeout_sec: float) -> None:
        super().__init__("rebotarm_visual_grasp_benchmark")
        namespace = namespace.strip("/")
        self._service_timeout_sec = float(service_timeout_sec)
        # 服务名由命名空间前缀拼出：调用方必须传与启动文件一致的命名空间，
        # 否则会一直等到超时（不会误打到别的机械臂命名空间上）。
        self._ready_client = self.create_client(Trigger, f"/{namespace}/visual_ready/move")
        self._grasp_client = self.create_client(Trigger, f"/{namespace}/visual_grasp/execute")

    def call_trigger(self, client, label: str) -> tuple[bool, str]:
        """同步调用一次 Trigger 服务，返回 (是否成功, 服务返回的消息)。

        ``label`` 只用于拼装错误文案（如 "visual_grasp service unavailable"）。
        请求体为空（Trigger.Request() 无字段）。超时、无服务、无响应一律返回 False
        且不抛异常，保证 benchmark 能跑完整轮。
        """
        if not client.wait_for_service(timeout_sec=self._service_timeout_sec):
            return False, f"{label} service unavailable"
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=self._service_timeout_sec)
        if not future.done():
            return False, f"{label} service call timed out"
        result = future.result()
        if result is None:
            return False, f"{label} returned no result"
        return bool(result.success), str(result.message)

    def run_attempts(self, *, attempts: int, return_ready_before_each: bool, wait_enter: bool) -> BenchmarkStats:
        """执行多轮"回准备位 -> 抓取"并返回统计结果。

        参数：
        - ``attempts``：尝试次数，索引从 1 开始，仅用于日志与交互提示。
        - ``return_ready_before_each``：每轮抓取前是否先调一次视觉准备位服务；
          准备位失败时本轮直接记为失败并跳过抓取（避免在未知位姿下继续动）。
        - ``wait_enter``：是否在抓取前阻塞等待操作者按回车，给人工摆放物体与确认
          现场安全留出时间；自动化回归时可关闭。

        副作用：向标准输入读取一行（``wait_enter`` 为真时）、写日志、调用服务。
        """
        stats = BenchmarkStats()
        for index in range(1, int(attempts) + 1):
            self.get_logger().info(f"benchmark attempt {index}/{attempts}")
            if return_ready_before_each:
                ok, message = self.call_trigger(self._ready_client, "visual_ready")
                self.get_logger().info(f"visual_ready: success={ok}, message={message}")
                if not ok:
                    stats.record(success=False, message=f"visual_ready failed: {message}")
                    continue
            if wait_enter:
                input(
                    f"[{index}/{attempts}] Place object, open gripper if needed, confirm the area is safe, "
                    "then press Enter to grasp..."
                )
            ok, message = self.call_trigger(self._grasp_client, "visual_grasp")
            stats.record(success=ok, message=message)
            self.get_logger().info(f"visual_grasp: success={ok}, message={message}")
        return stats


def build_parser() -> argparse.ArgumentParser:
    """构造命令行参数解析器。

    每个开关都有正向与 ``--no-*`` 反向形式（``store_false`` + ``dest``），默认值
    与实测推荐用法（先回准备位、再人工确认）一致。
    """
    parser = argparse.ArgumentParser(description="Run repeated visual grasp attempts and summarize success rate.")
    parser.add_argument("--attempts", type=int, default=20)
    parser.add_argument("--namespace", default="rebotarm")
    parser.add_argument("--service-timeout-sec", type=float, default=180.0)
    parser.add_argument("--wait-enter", action="store_true", default=True)
    parser.add_argument("--no-wait-enter", action="store_false", dest="wait_enter")
    parser.add_argument("--return-ready-before-each", action="store_true", default=True)
    parser.add_argument("--no-return-ready-before-each", action="store_false", dest="return_ready_before_each")
    return parser


def main(argv: Iterable[str] | None = None) -> None:
    """命令行入口：解析参数、初始化运行时、跑完尝试并打印汇总。

    ``attempts`` 至少为 1；无论成功失败都会在 finally 中销毁节点，且仅在运行时
    仍有效时才关闭，避免重复关闭报错。
    """
    args = build_parser().parse_args(list(argv) if argv is not None else None)
    rclpy.init()
    node = VisualGraspBenchmark(namespace=args.namespace, service_timeout_sec=args.service_timeout_sec)
    try:
        stats = node.run_attempts(
            attempts=max(1, int(args.attempts)),
            return_ready_before_each=bool(args.return_ready_before_each),
            wait_enter=bool(args.wait_enter),
        )
        for line in stats.summary_lines():
            node.get_logger().info(line)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

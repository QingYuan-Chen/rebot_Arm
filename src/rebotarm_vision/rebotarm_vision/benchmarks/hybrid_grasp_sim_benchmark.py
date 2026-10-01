"""混合抓取仿真基准测试（真实感知 + 仿真执行）。

用途与定位：
    本模块是一个**离线基准测试工具**（不是常驻生产节点），用于在「真实相机 + 真实感知链路」
    下反复运行视觉抓取，但只把抓取交给仿真/软件执行后端，从而量化抓取成功率并按失败阶段归因。
    与只调用服务的普通基准（visual_grasp_benchmark）相比，本基准额外订阅抓取规划话题，
    并要求「每轮都收到一条比上一轮更新且 valid 的规划」后才触发执行，避免把上一轮的旧规划
    计入本轮统计。

数据链路与接口：
    - 订阅：抓取规划话题（默认 /grasp/filtered_plan，消息类型 GraspPlan），用于确认本轮产生了新规划；
    - 调用服务（标准 Trigger 类型，两个服务都只接受空请求）：
        /{namespace}/visual_ready/move    回到视觉准备位姿（可选，按命令行开关决定是否调用）；
        /{namespace}/visual_grasp/execute 执行一次视觉抓取（仿真后端或显式授权的真机后端）；
    - 输出：统计汇总（总数 / 成功数 / 成功率 / 按阶段失败数）打印到节点日志。

安全边界：
    本工具自身不规划、不控制电机，只调用上层服务；仿真执行不会使能真实机械臂。
    真机执行仍由服务端的安全门控决定。

退出码：
    0 = 正常完成（且成功率达标）；2 = 成功率低于 --min-success-rate 阈值。
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass, field
from typing import Iterable

import rclpy
from rclpy.node import Node
from rebotarm_msgs.msg import GraspPlan
from std_srvs.srv import Trigger

from .visual_grasp_benchmark import classify_failure_stage


@dataclass
class HybridBenchmarkStats:
    """基准统计：成功数、按失败阶段的计数与原始消息记录。

    failures_by_stage 的键由同包 benchmark 模块的失败归因函数给出
    （例如 detect/filter/busy/executor/unknown），用于区分是感知、候选过滤、
    并发占用还是执行阶段出的问题。
    """

    total: int = 0
    # 成功轮次数（服务返回 success=true 的轮次）
    success: int = 0
    # 阶段 -> 失败次数；键为 classify_failure_stage 的返回值
    failures_by_stage: dict[str, int] = field(default_factory=dict)
    # 每轮的消息原文，便于失败后离线复核（不参与统计计算）
    messages: list[str] = field(default_factory=list)

    def record(self, *, success: bool, message: str) -> None:
        """登记一轮结果：总数自增，失败时再按消息内容归因到具体阶段。"""
        self.total += 1
        self.messages.append(str(message))
        if success:
            self.success += 1
            return
        stage = classify_failure_stage(message)
        self.failures_by_stage[stage] = self.failures_by_stage.get(stage, 0) + 1

    @property
    def success_rate(self) -> float:
        """成功率百分比（0~100）；总数为 0 时返回 0.0，避免除零。"""
        if self.total <= 0:
            return 0.0
        return 100.0 * float(self.success) / float(self.total)

    def summary_lines(self) -> list[str]:
        """生成汇总文本行：先一行总量，再按阶段名排序输出失败明细。

        排序是为了让多次运行的输出可逐行对比（阶段名固定顺序）。
        """
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


class HybridGraspSimBenchmark(Node):
    """混合基准节点：订阅规划话题 + 调用视觉准备/视觉抓取服务。

    节点名固定为 rebotarm_hybrid_grasp_sim_benchmark（对外可见，禁止改动）。
    运行模型为单线程客户端：由 run_attempts 主动 spin_once / spin_until_future_complete，
    不使用回调组，因此同一时刻只处理一个服务请求，天然不会并发触发抓取。
    """

    def __init__(
        self,
        *,
        namespace: str,
        plan_topic: str,
        service_timeout_sec: float,
        plan_timeout_sec: float,
    ) -> None:
        """创建服务客户端与规划订阅。

        参数：
            namespace：服务命名空间（前导/尾随 “/” 会被去掉），服务名拼接为
                /{namespace}/visual_ready/move 与 /{namespace}/visual_grasp/execute，
                真机与仿真使用不同命名空间以隔离后端。
            plan_topic：抓取规划话题名（默认 /grasp/filtered_plan）。
            service_timeout_sec：等待服务可用及等待响应的超时（s）。视觉抓取包含感知、
                IK 与规划，耗时可能达分钟级，故默认 180。
            plan_timeout_sec：每轮等待「新且 valid 的规划」的超时（s），默认 10。
        """
        super().__init__("rebotarm_hybrid_grasp_sim_benchmark")
        namespace = namespace.strip("/")
        self._plan_topic = str(plan_topic)
        self._service_timeout_sec = float(service_timeout_sec)
        self._plan_timeout_sec = float(plan_timeout_sec)
        # 最近一条规划及其「修订号」：修订号只增不减，用于判断规划是否为本轮新产生。
        self._latest_plan: GraspPlan | None = None
        self._plan_revision = 0

        self._ready_client = self.create_client(Trigger, f"/{namespace}/visual_ready/move")
        self._grasp_client = self.create_client(Trigger, f"/{namespace}/visual_grasp/execute")
        # 队列深度 10：只关心最新规划，积压过多也无意义。
        self.create_subscription(GraspPlan, self._plan_topic, self._on_plan, 10)

    def _on_plan(self, plan: GraspPlan) -> None:
        """缓存最新规划并递增修订号（不在此处判断 valid，判断留给等待函数）。"""
        self._latest_plan = plan
        self._plan_revision += 1

    def _wait_for_fresh_valid_plan(self, *, min_revision: int, timeout_sec: float) -> GraspPlan | None:
        """在超时内等待一条「修订号大于 min_revision 且 valid=true」的规划。

        判定条件故意要求同时满足两点：
        - 修订号必须比本轮开始时更新，否则可能是上一轮遗留的规划被重复使用；
        - valid 必须为真，invalid 规划代表候选链被过滤/门控拒绝，不能作为执行依据。
        超时（或 ROS 已关闭）返回 None，由调用方记为失败轮次。
        """
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        while rclpy.ok() and time.monotonic() < deadline:
            # 单次 spin 0.1 s：既能把订阅回调跑起来，也保证超时粒度足够细。
            rclpy.spin_once(self, timeout_sec=0.1)
            plan = self._latest_plan
            if self._plan_revision <= min_revision or plan is None:
                continue
            if bool(plan.valid):
                return plan
        return None

    def call_trigger(self, client, label: str) -> tuple[bool, str]:
        """调用一个空请求的 Trigger 服务，返回 (是否成功, 服务消息)。

        label 只用于拼装失败原因文本（如 "visual_ready service unavailable"），
        便于统计阶段归因。三种失败都会被转换成 (False, 原因) 而不是抛异常：
        服务不可用、调用超时、返回空结果。
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

    def run_attempts(
        self,
        *,
        attempts: int,
        wait_enter: bool,
        return_ready_before_each: bool,
        return_ready_after_each: bool,
    ) -> HybridBenchmarkStats:
        """按顺序执行 attempts 轮「等待新规划 -> 触发仿真抓取」，返回统计结果。

        参数：
            attempts：轮次数（调用方已保证 >= 1）。
            wait_enter：每轮触发前是否阻塞等待操作员按回车；真机/真相机场景下
                用于人工确认「机械臂在视觉准备位、相机能看到物体、现场安全」。
            return_ready_before_each：每轮开始前调用视觉准备服务回位；失败则本轮直接记为失败
                （避免在未回位状态下抓取，污染统计与安全边界）。
            return_ready_after_each：每轮结束后调用视觉准备服务回位；失败同样记为一次失败。

        说明：
            invalid 规划或规划超时只记失败并跳过执行，不会触发抓取；
            return_ready_after_each 的失败会额外计入一次失败，因此总失败数可能大于轮次数。
        """
        stats = HybridBenchmarkStats()
        for index in range(1, int(attempts) + 1):
            self.get_logger().info(f"hybrid grasp sim benchmark attempt {index}/{attempts}")

            if return_ready_before_each:
                ok, message = self.call_trigger(self._ready_client, "visual_ready")
                self.get_logger().info(f"visual_ready: success={ok}, message={message}")
                if not ok:
                    stats.record(success=False, message=f"visual_ready failed: {message}")
                    continue

            if wait_enter:
                # 阻塞式人工确认：这是安全门而非调试输出，勿改成自动继续。
                input(
                    f"[{index}/{attempts}] Confirm real arm is at visual_ready, real camera sees the object, "
                    "then press Enter to run simulated grasp..."
                )

            # 先记录当前修订号，只有比它更新的规划才被接受为本轮结果。
            min_revision = self._plan_revision
            plan = self._wait_for_fresh_valid_plan(
                min_revision=min_revision,
                timeout_sec=self._plan_timeout_sec,
            )
            if plan is None:
                message = f"no valid fresh grasp plan received on {self._plan_topic}"
                stats.record(success=False, message=message)
                self.get_logger().warn(message)
                continue

            self._log_plan_snapshot(plan)
            # 执行阶段：此处只调用服务，是否落到仿真或真机由后端与安全门控决定。
            ok, message = self.call_trigger(self._grasp_client, "visual_grasp")
            stats.record(success=ok, message=message)
            self.get_logger().info(f"visual_grasp(sim): success={ok}, message={message}")

            if return_ready_after_each:
                ready_ok, ready_message = self.call_trigger(self._ready_client, "visual_ready")
                self.get_logger().info(f"visual_ready(after): success={ready_ok}, message={ready_message}")
                if not ready_ok:
                    stats.record(success=False, message=f"visual_ready failed: {ready_message}")
        return stats

    def _log_plan_snapshot(self, plan: GraspPlan) -> None:
        """打印规划关键字段快照（来源、指爪宽度 m、预抓取/抓取位置 m、原因文本）。

        用于事后核对「候选来自哪条链路」「失败原因写的是什么」，不参与判定逻辑。
        """
        pre = plan.pregrasp_pose.position
        grasp = plan.grasp_pose.position
        self.get_logger().info(
            "plan source="
            f"{plan.source}, candidate_source={plan.candidate.source}, "
            f"jaw_width={float(plan.jaw_width):.4f}, "
            f"pregrasp=({pre.x:.3f}, {pre.y:.3f}, {pre.z:.3f}), "
            f"grasp=({grasp.x:.3f}, {grasp.y:.3f}, {grasp.z:.3f}), "
            f"reason={plan.reason}"
        )


def build_parser() -> argparse.ArgumentParser:
    """构造命令行参数解析器（所有参数名都是对外接口，禁止改动）。"""
    parser = argparse.ArgumentParser(
        description=(
            "Run repeated real-perception plus simulated-execution visual grasp attempts and summarize success rate."
        )
    )
    parser.add_argument("--attempts", type=int, default=20)  # 轮次数，运行时会取 max(1, 值)
    parser.add_argument("--namespace", default="rebotarm_sim")  # 服务命名空间前缀，默认仿真后端
    parser.add_argument("--plan-topic", default="/grasp/filtered_plan")  # 抓取规划话题名
    # 等待服务可用/响应的超时（s）；真实感知+规划较慢，默认 180
    parser.add_argument("--service-timeout-sec", type=float, default=180.0)
    parser.add_argument("--plan-timeout-sec", type=float, default=10.0)  # 等待新规划的超时（s）
    # 最低成功率门槛（%）；低于该值时进程以退出码 2 结束，供 CI/脚本判定
    parser.add_argument("--min-success-rate", type=float, default=0.0)
    # 以下两行是同一个开关的正反写法：默认 wait_enter=True，用 --no-wait-enter 关闭人工确认
    parser.add_argument("--wait-enter", action="store_true", default=True)
    parser.add_argument("--no-wait-enter", action="store_false", dest="wait_enter")
    # 每轮开始前/结束后是否调用视觉准备服务回位，默认都不调用（保持现状，不额外移动机械臂）
    parser.add_argument("--return-ready-before-each", action="store_true", default=False)
    parser.add_argument("--return-ready-after-each", action="store_true", default=False)
    return parser


def main(argv: Iterable[str] | None = None) -> None:
    """命令行入口：建节点、跑基准、打印汇总；成功率不达标时以退出码 2 结束。

    无论运行是否异常都会销毁节点并优雅关闭 rclpy，避免后台 DDS 资源泄漏。
    """
    args = build_parser().parse_args(list(argv) if argv is not None else None)
    rclpy.init()
    node = HybridGraspSimBenchmark(
        namespace=args.namespace,
        plan_topic=args.plan_topic,
        service_timeout_sec=args.service_timeout_sec,
        plan_timeout_sec=args.plan_timeout_sec,
    )
    exit_code = 0
    try:
        stats = node.run_attempts(
            attempts=max(1, int(args.attempts)),
            wait_enter=bool(args.wait_enter),
            return_ready_before_each=bool(args.return_ready_before_each),
            return_ready_after_each=bool(args.return_ready_after_each),
        )
        for line in stats.summary_lines():
            node.get_logger().info(line)
        if stats.success_rate < float(args.min_success_rate):
            # 成功率为百分比，阈值同为百分比，直接比较。
            exit_code = 2
            node.get_logger().error(
                f"success_rate {stats.success_rate:.1f}% < min_success_rate {float(args.min_success_rate):.1f}%"
            )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if exit_code:
        raise SystemExit(exit_code)


if __name__ == "__main__":
    main(sys.argv[1:])

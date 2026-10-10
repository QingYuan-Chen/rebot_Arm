"""抓取候选深度探针：判断近距离深度是否还够准，用于决定能否做预抓取位重定位。

用途与位置
----------
这是一个诊断/验收辅助节点，不参与正式抓取链路：它订阅抓取候选话题，连续采集若干帧
目标候选的深度值，输出均值/极值/标准差与逐样本的深度状态，最后给出
``gemini2_close_depth_usable`` 结论——近距离深度能否支撑"接近后按新深度重新定位
预抓取位"的做法。结论为不可用时，应改用一次性抓取（不重定位）或把粗预抓取位放远
一些，而不是硬用带噪声的深度继续接近。

接口与行为
----------
- 订阅：``input_topic``（默认 ``/grasp/candidates``，类型为抓取候选数组）。
- 发布：无。写服务/动作：无。本节点不发布任何运动请求，也不使能硬件。
- 采满 ``sample_count`` 个样本后打印汇总并主动关闭运行时退出，因此适合脚本化调用：
  ``ros2 run <本包> rebotarm_grasp_depth_probe``（可带 ``-p`` 覆盖参数）。
- 参数均在构造时声明，运行期每帧重新读取，因此可以用参数服务在线调整阈值。

坐标与单位
----------
``candidate.pose`` 在候选消息头声明的坐标系里（由候选生成侧决定，日志按相机坐标系
解读为 camera_xyz），位置单位为 m；这里取位置的 z 分量作为深度，并与绝对阈值、
相机可靠测距区间比较。夹爪宽度
``jaw_width`` 与 ``object_length`` 也是 m，只用于汇总展示（判断目标尺寸是否与
当前深度下的测量一致），不参与状态判定。
"""

from __future__ import annotations

import math
from statistics import mean, pstdev
from typing import Optional

import rclpy
from rclpy.node import Node

from rebotarm_msgs.msg import GraspCandidateArray


class GraspDepthProbeNode(Node):
    """采集目标候选深度样本并判定近距离深度可用性的节点。

    生命周期：由 ``main`` 创建后一直 spin，直到采满样本数（回调内主动关闭运行时）
    或被 Ctrl-C 中断。回调在单线程执行器里串行执行，样本列表无需加锁；样本只增不
    清，跨帧累积，因此要求候选在采集期间保持静止。

    参数（全部可在启动时覆盖）：
    - ``input_topic``：候选来源话题，默认 ``/grasp/candidates``。
    - ``sample_count``：汇总所需的样本数，构造时钳到至少 1。
    - ``absolute_min_depth_m``：绝对最近可信距离；更近直接判 ``too_close_unreliable``。
    - ``min_reliable_depth_m``：可靠区下限；介于绝对下限与它之间判 ``near_limit``。
    - ``max_reliable_depth_m``：可靠区上限；更远判 ``too_far``。
    - ``max_depth_std_m``：样本深度标准差上限，超过则整体判定不可用（离散度约束，
      与逐样本状态判定相互独立）。
    - ``target_class``：只采集该类别名的候选；为空表示不按类别筛选。
    """

    def __init__(self) -> None:
        super().__init__("rebotarm_grasp_depth_probe")
        self.declare_parameter("input_topic", "/grasp/candidates")
        self.declare_parameter("sample_count", 20)
        self.declare_parameter("min_reliable_depth_m", 0.20)
        self.declare_parameter("absolute_min_depth_m", 0.15)
        self.declare_parameter("max_reliable_depth_m", 5.0)
        self.declare_parameter("max_depth_std_m", 0.02)
        self.declare_parameter("target_class", "")
        self._samples: list[float] = []
        self._last_widths: list[float] = []
        self._last_lengths: list[float] = []
        # 样本数为 0 会导致永远无法汇总，这里构造时就钳到至少 1。
        self._sample_count = max(int(self.get_parameter("sample_count").value), 1)
        self._target_class = str(self.get_parameter("target_class").value).strip()
        self.create_subscription(
            GraspCandidateArray,
            str(self.get_parameter("input_topic").value),
            self._on_candidates,
            10,
        )
        self.get_logger().info(
            "grasp depth probe ready: "
            f"input={self.get_parameter('input_topic').value}, samples={self._sample_count}"
        )

    def _on_candidates(self, msg: GraspCandidateArray) -> None:
        """每帧候选回调：选一个目标、记一组样本、打印状态，采满后输出汇总并退出。"""
        candidate = self._select_candidate(msg)
        if candidate is None:
            self.get_logger().warn("no grasp candidate received in this frame")
            return
        z = float(candidate.pose.position.z)
        width = float(candidate.jaw_width)
        length = float(candidate.object_length)
        self._samples.append(z)
        self._last_widths.append(width)
        self._last_lengths.append(length)
        status = self._depth_status(z)
        self.get_logger().info(
            f"sample {len(self._samples)}/{self._sample_count}: "
            f"class={candidate.class_name} conf={float(candidate.confidence):.3f} "
            f"camera_xyz=({float(candidate.pose.position.x):.3f}, "
            f"{float(candidate.pose.position.y):.3f}, {z:.3f}) "
            f"jaw={width:.3f} object_length={length:.3f} depth_status={status}"
        )
        if len(self._samples) >= self._sample_count:
            self._print_summary()
            rclpy.shutdown()

    def _select_candidate(self, msg: GraspCandidateArray):
        """从一帧候选中挑出要采样的目标。

        指定 ``target_class`` 时按类别名精确匹配（区分大小写），找不到就返回 None——
        宁可本帧不采样，也不混入其他类别的深度。未指定类别时优先用数组的
        ``best_index``；下标非法（空列表为 -1 或越界）时退回第一个候选。
        """
        if not msg.candidates:
            return None
        if self._target_class:
            for candidate in msg.candidates:
                if str(candidate.class_name) == self._target_class:
                    return candidate
            return None
        best = int(getattr(msg, "best_index", -1))
        if 0 <= best < len(msg.candidates):
            return msg.candidates[best]
        return msg.candidates[0]

    def _depth_status(self, depth_m: float) -> str:
        """给出单个深度样本的状态标签。

        返回值为稳定的英文枚举（会被写入日志并被汇总逻辑复用，不可翻译）：
        "invalid"（非有限值或非正，通常是无效测量）、"too_close_unreliable"
        （近于绝对下限，测距不可信）、"near_limit"（在绝对下限与可靠下限之间，勉强
        可用）、"too_far"（超过可靠上限）、"reliable"（落在可靠区间内）。
        """
        if not math.isfinite(depth_m) or depth_m <= 0.0:
            return "invalid"
        absolute_min = float(self.get_parameter("absolute_min_depth_m").value)
        min_reliable = float(self.get_parameter("min_reliable_depth_m").value)
        max_reliable = float(self.get_parameter("max_reliable_depth_m").value)
        if depth_m < absolute_min:
            return "too_close_unreliable"
        if depth_m < min_reliable:
            return "near_limit"
        if depth_m > max_reliable:
            return "too_far"
        return "reliable"

    def _print_summary(self) -> None:
        """输出样本统计与最终结论。

        统计前先剔除无效深度（非有限或非正）。整体可用需同时满足两条：所有有效样本
        状态都是 "reliable"（即在可靠区间内），且样本标准差不超过 ``max_depth_std_m``
        （重复测量一致性足够）。只有一个样本时标准差按 0 处理。标准差用的是总体
        标准差，样本本身就是全部观测，不做无偏修正。
        """
        depths = [value for value in self._samples if math.isfinite(value) and value > 0.0]
        if not depths:
            self.get_logger().error("summary: no valid depth samples")
            return
        avg = mean(depths)
        std = pstdev(depths) if len(depths) > 1 else 0.0
        min_depth = min(depths)
        max_depth = max(depths)
        max_std = float(self.get_parameter("max_depth_std_m").value)
        reliable_count = sum(1 for value in depths if self._depth_status(value) == "reliable")
        usable = reliable_count == len(depths) and std <= max_std
        self.get_logger().info(
            "summary: "
            f"samples={len(depths)}, depth_min={min_depth:.3f}, depth_max={max_depth:.3f}, "
            f"depth_mean={avg:.3f}, depth_std={std:.3f}, "
            f"jaw_mean={mean(self._last_widths):.3f}, object_length_mean={mean(self._last_lengths):.3f}, "
            f"gemini2_close_depth_usable={usable}"
        )
        if usable:
            self.get_logger().info("result: depth appears usable for close-range re-localization")
        else:
            self.get_logger().warn(
                "result: depth is not reliable enough for pregrasp re-localization; "
                "prefer one-shot grasping or a farther coarse-pregrasp"
            )


def main(args: Optional[list[str]] = None) -> None:
    """命令行入口：初始化运行时、创建节点并 spin。

    Ctrl-C（KeyboardInterrupt）视为正常结束。注意收尾顺序：只有在运行时仍然有效时才
    销毁节点并关闭；若回调已经采满样本主动关闭过运行时，这里不能再关一次。
    """
    rclpy.init(args=args)
    node = GraspDepthProbeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()

"""抓取候选的 IK 与碰撞可行性过滤节点（视觉链路中候选生成之后、执行之前的一层）。

职责与在系统中的位置：订阅上游视觉节点给出的抓取候选数组，为每个候选展开多组
"接近点 + 抓取点"目标位姿，逐个调用 MoveIt 的逆解服务求关节解、再用状态有效性服务
做碰撞检查，最后按几何闸门与可动性代价打分排序，发布过滤后的候选数组与一份抓取计划。
本节点只做校验与排序：既不发送轨迹，也不直接驱动硬件；真正的规划、碰撞检查与执行
门控由下游执行器负责。

对外接口：
- 订阅 ``input_topic``（默认 /grasp/candidates）：上游候选数组，位姿坐标系由消息
  header.frame_id 给出；
- 订阅 ``joint_state_topic``（默认 /rebotarm/visual_joint_states）：当前关节角，
  既作 IK 的种子状态，也用于计算关节位移代价；
- 发布 ``output_topic``（默认 /grasp/filtered_candidates）：保留下来并按得分降序的候选；
- 发布 ``output_plan_topic``（默认 /grasp/filtered_plan）：最优候选的完整计划
  （source=candidate_ik_filter），供执行器消费；
- 调用 ``moveit_ik_service``（默认 /compute_ik）与 ``collision_check_service``
  （默认 /check_state_validity）。

单帧处理流程：候选级预检（置信度、夹爪宽度）→ 位姿变体展开（位姿策略、偏航角偏移、
高度偏移、平行夹爪对称姿态）→ 几何闸门（夹爪宽度、最低抓取高度、可选工作空间盒）→
逐变体 IK + 碰撞检查 → 关节位移与 joint6 变化量评估 → 同一候选取最高分变体 → 整帧排序发布。

安全约束：帧处理串行化并只保留最新一帧，避免服务调用堆积；没有有效关节状态时直接
跳过 IK，不把空的机器人状态交给求解器；服务不可用、超时、返回空等异常一律按"不可行"
处理（保守失败），绝不因为校验环节故障而放行候选。
"""

from __future__ import annotations

import math
import time
from copy import deepcopy
from threading import RLock

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPositionIK, GetStateValidity
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

from rebotarm_msgs.msg import GraspCandidateArray, GraspPlan

from ..policies.candidate_filter_policy import filter_candidate_array_by_reachability
from ..candidate_ik_config import CandidateIkConfig
from ..candidate_ik_gateway import CandidateIkGateway
from ..policies.candidate_ik_policy import CandidateIkPolicy
from ..candidate_ik_runtime import CandidateIkRuntime
from ..candidate_tf_adapter import transform_candidate_pose_to_target_frame
from ..utils.latest_only_work_queue import LatestOnlyWorkQueue
from ..visual_grasp_sequence import PoseTarget


def _parameter(node, name: str):
    reader = getattr(node, "_parameter", None)
    return reader(name) if reader is not None else node.get_parameter(name)


def _pose_from_target(target: PoseTarget) -> Pose:
    """把内部位姿目标转换为 ROS 位姿消息。

    内部约定：位置为 (x, y, z) 三元组、单位 m；姿态为 (x, y, z, w) 四元数，顺序与 ROS
    字段一致，因此此处只做逐字段拷贝与 float 转换，不做任何坐标变换。
    """
    pose = Pose()
    pose.position.x = float(target.position[0])
    pose.position.y = float(target.position[1])
    pose.position.z = float(target.position[2])
    pose.orientation.x = float(target.orientation[0])
    pose.orientation.y = float(target.orientation[1])
    pose.orientation.z = float(target.orientation[2])
    pose.orientation.w = float(target.orientation[3])
    return pose


class CandidateIkFilterNode(Node):
    """抓取候选 IK / 碰撞可行性过滤节点。

    生命周期：构造时声明全部参数、建立 TF 缓冲与监听、创建逆解与状态有效性服务客户端、
    两个发布者和两个订阅者；``main`` 中用 3 线程多线程执行器自旋。节点不保存跨帧的抓取
    状态，处理完一帧立即发布结果，因此随时可以重启而不遗留状态。

    线程模型：候选回调与关节状态回调位于同一个可重入回调组，可以被并发调用；但耗时处理
    由"只保留最新一帧"的串行队列保护，同一时刻只有一个工作项在跑，处理期间到达的帧被
    合并丢弃，从而限制服务调用速率、保证下一帧永远基于最新输入。
    """

    def __init__(self) -> None:
        super().__init__("rebotarm_grasp_candidate_ik_filter")
        self._callback_group = ReentrantCallbackGroup()
        # ---- 话题、坐标系与服务 ----
        # 输入候选话题：上游视觉节点发布的抓取候选数组，位姿坐标系见消息 header.frame_id
        self.declare_parameter("input_topic", "/grasp/candidates")
        # 输出候选话题：过滤并排序后的候选数组，header 沿用输入帧
        self.declare_parameter("output_topic", "/grasp/filtered_candidates")
        # 输出计划话题：只含最优候选的抓取计划（接近点 + 抓取点），供下游执行器消费
        self.declare_parameter("output_plan_topic", "/grasp/filtered_plan")
        # 统一的求解与发布坐标系（默认机械臂基座系 base_link），所有位姿单位 m
        self.declare_parameter("target_frame", "base_link")
        # 逆解服务名（MoveIt 的 compute_ik）；该服务只求关节解，不产生任何运动
        self.declare_parameter("moveit_ik_service", "/compute_ik")
        # 关节状态话题：IK 的种子状态与关节位移代价的当前值，必须来自有反馈的关节源
        self.declare_parameter("joint_state_topic", "/rebotarm/visual_joint_states")
        # 是否调用状态有效性服务做碰撞/越限检查；关闭后仅凭 IK 有解判定可行，属调试手段
        self.declare_parameter("collision_check_enabled", True)
        # 状态有效性服务名（MoveIt 的 check_state_validity）
        self.declare_parameter("collision_check_service", "/check_state_validity")
        # 碰撞检查使用的规划组：必须包含夹爪，否则夹爪与目标/桌面的干涉查不出来
        self.declare_parameter("collision_group_name", "arm_with_gripper")
        # 逆解使用的规划组名（只含手臂关节）
        self.declare_parameter("moveit_group_name", "arm")
        # 逆解的目标连杆（TCP 参考连杆），与 tcp_offset_xyz 配套解读
        self.declare_parameter("ee_frame_id", "end_link")
        # 服务等待与单次调用超时，单位 s；调小会让慢响应被误判为无解，调大会拖慢整帧
        self.declare_parameter("service_timeout_sec", 5.0)
        # ---- 候选位姿变体生成 ----
        # 正式候选过滤路线只保留 GraspNet 的完整候选姿态。
        self.declare_parameter("pose_policy", "preserve_candidate_pose")
        # base_axis 策略使用的固定末端姿态（四元数 x,y,z,w）。当前安装把上游 +X 工作区绕基座 Z
        # 旋转了 -90°，因此默认值是绕 Z 轴 -90° 的四元数；只在 base_axis 及其回退变体中生效
        self.declare_parameter(
            "fixed_grasp_orientation_xyzw",
            [0.0, 0.0, 0.0, 1.0],
        )
        # 基座系进给轴方向（单位向量）：接近点 = 抓取点沿该方向后退 base_pregrasp_distance_m，
        # 默认沿基座 -Y 进入，与上面旋转后的工作区朝向一致
        self.declare_parameter("base_approach_axis_xyz", [1.0, 0.0, 0.0])
        # 接近点到抓取点的距离，单位 m：太小会侧向蹭到目标，太大会拉长接近行程与节拍
        self.declare_parameter("base_pregrasp_distance_m", 0.08)
        # 偏航角偏移列表，单位 rad；每个元素生成一组姿态变体，用于绕竖直轴尝试不同抓取朝向
        self.declare_parameter("orientation_yaw_offsets_rad", [0.0])
        # 抓取高度附加偏移列表，单位 m；每个元素生成一组变体，用于在目标高度方向微调
        self.declare_parameter("candidate_grasp_z_offsets_m", [0.0])
        # 每帧最多处理的候选数（下限被钳到 1）；超出部分直接丢弃，用于限制单帧 IK 调用量
        self.declare_parameter("max_candidates_per_frame", 20)
        # 置信度下限，无量纲 [0, 1]；低于该值的候选在生成位姿前就被丢弃
        self.declare_parameter("candidate_min_confidence", 0.0)
        # 过滤统计日志的最小间隔，单位 s；<= 0 表示每帧都打印
        self.declare_parameter("filter_stats_log_interval_sec", 5.0)
        # TCP 相对末端连杆的偏移（末端连杆局部坐标系，单位 m）；求逆解前会先从目标点减去该
        # 偏移，把"TCP 应到达的点"换算成"末端连杆应到达的位姿"
        self.declare_parameter("tcp_offset_xyz", [-0.04, 0.0, 0.0])
        # 目标点在基座系下的平移修正，单位 m；用于补偿手眼/TCP 标定残差
        self.declare_parameter("target_base_offset_xyz", [0.0, 0.0, 0.0])
        # 接近点最低高度钳位，单位 m；低于该值会被抬高到该值，避免接近点插进桌面
        self.declare_parameter("candidate_pregrasp_min_z_m", 0.04)
        # 抓取点整体高度偏移，单位 m；与每个变体的高度偏移叠加
        self.declare_parameter("grasp_base_z_offset_m", 0.0)
        # ---- 几何闸门与打分权重 ----
        # 夹爪可信行程下限，单位 m；小于该开口认为抓不稳目标
        self.declare_parameter("candidate_min_jaw_width_m", 0.006)
        # 夹爪可信行程上限，单位 m；大于该开口认为夹爪张不到位（会被夹爪行程截断）
        self.declare_parameter("candidate_max_jaw_width_m", 0.085)
        # 抓取点最低允许高度，单位 m（基座系）；低于该值判为贴地或穿桌
        self.declare_parameter("candidate_min_grasp_z_m", 0.0)
        # 是否启用工作空间盒闸门；默认关闭，启用后抓取点必须落在下面的盒范围内
        self.declare_parameter("candidate_workspace_gate_enabled", False)
        # 工作空间盒最小角（基座系，单位 m）；沿用旧仓库 +X 工作区。
        self.declare_parameter("candidate_workspace_min_xyz", [0.18, -0.35, 0.0])
        # 工作空间盒最大角（基座系，单位 m）
        self.declare_parameter("candidate_workspace_max_xyz", [0.64, 0.35, 0.45])
        # 抓取点到目标中心的最大允许距离，单位 m；超出说明抓取点已偏离物体（深度或分割异常）
        self.declare_parameter("candidate_max_grasp_to_object_center_m", 0.15)
        # 关节位移在评分中的权重（每 rad 的扣分）；越大越偏好关节动作小的解
        self.declare_parameter("candidate_score_joint_distance_weight", 0.15)
        # joint6 变化量在评分中的权重（每 rad 的扣分）；腕部旋转代价单独加权，抑制绕线
        self.declare_parameter("candidate_score_joint6_weight", 0.35)
        # joint6 变化量上限，单位 rad（默认 1.5708 ≈ 90°）；超过直接判该解不可行，防止腕部大幅翻转
        self.declare_parameter("candidate_max_joint6_delta_rad", 1.5708)
        # 是否追加平行夹爪对称姿态变体：平行夹爪绕末端 x 轴转 180° 后仍是同一次抓取
        self.declare_parameter("candidate_joint6_symmetry_enabled", True)
        # 对称变体的旋转角，单位 rad（默认 π）；只有平行夹爪的 180° 对称才物理等价
        self.declare_parameter("candidate_joint6_symmetry_angle_rad", math.pi)

        self._input_topic = str(self.get_parameter("input_topic").value)
        self._target_frame = str(self.get_parameter("target_frame").value)
        self._service_timeout_sec = float(self.get_parameter("service_timeout_sec").value)
        # 最近一帧有效关节状态，作为 IK 种子；None 表示尚未收到可用反馈
        self._latest_joint_state: JointState | None = None
        self._joint_state_lock = RLock()
        # 关节状态缺失告警的去重标志，避免每帧刷屏
        self._warned_missing_joint_state = False
        # 只保留最新一帧的串行工作队列：处理耗时的帧运行期间到达的帧会被合并丢弃
        self._work_queue: LatestOnlyWorkQueue[GraspCandidateArray] = LatestOnlyWorkQueue()
        # 上次打印过滤统计的时刻（单调时钟，s），配合 filter_stats_log_interval_sec 限流
        self._last_filter_stats_log_at = 0.0
        self._config = CandidateIkConfig.from_node(self)
        # TF 缓冲与监听：用于把候选位姿从相机系等来源坐标系变换到 target_frame
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        # 逆解客户端：仅用于可行性校验，绝不触发任何运动
        self._ik_client = self.create_client(
            GetPositionIK,
            str(_parameter(self, "moveit_ik_service").value),
            callback_group=self._callback_group,
        )
        # 状态有效性客户端：对 IK 解做碰撞/越限检查
        self._state_validity_client = self.create_client(
            GetStateValidity,
            str(_parameter(self, "collision_check_service").value),
            callback_group=self._callback_group,
        )
        # 过滤后的候选数组发布者
        self._candidates_pub = self.create_publisher(
            GraspCandidateArray,
            str(_parameter(self, "output_topic").value),
            10,
        )
        # 抓取计划发布者：只有最优候选的接近点/抓取点，是执行器真正消费的消息
        self._plan_pub = self.create_publisher(
            GraspPlan,
            str(_parameter(self, "output_plan_topic").value),
            10,
        )
        self._gateway = self._make_gateway(self._config, None)
        # 候选订阅：回调可重入，实际重活由下面的串行队列限制并发
        self.create_subscription(
            GraspCandidateArray,
            self._input_topic,
            self._on_candidates,
            10,
            callback_group=self._callback_group,
        )
        # 关节状态订阅：只缓存最新有效帧，供逆解与位移代价使用
        self.create_subscription(
            JointState,
            str(_parameter(self, "joint_state_topic").value),
            self._on_joint_state,
            10,
            callback_group=self._callback_group,
        )
        self.get_logger().info(
            "candidate IK filter ready: "
            f"input={self._input_topic}, output={str(_parameter(self, 'output_topic').value)}, "
            f"plan={str(_parameter(self, 'output_plan_topic').value)}"
        )

    def _make_gateway(self, config, joint_state):
        return CandidateIkGateway(
            config=config, ik_client=self._ik_client,
            validity_client=self._state_validity_client, tf_buffer=self._tf_buffer,
            joint_state=joint_state, logger=self.get_logger(),
            publish_ranked=self._publish_ranked,
            publish_empty=lambda msg: self._publish_filtered(msg, []),
        )

    def _parameter(self, name: str):
        """读取当前候选帧配置；构造阶段回退到 ROS 参数对象。"""
        config = getattr(self, "_config", None)
        if config is not None and name in config.values:
            return type("ParameterValue", (), {"value": config.values[name]})()
        try:
            return super().get_parameter(name)
        except AttributeError:
            return self.get_parameter(name)

    def _refresh_config(self) -> None:
        """在每帧开始时复制一次参数，保证该帧不混用新旧值。"""
        self._config = CandidateIkConfig.from_node(self)
        self._input_topic = self._config.input_topic
        self._target_frame = self._config.target_frame
        self._service_timeout_sec = self._config.service_timeout_sec



    def _on_joint_state(self, msg: JointState) -> None:
        """缓存最近一帧有效关节状态；空或不完整的帧只告警一次，不污染 IK 种子。"""
        if self._valid_joint_state(msg):
            with self._joint_state_lock:
                self._latest_joint_state = deepcopy(msg)
                self._warned_missing_joint_state = False
            return
        with self._joint_state_lock:
            should_warn = not self._warned_missing_joint_state
            self._warned_missing_joint_state = True
        if should_warn:
            self.get_logger().warn(
                "candidate IK filter ignored empty or incomplete joint state; "
                "waiting before calling MoveIt IK"
            )

    def _joint_state_snapshot(self) -> JointState | None:
        """返回供单帧处理使用的独立关节状态快照。"""
        with self._joint_state_lock:
            return deepcopy(self._latest_joint_state)

    def _valid_joint_state(self, msg: JointState | None) -> bool:
        """判断关节状态能否作为 IK 种子。

        要求：消息非空、name 与 position 都非空、position 数量不少于 name 数量，且 name
        对应的 position 全为有限值。数量不足会让求解器拿到缺关节的机器人状态；NaN/Inf
        则会让逆解直接失败或返回不可预期的解，因此都必须在入口挡住。
        """
        if msg is None:
            return False
        if not msg.name or not msg.position:
            return False
        if len(msg.position) < len(msg.name):
            return False
        return all(math.isfinite(float(value)) for value in msg.position[: len(msg.name)])

    def _on_candidates(self, msg: GraspCandidateArray) -> None:
        """候选数组订阅回调：交给最新优先队列后串行处理。

        队列保证同一时刻只有一个工作项在跑，处理期间新到的帧只保留最新一帧，因此这里用
        while 循环把"当前帧 + 处理期间积压的最新帧"依次处理完，直到队列空闲。
        单帧内部异常只记录错误日志，不打断后续帧的处理。
        """
        work = self._work_queue.submit(msg, received_at=time.monotonic())
        while work is not None:
            started_at = time.monotonic()
            counts: dict[str, int] = {}
            try:
                counts = self._on_candidates_unlocked(work.item)
            except Exception as exc:
                self.get_logger().error(f"candidate IK filter frame failed: {exc}")
            completed_at = time.monotonic()
            # 单帧实际处理耗时（含服务调用等待），单位 ms
            processing_ms = (completed_at - started_at) * 1000.0
            # 出队即表示本帧处理结束；返回不为空说明处理期间又积压了更新的一帧
            work = self._work_queue.complete(completed_at=completed_at)
            self._log_filter_stats(
                processing_ms=processing_ms,
                pending_age_ms=0.0 if work is None else work.pending_age_ms,
                counts=counts,
            )

    def _log_filter_stats(
        self,
        *,
        processing_ms: float,
        pending_age_ms: float,
        counts: dict[str, int],
    ) -> None:
        """按 filter_stats_log_interval_sec 限流打印过滤统计。

        统计内容包括队列的收到/开始/完成/合并帧数与忙碌状态、本帧耗时、积压帧年龄，
        以及本帧各阶段的候选计数；限流是为了避免高频相机帧把日志刷爆。
        """
        now = time.monotonic()
        # 间隔 <= 0 视为不限流（每帧都打印）；否则未到间隔直接返回
        interval = max(0.0, float(_parameter(self, "filter_stats_log_interval_sec").value))
        if interval > 0.0 and now - self._last_filter_stats_log_at < interval:
            return
        self._last_filter_stats_log_at = now
        stats = self._work_queue.snapshot()
        self.get_logger().info(
            "candidate_filter_stats "
            f"received={stats.received} started={stats.started} completed={stats.completed} "
            f"coalesced={stats.coalesced} busy={str(stats.busy).lower()} "
            f"pending={str(stats.pending).lower()} processing_ms={processing_ms:.1f} "
            f"pending_age_ms={pending_age_ms:.1f} "
            f"input_candidates={counts.get('input_candidates', 0)} "
            f"precheck_passed={counts.get('precheck_passed', 0)} "
            f"geometry_variants_passed={counts.get('geometry_variants_passed', 0)} "
            f"ranked={counts.get('ranked', 0)}"
        )

    # 运行时负责处理循环；节点方法作为策略与网关适配器注入。
    def _on_candidates_unlocked(self, msg: GraspCandidateArray) -> dict[str, int]:
        """把一帧候选交给运行时编排器处理。"""
        self._refresh_config()
        config = self._config
        joint_state = self._joint_state_snapshot()
        gateway = self._make_gateway(config, joint_state)
        policy = CandidateIkPolicy(
            config=config, joint_state=joint_state,
            transform_pose=lambda pose, frame, stamp=None: transform_candidate_pose_to_target_frame(
                pose, source_frame=frame, target_frame=config.target_frame,
                stamp=stamp, lookup_transform=gateway.lookup_transform,
            ), logger=self.get_logger(),
        )
        self._gateway = gateway
        max_candidates = config.max_candidates_per_frame
        runtime = CandidateIkRuntime(
            policy=policy, gateway=gateway, max_candidates=max_candidates, logger=self.get_logger()
        )
        return runtime.filter_frame(msg)





    def _lookup_transform(self, target_frame: str, source_frame: str, stamp=None):
        """查询候选采集时刻的 TF，超时 0.2 s。

        取不到变换时本方法会抛异常，由调用方按"无法校验"处理：候选级异常会丢弃该候选，
        目标中心变换失败则跳过依赖该变换的距离检查。
        """
        return self._gateway.lookup_transform(target_frame, source_frame, stamp)

    def _transform_pose_to_target_frame(self, pose: Pose, source_frame: str, stamp=None) -> Pose:
        """把位姿从 source_frame 变换到 target_frame；两者相同或为空时返回原样副本。"""
        return transform_candidate_pose_to_target_frame(
            pose,
            source_frame=source_frame,
            target_frame=self._target_frame,
            stamp=stamp,
            lookup_transform=self._lookup_transform,
        )



    def _check_ik_and_collision(self, target: PoseTarget, label: str):
        """对单个目标位姿做完整可行性检查：逆解有解且状态有效才返回解，否则返回 None。

        返回值为逆解得到的机器人状态；碰撞检查关闭时只做逆解。
        """
        solution = self._gateway.solve_ik(target, label)
        if solution is None:
            return None
        if not self._gateway.check_state_validity(solution, label):
            return None
        return solution



    def _target_debug_text(self, target: PoseTarget) -> str:
        """把目标位姿格式化成日志文本（位置 3 位小数，单位 m；四元数 4 位小数）。"""
        return (
            f"target=({target.position[0]:.3f}, {target.position[1]:.3f}, {target.position[2]:.3f}), "
            f"orientation=({target.orientation[0]:.4f}, {target.orientation[1]:.4f}, "
            f"{target.orientation[2]:.4f}, {target.orientation[3]:.4f})"
        )




    def _publish_filtered(
        self,
        original: GraspCandidateArray,
        reachable: list[bool],
        reachable_targets: list[tuple[PoseTarget, PoseTarget]] | None = None,
    ) -> None:
        """按可达标记过滤后发布候选数组，并发布由过滤结果组装的抓取计划。

        reachable 是与输入候选一一对应的布尔标记（本节点在无候选可处理时传入空列表）；
        reachable_targets 为空时，计划退化为候选原始位姿，不带接近点/抓取点。
        """
        filtered = filter_candidate_array_by_reachability(original, reachable)
        self._candidates_pub.publish(filtered)
        plan = self._plan_from_filtered(filtered, reachable_targets or [])
        self._plan_pub.publish(plan)

    def _publish_ranked(
        self,
        original: GraspCandidateArray,
        ranked: list[tuple[float, int, object, tuple[PoseTarget, PoseTarget], str, str]],
    ) -> None:
        """按得分降序发布候选数组与最优抓取计划。

        排序键为 (-得分, 原始下标)：得分高者在前，同分时保持上游顺序，结果稳定可复现。
        计划中的 best_index 在列表非空时固定为 0（已排序），空列表时为 -1。
        """
        ranked = sorted(ranked, key=lambda item: (-float(item[0]), int(item[1])))
        filtered = GraspCandidateArray()
        filtered.header = original.header
        filtered.best_index = 0 if ranked else -1
        # 候选消息整体深拷贝：下游可能改写内容，不能与输入消息共用对象
        filtered.candidates = [deepcopy(item[2]) for item in ranked]
        targets = [item[3] for item in ranked]
        self._candidates_pub.publish(filtered)
        plan = self._plan_from_filtered(filtered, targets)
        if ranked:
            score, original_index, _candidate, _targets, label, motion_reason = ranked[0]
            # 计划里的 reason 在有效时用于记录选择依据：原始下标、得分、变体标签与可动性说明
            plan.reason = (
                f"best_candidate original_index={original_index}, score={score:.2f}, "
                f"variant={label}, {motion_reason}"
            )
            self.get_logger().info(f"candidate IK filter best: {plan.reason}")
        self._plan_pub.publish(plan)

    def _plan_from_filtered(
        self,
        filtered: GraspCandidateArray,
        reachable_targets: list[tuple[PoseTarget, PoseTarget]],
    ) -> GraspPlan:
        """由过滤后的候选数组与对应的可达目标点组装抓取计划。

        best_index < 0 或候选为空时返回 valid=False 的计划（reason 说明原因），下游必须
        据此放弃抓取。有可达目标点时，计划头改用 target_frame 并写入该变体的接近点/抓取点；
        没有目标点时退化为候选原始位姿（接近点与抓取点相同）。
        """
        plan = GraspPlan()
        plan.header = filtered.header
        # 计划来源标识：下游执行器据此判断位姿已由本节点在 target_frame 中生成，无需再套用位姿策略
        plan.source = "candidate_ik_filter"
        if filtered.best_index < 0 or not filtered.candidates:
            plan.valid = False
            plan.reason = "no IK-reachable grasp candidates"
            return plan
        candidate = filtered.candidates[int(filtered.best_index)]
        plan.candidate = candidate
        if reachable_targets:
            pregrasp, grasp = reachable_targets[int(filtered.best_index)]
            plan.pregrasp_pose = _pose_from_target(pregrasp)
            plan.grasp_pose = _pose_from_target(grasp)
            # 位姿已在 target_frame 下生成，计划头必须同步改写，否则下游会按原坐标系解读
            plan.header.frame_id = self._target_frame
        else:
            # 没有可达目标点时只能退化为候选原始位姿，接近点与抓取点相同
            plan.grasp_pose = candidate.pose
            plan.pregrasp_pose = candidate.pose
        plan.jaw_width = float(candidate.jaw_width)
        plan.valid = True
        plan.reason = ""
        return plan


def main(args=None) -> None:
    """节点入口：用 3 线程的多线程执行器自旋，Ctrl+C 或外部关闭时正常退出。"""
    rclpy.init(args=args)
    node = CandidateIkFilterNode()
    # 3 线程：候选回调在等待 IK / 状态有效性服务返回期间，仍需处理关节状态与 TF 回调
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        # 用户中断与外部关闭都按正常退出处理，不再向上抛异常
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        # 仅在上下文仍有效时关闭，避免二次 shutdown 抛错
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

from __future__ import annotations

from copy import deepcopy
import time

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from rebotarm_msgs.msg import GraspCandidateArray, GraspPlan
from rebotarm_msgs.srv import ExecutePose, GraspGripper, PublishTrajectoryPreview, SetGripper

from ..utils.message_freshness import is_message_fresh, message_age_sec
from ..visual_grasp_config import VisualGraspConfig
from ..visual_grasp_io import VisualGraspIoGateway
from ..visual_grasp_io import io_gateway_for as _io_gateway_for
from ..visual_grasp_sequence import (
    PoseTarget,
)
from ..visual_grasp_state import VisualGraspState, _state_for
from ..visual_grasp_workflow import VisualGraspWorkflow


def _parameter(node, name: str):
    reader = getattr(node, "_parameter", None)
    return reader(name) if reader is not None else node.get_parameter(name)



# 视觉抓取执行节点：把"视觉给出的抓取计划"翻译成一条受门控的机械臂动作序列。
#
# 本节点是视觉链路的执行入口，本身不做感知，也不直接驱动硬件，只做编排。
#
# 输入
#   - 订阅 `input_topic`（默认 `/grasp/filtered_plan`，类型为抓取计划消息）：
#     上游已过滤/校验过的抓取计划，含接近点位姿、抓取点位姿、类别、置信度与夹爪开口宽度。
#   - 订阅 `candidates_topic`（默认 `/grasp/filtered_candidates`，类型为抓取候选数组消息）：
#     同一次规划的候选列表，用于失败后按顺序自动换候选重试。
#
# 输出（全部为服务调用，服务名前缀 `/{arm_namespace}`，默认命名空间 `rebotarm`）
#   - `/visual_grasp/execute`（Trigger）：收到请求后同步跑完整条抓取序列，返回值即最终结论。
#   - `/visual_grasp/stop`（Trigger）：请求中止，置运行标志为假并调用运动停止。
#   - stop 成功须确认旧请求结束、Action 终态和新鲜静止反馈；不自动回位。
#   - 向下游调用 `/motion_execution/execute_pose`（末端位姿规划/执行）、
#     `/motion_execution/stop` 与 `/trajectory_stop`（急停）、`/safe_home`（回安全位）、
#     `/gripper/set`（夹爪位置控制）与 `/gripper/grasp`（力闭合抓取）。
#
# 典型阶段序列（由序列构建策略按参数生成）：
#   [open_gripper] → move_to_pregrasp → approach_grasp → close_gripper
#   → [safe_retreat] → [safe_home] → [move_to_place → open_gripper_at_place → place_retreat]
#   方括号表示由参数开关控制的可选阶段。
#
# 安全设计（阅读时请重点注意）
#   1. `execution_mode` 决定一切动作是否真正下发：只有取值 execute / real 才视为"执行"；
#      其余（含默认 plan_only）下所有运动只做规划干跑、夹爪与回零命令一律跳过，
#      各段规划成功后由运动层一次性发布连续 RViz 预览；不会逐段重启动画。
#   2. 抓取计划具有时效性：`max_plan_age_sec` 之外的旧计划、时间戳未设置的计划一律拒收，
#      避免用几秒前的检测结果去驱动现在的机械臂。
#   3. 抓取点高度受 `min_grasp_z_m` 下限保护（在序列构建策略中校验），防止规划到桌面以下。
#   4. 每个阶段失败都会走恢复策略：先请求运动停止，再（可选）撤到接近点，
#      只有"允许自动重试 + 该阶段可重试 + 还有剩余候选"三者同时成立才会换候选继续。
#   5. 闭合后立即做抓取验证（接触 + 闭合行程），通过后才沿接近路径反向撤退。
#
# 线程模型：全部回调注册在可重入回调组上，主函数使用三线程执行器。执行服务回调会长时间
# 阻塞（内部用 `rclpy.ok()` 与 `_running` 标志轮询等待），停止服务可在另一个线程里打断它。
# 坐标系：`target_frame`（默认 `base_link`）是下发给运动层的目标坐标系；计划消息里
# `header.frame_id` 与它不一致时用 TF 换算，查询超时 0.2 s、取最新可用变换。


def pose_to_target(pose: Pose) -> PoseTarget:
    """把位姿消息转成内部轻量元组表示（位置 3 元组 + 四元数 4 元组，单位均为 m / 无量纲）。"""

    return PoseTarget(
        position=(float(pose.position.x), float(pose.position.y), float(pose.position.z)),
        orientation=(
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        ),
    )


def target_to_pose_stamped(target: PoseTarget, frame_id: str) -> PoseStamped:
    """把内部位姿目标打包成带坐标系的位姿消息。

    `frame_id` 即目标坐标系（本节点默认 `base_link`），下游运动层按该坐标系解释位置。
    时间戳刻意填 0（rclpy 的零时刻），表示"使用最新可用变换"，避免用本地时钟去要求
    运动层做时间对齐；位置单位为 m，姿态为四元数。
    """

    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = rclpy.time.Time().to_msg()
    msg.pose.position.x = float(target.position[0])
    msg.pose.position.y = float(target.position[1])
    msg.pose.position.z = float(target.position[2])
    msg.pose.orientation.x = float(target.orientation[0])
    msg.pose.orientation.y = float(target.orientation[1])
    msg.pose.orientation.z = float(target.orientation[2])
    msg.pose.orientation.w = float(target.orientation[3])
    return msg


class VisualGraspExecutorNode(Node):
    """抓取执行器节点：订阅抓取计划/候选，对外提供执行与中止两个服务。

    生命周期：进程启动即建立 TF 监听、六个下游服务客户端、两个订阅与两个服务；
    节点本身不保存"机械臂状态"，只保存最近一次有效计划与候选，以及本轮执行过程的
    快照（最近夹爪到位位置、是否检出接触、闭合行程），供失败诊断使用。

    回调模型：所有订阅与服务都在同一个可重入回调组，主线程池为三线程。抓取序列在
    `/visual_grasp/execute` 的回调里同步执行，期间由 `_running` 标志与 `rclpy.ok()`
    控制所有等待循环；`/visual_grasp/stop` 只需把该标志置假即可让序列在下一个检查点退出。

    安全语义：本类不对硬件做任何直接操作，所有动作都必须经过下游运动执行服务的
    规划与执行门控；执行模式不是 `execute`/`real` 时，运动命令降级为纯规划干跑，
    夹爪与回零命令被跳过，因此默认参数下不会让机械臂真实运动。
    """

    def __init__(self) -> None:
        super().__init__("rebotarm_visual_grasp_executor")
        self._callback_group = ReentrantCallbackGroup()
        self._stop_callback_group = MutuallyExclusiveCallbackGroup()

        # ── 输入话题与坐标系 ────────────────────────────────────────────────
        # 机械臂命名空间，用于拼接所有下游服务名；前导斜杠会被 strip 掉
        self.declare_parameter("arm_namespace", "rebotarm")
        # 抓取计划订阅话题；上游候选过滤节点的输出，必须与本节点收到的计划格式一致
        self.declare_parameter("input_topic", "/grasp/filtered_plan")
        # 候选数组订阅话题；仅用于失败后换候选重试，缺省时退化为只执行最优计划
        self.declare_parameter("candidates_topic", "/grasp/filtered_candidates")
        # 下发给运动层的目标坐标系；计划消息 frame_id 与之不同则先用 TF 换算
        self.declare_parameter("target_frame", "base_link")

        # ── 位姿换算与接近点构造 ────────────────────────────────────────────
        # TCP（夹持中心）相对末端法兰 end_link 的偏移（m），表达在末端自身坐标系中：
        # 先用目标姿态把它旋转到基座系，再从目标位姿里扣掉，得到法兰应到达的位置。
        # 本站实测量为 [-0.04, 0, 0]，即夹持中心位于法兰沿末端 X 轴前伸 4 cm 处
        # （与仿真模型里 end_link 下的 ee_site 位姿一致）
        self.declare_parameter("tcp_offset_xyz", [-0.04, 0.0, 0.0])
        # 目标点整体平移补偿（m），用于补偿标定/安装残差；默认全 0 表示不补偿
        self.declare_parameter("target_base_offset_xyz", [0.0, 0.0, 0.0])
        # 抓取点额外的 z 偏移（m），仅旧式 visual_pose 策略使用
        self.declare_parameter("grasp_base_z_offset_m", 0.0)
        # 位姿生成策略：base_axis = 用固定姿态 + 基座接近轴现算接近点；
        # visual_pose/source_pose/legacy = 直接采用计划里的位姿；抓取点可叠加上述 z 偏移
        self.declare_parameter("pose_policy", "base_axis")
        # 固定抓取姿态四元数 xyzw，默认绕 Z 转 -90°（本站工作区相对上游 +X 布局旋转而来），
        # 使夹爪闭合方向与目标摆放方向一致
        self.declare_parameter(
            "fixed_grasp_orientation_xyzw",
            [0.0, 0.0, 0.0, 1.0],
        )
        # 基座系下的接近方向单位向量（会归一化）：接近点 = 抓取点沿该轴反向退开一段距离。
        # 默认 [0, -1, 0] 表示从 -Y 侧进给
        self.declare_parameter("base_approach_axis_xyz", [1.0, 0.0, 0.0])
        # 接近点与抓取点的距离（m），越大越保守，但要求工作空间更大
        self.declare_parameter("base_pregrasp_distance_m", 0.08)
        # 抓取点 z 下限（m），低于此值直接判序列构建失败，属桌面碰撞护栏
        self.declare_parameter("min_grasp_z_m", 0.0)

        # ── 夹爪开合与宽度自适应 ────────────────────────────────────────────
        # 是否在接近前先张开夹爪；张开会提前占用空间，狭窄场景可关掉
        self.declare_parameter("open_before_approach", False)
        # 固定张开宽度（m，指两指间距），auto_gripper_width 为真且检测到宽度时被覆写
        self.declare_parameter("open_position_m", 0.09)
        # 固定闭合宽度（m）；同上，可能被自适应结果覆写
        self.declare_parameter("close_position_m", 0.025)
        # 闭合阶段最大力矩（N·m），是实际夹持力旋钮（越大越紧、越容易顶坏目标）
        self.declare_parameter("close_max_effort", 0.4)
        # 是否按检测到的目标宽度自动推算张开/闭合宽度；关闭或未测到宽度时用上下两行的固定值
        self.declare_parameter("auto_gripper_width", True)
        # 张开宽度在检测宽度基础上额外留的余量（m），默认 0 表示刚好等于检测宽度
        self.declare_parameter("open_clearance_m", 0.0)
        # 闭合宽度比检测宽度收窄的量（m），保证闭合时对目标有预压
        self.declare_parameter("close_margin_m", 0.012)
        # 自适应张开宽度下限（m），防止算出的开口过小夹不住
        self.declare_parameter("min_open_position_m", 0.035)
        # 自适应张开宽度上限（m），受夹爪行程限制
        self.declare_parameter("max_open_position_m", 0.09)
        # 自适应闭合宽度下限（m），0 附近即完全闭合
        self.declare_parameter("min_close_position_m", 0.006)
        # 自适应闭合宽度上限（m），防止闭合指令等于张开指令而夹不住
        self.declare_parameter("max_close_position_m", 0.08)
        # 是否按统一上下限约束夹持力矩
        self.declare_parameter("auto_gripper_effort", True)
        # 力矩下限（N·m），太小会夹不住、滑落
        self.declare_parameter("min_gripper_effort", 0.22)
        # 力矩上限（N·m），太大会压坏目标或让夹爪堵转报警
        self.declare_parameter("max_gripper_effort", 0.60)
        # 允许抓取的最大目标宽度（m），超过直接判策略不允许抓（不算夹爪行程上限，留了余量）
        self.declare_parameter("max_allowed_grasp_width_m", 0.085)

        # ── 位置闭合的"接触"判据（无接触传感器，靠行程与位移推断）────────────
        # 是否启用"闭合未到位但疑似夹到"的放宽判据
        self.declare_parameter("close_contact_success_enabled", True)
        # 允许停目标外侧的余量（m）：实际到位 >= 目标 + 该余量即认为被目标挡住
        self.declare_parameter("close_contact_margin_m", 0.004)
        # 相对上次张开位置的最小闭合行程（m），行程不足说明没夹到东西
        self.declare_parameter("close_contact_min_closure_delta_m", 0.015)

        # ── 力闭合抓取服务（堵转推断接触 + 有界保持）────────────────────────
        # close_gripper 阶段是否改用专用抓取服务而非普通位置服务
        self.declare_parameter("gripper_grasp_enabled", True)
        # 闭合阶段力矩（N·m）；硬件层会夹到 [0.05, 1.0]
        self.declare_parameter("gripper_grasp_close_force", 0.4)
        # 闭合超时（s），超时未堵转即判失败
        self.declare_parameter("gripper_grasp_timeout_sec", 8.0)
        # 最短闭合时间（s），防止刚起步速度未建立就被误判为接触
        self.declare_parameter("gripper_grasp_min_close_time_sec", 0.08)
        # 判定"已停住"的角速度阈值（rad/s），低于它才算堵转
        self.declare_parameter("gripper_grasp_velocity_threshold", 0.04)
        # 判定接触所需的最小闭合行程（m）
        self.declare_parameter("gripper_grasp_min_closure_distance_m", 0.006)

        # ── 抓取后安全撤退 ─────────────────────────────────────────────────
        # 是否在闭合后沿本次接近路径反向撤退
        self.declare_parameter("safe_retreat_enabled", True)
        # 撤退平移距离（m）
        self.declare_parameter("safe_retreat_distance_m", 0.06)
        # 是否在序列末尾回安全位（默认关，避免未经授权的大范围回零动作）
        self.declare_parameter("safe_home_after_grasp", False)

        # ── 超时与阶段间停顿 ───────────────────────────────────────────────
        # 等待下游服务可用的超时（s）
        self.declare_parameter("service_timeout_sec", 20.0)
        # move_to_pregrasp 之后的稳定等待（s）
        self.declare_parameter("pregrasp_wait_sec", 0.5)
        # approach_grasp 之后的稳定等待（s）
        self.declare_parameter("approach_wait_sec", 0.2)
        # 是否真正下发夹爪命令；计划模式或仿真下会被强制跳过
        self.declare_parameter("execute_gripper", True)
        # 夹爪开/合阶段的等待（s）
        self.declare_parameter("gripper_wait_sec", 1.0)
        # safe_retreat 阶段完成后的稳定等待（s）
        self.declare_parameter("retreat_wait_sec", 0.5)
        # 单次运动执行的服务端超时（s），会随请求一起下发，因此要覆盖规划耗时
        self.declare_parameter("motion_result_timeout_sec", 45.0)

        # ── 执行模式与速度缩放 ─────────────────────────────────────────────
        # plan_only = 只规划干跑（默认）；execute/real = 真正下发轨迹与夹爪命令
        self.declare_parameter("execution_mode", "plan_only")
        # 仿真停止后的复位由本节点编排；真实硬件仍由操作者确认后单独恢复。
        self.declare_parameter("use_hardware", False)
        # 常规移动的速度缩放，无量纲 (0, 1]；越小越慢越安全
        self.declare_parameter("move_velocity_scaling", 0.25)
        # 接近段速度缩放；比常规更慢，因为此时离目标与台面最近
        self.declare_parameter("approach_velocity_scaling", 0.08)
        # 撤退段速度缩放；比接近段稍快，兼顾"已夹住"与效率
        self.declare_parameter("retreat_velocity_scaling", 0.15)
        # 加速度缩放，所有阶段共用
        self.declare_parameter("acceleration_scaling", 0.12)
        # plan_only 阶段间诊断等待（s）；默认 0，连续预览无需人为停顿
        self.declare_parameter("plan_only_stage_pause_sec", 0.0)

        # ── 到接近点后刷新计划 ─────────────────────────────────────────────
        # 是否在到达接近点后等待更新版本的抓取计划（近距离视觉更准）
        self.declare_parameter("refresh_plan_at_pregrasp_enabled", True)
        # 刷新是否强制：开启且等不到新计划时本次尝试判失败（而不是沿用旧计划）
        self.declare_parameter("refresh_plan_at_pregrasp_required", True)
        # 等待新计划的最长时间（s）
        self.declare_parameter("refresh_plan_timeout_sec", 1.0)

        # ── 接近段视觉伺服（小步逼近 + 用新计划纠偏）──────────────────────
        # 是否用迭代小步替代一次到位的 approach_grasp
        self.declare_parameter("approach_visual_servo_enabled", False)
        # 最大迭代步数，至少为 1
        self.declare_parameter("approach_visual_servo_max_iterations", 5)
        # 单步最大位移（m），限制每步风险
        self.declare_parameter("approach_visual_servo_max_step_m", 0.02)
        # 位置误差容差（m），误差小于它就认为已到位
        self.declare_parameter("approach_visual_servo_position_tolerance_m", 0.008)
        # 是否要求每一步都基于更新的计划；关掉后允许沿用最近一次计划
        self.declare_parameter("approach_visual_servo_require_fresh_plan", True)

        # ── 失败恢复与抓取验证 ─────────────────────────────────────────────
        # 是否允许失败后自动换下一个候选重试（默认关，保持"一次命令一次动作"）
        self.declare_parameter("auto_retry_enabled", False)
        # 自动重试的候选数上限
        self.declare_parameter("auto_retry_max_attempts", 3)
        # 重试前是否先撤到接近点（避免贴着目标换位形）
        self.declare_parameter("safe_retreat_before_retry", True)
        # 是否在闭合后用接触与闭合行程做抓取成功性验证
        self.declare_parameter("grasp_verification_enabled", True)
        # 判定"确实夹住"所需的最小闭合行程（m）
        self.declare_parameter("grasp_verification_min_closure_distance_m", 0.006)
        # 是否要求检出接触，缺失即判验证失败
        self.declare_parameter("grasp_verification_require_contact", True)

        # ── 抓取后放置（可选）──────────────────────────────────────────────
        # 是否在抓取成功后继续执行放置序列
        self.declare_parameter("place_after_grasp_enabled", False)
        # 放置点位置（基座系，m）
        self.declare_parameter("place_position_xyz", [-0.20, -0.20, 0.25])
        # 放置点姿态四元数 xyzw，默认同样是绕 Z 转 -90°，与抓取姿态保持一致
        self.declare_parameter(
            "place_orientation_xyzw",
            [0.0, 0.0, 0.0, 1.0],
        )
        # 放置点松开夹爪的开口宽度（m）
        self.declare_parameter("place_open_position_m", 0.08)
        # 放置点松开夹爪的力矩（N·m）
        self.declare_parameter("place_open_max_effort", 0.25)
        # 松爪后向上撤开的高度（m）
        self.declare_parameter("place_retreat_z_m", 0.06)

        # ── 执行前预检与计划时效 ───────────────────────────────────────────
        # 真正执行前先用 execute=False 走一次规划预检，规划不通过就不执行
        self.declare_parameter("trajectory_precheck_enabled", True)
        # 抓取计划的最大允许时延（s），超时视为过期并拒收
        self.declare_parameter("max_plan_age_sec", 1.0)

        # 读取派生元组前，先构造经过校验的参数快照。
        # _tuple3 统一从该快照读取，使启动配置和显式覆盖值
        # 使用相同的转换流程。
        self._config = VisualGraspConfig.from_node(self)
        self._arm_namespace = str(self.get_parameter("arm_namespace").value).strip("/")
        self._input_topic = str(self.get_parameter("input_topic").value)
        self._candidates_topic = str(self.get_parameter("candidates_topic").value)
        self._target_frame = str(self.get_parameter("target_frame").value).strip()
        self._tcp_offset_xyz = self._tuple3("tcp_offset_xyz")
        self._target_base_offset_xyz = self._tuple3("target_base_offset_xyz")
        self._grasp_base_z_offset_m = float(self.get_parameter("grasp_base_z_offset_m").value)
        self._service_timeout_sec = float(self.get_parameter("service_timeout_sec").value)
        self._motion_result_timeout_sec = float(self.get_parameter("motion_result_timeout_sec").value)
        self._max_plan_age_sec = float(self.get_parameter("max_plan_age_sec").value)
        self._execution_mode = str(self.get_parameter("execution_mode").value).strip().lower()
        # 阶段名 → 该阶段成功后的固定等待（s）。等待只用于让机械/夹爪稳定，未列出的阶段不等待；
        # 具体时长来自参数；plan_only 不需要机械稳定等待，只使用其专用诊断等待参数
        self._stage_waits = {
            "move_to_pregrasp": float(self.get_parameter("pregrasp_wait_sec").value),
            "approach_grasp": float(self.get_parameter("approach_wait_sec").value),
            "close_gripper": float(self.get_parameter("gripper_wait_sec").value),
            "open_gripper": float(self.get_parameter("gripper_wait_sec").value),
            "safe_retreat": float(self.get_parameter("retreat_wait_sec").value),
        }

        # ── 运行状态 ───────────────────────────────────────────────────────
        self._state = VisualGraspState()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ── 下游服务客户端（全部挂在可重入回调组上）────────────────────────
        self._execute_pose_client = self.create_client(
            ExecutePose,
            f"/{self._arm_namespace}/motion_execution/execute_pose",
            callback_group=self._callback_group,
        )
        self._publish_preview_client = self.create_client(
            PublishTrajectoryPreview,
            f"/{self._arm_namespace}/motion_execution/publish_trajectory_preview",
            callback_group=self._callback_group,
        )
        self._motion_stop_client = self.create_client(
            Trigger,
            f"/{self._arm_namespace}/motion_execution/stop",
            callback_group=self._callback_group,
        )
        self._trajectory_stop_client = self.create_client(
            Trigger,
            f"/{self._arm_namespace}/trajectory_stop",
            callback_group=self._callback_group,
        )
        self._safe_home_client = self.create_client(
            Trigger,
            f"/{self._arm_namespace}/safe_home",
            callback_group=self._callback_group,
        )
        self._gripper_client = self.create_client(
            SetGripper,
            f"/{self._arm_namespace}/gripper/set",
            callback_group=self._callback_group,
        )
        self._gripper_grasp_client = self.create_client(
            GraspGripper,
            f"/{self._arm_namespace}/gripper/grasp",
            callback_group=self._callback_group,
        )
        self._io_gateway = VisualGraspIoGateway(
            logger=self.get_logger(),
            clients={
                "execute_pose": self._execute_pose_client,
                "publish_preview": self._publish_preview_client,
                "motion_stop": self._motion_stop_client,
                "trajectory_stop": self._trajectory_stop_client,
                "safe_home": self._safe_home_client,
                "gripper": self._gripper_client,
                "gripper_grasp": self._gripper_grasp_client,
            },
            cancelled=lambda: not _state_for(self).is_running(),
            on_timeout=lambda name: self._handle_io_timeout(name),
        )
        # 队列深度 10：计划/候选都是"最新值优先"语义，积压没有意义，_on_plan 会自行丢弃过期消息
        self.create_subscription(GraspPlan, self._input_topic, self._on_plan, 10, callback_group=self._callback_group)
        self.create_subscription(
            GraspCandidateArray,
            self._candidates_topic,
            self._on_candidates,
            10,
            callback_group=self._callback_group,
        )
        self.create_service(Trigger, f"/{self._arm_namespace}/visual_grasp/execute", self._execute_visual_grasp, callback_group=self._callback_group)
        self.create_service(Trigger, f"/{self._arm_namespace}/visual_grasp/stop", self._stop_visual_grasp, callback_group=self._stop_callback_group)
        self.create_timer(0.05, self._poll_stop_confirmation, callback_group=self._callback_group)
        self.get_logger().info(
            "visual grasp executor ready: "
            f"input={self._input_topic}, namespace=/{self._arm_namespace}, target_frame={self._target_frame}, "
            "motion_execution=/motion_execution/execute_pose"
        )

    def _io_gateway_for(self) -> VisualGraspIoGateway:
        gateway = getattr(self, "_io_gateway", None)
        if gateway is None:
            gateway = VisualGraspIoGateway(
                logger=self.get_logger(),
                clients={
                    "execute_pose": self._execute_pose_client,
                    "publish_preview": self._publish_preview_client,
                    "motion_stop": self._motion_stop_client,
                    "trajectory_stop": self._trajectory_stop_client,
                "safe_home": self._safe_home_client,
                    "gripper": self._gripper_client,
                    "gripper_grasp": self._gripper_grasp_client,
                },
                cancelled=lambda: not _state_for(self).is_running(),
                on_timeout=lambda name: self._handle_io_timeout(name),
            )
            self._io_gateway = gateway
        return gateway

    def _handle_io_timeout(self, name: str) -> None:
        """超时后下游运动状态不确定，返回前必须请求停止。"""
        state = _state_for(self)
        state.mark_aborting(f"{name} timed out")
        self._request_stop()

    def _parameter(self, name: str):
        """读取当前任务配置；构造阶段尚未建立快照时回退到 ROS 参数。"""
        config = getattr(self, "_config", None)
        if config is not None and name in config.values:
            return type("ParameterValue", (), {"value": config.get(name)})()
        try:
            return super().get_parameter(name)
        except AttributeError:
            return self.get_parameter(name)

    def _refresh_config(self) -> None:
        """在每次执行前建立一致的参数快照，并刷新本地派生值。"""
        self._config = VisualGraspConfig.from_node(self)
        self._input_topic = self._config.input_topic
        self._target_frame = str(self._config.get("target_frame", self._target_frame)).strip()
        self._tcp_offset_xyz = self._tuple3("tcp_offset_xyz")
        self._target_base_offset_xyz = self._tuple3("target_base_offset_xyz")
        self._grasp_base_z_offset_m = float(_parameter(self, "grasp_base_z_offset_m").value)
        self._service_timeout_sec = float(_parameter(self, "service_timeout_sec").value)
        self._motion_result_timeout_sec = float(_parameter(self, "motion_result_timeout_sec").value)
        self._max_plan_age_sec = self._config.max_plan_age_sec
        self._execution_mode = str(_parameter(self, "execution_mode").value).strip().lower()
        self._stage_waits = {
            "move_to_pregrasp": float(_parameter(self, "pregrasp_wait_sec").value),
            "approach_grasp": float(_parameter(self, "approach_wait_sec").value),
            "close_gripper": float(_parameter(self, "gripper_wait_sec").value),
            "open_gripper": float(_parameter(self, "gripper_wait_sec").value),
            "safe_retreat": float(_parameter(self, "retreat_wait_sec").value),
        }

    def _tuple3(self, name: str) -> tuple[float, float, float]:
        return tuple(float(value) for value in self._config.get(name))

    def _on_plan(self, plan: GraspPlan) -> None:
        """原子替换缓存；拒绝新输入时，同时撤销旧缓存的执行资格。"""
        state = _state_for(self)
        with state.lock:
            if state.plans_blocked:
                return
            reason = ""
            if not plan.valid:
                reason = f"invalid grasp plan: {plan.reason or 'upstream reported invalid'}"
            elif not self._plan_is_fresh(plan):
                age = message_age_sec(plan.header.stamp, now_ns=int(self.get_clock().now().nanoseconds))
                reason = f"grasp plan expired on arrival: age_sec={age}, max_plan_age_sec={self._max_plan_age_sec}"
            state.plan_revision += 1
            state.last_plan_rejection = reason
            if reason:
                state.latest_plan = None
                state.latest_candidates = None
            else:
                state.latest_plan = deepcopy(plan)

    def _plan_is_fresh(self, plan: GraspPlan) -> bool:
        return is_message_fresh(
            plan.header.stamp, now_ns=int(self.get_clock().now().nanoseconds),
            max_age_sec=self._max_plan_age_sec,
        )

    def _on_candidates(self, candidates: GraspCandidateArray) -> None:
        """候选数组订阅回调；空数组撤销待执行计划和候选缓存。"""

        state = _state_for(self)
        with state.lock:
            if state.plans_blocked:
                return
            if not candidates.candidates:
                state.latest_candidates = None
                state.latest_plan = None
                state.plan_revision += 1
                state.last_plan_rejection = "upstream published no grasp candidates"
            else:
                state.latest_candidates = deepcopy(candidates)

    def _execute_visual_grasp(self, _request, response):
        """ROS Trigger 适配：执行流程由 VisualGraspRuntime 编排。"""
        state = _state_for(self)
        with state.lock:
            if state.running or state.phase != "IDLE" or self._io_gateway.stop_pending:
                response.success = False
                response.message = state.abort_reason or "visual grasp running or stop not confirmed"
                return response
            if not state.begin_run():
                response.success = False
                response.message = "visual grasp stop not confirmed"
                return response
            request_id = state.active_request_id
        try:
            self._refresh_config()
            ok, reason = self._wait_for_fresh_plan()
            if not ok:
                response.success = False
                response.message = reason
                return response
            # 运行时编排器位于包根目录，即 nodes 子包的上一层。
            from ..visual_grasp_runtime import VisualGraspRuntime
            workflow = VisualGraspWorkflow(
                state=state, config=self._config, io_gateway=self._io_gateway,
                tf_buffer=self._tf_buffer, clock=self.get_clock(), logger=self.get_logger(),
            )
            self._runtime = VisualGraspRuntime(workflow)
            return self._runtime.execute(_request, response, admitted=True)
        except (ValueError, TypeError) as exc:
            response.success = False
            response.message = f"invalid visual grasp configuration: {exc}"
            return response
        finally:
            with state.lock:
                if state.active_request_id == request_id:
                    state.finish_run(
                        preserve_abort=state.phase in ("STOP_REQUESTED", "ABORTING")
                    )

    def _wait_for_fresh_plan(self) -> tuple[bool, str]:
        """允许实时感知链路补充缺失输入或替换过期输入。

        等待受 service_timeout_sec 限制，并可由停止请求中断。
        新鲜度始终按采集时间判断，不以消息到达时间替代。
        """
        state = _state_for(self)
        timeout = max(float(self._service_timeout_sec), 0.0)
        deadline = time.monotonic() + timeout
        while rclpy.ok():
            with state.lock:
                if not state.running or state.plans_blocked:
                    return False, state.abort_reason or "stopped while waiting for a fresh plan"
                plan = state.latest_plan
                if plan is not None and self._plan_is_fresh(plan):
                    return True, "fresh plan available"
                if plan is not None:
                    age = message_age_sec(plan.header.stamp, now_ns=int(self.get_clock().now().nanoseconds))
                    state.latest_plan = None
                    state.latest_candidates = None
                    state.last_plan_rejection = (
                        f"cached grasp plan expired: age_sec={age}, "
                        f"max_plan_age_sec={self._max_plan_age_sec}"
                    )
                reason = state.last_plan_rejection or "no grasp plan received yet"
            if time.monotonic() >= deadline:
                return False, f"fresh grasp plan wait timed out after {timeout:g}s: {reason}"
            time.sleep(0.02)
        return False, "ROS shutdown while waiting for a fresh plan"

    def _poll_stop_confirmation(self):
        state = _state_for(self)
        with state.lock:
            if state.run_callback_active or not self._io_gateway.stop_pending:
                return
            if not state.plans_blocked:
                state.request_stop("waiting for motion stop confirmation")
            ok, message = self._io_gateway.poll_stop_confirmation()
            if ok:
                state.confirm_stop()
                self.get_logger().info("visual grasp stop confirmed; waiting for a new plan")
            elif ok is False:
                if state.abort_reason != message:
                    self.get_logger().error(message)
                state.mark_aborting(message)

    def _stop_visual_grasp(self, _request, response):
        """停止并保持当前位置；成功表示已确认停止，不附带自动回位。"""
        state = _state_for(self)
        with state.lock:
            state.request_stop("operator stop; waiting for confirmation")
            self._request_stop()
        deadline = time.monotonic() + 16.0
        while rclpy.ok() and time.monotonic() < deadline:
            with state.lock:
                if not state.plans_blocked:
                    response.success = True
                    response.message = "visual grasp stopped and confirmed; holding current position"
                    return response
                if state.phase == "ABORTING" and not state.run_callback_active:
                    response.success = False
                    response.message = state.abort_reason
                    return response
            time.sleep(.02)
        response.success = False
        response.message = "stop not confirmed; execution remains blocked"
        return response

    def _request_stop(self) -> None:
        """同时向运动执行与轨迹层发出停止请求，双通道保险，任一可用即可生效。"""

        self._request_stop_service(self._motion_stop_client, "motion execution stop")
        self._request_stop_service(self._trajectory_stop_client, "trajectory_stop")

    def _request_stop_service(self, client, label: str) -> None:
        """尽力而为地调用一个停止服务：服务端 0.2 s 内不可用就放弃，异常只告警不抛出。

        停止路径本身不能失败退出，否则会把异常传播到执行回调，掩盖真正的失败原因。
        """

        name = "motion_stop" if client is self._motion_stop_client else "trajectory_stop"
        _io_gateway_for(self).stop(name)


def main(args=None) -> None:
    """进程入口：初始化 rclpy，跑一个三线程执行器。

    线程数 3 是为了让长时间运行的执行服务回调（内含轮询等待）不至于饿死订阅回调与
    停止服务；`KeyboardInterrupt` 与外部关闭异常按正常退出处理（不打印堆栈），
    退出时先关执行器再销毁节点，最后只在 rclpy 仍有效时调用 shutdown。
    """

    rclpy.init(args=args)
    node = VisualGraspExecutorNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

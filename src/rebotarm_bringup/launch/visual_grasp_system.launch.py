# 视觉抓取系统（visual grasp）主启动文件
#
# 用途：统一提供无硬件感知预览、plan-only 和受控执行。完整组合为
# 就绪摆位 → 相机与检测 → 抓取网络候选 → MoveIt 逆解与碰撞过滤 →
# 点到点执行 → 抓取执行器，并在 RViz/Open3D 中显示数据、候选与规划预览。
#
# 节点组合与启动顺序：
#   1) 先包含共享底层栈（MoveIt 规划、RViz、真实或仿真的关节状态来源）；
#   2) 再启动一次性的视觉就绪实例，把机械臂摆到观察位姿后退出；
#   3) 用进程退出事件在它退出后启动后续全部视觉抓取节点；若不执行启动摆位，则用取反条件直接启动这些节点。
#   这样可保证相机与检测只在手眼标定所用的位姿下才开始工作。
#
# 真实/仿真后端选择：
#   - use_hardware=true：共享底层栈启动真实电机控制器，轨迹执行落在硬件层；同时本文件的仿真轨迹控制器被条件排除，避免出现两个同名轨迹服务。
#   - use_hardware=false（默认）：由共享底层栈提供 MoveIt 或仿真的关节状态；仿真轨迹控制器只在 start_sim_trajectory_controller=true 且非真实硬件时启动。
#
# 参数来源：
#   - 运动与视觉策略档（只读引用）来自视觉包与运动包的 config 目录；
#   - 其余数值默认值以启动参数暴露，均可用 ros2 launch 的 name:=value 覆盖。
#
# 安全默认值（改动前请确认影响）：
#   - use_hardware=false、execution_mode=plan_only：默认只规划，不上电、不下发真实动作；
#   - shutdown_safe_home=false：退出时不做未经确认的整机回零；
#   - auto_retry_enabled=false、place_after_grasp_enabled=false：失败即停，不自动连续动作；
#   - 台面高度闸门、夹爪开口与夹持力上下限均为 fail-closed：不满足即拒绝，而不是勉强执行。
#
# 职责边界：本文件只做启动组合，不实现电机控制、运动规划、感知或标定算法。

import os
from pathlib import Path

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# 解析工作区相对路径型默认值：优先读取环境变量；未设置时从本文件位置逐级向上查找，
# 找到即返回绝对路径，都找不到返回空串（由节点侧判空并报未配置，而不是猜路径）。
# 这样源码树与安装后的 share 布局都能定位模型权重，且不硬编码任何 home 路径。
def _workspace_path(environment_name: str, relative_path: str) -> str:
    configured = os.environ.get(environment_name, "").strip()
    if configured:
        return configured
    source = Path(__file__).resolve()
    for parent in source.parents:
        candidate = parent / relative_path
        if candidate.exists():
            return str(candidate)
    return ""


def _read_policy(context, profile, node_name):
    """Read one node's defaults; fail with the resource and node in the error."""
    path = profile.perform(context)
    try:
        with open(path, encoding="utf-8") as stream:
            values = yaml.safe_load(stream)[node_name]["ros__parameters"]
        if not isinstance(values, dict):
            raise TypeError("ros__parameters must be a mapping")
        return values
    except (OSError, yaml.YAMLError, KeyError, TypeError) as exc:
        raise RuntimeError(f"Cannot load policy {path} for {node_name}: {exc}") from exc


# 生成完整启动描述。两条顺序约束必须保持：
#   1) 声明顺序即启动顺序，共享底层栈先起、就绪实例随后；
#   2) 视觉链路节点全部挂在 post_visual_ready_actions 里，由就绪实例的退出事件触发。
def generate_launch_description():
    # ── 参数档来源（只读引用；数值定义在各属主包的 config 目录）──
    # 上层启动只决定把哪一份策略档传给哪个节点，策略本身不在本文件定义。
    bringup_share = FindPackageShare("rebotarm_bringup")
    vision_share = FindPackageShare("rebotarm_vision")
    grasp_pose_policy_params = PathJoinSubstitution([vision_share, "config", "grasp_pose_policy.yaml"])  # 抓取姿态与候选过滤策略档（候选过滤节点保留候选姿态，执行器用 base_axis）
    gripper_policy_params = PathJoinSubstitution([vision_share, "config", "gripper_policy.yaml"])  # 夹爪开合宽度与夹持力策略档
    retry_policy_params = PathJoinSubstitution([vision_share, "config", "retry_policy.yaml"])  # 重试、抓取验证与放置策略档
    retreat_policy_params = PathJoinSubstitution([vision_share, "config", "retreat_policy.yaml"])  # 抓取后安全撤退策略档
    visual_servo_params = PathJoinSubstitution([vision_share, "config", "visual_servo.yaml"])  # 计划刷新与接近段视觉伺服档
    table_safety_params = PathJoinSubstitution([vision_share, "config", "table_safety.yaml"])  # 台面高度安全档（候选过滤与执行器共享同一批高度下限）
    graspnet_ubuntu_params = PathJoinSubstitution([vision_share, "config", "graspnet_ubuntu.yaml"])  # 抓取网络参数档（Ubuntu 原生档，模型路径由启动参数另行注入）
    visual_ready_params = PathJoinSubstitution([FindPackageShare("rebotarm_motion"), "config", "visual_ready.yaml"])  # 视觉就绪位姿档（启动实例与常驻实例共用同一份关节角）

    # ── 命名空间与后端选择 ──
    # 下面的 LaunchConfiguration 只是把启动参数读进来；每个参数的完整含义见文末声明处。
    arm_namespace = LaunchConfiguration("arm_namespace")
    channel = LaunchConfiguration("channel")
    use_hardware = LaunchConfiguration("use_hardware")
    # ── 夹爪安全与硬件反馈（转发给真实控制器；仿真下不生效）──
    gripper_position_torque_cap_nm = LaunchConfiguration("gripper_position_torque_cap_nm")
    gripper_position_max_speed_rad_s = LaunchConfiguration("gripper_position_max_speed_rad_s")
    gripper_position_timeout_margin_sec = LaunchConfiguration("gripper_position_timeout_margin_sec")
    gripper_feedback_stale_timeout_sec = LaunchConfiguration("gripper_feedback_stale_timeout_sec")
    hardware_feedback_rate_hz = LaunchConfiguration("hardware_feedback_rate_hz")
    grasp_hold_timeout_sec = LaunchConfiguration("grasp_hold_timeout_sec")
    shutdown_safe_home = LaunchConfiguration("shutdown_safe_home")
    # ── 本地显示与执行模式 ──
    use_local_rviz = LaunchConfiguration("use_local_rviz")
    execution_mode = LaunchConfiguration("execution_mode")
    # ── 视觉输入链路（原生 Ubuntu 档，无网络输入路径）──
    start_vision = LaunchConfiguration("start_vision")
    vision_python_executable = LaunchConfiguration("vision_python_executable")
    vision_profile = LaunchConfiguration("vision_profile")
    vision_camera_config = LaunchConfiguration("vision_camera_config")
    vision_handeye_config = LaunchConfiguration("vision_handeye_config")
    vision_yolo_model_path = LaunchConfiguration("vision_yolo_model_path")
    # ── 抓取网络候选生成（主候选来源）──
    start_graspnet_baseline = LaunchConfiguration("start_graspnet_baseline")
    graspnet_candidates_topic = LaunchConfiguration("graspnet_candidates_topic")
    graspnet_output_frame_id = LaunchConfiguration("graspnet_output_frame_id")
    graspnet_config = LaunchConfiguration("graspnet_config")
    graspnet_max_input_skew_ms = LaunchConfiguration("graspnet_max_input_skew_ms")
    graspnet_python_executable = LaunchConfiguration("graspnet_python_executable")
    graspnet_model_root = LaunchConfiguration("graspnet_model_root")
    graspnet_checkpoint_path = LaunchConfiguration("graspnet_checkpoint_path")
    graspnet_device = LaunchConfiguration("graspnet_device")
    graspnet_backend_module = LaunchConfiguration("graspnet_backend_module")
    graspnet_max_grasps = LaunchConfiguration("graspnet_max_grasps")
    graspnet_max_points = LaunchConfiguration("graspnet_max_points")
    # ── 预览与候选逆解过滤 ──
    start_grasp_preview = LaunchConfiguration("start_grasp_preview")
    start_candidate_ik_filter = LaunchConfiguration("start_candidate_ik_filter")
    candidate_ik_input_topic = LaunchConfiguration("candidate_ik_input_topic")
    # ── 视觉就绪位姿（一次性启动实例 + 常驻服务实例）──
    start_visual_ready = LaunchConfiguration("start_visual_ready")
    move_to_visual_ready_on_start = LaunchConfiguration("move_to_visual_ready_on_start")
    visual_ready_joint_positions = LaunchConfiguration("visual_ready_joint_positions")
    visual_ready_duration_sec = LaunchConfiguration("visual_ready_duration_sec")
    visual_ready_wait_timeout_sec = LaunchConfiguration("visual_ready_wait_timeout_sec")
    visual_ready_max_start_delta_rad = LaunchConfiguration("visual_ready_max_start_delta_rad")
    visual_ready_startup_delay_sec = LaunchConfiguration("visual_ready_startup_delay_sec")
    # ── 执行侧节点开关 ──
    start_visual_grasp_executor = LaunchConfiguration("start_visual_grasp_executor")
    start_visual_grasp_markers = LaunchConfiguration("start_visual_grasp_markers")
    start_raw_candidate_markers = LaunchConfiguration("start_raw_candidate_markers")
    start_open3d_viewer = LaunchConfiguration("start_open3d_viewer")
    start_motion_execution = LaunchConfiguration("start_motion_execution")
    execute_gripper = LaunchConfiguration("execute_gripper")
    start_sim_trajectory_controller = LaunchConfiguration("start_sim_trajectory_controller")
    # ── RViz 可视化标记（仅观察，不参与运动决策）──
    gripper_open_axis_local_xyz = LaunchConfiguration("gripper_open_axis_local_xyz")
    show_tcp_markers = LaunchConfiguration("show_tcp_markers")
    show_approach_arrow = LaunchConfiguration("show_approach_arrow")
    show_gripper_open_axis = LaunchConfiguration("show_gripper_open_axis")
    # ── 话题与坐标系约定（改动即改变节点间连线，需上下游同步）──
    filtered_candidates_topic = LaunchConfiguration("filtered_candidates_topic")
    filtered_plan_topic = LaunchConfiguration("filtered_plan_topic")
    executor_input_topic = LaunchConfiguration("executor_input_topic")
    candidate_joint_state_topic = LaunchConfiguration("candidate_joint_state_topic")
    candidate_filter_service_timeout_sec = LaunchConfiguration("candidate_filter_service_timeout_sec")
    candidate_collision_check_enabled = LaunchConfiguration("candidate_collision_check_enabled")
    candidate_collision_check_service = LaunchConfiguration("candidate_collision_check_service")
    candidate_collision_group_name = LaunchConfiguration("candidate_collision_group_name")
    # ── 位姿策略与 TCP/目标偏移 ──
    # tcp_offset_xyz 是操作员实测值，vision 包的配置档与仿真模型必须保持一致。
    pose_mode = LaunchConfiguration("pose_mode")
    tcp_offset_xyz = LaunchConfiguration("tcp_offset_xyz")
    target_base_offset_xyz = LaunchConfiguration("target_base_offset_xyz")
    min_target_z_m = LaunchConfiguration("min_target_z_m")
    grasp_base_z_offset_m = LaunchConfiguration("grasp_base_z_offset_m")
    pose_policy = LaunchConfiguration("pose_policy")
    fixed_grasp_orientation_xyzw = LaunchConfiguration("fixed_grasp_orientation_xyzw")
    base_approach_axis_xyz = LaunchConfiguration("base_approach_axis_xyz")
    base_pregrasp_distance_m = LaunchConfiguration("base_pregrasp_distance_m")
    # ── 候选闸门与评分（逆解过滤节点）──
    # 高度、工作空间、夹爪开口、关节运动量都是 fail-closed：不满足即丢弃候选。
    candidate_pose_policy = LaunchConfiguration("candidate_pose_policy")
    candidate_orientation_yaw_offsets_rad = LaunchConfiguration("candidate_orientation_yaw_offsets_rad")
    candidate_grasp_z_offsets_m = LaunchConfiguration("candidate_grasp_z_offsets_m")
    candidate_max_candidates_per_frame = LaunchConfiguration("candidate_max_candidates_per_frame")
    candidate_min_confidence = LaunchConfiguration("candidate_min_confidence")
    candidate_min_jaw_width_m = LaunchConfiguration("candidate_min_jaw_width_m")
    candidate_max_jaw_width_m = LaunchConfiguration("candidate_max_jaw_width_m")
    candidate_min_grasp_z_m = LaunchConfiguration("candidate_min_grasp_z_m")
    candidate_pregrasp_min_z_m = LaunchConfiguration("candidate_pregrasp_min_z_m")
    candidate_workspace_gate_enabled = LaunchConfiguration("candidate_workspace_gate_enabled")
    candidate_workspace_min_xyz = LaunchConfiguration("candidate_workspace_min_xyz")
    candidate_workspace_max_xyz = LaunchConfiguration("candidate_workspace_max_xyz")
    candidate_max_grasp_to_object_center_m = LaunchConfiguration("candidate_max_grasp_to_object_center_m")
    candidate_score_joint_distance_weight = LaunchConfiguration("candidate_score_joint_distance_weight")
    candidate_score_joint6_weight = LaunchConfiguration("candidate_score_joint6_weight")
    candidate_max_joint6_delta_rad = LaunchConfiguration("candidate_max_joint6_delta_rad")
    candidate_joint6_symmetry_enabled = LaunchConfiguration("candidate_joint6_symmetry_enabled")
    candidate_joint6_symmetry_angle_rad = LaunchConfiguration("candidate_joint6_symmetry_angle_rad")
    # ── 夹爪几何与夹持力（执行器）──
    # 归一化夹持力量纲与硬件层不同，硬件层会再做区间截断。
    close_position_m = LaunchConfiguration("close_position_m")
    close_max_effort = LaunchConfiguration("close_max_effort")
    open_before_approach = LaunchConfiguration("open_before_approach")
    auto_gripper_width = LaunchConfiguration("auto_gripper_width")
    auto_gripper_effort = LaunchConfiguration("auto_gripper_effort")
    open_clearance_m = LaunchConfiguration("open_clearance_m")
    close_margin_m = LaunchConfiguration("close_margin_m")
    min_gripper_effort = LaunchConfiguration("min_gripper_effort")
    max_gripper_effort = LaunchConfiguration("max_gripper_effort")
    max_allowed_grasp_width_m = LaunchConfiguration("max_allowed_grasp_width_m")
    # ── 抓取夹持服务与接触判据 ──
    gripper_grasp_enabled = LaunchConfiguration("gripper_grasp_enabled")
    gripper_grasp_close_force = LaunchConfiguration("gripper_grasp_close_force")
    gripper_grasp_timeout_sec = LaunchConfiguration("gripper_grasp_timeout_sec")
    gripper_grasp_min_close_time_sec = LaunchConfiguration("gripper_grasp_min_close_time_sec")
    gripper_grasp_velocity_threshold = LaunchConfiguration("gripper_grasp_velocity_threshold")
    gripper_grasp_min_closure_distance_m = LaunchConfiguration("gripper_grasp_min_closure_distance_m")
    # ── 抓取后安全撤退、回零与 MoveIt 规划 ──
    safe_retreat_enabled = LaunchConfiguration("safe_retreat_enabled")
    safe_retreat_distance_m = LaunchConfiguration("safe_retreat_distance_m")
    safe_home_after_grasp = LaunchConfiguration("safe_home_after_grasp")
    moveit_planning_time = LaunchConfiguration("moveit_planning_time")
    moveit_num_planning_attempts = LaunchConfiguration("moveit_num_planning_attempts")
    move_velocity_scaling = LaunchConfiguration("move_velocity_scaling")
    approach_velocity_scaling = LaunchConfiguration("approach_velocity_scaling")
    retreat_velocity_scaling = LaunchConfiguration("retreat_velocity_scaling")
    acceleration_scaling = LaunchConfiguration("acceleration_scaling")
    plan_only_stage_pause_sec = LaunchConfiguration("plan_only_stage_pause_sec")
    # ── 接近段视觉伺服（默认关闭的增强项）──
    approach_visual_servo_enabled = LaunchConfiguration("approach_visual_servo_enabled")
    approach_visual_servo_max_iterations = LaunchConfiguration("approach_visual_servo_max_iterations")
    approach_visual_servo_max_step_m = LaunchConfiguration("approach_visual_servo_max_step_m")
    approach_visual_servo_position_tolerance_m = LaunchConfiguration("approach_visual_servo_position_tolerance_m")
    approach_visual_servo_require_fresh_plan = LaunchConfiguration("approach_visual_servo_require_fresh_plan")
    # ── 重试、抓取验证与放置（默认不自动继续动作）──
    auto_retry_enabled = LaunchConfiguration("auto_retry_enabled")
    auto_retry_max_attempts = LaunchConfiguration("auto_retry_max_attempts")
    safe_retreat_before_retry = LaunchConfiguration("safe_retreat_before_retry")
    grasp_verification_enabled = LaunchConfiguration("grasp_verification_enabled")
    grasp_verification_min_closure_distance_m = LaunchConfiguration("grasp_verification_min_closure_distance_m")
    grasp_verification_require_contact = LaunchConfiguration("grasp_verification_require_contact")
    place_after_grasp_enabled = LaunchConfiguration("place_after_grasp_enabled")
    place_position_xyz = LaunchConfiguration("place_position_xyz")
    place_orientation_xyzw = LaunchConfiguration("place_orientation_xyzw")
    place_open_position_m = LaunchConfiguration("place_open_position_m")
    place_open_max_effort = LaunchConfiguration("place_open_max_effort")
    place_retreat_z_m = LaunchConfiguration("place_retreat_z_m")
    # ── 轨迹预检与计划新鲜度 ──
    trajectory_precheck_enabled = LaunchConfiguration("trajectory_precheck_enabled")
    max_plan_age_sec = LaunchConfiguration("max_plan_age_sec")

    # ── 共享底层栈（包含启动）──
    # 该包含决定谁来执行轨迹：use_hardware=true 时启动真实电机控制器，
    # 否则由 MoveIt 侧提供关节状态（本文件已显式关闭假关节状态源，避免 RViz 抖动）。
    # use_moveit_preview 置 true 表示本次启动必须提供 MoveIt 规划能力（move_group + RViz）。
    # 安全默认：use_hardware=false、shutdown_safe_home=false（退出不回零）。
    interactive_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "interactive_system.launch.py"])
        ),
        launch_arguments={
            "arm_namespace": arm_namespace,
            "use_moveit_preview": "true",
            "use_hardware": use_hardware,
            "channel": channel,
            "shutdown_safe_home": shutdown_safe_home,
            "gripper_position_torque_cap_nm": gripper_position_torque_cap_nm,
            "gripper_position_max_speed_rad_s": gripper_position_max_speed_rad_s,
            "gripper_position_timeout_margin_sec": gripper_position_timeout_margin_sec,
            "gripper_feedback_stale_timeout_sec": gripper_feedback_stale_timeout_sec,
            "hardware_feedback_rate_hz": hardware_feedback_rate_hz,
            "grasp_hold_timeout_sec": grasp_hold_timeout_sec,
            "use_local_rviz": use_local_rviz,
            "start_passive_joint_state_publisher": "false",
            "use_moveit_fake_joint_states": "false",
            "rviz_config": PathJoinSubstitution([bringup_share, "rviz", "visual_grasp.rviz"]),
        }.items(),
    )
    # 无硬件 plan-only 模式需要一个唯一的运动学状态/轨迹后端。它必须和 MoveIt
    # 同时启动，而不能等到 visual_ready 启动实例退出后才启动：MoveIt 初始化规划场景
    # 时会等待当前关节状态，visual_ready 自动摆位也依赖同一个动作与状态接口。
    # 真机模式下条件恒为 false；外部 MuJoCo 等后端接管时由
    # start_sim_trajectory_controller=false 显式关闭，避免第二个同名 Action server。
    sim_trajectory_controller = Node(
        package="rebotarm_simulation",
        executable="rebotarm_sim_trajectory_controller",
        name="rebotarm_sim_trajectory_controller",
        output="screen",
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    use_hardware,
                    "'.lower() != 'true' and '",
                    start_sim_trajectory_controller,
                    "'.lower() == 'true'",
                ]
            )
        ),
        parameters=[
            {
                "arm_namespace": arm_namespace,
                "initial_joint_positions": visual_ready_joint_positions,
            }
        ],
    )
    # 只有两个开关都为 true 才执行一次启动摆位。默认
    # move_to_visual_ready_on_start=false 时直接启动后续视觉链，同时仍保留常驻的
    # /visual_ready/move 服务；不能启动一个不会退出的一次性节点来阻塞整个链路。
    run_visual_ready_startup = PythonExpression(
        [
            "'",
            start_visual_ready,
            "'.lower() == 'true' and '",
            move_to_visual_ready_on_start,
            "'.lower() == 'true'",
        ]
    )
    # 启动实例（一次性）：把机械臂摆到视觉观察位姿，摆位结束后退出进程。
    # 必要性：相机视角与手眼关系是在该位姿下标定的，检测必须在位姿就绪后才开始。
    # 它带 startup_delay_sec 延迟与 max_start_delta_rad 起始偏差保护；
    # exit_after_startup_move 为 true，因此不论移动成败都会退出，用退出事件衔接后续节点。
    visual_ready_startup = Node(
        package="rebotarm_motion",
        executable="rebotarm_visual_ready",
        name="rebotarm_visual_ready_startup",
        output="screen",
        condition=IfCondition(run_visual_ready_startup),
        parameters=[
            visual_ready_params,
            {
                "arm_namespace": arm_namespace,
                "auto_move_on_start": move_to_visual_ready_on_start,
                "exit_after_startup_move": True,  # 启动实例摆位结束后立即退出进程，由进程退出事件驱动后续节点（成败只体现在日志）
                "startup_delay_sec": visual_ready_startup_delay_sec,
                "joint_positions": visual_ready_joint_positions,
                "duration_sec": visual_ready_duration_sec,
                "wait_timeout_sec": visual_ready_wait_timeout_sec,
                "max_start_delta_rad": visual_ready_max_start_delta_rad,
            }
        ],
    )

    def _launch_executor(context):
        # Flatten named YAML entries before Node normalization so explicit overrides
        # have predictable precedence, including values equal to former defaults.
        policy_defaults = {}
        for profile in (
            grasp_pose_policy_params,
            gripper_policy_params,
            retry_policy_params,
            retreat_policy_params,
            visual_servo_params,
            table_safety_params,
        ):
            policy_defaults.update(_read_policy(context, profile, "rebotarm_visual_grasp_executor"))
        # Keep launch compatibility: only values different from YAML defaults
        # are added as overrides. Equal defaults remain owned by the profile.
        override_names = ('close_position_m', 'close_max_effort', 'open_before_approach', 'auto_gripper_width', 'auto_gripper_effort', 'open_clearance_m', 'close_margin_m', 'min_gripper_effort', 'max_gripper_effort', 'max_allowed_grasp_width_m', 'gripper_grasp_enabled', 'gripper_grasp_close_force', 'gripper_grasp_timeout_sec', 'gripper_grasp_min_close_time_sec', 'gripper_grasp_velocity_threshold', 'gripper_grasp_min_closure_distance_m',)
        for name in override_names:
            value = LaunchConfiguration(name).perform(context).strip().lower()
            default = str(policy_defaults[name]).strip().lower()
            if value != default:
                policy_defaults[name] = LaunchConfiguration(name)
        return [Node(
    package="rebotarm_vision",
    executable="rebotarm_visual_grasp_executor",
    name="rebotarm_visual_grasp_executor",
    output="screen",
    condition=IfCondition(start_visual_grasp_executor),
    parameters=[
        {
            **policy_defaults,
            "arm_namespace": arm_namespace,
            "input_topic": executor_input_topic,
            "candidates_topic": filtered_candidates_topic,
            "target_frame": "base_link",
            "tcp_offset_xyz": tcp_offset_xyz,
            "target_base_offset_xyz": target_base_offset_xyz,
            "grasp_base_z_offset_m": grasp_base_z_offset_m,
            "pose_policy": pose_policy,
            "fixed_grasp_orientation_xyzw": fixed_grasp_orientation_xyzw,
            "base_approach_axis_xyz": base_approach_axis_xyz,
            "base_pregrasp_distance_m": base_pregrasp_distance_m,
            "min_grasp_z_m": min_target_z_m,
            "safe_retreat_enabled": safe_retreat_enabled,
            "safe_retreat_distance_m": safe_retreat_distance_m,
            "safe_home_after_grasp": safe_home_after_grasp,
            "move_velocity_scaling": move_velocity_scaling,
            "approach_velocity_scaling": approach_velocity_scaling,
            "retreat_velocity_scaling": retreat_velocity_scaling,
            "acceleration_scaling": acceleration_scaling,
            "execute_gripper": execute_gripper,
            "execution_mode": execution_mode,
            "max_plan_age_sec": max_plan_age_sec,
            "plan_only_stage_pause_sec": plan_only_stage_pause_sec,
            "approach_visual_servo_enabled": approach_visual_servo_enabled,
            "approach_visual_servo_max_iterations": approach_visual_servo_max_iterations,
            "approach_visual_servo_max_step_m": approach_visual_servo_max_step_m,
            "approach_visual_servo_position_tolerance_m": approach_visual_servo_position_tolerance_m,
            "approach_visual_servo_require_fresh_plan": approach_visual_servo_require_fresh_plan,
            "auto_retry_enabled": auto_retry_enabled,
            "auto_retry_max_attempts": auto_retry_max_attempts,
            "safe_retreat_before_retry": safe_retreat_before_retry,
            "grasp_verification_enabled": grasp_verification_enabled,
            "grasp_verification_min_closure_distance_m": grasp_verification_min_closure_distance_m,
            "grasp_verification_require_contact": grasp_verification_require_contact,
            "place_after_grasp_enabled": place_after_grasp_enabled,
            "place_position_xyz": place_position_xyz,
            "place_orientation_xyzw": place_orientation_xyzw,
            "place_open_position_m": place_open_position_m,
            "place_open_max_effort": place_open_max_effort,
            "place_retreat_z_m": place_retreat_z_m,
            "trajectory_precheck_enabled": trajectory_precheck_enabled,
        }
    ],
)]

    # ── 就绪之后才启动的完整视觉抓取链路（按依赖顺序排列）──
    # 顺序：就绪常驻服务 → 相机与检测 → 抓取网络候选 → 逆解与碰撞过滤 →
    #       MoveIt 位姿执行 → 抓取执行器。无硬件状态后端已在本列表之前启动。
    # 触发方式：明确启用启动自动摆位时由进程退出事件触发；
    #           否则由文末的分组动作直接启动（跳过自动摆位）。
    post_visual_ready_actions = [
        # 常驻实例：不自动运动，只提供 visual_ready/move 服务，
        # 供界面或操作者在需要时把机械臂摆回观察位姿（与启动实例共用同一份位姿参数）。
        Node(
            package="rebotarm_motion",
            executable="rebotarm_visual_ready",
            name="rebotarm_visual_ready",
            output="screen",
            condition=IfCondition(start_visual_ready),
            parameters=[
                visual_ready_params,
                {
                    "arm_namespace": arm_namespace,
                    "auto_move_on_start": False,  # 常驻实例不自动运动，只响应 visual_ready/move 服务调用
                    "joint_positions": visual_ready_joint_positions,
                    "duration_sec": visual_ready_duration_sec,
                    "wait_timeout_sec": visual_ready_wait_timeout_sec,
                    "max_start_delta_rad": visual_ready_max_start_delta_rad,
                }
            ],
        ),
        # 相机与检测链路：仅当 start_vision=true 且 vision_profile=ubuntu_native 时启动，
        # 内容为原生 Gemini 2 驱动、YOLO 分割与深度/内参发布，全部为本地输入。
        # 相机档、手眼档、模型路径均可由启动参数覆盖；解释器由 vision_python_executable 单独指定，
        # 只影响本链路，不向整个启动组注入环境变量。
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([vision_share, "launch", "vision.launch.py"])
            ),
            condition=IfCondition(
                PythonExpression(
                    [
                        "'",
                        start_vision,
                        "' == 'true' and '",
                        vision_profile,
                        "' == 'ubuntu_native'",
                    ]
                )
            ),
            launch_arguments={
                "camera_config": vision_camera_config,
                "handeye_config": vision_handeye_config,
                "yolo_model_path": vision_yolo_model_path,
                "vision_python_executable": vision_python_executable,
                "yolo_device": "0",
            }.items(),
        ),
        # 抓取预览发送器（默认关闭）：把过滤后的计划转成操作者可预览的目标位姿，
        # pose_mode=pregrasp 时只发预抓取位姿，避免末端贴住物体。
        # 它只发布目标、不产生运动；publish_count 用于抵消话题发现延迟。
        Node(
            package="rebotarm_vision",
            executable="rebotarm_send_grasp_preview",
            name="rebotarm_grasp_preview_sender",
            output="screen",
            condition=IfCondition(start_grasp_preview),
            parameters=[
                grasp_pose_policy_params,
                {
                    "input_topic": executor_input_topic,
                    "output_topic": ["/", arm_namespace, "/interactive_control/pose_target"],
                    "pose_mode": pose_mode,
                    "target_frame": "base_link",
                    "tcp_offset_xyz": tcp_offset_xyz,
                    "target_base_offset_xyz": target_base_offset_xyz,
                    "min_target_z_m": min_target_z_m,
                    "publish_count": 5,  # 重复发布 5 次，抵消话题发现延迟
                    "exit_after_publish": False,  # 发布后不退出，保持常驻以便反复预览
                }
            ],
        ),
        # RViz 可视化标记：物体包围框、预抓取与抓取 TCP 标记、接近箭头、夹爪开合轴。
        # 只用于观察，不参与任何运动或抓取判定；尺寸下限仅影响可见性。
        Node(
            package="rebotarm_vision",
            executable="rebotarm_visual_grasp_markers",
            name="rebotarm_visual_grasp_markers",
            output="screen",
            condition=IfCondition(start_visual_grasp_markers),
            parameters=[
                grasp_pose_policy_params,
                {
                    "input_topic": executor_input_topic,
                    "output_topic": "/grasp/visual_markers",  # RViz 标记输出话题
                    "target_frame": "base_link",
                    "object_min_diameter_m": 0.06,  # 物体标记最小直径 6 cm，仅影响 RViz 可见性
                    "object_min_height_m": 0.12,  # 物体标记最小高度 12 cm，仅影响 RViz 可见性
                    "upright_object_marker": True,  # 物体框画成竖直方向，便于观察物体位置
                    # 候选消息只有类别、置信度和抓取尺度，并不包含真实物体轮廓。
                    # 关闭示意圆柱、中心绿点和文字，避免把它们误认为视觉测得的瓶子外形；
                    # TCP、接近方向和夹爪开合轴仍保留，用于核对实际抓取计划。
                    "show_object_marker": False,
                    "show_object_center_marker": False,
                    "show_object_label": False,
                    "tcp_offset_xyz": tcp_offset_xyz,
                    "gripper_open_axis_local_xyz": gripper_open_axis_local_xyz,
                    "show_tcp_markers": show_tcp_markers,
                    "show_approach_arrow": show_approach_arrow,
                    "show_gripper_open_axis": show_gripper_open_axis,
                }
            ],
        ),
        # 抓取网络候选生成：本地进程内推理（不是远程服务），
        # 输入 RGB-D 与检测结果，输出 6D 抓取候选数组供逆解过滤节点消费。
        # prefix 只为该进程选择解释器，不污染整个启动组；
        # 模型根目录、权重、设备与后端模块默认落在仓库内 .local-models 或对应环境变量。
        Node(
            package="rebotarm_vision",
            executable="rebotarm_graspnet_baseline_node",
            name="rebotarm_graspnet_baseline_node",
            output="screen",
            prefix=graspnet_python_executable,
            condition=IfCondition(start_graspnet_baseline),
            parameters=[
                graspnet_config,
                {
                    "input_color_topic": "/camera/color/image_raw",  # 彩色图输入话题
                    "input_depth_topic": "/camera/depth/image_raw",  # 深度图输入话题（16UC1，原始单位换算见参数档）
                    "input_detections_topic": "/grasp/detections",  # 检测结果输入话题（用于把点云裁剪到目标区域）
                    "output_candidates_topic": graspnet_candidates_topic,
                    "output_frame_id": graspnet_output_frame_id,
                    "max_input_skew_ms": graspnet_max_input_skew_ms,
                    "model_root": graspnet_model_root,
                    "checkpoint_path": graspnet_checkpoint_path,
                    "device": graspnet_device,
                    "backend_module": graspnet_backend_module,
                    "max_grasps": graspnet_max_grasps,
                    "max_jaw_width_m": candidate_max_jaw_width_m,
                    "max_points": graspnet_max_points,
                }
            ],
        ),
        # 原始 GraspNet 候选的 RViz 标记。它只做坐标变换与绘制，不参与候选过滤，
        # 用于对比网络原始输出和下游最终选中的抓取计划。
        Node(
            package="rebotarm_vision",
            executable="rebotarm_grasp_candidate_markers",
            name="rebotarm_grasp_candidate_markers",
            output="screen",
            condition=IfCondition(start_raw_candidate_markers),
            parameters=[
                {
                    "input_topic": graspnet_candidates_topic,
                    "output_topic": "/grasp/raw_candidate_markers",
                    "target_frame": "base_link",
                    "max_candidates": 5,
                }
            ],
        ),
        # Open3D 只读查看器：复用现有 RGB-D、内参与原始候选，不打开第二份相机或模型，
        # 也不调用任何规划、控制器或夹爪接口。无桌面环境时显式关闭。
        Node(
            package="rebotarm_vision",
            executable="rebotarm_graspnet_open3d_viewer",
            name="rebotarm_graspnet_open3d_viewer",
            output="screen",
            prefix=graspnet_python_executable,
            condition=IfCondition(start_open3d_viewer),
            parameters=[
                {
                    "input_color_topic": "/camera/color/image_raw",
                    "input_depth_topic": "/camera/depth/image_raw",
                    "input_camera_info_topic": "/camera/depth/camera_info",
                    "input_candidates_topic": graspnet_candidates_topic,
                }
            ],
        ),
        # 候选逆解与碰撞过滤：候选姿态变体 → MoveIt 逆解 → 状态有效性（碰撞）检查 →
        # 关节运动量/工作空间/台面高度闸门 → 评分排序，输出筛选后的候选与抓取计划。
        # 它只做规划校验，不下发任何轨迹；执行由下游执行器负责。
        Node(
            package="rebotarm_vision",
            executable="rebotarm_grasp_candidate_ik_filter",
            name="rebotarm_grasp_candidate_ik_filter",
            output="screen",
            condition=IfCondition(start_candidate_ik_filter),
            parameters=[
                # 该节点的完整候选过滤参数必须集中在同一个 launch 字典里。
                # 在 ROS 2 Jazzy 上，按节点名分组的 YAML 条目会覆盖 launch_ros 生成的
                # 通配字典（即使本字典写在后面），把参数拆到 YAML 里会让这些启动参数失效。
                # 保持单一完整档即可绕开该覆盖问题；这样候选过滤相关的闸门与评分才有唯一来源。
                {
                    "input_topic": candidate_ik_input_topic,
                    "output_topic": filtered_candidates_topic,
                    "output_plan_topic": filtered_plan_topic,
                    "joint_state_topic": candidate_joint_state_topic,
                    "service_timeout_sec": candidate_filter_service_timeout_sec,
                    "target_frame": "base_link",
                    "moveit_ik_service": "/compute_ik",  # MoveIt 逆解服务名
                    "collision_check_enabled": candidate_collision_check_enabled,
                    "collision_check_service": candidate_collision_check_service,
                    "collision_group_name": candidate_collision_group_name,
                    "moveit_group_name": "arm",  # 逆解用规划组（仅手臂；夹爪碰撞由碰撞检查组覆盖）
                    "ee_frame_id": "end_link",
                    "pose_policy": candidate_pose_policy,
                    "fixed_grasp_orientation_xyzw": fixed_grasp_orientation_xyzw,
                    "base_approach_axis_xyz": base_approach_axis_xyz,
                    "base_pregrasp_distance_m": base_pregrasp_distance_m,
                    "orientation_yaw_offsets_rad": candidate_orientation_yaw_offsets_rad,
                    "candidate_grasp_z_offsets_m": candidate_grasp_z_offsets_m,
                    "max_candidates_per_frame": candidate_max_candidates_per_frame,
                    "candidate_min_confidence": candidate_min_confidence,
                    "candidate_min_jaw_width_m": candidate_min_jaw_width_m,
                    "candidate_max_jaw_width_m": candidate_max_jaw_width_m,
                    "candidate_min_grasp_z_m": candidate_min_grasp_z_m,
                    "candidate_workspace_gate_enabled": candidate_workspace_gate_enabled,
                    "candidate_workspace_min_xyz": candidate_workspace_min_xyz,
                    "candidate_workspace_max_xyz": candidate_workspace_max_xyz,
                    "candidate_max_grasp_to_object_center_m": candidate_max_grasp_to_object_center_m,
                    "candidate_score_joint_distance_weight": candidate_score_joint_distance_weight,
                    "candidate_score_joint6_weight": candidate_score_joint6_weight,
                    "candidate_max_joint6_delta_rad": candidate_max_joint6_delta_rad,
                    "candidate_joint6_symmetry_enabled": candidate_joint6_symmetry_enabled,
                    "candidate_joint6_symmetry_angle_rad": candidate_joint6_symmetry_angle_rad,
                    "tcp_offset_xyz": tcp_offset_xyz,
                    "target_base_offset_xyz": target_base_offset_xyz,
                    "candidate_pregrasp_min_z_m": candidate_pregrasp_min_z_m,
                    "grasp_base_z_offset_m": grasp_base_z_offset_m,
                }
            ],
        ),
        # 点到点位姿执行节点：接收位姿目标，调用 MoveIt 规划后交由下层执行
        # （真实硬件走电机控制器，仿真走仿真后端）。
        # 默认速度与加速度使用视觉夹取暂定上限档，并由启动参数统一传入；
        # 规划时间与尝试次数同样由启动参数暴露。
        Node(
            package="rebotarm_motion",
            executable="PoseExecutionNode",
            name="motion_execution",
            output="screen",
            condition=IfCondition(start_motion_execution),
            parameters=[
                {
                    "arm_namespace": arm_namespace,
                    "frame_id": "base_link",  # 目标位姿参考坐标系：base 系，z 轴向上
                    "ee_frame_id": "end_link",
                    "moveit_planning_time": moveit_planning_time,
                    "moveit_num_planning_attempts": moveit_num_planning_attempts,
                    # 只在纯规划模式发布 RViz 幻影；真机执行/预检不发布虚拟轨迹。
                    "publish_plan_only_preview": PythonExpression(
                        ["'", execution_mode, "'.lower() == 'plan_only'"]
                    ),
                    "default_velocity_scaling": move_velocity_scaling,
                    "default_acceleration_scaling": acceleration_scaling,
                }
            ],
        ),
        # 视觉抓取执行器（链路末端）：把过滤后的计划展开为
        # 接近 → 预抓取 → 抓取 → 闭合验证 →（可选 沿接近路径反向撤退/放置）的阶段序列，
        # 逐阶段调用运动层与夹爪服务，并施加台面高度、夹持力、验证与撤退等安全策略。
        # 默认 execution_mode=plan_only：只规划干跑，不向硬件下发动作。
        OpaqueFunction(function=_launch_executor),
    ]

    # ── 启动参数声明与实体组装 ──
    # 下列声明是本文件对外的全部可调接口，默认值体现安全取向：
    # 默认不接硬件、只规划、失败即停，大范围动作（回零、放置、自动重试）需显式开启。
    return LaunchDescription(
        [
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),  # 机械臂 ROS 命名空间；所有话题/服务/动作都以此为前缀，真机与仿真并存时用于隔离
            DeclareLaunchArgument("channel", default_value="auto"),  # 真实硬件总线通道；auto 表示由控制器自动探测，也可填具体串口设备路径（仅 use_hardware=true 生效）
            DeclareLaunchArgument("use_hardware", default_value="false"),  # 是否启用真实电机后端。安全默认 false：只做规划与仿真，绝不上电使能
            # 夹爪位置命令的力矩上限（N·m）。硬件层限制在 [0.05, 1.5]；
            # 调大夹得更紧，但堵转力矩与过载风险上升。
            DeclareLaunchArgument(
                "gripper_position_torque_cap_nm", default_value="1.0"
            ),
            DeclareLaunchArgument("gripper_position_max_speed_rad_s", default_value="1.5"),  # 夹爪开合角速度上限（rad/s）；硬件层限制 [0.05, 3.0]，调大更快但冲击与堵转力矩更大
            DeclareLaunchArgument("gripper_position_timeout_margin_sec", default_value="1.5"),  # 夹爪到位超时余量（s）；实际超时 = 行程/速度 + 该余量，硬件层限制 [0.1, 10.0]
            DeclareLaunchArgument("gripper_feedback_stale_timeout_sec", default_value="0.15"),  # 夹爪反馈新鲜度阈值（s）；超时即判反馈过期并拒绝新命令，硬件层限制 [0.05, 2.0]
            DeclareLaunchArgument("hardware_feedback_rate_hz", default_value="50.0"),  # 硬件反馈刷新频率上限（Hz）；硬件层要求落在 [20, 100]
            DeclareLaunchArgument("grasp_hold_timeout_sec", default_value="30.0"),  # 抓取保持最长时间（s）；到期自动松开，避免电机持续堵转发热，硬件层限制 [0.1, 120.0]
            DeclareLaunchArgument("shutdown_safe_home", default_value="false"),  # 进程退出时是否回安全位。默认 false：不做未经确认的大范围回零动作
            DeclareLaunchArgument("use_local_rviz", default_value="true"),  # 是否在本机启动 RViz；远程无界面运行时置 false
            DeclareLaunchArgument("execution_mode", default_value="plan_only"),  # 执行模式。plan_only = 只规划干跑（默认安全值）；execute/real = 真正下发轨迹与夹爪命令
            DeclareLaunchArgument("start_vision", default_value="true"),  # 是否启动相机与检测链路（还需 vision_profile=ubuntu_native）
            # 视觉输入档位。当前只支持 ubuntu_native，即原生 Gemini 2 驱动，
            # 不接受网络/远程 JSON 等已废弃输入路径。
            DeclareLaunchArgument(
                "vision_profile",
                default_value="ubuntu_native",
                choices=["ubuntu_native"],
                description="Ubuntu native Gemini 2 vision input",
            ),
            # 相机参数文件路径；原生档默认取视觉包 config/camera_ubuntu.yaml。
            DeclareLaunchArgument(
                "vision_camera_config",
                default_value=PathJoinSubstitution([vision_share, "config", "camera_ubuntu.yaml"]),
            ),
            # 手眼标定参数文件路径；提供相机到末端的变换与残差阈值。
            DeclareLaunchArgument(
                "vision_handeye_config",
                default_value=PathJoinSubstitution([vision_share, "config", "handeye.yaml"]),
            ),
            # YOLO 分割模型权重路径；默认取视觉包 models/yolo26s-seg.pt。
            DeclareLaunchArgument(
                "vision_yolo_model_path",
                default_value=PathJoinSubstitution(
                    [
                        vision_share,
                        "models",
                        "yolo26s-seg.pt",
                    ]
                ),
            ),
            DeclareLaunchArgument("start_graspnet_baseline", default_value="true"),  # 是否启动抓取网络候选生成节点（主候选来源）
            DeclareLaunchArgument("graspnet_candidates_topic", default_value="/grasp/graspnet_candidates"),  # 候选数组输出话题；同时也是下游逆解过滤节点的输入话题
            DeclareLaunchArgument("graspnet_output_frame_id", default_value="camera_depth_frame"),  # 候选位姿输出坐标系，必须与深度图 frame_id 一致，否则 TF 换算失配
            DeclareLaunchArgument("graspnet_config", default_value=graspnet_ubuntu_params),  # 抓取网络参数档；默认用 Ubuntu 原生档（输入话题、深度换算系数、候选数上限）
            DeclareLaunchArgument("graspnet_max_input_skew_ms", default_value="100"),  # 彩色/深度/检测三者的最大时间偏差（ms）；超出则本帧不推理
            # 抓取网络进程专用 Python 解释器；默认读取 GRASPNET_PYTHON 环境变量，
            # 未设置则用 python3。只作用于该进程，不影响其它节点。
            DeclareLaunchArgument(
                "graspnet_python_executable",
                default_value=EnvironmentVariable("GRASPNET_PYTHON", default_value="python3"),
            ),
            # 视觉链路进程专用 Python 解释器；默认读取 REBOTARM_VISION_PYTHON 环境变量，
            # 未设置则用 python3。按进程选择解释器，不向整个启动组注入解释器搜索路径。
            DeclareLaunchArgument(
                "vision_python_executable",
                default_value=EnvironmentVariable("REBOTARM_VISION_PYTHON", default_value="python3"),
            ),
            # 抓取网络模型代码根目录；默认按工作区相对路径 third_party/graspnet-baseline 解析，
            # 可用环境变量 GRASPNET_MODEL_ROOT 覆盖；解析不到时为空串，由节点报未配置。
            DeclareLaunchArgument(
                "graspnet_model_root",
                default_value=_workspace_path(
                    "GRASPNET_MODEL_ROOT", "third_party/graspnet-baseline"
                ),
            ),
            # 模型权重文件路径；默认 .local-models/checkpoints/checkpoint-rs.tar，
            # 可用环境变量 GRASPNET_CHECKPOINT_PATH 覆盖；找不到时解析为空串。
            DeclareLaunchArgument(
                "graspnet_checkpoint_path",
                default_value=_workspace_path(
                    "GRASPNET_CHECKPOINT_PATH",
                    ".local-models/checkpoints/checkpoint-rs.tar",
                ),
            ),
            DeclareLaunchArgument("graspnet_device", default_value="cuda:0"),  # 推理设备标识（cuda:0 或 cpu），由后端模块解释
            DeclareLaunchArgument("graspnet_backend_module", default_value="graspnet_baseline_inference"),  # 后端包装模块名；模块内需提供 GraspNetBaselineInference 类
            DeclareLaunchArgument("graspnet_max_grasps", default_value="10"),  # 单帧最多输出的候选数；调大提高可行解命中率，但下游逆解调用量增加
            DeclareLaunchArgument("graspnet_max_points", default_value="20000"),  # 送入网络的点云采样上限；调大更准，但显存占用与单帧耗时上升
            DeclareLaunchArgument("start_grasp_preview", default_value="false"),  # 是否启动抓取预览发送器；默认关闭，仅在需要人工观察目标位姿时打开
            DeclareLaunchArgument("start_candidate_ik_filter", default_value="true"),  # 是否启动候选逆解与碰撞过滤节点（候选到可执行计划的必需环节）
            DeclareLaunchArgument("candidate_ik_input_topic", default_value="/grasp/graspnet_candidates"),  # 逆解过滤节点的输入候选话题
            DeclareLaunchArgument("start_visual_ready", default_value="true"),  # 是否提供视觉就绪摆位流程/服务；只有下面的自动移动开关也为 true 时才先摆位，否则直接启动视觉链并保留手动服务
            DeclareLaunchArgument("move_to_visual_ready_on_start", default_value="false"),  # 是否在启动时自动执行一次就绪移动；默认 false，不阻塞视觉链、不产生运动
            # 观察位姿的 6 个手臂关节角（rad，顺序 joint1..joint6，不含夹爪）。
            # joint1=-pi/2 表示本站工作区在 base 的 -Y 方向，必须与手眼标定所用位姿一致。
            DeclareLaunchArgument(
                "visual_ready_joint_positions",
                default_value="[0.0, -0.1, -0.2, 0.2, 0.0, 0.0]",
            ),
            DeclareLaunchArgument("visual_ready_duration_sec", default_value="4.0"),  # 就绪轨迹总时长（s），节点内部下限 0.2 s；调小会提高关节速度与加速度
            DeclareLaunchArgument("visual_ready_wait_timeout_sec", default_value="12.0"),  # 等待首帧关节反馈与动作服务上线的时间上限（s）；超时即失败且不自动重试
            DeclareLaunchArgument("visual_ready_max_start_delta_rad", default_value="2.5"),  # 起始偏差保护（rad）：单关节偏差超过该值就拒绝移动，防止从远处大幅横扫
            DeclareLaunchArgument("visual_ready_startup_delay_sec", default_value="0.0"),  # 启动移动前的等待时间（s），用于等控制器/时钟就绪；只推迟开始不改变轨迹
            DeclareLaunchArgument("start_visual_grasp_executor", default_value="true"),  # 是否启动抓取执行器（阶段序列编排 + 台面/夹持力/验证/撤退等安全策略）
            DeclareLaunchArgument("start_visual_grasp_markers", default_value="true"),  # 是否启动 RViz 可视化标记节点
            DeclareLaunchArgument("start_raw_candidate_markers", default_value="true"),  # 是否把 GraspNet 原始候选发布成 RViz MarkerArray；只读显示，不参与规划与执行
            DeclareLaunchArgument("start_open3d_viewer", default_value="true"),  # 是否弹出 Open3D 点云/原始夹爪查看器；无桌面环境设为 false
            DeclareLaunchArgument("start_motion_execution", default_value="true"),  # 是否启动点到点位姿执行节点（MoveIt 规划与执行入口）
            DeclareLaunchArgument("execute_gripper", default_value="true"),  # 执行器是否真正下发夹爪命令；plan_only 或仿真下会被强制跳过
            # 是否在没有外部仿真后端接管命名空间时启动仅供 RViz 使用的运动学轨迹控制器。
            # 真实硬件或已有外部仿真时应置 false，避免出现第二个同名轨迹服务。
            DeclareLaunchArgument(
                "start_sim_trajectory_controller",
                default_value="true",
                description="Start the RViz-only kinematic action server when no external simulation backend owns the namespace",
            ),
            DeclareLaunchArgument("gripper_open_axis_local_xyz", default_value="[0.0, 1.0, 0.0]"),  # 夹爪开合轴在局部坐标系中的方向向量；本站为局部 Y 轴
            DeclareLaunchArgument("show_tcp_markers", default_value="true"),  # 是否显示 TCP 标记（预抓取点与抓取点）
            DeclareLaunchArgument("show_approach_arrow", default_value="true"),  # 是否显示接近方向箭头
            DeclareLaunchArgument("show_gripper_open_axis", default_value="true"),  # 是否显示夹爪开合轴指示
            DeclareLaunchArgument("filtered_candidates_topic", default_value="/grasp/filtered_candidates"),  # 过滤后候选数组话题，供执行器换候选重试时使用
            DeclareLaunchArgument("filtered_plan_topic", default_value="/grasp/filtered_plan"),  # 过滤后抓取计划话题，记录已通过 IK 与碰撞检查的位姿
            DeclareLaunchArgument("executor_input_topic", default_value="/grasp/filtered_plan"),  # 执行器/预览/标记节点的输入话题；默认直接消费过滤后的计划
            DeclareLaunchArgument("candidate_joint_state_topic", default_value="/rebotarm/visual_joint_states"),  # 逆解使用的当前关节状态话题（IK 初值与关节运动量评分的输入）
            DeclareLaunchArgument("candidate_filter_service_timeout_sec", default_value="5.0"),  # 调用 MoveIt 逆解/碰撞检查服务的超时（s）
            DeclareLaunchArgument("candidate_collision_check_enabled", default_value="true"),  # 是否对候选解做 MoveIt 状态有效性（碰撞）检查
            DeclareLaunchArgument("candidate_collision_check_service", default_value="/check_state_validity"),  # 碰撞检查服务名
            DeclareLaunchArgument("candidate_collision_group_name", default_value="arm_with_gripper"),  # 碰撞检查用规划组；含夹爪以避免夹爪本体碰撞漏检
            DeclareLaunchArgument("pose_mode", default_value="pregrasp"),  # 预览发送的位姿模式；pregrasp = 预抓取位姿（更安全），另一可选为抓取位姿
            DeclareLaunchArgument("tcp_offset_xyz", default_value="[-0.04, 0.0, 0.0]"),  # 末端法兰到夹持中心 TCP 的平移（m，沿末端 X 轴 -4 cm），操作员实测值；改动会让抓取点整体偏移
            DeclareLaunchArgument("target_base_offset_xyz", default_value="[0.0, 0.0, 0.0]"),  # base 系下的整体平移补偿（m），用于吸收标定残差；全 0 表示不补偿
            DeclareLaunchArgument("min_target_z_m", default_value="0.0"),  # 预览/执行允许的最低目标高度（m）；大于 0 时把目标抬高到该值，0 表示不做下限抬升
            DeclareLaunchArgument("grasp_base_z_offset_m", default_value="0.0"),  # 抓取位姿在 base Z 方向的额外偏移（m）；0 表示取深度反投影的原始高度
            DeclareLaunchArgument("pose_policy", default_value="base_axis"),  # 执行器使用的姿态策略；base_axis = 按 base 接近轴构造确定姿态（候选过滤节点另用混合策略）
            # 固定抓取姿态四元数 (x, y, z, w)；沿用旧仓库 +X 工作区的单位姿态。
            # 它与 base_approach_axis_xyz 是同一安装朝向的两种表达，必须保持一致。
            DeclareLaunchArgument(
                "fixed_grasp_orientation_xyzw",
                default_value="[0.0, 0.0, 0.0, 1.0]",
            ),
            DeclareLaunchArgument("base_approach_axis_xyz", default_value="[1.0, 0.0, 0.0]"),  # base 系下的接近方向单位向量（旧仓库沿 +X 进入）
            DeclareLaunchArgument("base_pregrasp_distance_m", default_value="0.06"),  # 预抓取点沿接近轴后退的距离（m）；本启动档取 6 cm，覆盖策略档中的 8 cm
            DeclareLaunchArgument("candidate_pose_policy", default_value="preserve_candidate_pose"),  # 候选姿态策略；preserve_candidate_pose = 原样保留网络给出的 6D 姿态，由下游闸门统一过滤
            DeclareLaunchArgument("candidate_orientation_yaw_offsets_rad", default_value="[0.0]"),  # 绕竖直轴尝试的偏航角偏移列表（rad）；[0.0] 表示不做偏航试探
            DeclareLaunchArgument("candidate_grasp_z_offsets_m", default_value="[0.0]"),  # 抓取点高度偏移试探列表（m）；[0.0] 表示不做高度试探
            DeclareLaunchArgument("candidate_max_candidates_per_frame", default_value="20"),  # 每帧最多处理的候选数（下限钳到 1）；调大更全面，但逆解与碰撞检查量成倍上升
            DeclareLaunchArgument("candidate_min_confidence", default_value="0.4"),  # 候选最低置信度阈值（0~1）；低于该值直接跳过
            DeclareLaunchArgument("candidate_min_jaw_width_m", default_value="0.006"),  # 夹爪开口下限（m，6 mm）；更小说明夹爪几乎闭合，视为无效抓取
            DeclareLaunchArgument("candidate_max_jaw_width_m", default_value="0.085"),  # 夹爪开口上限（m，85 mm，接近最大行程）；超过即夹不住目标
            DeclareLaunchArgument("candidate_min_grasp_z_m", default_value="0.0"),  # 抓取点最低高度（m，base 系 z 轴向上）；低于该值判为贴地或穿桌
            DeclareLaunchArgument("candidate_pregrasp_min_z_m", default_value="0.04"),  # 预抓取目标在 base_link 下的 Z 下限（4 cm）；低于时抬高到此值，不代表离桌面4 cm
            DeclareLaunchArgument("candidate_workspace_gate_enabled", default_value="true"),  # 是否启用工作空间包围盒闸门；本档为 true，候选必须落在下面的盒内
            DeclareLaunchArgument("candidate_workspace_min_xyz", default_value="[0.18, -0.35, 0.0]"),  # 旧仓库工作空间盒最小角（m，base 系）
            DeclareLaunchArgument("candidate_workspace_max_xyz", default_value="[0.64, 0.35, 0.45]"),  # 旧仓库工作空间盒最大角（m，base 系）
            DeclareLaunchArgument("candidate_max_grasp_to_object_center_m", default_value="0.15"),  # 抓取点到物体中心的最大允许偏差（m，15 cm）；超出判该候选不可信
            DeclareLaunchArgument("candidate_score_joint_distance_weight", default_value="0.15"),  # 候选评分中关节位移的权重（每 rad 扣分）；调大更偏好动作幅度小的解
            DeclareLaunchArgument("candidate_score_joint6_weight", default_value="0.35"),  # 候选评分中 joint6 变化量的独立权重；调大更倾向保持腕部姿态、抑制绕线
            DeclareLaunchArgument("candidate_max_joint6_delta_rad", default_value="1.5708"),  # joint6 单关节最大允许角差（rad，约 90 度）；超过直接否决该候选
            DeclareLaunchArgument("candidate_joint6_symmetry_enabled", default_value="true"),  # 是否启用夹爪 180 度对称性补偿（平行夹爪绕张合轴转 pi 后仍是同一次物理抓取）
            DeclareLaunchArgument("candidate_joint6_symmetry_angle_rad", default_value="3.141592653589793"),  # 对称角（rad，等于 pi）；只有 180 度对称才物理等价，不应随意改动
            # 夹爪 launch 参数直接读取 YAML 默认值，显式传入的值仍覆盖配置。
            DeclareLaunchArgument("close_position_m", default_value="0.025"),  # 固定合爪目标位置（两指间距，m）；自适应模式开启且测得有效宽度时会被覆写
            DeclareLaunchArgument("close_max_effort", default_value="0.4"),  # 合爪阶段最大夹持力（归一化量纲，非牛顿；本站约定 0~0.6）；调大更紧但可能压坏目标
            DeclareLaunchArgument("open_before_approach", default_value="true"),  # 是否在接近目标前先张开夹爪；狭窄场景可置 false，改为到达接近点后再张开
            DeclareLaunchArgument("auto_gripper_width", default_value="true"),  # 是否按检测到的物体宽度自动推算开合爪宽度（仍受 max_allowed_grasp_width_m 约束）
            DeclareLaunchArgument("auto_gripper_effort", default_value="true"),  # 自动夹持力开关；当前实现不参与计算（合力恒取 close_max_effort），仅为参数对齐保留
            DeclareLaunchArgument("open_clearance_m", default_value="0.0"),  # 开爪宽度相对物体宽度的余量（m）；太小接近时易刮碰，太大易碰到周围物体
            DeclareLaunchArgument("close_margin_m", default_value="0.012"),  # 合爪宽度相对物体宽度收窄的量（m，12 mm），用于形成夹持预紧力；太小夹不牢，太大压坏目标
            DeclareLaunchArgument("min_gripper_effort", default_value="0.22"),  # 夹持力安全下限（归一化）；低于此值夹不紧，抬起时容易滑落
            DeclareLaunchArgument("max_gripper_effort", default_value="0.60"),  # 夹持力安全上限（归一化）；高于此值有过载与压坏目标的风险
            DeclareLaunchArgument("max_allowed_grasp_width_m", default_value="0.085"),  # 允许抓取的最大物体宽度（m）；超过即 fail-closed 拒绝本次抓取
            DeclareLaunchArgument("gripper_grasp_enabled", default_value="true"),  # 是否改用专用抓取服务（堵转推断接触 + 有界保持）代替普通位置闭合
            DeclareLaunchArgument("gripper_grasp_close_force", default_value="0.4"),  # 抓取服务的闭合夹持力（归一化，非牛顿）；硬件层会再截断到其允许区间
            DeclareLaunchArgument("gripper_grasp_timeout_sec", default_value="8.0"),  # 合爪超时（s）；超时仍未判定堵转即判本次抓取失败
            DeclareLaunchArgument("gripper_grasp_min_close_time_sec", default_value="0.08"),  # 最短合爪时间（s）；防止刚起步、速度尚未建立就被误判为已接触
            DeclareLaunchArgument("gripper_grasp_velocity_threshold", default_value="0.04"),  # 判定已停住（堵转）的角速度阈值（rad/s）；调大更易误判接触，调小更难判定
            DeclareLaunchArgument("gripper_grasp_min_closure_distance_m", default_value="0.006"),  # 判定接触所需的最小闭合行程（m）；行程不足说明是空夹，而不是夹到物体
            DeclareLaunchArgument("safe_retreat_enabled", default_value="true"),  # 闭合并验证后是否沿本次接近路径反向撤退
            DeclareLaunchArgument("safe_retreat_distance_m", default_value="0.06"),  # 沿 grasp -> pregrasp 方向撤退的距离（m，6 cm）
            DeclareLaunchArgument("safe_home_after_grasp", default_value="false"),  # 序列末尾是否回安全位。默认 false：回零是大范围动作，必须由操作员显式开启
            DeclareLaunchArgument("moveit_planning_time", default_value="8.0"),  # MoveIt 单次规划时间上限（s）；调大更可能规划成功，但整体节拍变慢
            DeclareLaunchArgument("moveit_num_planning_attempts", default_value="5"),  # MoveIt 规划尝试次数；调大提高成功率，但耗时成比例增加
            # 视觉夹取暂定上限默认档。这里的百分比是相对 MoveIt/URDF 关节上限的缩放，
            # 不是机械臂已完成真机安全验收的绝对速度；后续可在命令行逐项覆盖回退。
            DeclareLaunchArgument("move_velocity_scaling", default_value="0.25"),  # 普通移动速度：25%
            DeclareLaunchArgument("approach_velocity_scaling", default_value="0.08"),  # 最终接近抓取速度：8%
            DeclareLaunchArgument("retreat_velocity_scaling", default_value="0.15"),  # 抓取后撤退速度：15%
            DeclareLaunchArgument("acceleration_scaling", default_value="0.12"),  # 各运动阶段加速度：12%
            DeclareLaunchArgument("plan_only_stage_pause_sec", default_value="0.0"),  # plan_only 各规划阶段间的诊断等待（s）；完整轨迹最后一次发布，默认无需停顿
            DeclareLaunchArgument("approach_visual_servo_enabled", default_value="false"),  # 是否用迭代小步逼近加逐步纠偏替代一次到位的接近；默认关闭的增强项
            DeclareLaunchArgument("approach_visual_servo_max_iterations", default_value="5"),  # 视觉伺服最大迭代步数（至少 1）；调大更可能收敛，但接近段耗时成比例增加
            DeclareLaunchArgument("approach_visual_servo_max_step_m", default_value="0.02"),  # 单步最大位移（m，2 cm）；限制每步风险，调大更快但更接近一次到位的碰撞风险
            DeclareLaunchArgument("approach_visual_servo_position_tolerance_m", default_value="0.008"),  # 位置误差容差（m，8 mm）；当前点到目标点距离小于它即认为到位
            DeclareLaunchArgument("approach_visual_servo_require_fresh_plan", default_value="true"),  # 是否要求每一步都基于更新的计划；false 会允许带偏差的盲走
            DeclareLaunchArgument("auto_retry_enabled", default_value="false"),  # 阶段失败后是否自动换下一个候选重试。默认 false：失败即停并回报原因
            DeclareLaunchArgument("auto_retry_max_attempts", default_value="3"),  # 自动重试最多使用的候选个数（含首次）；仅在允许重试且该阶段可重试时生效
            DeclareLaunchArgument("safe_retreat_before_retry", default_value="true"),  # 重试前是否先撤到接近点，避免贴着目标原地换位形刮碰物体
            DeclareLaunchArgument("grasp_verification_enabled", default_value="true"),  # 是否在闭合后验证抓取成功性（接触 + 闭合行程）
            DeclareLaunchArgument("grasp_verification_min_closure_distance_m", default_value="0.006"),  # 判定确实夹住所需要的最小闭合行程（m）；行程不足说明是空夹
            DeclareLaunchArgument("grasp_verification_require_contact", default_value="true"),  # 是否必须检出夹爪接触；置 false 后仅凭闭合行程判成功，误判风险上升
            DeclareLaunchArgument("place_after_grasp_enabled", default_value="false"),  # 抓取成功后是否继续执行放置序列；默认 false，防止未经授权的大范围搬运
            DeclareLaunchArgument("place_position_xyz", default_value="[-0.20, -0.20, 0.25]"),  # 放置点位置（base 系，m）；默认落在操作者一侧、抬高的安全位置
            # 放置点姿态四元数 (x, y, z, w)；默认绕 Z 轴 -90 度，与本站抓取姿态一致，
            # 使松爪方向与目标摆放方向匹配。
            DeclareLaunchArgument(
                "place_orientation_xyzw",
                default_value="[0.0, 0.0, 0.0, 1.0]",
            ),
            DeclareLaunchArgument("place_open_position_m", default_value="0.08"),  # 放置点松爪开口宽度（m）；太小松不开带动物体，太大可能碰倒目标
            DeclareLaunchArgument("place_open_max_effort", default_value="0.25"),  # 放置点松爪夹持力（归一化，非牛顿）；松爪只需克服残余摩擦，故小于抓取力
            DeclareLaunchArgument("place_retreat_z_m", default_value="0.06"),  # 松爪后向上撤离的高度（m，6 cm），保证先垂直离开再平移
            DeclareLaunchArgument("trajectory_precheck_enabled", default_value="true"),  # 真正执行前是否先用 execute=False 走一次规划预检；false 会跳过这道门
            DeclareLaunchArgument(
                "max_plan_age_sec",
                default_value=PythonExpression(
                    [
                        "'10.0' if '",
                        execution_mode,
                        "'.lower() == 'plan_only' else '4.0'",
                    ]
                ),
                # 摄像头采帧→GraspNet→Top-10 IK/碰撞过滤实测可耗 3~5 s；
                # plan_only 再留出人工观察 RViz/触发服务的时间。只限不下发运动的预览。
                # execute 使用 4 s 默认门限，覆盖当前 RGB-D→GraspNet→IK/碰撞过滤实测延迟；
                # 更短或更长的窗口仍可由调用方显式覆盖。
                description="Maximum grasp-plan age in seconds (default: 10.0 for plan_only, 4.0 for execute)",
            ),
            # 无硬件状态后端先注册启动，随后 MoveIt 才开始等待当前状态。
            sim_trajectory_controller,
            interactive_system,
            visual_ready_startup,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=visual_ready_startup,
                    on_exit=post_visual_ready_actions,
                )
            ),
            GroupAction(
                condition=UnlessCondition(run_visual_ready_startup),
                actions=post_visual_ready_actions,
            ),
        ]
    )

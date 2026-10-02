# rebotarm_bringup 启动结构与功能

本文描述 `src/rebotarm_bringup/launch` 的当前结构、包含关系和功能边界。所有真机组合共享
同一个硬件底层片段，禁止在其它 bringup 启动文件中再次直接声明
`reBotArmController`。

`driver_only.launch.py` 已删除：它只是无附加行为地包含
`hardware_controller.launch.py`，保留两个名字反而容易让人误以为有两种硬件启动方式。
只启动底层控制器时直接使用：

```bash
ros2 launch rebotarm_bringup hardware_controller.launch.py
```

## 结构总览

```text
src/rebotarm_bringup/launch/
├── hardware_controller.launch.py
├── bringup.launch.py
├── moveit_hardware.launch.py
├── moveit_demo.launch.py
├── mujoco_moveit_sim.launch.py
├── mujoco_headless.launch.py
├── mujoco_rviz_viewer.launch.py
├── interactive_system.launch.py
├── rebotarm_app.launch.py
├── teleop_keyboard.launch.py
├── teleop_system.launch.py
├── rviz_ee_drag_sim.launch.py
├── visual_grasp_system.launch.py       # 兼容总入口（完整组合）
├── visual_readonly.launch.py           # 只读感知与候选
├── visual_plan_only.launch.py          # 规划预览
├── visual_execute.launch.py            # 显式授权后的执行组合
└── visual_ready.launch.py              # 独立就绪位姿服务
```

## 分层和包含关系

```text
hardware_controller.launch.py                  唯一真实硬件控制器定义
├── bringup.launch.py                          + 状态发布 + 基础 RViz
├── interactive_system.launch.py               状态源/MoveIt/RViz 的共享实现
│   ├── moveit_hardware.launch.py              固定真机后端的薄入口
│   │   └── rebotarm_app.launch.py             + 示教录制 + Dashboard + 状态 RViz（不含键盘）
│   ├── rviz_ee_drag_sim.launch.py             仿真 MotionPlanning 入口
│   └── visual_grasp_system.launch.py          感知预览、plan-only 与完整视觉抓取组合
├── teleop_keyboard.launch.py                  键盘 + 状态发布 + 基础 RViz（控制器点动适配）
│   └── teleop_system.launch.py                + 示教录制 + Dashboard（控制器点动适配）
```

`interactive_system.launch.py` 只有在 `use_hardware=true` 时才包含硬件片段；纯预览和仿真
分支不会打开串口。上层入口必须确保同一命名空间只有一个真实或仿真轨迹执行后端。

## 文件功能

| 文件 | 类型 | 功能和边界 |
|---|---|---|
| `hardware_controller.launch.py` | 底层片段 | 唯一直接启动 `reBotArmController` 的文件；统一串口、反馈频率、夹爪保护、仲裁、命名空间和坐标系参数 |
| `bringup.launch.py` | 基础真机入口 | 硬件、夹爪可视化状态桥、TF 和可选基础 RViz；不启动 MoveIt |
| `moveit_hardware.launch.py` | 唯一真机 MoveIt 用户入口 | 薄包装 `interactive_system.launch.py`，固定真机、MoveIt、真实关节状态和唯一状态源；启动后仍需显式 Enable |
| `moveit_demo.launch.py` | 离线 MoveIt 演示入口 | 使用 `rebotarm_moveit_config` 的模型与规划参数，启动 MoveIt、RViz 和夹爪可视化桥；不接真机 |
| `mujoco_moveit_sim.launch.py` | MuJoCo + MoveIt 组合 | 启动唯一 MuJoCo 执行后端并包含本包的 MoveIt 演示入口；无头和桌面薄入口位于同目录 |
| `interactive_system.launch.py` | 共享实现 | 统一拥有状态发布、MoveIt、RViz 以及真机/无硬件状态源的互斥选择；不建议用户手写其内部组合参数启动真机 |
| `rebotarm_app.launch.py` | 完整真机入口 | 包含真机 MoveIt 组合，再增加示教录制、Dashboard 和状态 RViz；不启动键盘节点 |
| `teleop_keyboard.launch.py` | 遥操作入口 | 可选硬件、键盘关节点动、状态发布和 RViz；默认不接真机。无硬件模式使用轻量仿真轨迹控制器，按键可以改变 RViz 姿态，但不代表物理仿真 |
| `teleop_system.launch.py` | 遥操作组合 | 包含键盘入口，再增加示教录制和默认只读 Dashboard；不启动 MoveIt，默认不接真机。无硬件模式可驱动 RViz 仿真姿态，真机点动仍由控制器执行 |
| `rviz_ee_drag_sim.launch.py` | 仿真规划入口 | 使用仿真轨迹控制器提供 Plan/Execute，不打开真机 |
| `visual_grasp_system.launch.py` | 兼容总入口 | 保留旧参数接口，组合所有阶段；默认 `use_hardware=false`、`execution_mode=plan_only`，不打开真机 |
| `visual_readonly.launch.py` | 只读感知入口 | 相机/YOLO/TCP TF、GraspNet 候选与可视化；不启动 MoveIt、IK、轨迹或夹爪 |
| `visual_plan_only.launch.py` | 规划入口 | 感知、候选、IK/碰撞过滤和 MoveIt 规划预览；固定 `plan_only`，不执行夹爪 |
| `visual_execute.launch.py` | 执行入口 | 在显式 `use_hardware:=true` 和控制器 Enable 门控后使用；默认仍为无硬件 |
| `visual_ready.launch.py` | 就绪位姿入口 | 只提供视觉观察位姿服务/一次性摆位，不启动视觉链或执行器 |

## 两组容易混淆的入口

| 需求 | 应使用 | 不包含 |
|---|---|---|
| 只需真机 MoveIt 规划/执行 | `moveit_hardware.launch.py` | Dashboard、键盘遥操、示教录制 |
| 需要真机 MoveIt + 示教录制 + Dashboard 完整工作台 | `rebotarm_app.launch.py` | 键盘遥操 |
| 只需键盘点动和基础 RViz | `teleop_keyboard.launch.py` | MoveIt、Dashboard、示教录制；无硬件模式使用轻量仿真执行键盘点动 |
| 需要键盘 + 示教录制 + Dashboard，且不需要 MoveIt | `teleop_system.launch.py` | MoveIt |

因此它们共享部分界面，但不是重复实现。`moveit_hardware.launch.py` 和
`teleop_keyboard.launch.py` 是可独立使用的基础组合；各自的上层入口只叠加自己的功能。
`rebotarm_app.launch.py` 与 `teleop_system.launch.py` 不应在同一命名空间同时启动。

视觉阶段现在按边界提供独立入口。只读感知诊断使用：

```bash
ros2 launch rebotarm_bringup visual_readonly.launch.py \
  start_open3d_viewer:=true
```

规划预览使用：

```bash
ros2 launch rebotarm_bringup visual_plan_only.launch.py \
  use_hardware:=false
```

受控执行使用 `visual_execute.launch.py`，只有在完成现场安全检查、获取新鲜反馈并由
操作员显式 Enable 后才允许传入 `use_hardware:=true`。兼容总入口仍保留，便于旧脚本
迁移期间逐步切换：

```bash
ros2 launch rebotarm_bringup visual_grasp_system.launch.py \
  use_hardware:=false \
  execution_mode:=plan_only \
  execute_gripper:=false \
  start_visual_ready:=false \
  start_motion_execution:=false \
  start_visual_grasp_executor:=false \
  start_open3d_viewer:=true \
  use_local_rviz:=true
```

该组合仍保留无硬件状态后端与 MoveIt 服务供 TF、IK 和候选过滤使用，但没有抓取执行
服务，也不会打开串口。真实反馈 plan-only 或真机执行仍使用同一 launch，并通过
`use_hardware`、`execution_mode` 和显式 Enable 安全门逐级启用。

旧的 `rviz_ee_drag_real.launch.py` 已删除：它与
`moveit_hardware.launch.py use_rviz:=true` 提供同一套真机 MotionPlanning 能力，
而且两者都需要显式 `/rebotarm/enable`。仿真入口 `rviz_ee_drag_sim.launch.py`
因拥有独立仿真轨迹后端而继续保留。

`moveit_hardware.launch.py` 不再维护第二套硬件与 MoveIt 节点定义，而是固定参数后
包含 `interactive_system.launch.py`。这样共享实现只有一份，同时保留简短、安全的
真机用户命令。

### 示教回放入口

Dashboard 的 `TeachReplayWorkflow` 是唯一正式回放实现，负责质量分析、预处理、dry-run 令牌、MoveIt 起点对齐、碰撞预检和运行期跟踪监控。旧的 `TeachReplayNode` 和 `teach_replay.launch.py` 已删除，避免两套回放策略产生分歧。示教回放使用完整工作台的 Teach Trajectory 卡片：先 Check/Dry-run，再在显式执行授权下 Replay。

## 硬件公共参数

`hardware_controller.launch.py` 是以下硬件默认值的权威定义：

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `channel` | 空串 | 由配置决定硬件通道；上层可传入探测后的设备路径 |
| `shutdown_safe_home` | `false` | 退出时不自动产生回位运动 |
| `joint_state_rate` | `100.0` | 关节状态发布频率，Hz |
| `hardware_feedback_rate_hz` | `50.0` | 硬件反馈刷新频率，Hz |
| `gripper_position_torque_cap_nm` | `1.0` | 夹爪位置模式力矩上限，N·m |
| `gripper_position_max_speed_rad_s` | `1.5` | 夹爪位置模式速度上限，rad/s |
| `gripper_position_timeout_margin_sec` | `1.5` | 夹爪到位超时余量，s |
| `gripper_feedback_stale_timeout_sec` | `0.15` | 夹爪反馈过期阈值，s |
| `grasp_hold_timeout_sec` | `30.0` | 夹持保持时限，s |
| `cmd_arbitration` | `reject` | 轨迹期间拒绝冲突的透传命令 |
| `arm_namespace` | `rebotarm` | 话题、服务和动作命名空间 |
| `frame_id` / `ee_frame_id` | `base_link` / `end_link` | 基座和末端坐标系 |

上层文件可以声明同名参数作为自己的公开接口并转发覆盖值，但不得复制控制器 `Node`。

### 为什么 `DeclareLaunchArgument` 看起来重复

ROS 2 的每个 launch 文件都有自己的命令行接口。子 launch 声明“我接受哪些参数”；
父 launch 若希望用户仍能在最外层传该参数，需要将它作为自己的公开参数并转发。
因此“同名声明 + 转发”是接口的逐层导出，不等于重复启动节点。

本目录按以下规则维护：

1. 控制器节点只由 `hardware_controller.launch.py` 拥有，硬件默认值以它为权威定义；
2. 上层只重新声明需要从该入口命令行对外暴露或本层节点需要使用的参数；
3. 父层可以选择更保守的场景默认值，但必须在文档中说明；
4. 不为了减少几行声明而隐藏顶层入口的 `--show-args` 接口。

## 安全约束

- 真实硬件启动后保持失能；新鲜反馈、现场检查和显式 `/rebotarm/enable` 缺一不可。
- 一个串口和一个机械臂命名空间只能有一个 `reBotArmController`。
- 真机入口、仿真入口和完整工作台入口不能并行启动在同一命名空间。
- 仿真/预览分支不能包含 `hardware_controller.launch.py` 的有效实例。
- `visual_grasp_system` 的无硬件 `plan_only` 分支必须在 MoveIt 初始化前提供关节状态；默认不自动摆到视觉准备位，也不得因此阻塞后续视觉链。
- `plan_only` 不发送 `FollowJointTrajectory`；后一阶段使用上一阶段轨迹终点作为虚拟规划起点。所有阶段规划成功后，运动层才通过一条 `/display_planned_path` 消息连续播放完整预览，阶段失败时不发布半条序列。带虚拟起点的 `execute=true` 请求必须拒绝。
- 眼在手上的相机使用 `use_hardware:=false` 时，`base_link -> camera` 来自配置的假关节角；因此它只能验收软件规划链，不能用 RViz 中的基座坐标判断实物位置。完整视觉入口默认不显示容易被误认为实测轮廓的物体示意圆柱、中心点和文字。
- MoveIt Plan 成功不是 Execute 成功；执行链仍需要 `moveit_simple_controller_manager`。
- 健康真机发生可恢复任务失败时保持 enabled hold，或在守护下回到已验证基线后再失能。

## 结构验证

结构测试必须保证：

1. 只有 `hardware_controller.launch.py` 包含 `package="rebotarmcontroller"`；
2. 所有需要真机的组合都包含该文件；
3. 只读感知和纯仿真入口不直接引用真实控制器；
4. `setup.py` 持续安装全部 `*.launch.py`，新增片段在安装空间可被解析。

# 命令参考索引

> 状态：REFERENCE；类型：可复制命令索引；适用范围：当前保留的启动、检查、构建和停止命令。

## 命令归属

每类命令只在一个文件中维护；其他文档只链接，不复制完整命令。

| 主题 | 权威文件 | 内容边界 |
|---|---|---|
| 系统环境、控制器、仿真和软件检查 | [system_runtime.md](system_runtime.md) | 通用启动、状态和验证 |
| 网页/键盘遥操作、示教录制和回放 | [rebotarm_common_commands.md](rebotarm_common_commands.md) | 操作员工作流 |
| RViz MoveIt 和真机 Plan/Execute | [rebotarm_feature_commands.md](rebotarm_feature_commands.md) | MoveIt 操作 |
| Ubuntu 视觉只读链路 | [ubuntu_vision_readonly_test_zh.md](ubuntu_vision_readonly_test_zh.md) | 相机、YOLO、GraspNet 只读验证 |
| 视觉候选、plan-only 和视觉执行边界 | [visual_grasp_commands.md](visual_grasp_commands.md) | 视觉抓取工作流 |
| 网页手眼/TCP 标定 | [calibration_web.md](calibration_web.md) | 标定软件操作 |
| MuJoCo 示教预演 | [mujoco_teach_preview.md](mujoco_teach_preview.md) | 六轴轨迹预演 |
| mjlab GPU Reach/RL（实验性） | [mjlab_rl.md](mjlab_rl.md) | 独立 mjlab/Warp/PyTorch 训练环境 |

## 源码入口覆盖

下面的表是本目录与当前源码入口的对照表。“用户入口”可按表直接运行；“组合内部”
由上层 launch 自动启动，不再复制逐节点流程；“维护入口”用于开发、模型或数据检查。

### Launch 入口

| 入口 | 分类 | 权威说明 |
|---|---|---|
| `rebotarm_bringup hardware_controller.launch.py` | 用户入口：真机底层连接 | [system_runtime.md](system_runtime.md) |
| `rebotarm_bringup bringup.launch.py` | 用户入口：硬件 + 基础状态/RViz | [system_runtime.md](system_runtime.md) |
| `rebotarm_bringup moveit_hardware.launch.py` | 用户入口：真机 MoveIt | [rebotarm_feature_commands.md](rebotarm_feature_commands.md) |
| `rebotarm_bringup rebotarm_app.launch.py` | 用户入口：完整工作台 | [rebotarm_common_commands.md](rebotarm_common_commands.md) |
| `rebotarm_bringup teleop_keyboard.launch.py` | 用户入口：键盘点动 | [rebotarm_common_commands.md](rebotarm_common_commands.md) |
| `rebotarm_bringup teleop_system.launch.py` | 用户入口：键盘 + Dashboard + 示教 | [rebotarm_common_commands.md](rebotarm_common_commands.md) |
| `rebotarm_bringup rviz_ee_drag_sim.launch.py` | 用户入口：仿真 RViz 拖动 | [rebotarm_feature_commands.md](rebotarm_feature_commands.md) |
| `rebotarm_bringup visual_grasp_system.launch.py` | 用户入口：视觉只读/plan-only/受控执行 | [visual_grasp_commands.md](visual_grasp_commands.md) |
| `rebotarm_bringup interactive_system.launch.py` | 组合内部：真机/无硬件状态源和 MoveIt 共享实现 | [src/rebotarm_bringup/launch/README.md](../../../src/rebotarm_bringup/launch/README.md) |
| `rebotarm_bringup moveit_demo.launch.py` | 组合内部：MoveIt 配置实现 | [src/rebotarm_bringup/launch/README.md](../../../src/rebotarm_bringup/launch/README.md) |
| `rebotarm_vision vision.launch.py` / `vision_ubuntu.launch.py` | 组合内部：由视觉总入口包含 | [ubuntu_vision_readonly_test_zh.md](ubuntu_vision_readonly_test_zh.md) |
| `rebotarm_bringup mujoco_headless.launch.py` | 用户入口：无头物理仿真 | [system_runtime.md](system_runtime.md) |
| `rebotarm_bringup mujoco_moveit_sim.launch.py` | 用户入口：MuJoCo + MoveIt/RViz | [system_runtime.md](system_runtime.md) |
| `rebotarm_bringup mujoco_rviz_viewer.launch.py` | 用户入口：MuJoCo Viewer + RViz | [system_runtime.md](system_runtime.md) |
| `rebotarm_simulation mujoco_sim.launch.py` | 维护/底层入口：单独 ROS 仿真后端 | [system_runtime.md](system_runtime.md) |

### `ros2 run` 入口

| 入口类别 | 当前入口 | 说明 |
|---|---|---|
| 标定用户/维护 | `rebotarm_handeye_capture`、`rebotarm_handeye_calibration`、`rebotarm_handeye_residual`、`rebotarm_tcp_calibration` | 网页采样、离线求解、残差报告、TCP 交互；见 [calibration_web.md](calibration_web.md) |
| MuJoCo 维护 | `rebotarm_mujoco_health`、`rebotarm_mujoco_cli`（别名 `rebotarm_mujoco`）、`rebotarm_urdf_to_mjcf`、`rebotarm_mujoco_teach_preview` | 启动前健康检查、无头 CLI、URDF→MJCF 生成/一致性检查及示教预演；见 [system_runtime.md](system_runtime.md) 和 [mujoco_teach_preview.md](mujoco_teach_preview.md) |
| MuJoCo 组合内部 | `rebotarm_mujoco_node`、`rebotarm_sim_trajectory_controller` | 由仿真 launch 启动，不作为日常独立流程 |
| 视觉组合内部/维护 | `rebotarm_vision_node`、`rebotarm_graspnet_baseline_node`、`rebotarm_grasp_candidate_ik_filter`、`rebotarm_visual_grasp_executor` 等 | 由 `visual_grasp_system.launch.py` 按参数组合；不重复维护逐节点启动文档 |
| 操作与示教 | `TeleopKeyboardNode`、`TeleopStatusPanelNode`、`TeachRecorderNode` | 通过对应组合 launch 使用 |
| 控制器示例 | `GravityCompensation`、`GripperControl`、`MoveTo`、`MoveToPose` | 示例客户端，不是推荐的正式用户入口；正式流程使用 Dashboard/MoveIt/视觉命令 |

## 维护规则

新增命令先判断是否属于已有主题；属于已有主题时修改对应权威文件，不新建重复说明。命令失效时从索引移除或改为明确的退役说明，不保留可复制的旧入口。

每个命令文件只保留一个用户任务的完整流程。功能说明放在 `implemented/features/`，节点和参数说明放在 `reference/topology/`、`reference/parameters/`，测试原始记录放在 `Agent/evidence/`。

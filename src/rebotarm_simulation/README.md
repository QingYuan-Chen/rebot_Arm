# rebotarm_simulation

MuJoCo 离线物理与 ROS 2 仿真后端包。它提供模型、物理步进、仿真控制器、Viewer 和离线指标；不导入真实电机 SDK，不启动 `rebotarmcontroller`，也不依赖 `rebotarm_motion` 的实现。详细安装与运行说明见 [`README_mujoco.md`](README_mujoco.md)。

选择 `models/rebotarm/scene_bottle.xml` 后可用 `RebotArmMujoco.randomize_bottle_pose(seed=...)` 做可复现的位姿变化；
`save_state` / `restore_state` 可回放同一仿真状态。接触快照包含力、法向与穿透深度，
ROS 节点在 `/diagnostics` 发布仿真控制与接触告警。

示教轨迹的独立 MuJoCo 预演见 [操作说明](../../docs/reference/commands/mujoco_teach_preview.md)。

## 目录结构

```text
rebotarm_simulation/
├── rebotarm_simulation/
│   ├── core/          # MuJoCo入口、状态类型、模型命名契约、资源定位
│   ├── control/       # 纯控制算法、控制参数加载、夹爪映射
│   ├── execution/     # 轨迹采样、离线执行、ROS执行状态与并发闸门
│   ├── ros/           # ROS节点、消息转换与诊断
│   ├── model_tools/   # URDF→MJCF与相机载荷生成
│   ├── diagnostics/   # 模型健康、限位、响应/抓取试验和指标
│   └── apps/          # CLI、Viewer、示教物理预演
├── models/rebotarm/    # robot.xml、scene.xml、reach_scene.xml与网格
├── config/            # 仿真与控制标定参数
├── launch/            # 单独MuJoCo节点入口
└── setup.py / package.xml / requirements-mujoco.txt
```

MuJoCo + MoveIt、无头和桌面组合入口位于 `rebotarm_bringup/launch`。

## 当前功能分组

| 分组 | 当前实现 | 主要文件/入口 | 当前状态 |
|---|---|---|---|
| 模型与场景 | URDF→MJCF 生成、模型一致性检查、机器人本体、空桌面场景、Reach 场景 | `model_tools/urdf_to_mjcf.py`、`model_tools/gemini2_payload.py`、`models/rebotarm/` | 已接入；`robot.xml` 是机器人本体，`scene.xml` 是桌面物理场景，`reach_scene.xml` 是 Reach 训练场景 |
| CPU MuJoCo 物理核心 | 物理步进、关节/夹爪控制、状态、接触、保存恢复、瓶子位姿 | `core/mujoco_sim.py`、`control/motor_control.py`、`core/mujoco_types.py`、`control/sim_gripper.py` | 当前 CPU 正确性和轨迹基准 |
| ROS / MoveIt 执行 | FollowJointTrajectory、关节/夹爪状态、仿真时钟、停止、容差和诊断 | `ros/mujoco_ros_node.py`、`execution/trajectory_sampler.py`、`ros_diagnostics.py` | 已接入 MuJoCo 组合 launch |
| CLI / Viewer 交互 | 无头 CLI、桌面 MuJoCo Viewer、键盘关节和夹爪操作 | `apps/mujoco_cli.py`、`apps/mujoco_viewer.py` | 已接入，属于人工调试/观察入口 |
| 示教轨迹预演 | 示教记录质量检查、准备、MuJoCo 轨迹执行和接触统计 | `apps/teach_preview.py`、`offline_trajectory.py` | 已接入，属于离线软件仿真 |
| 健康检查与离线评估 | 模型健康、限位、阶跃响应、抓取质量、成对轨迹统计 | `diagnostics/mujoco_health.py`、`diagnostics/mujoco_limit_checks.py`、`diagnostics/mujoco_runner.py`、`diagnostics/mujoco_grasp_quality.py`、`diagnostics/paired_trajectory_analysis.py` | 维护/测试/评估工具；不自动连接硬件 |

MJLab 训练任务和策略配对评估位于独立工程 `/home/a/project/rebot_Arm_rl/MJLab/`，见[本地 MJLab 命令说明](../../docs/reference/commands/mjlab_rl.md)。本包保留共享模型、CPU 物理后端及已有 Reach/Pick 离线验证能力。

### RViz 预演的边界

`rebotarm_sim_trajectory_controller` 由独立的 `rebotarm_preview` 包提供，只是在内存中对目标轨迹做插值并发布关节状态。
它不加载 MJCF，不推进 MuJoCo 物理，不计算动力学、执行器力、物体接触或碰撞结果。
因此以下入口中的“执行成功”只能证明 ROS/MoveIt/视觉流程和 RViz 姿态链路可运行：

- `teleop_keyboard.launch.py` 的无硬件模式；
- `rviz_ee_drag_sim.launch.py`；
- `visual_grasp_system.launch.py` 的 `use_hardware:=false` 模式。

需要验证物理、动力学或接触时，使用 `mujoco_moveit_sim.launch.py` 或单独的
`mujoco_sim.launch.py`。完整入口通过 `use_rviz` 和 `use_mujoco_viewer` 控制窗口。

## 执行后端的区别

| 后端 | 使用入口 | 执行方式 |
|---|---|---|
| `rebotarm_mujoco_node` | `mujoco_sim`、`mujoco_moveit_sim` | 加载 MJCF，推进物理并反馈实际仿真状态 |
| `rebotarm_sim_trajectory_controller`（`rebotarm_preview`） | 无硬件键盘、`rviz_ee_drag_sim`、无硬件视觉抓取组合 | 在内存中插值轨迹，用于 ROS 接口和 RViz 流程预演 |

RViz 预演不计算动力学、力矩或物体接触，也不自行检查碰撞。视觉组合的
`use_hardware:=false` 当前选择 RViz 预演后端；物理验证使用 MuJoCo 组合入口。
两种后端提供相同的控制器话题、服务和 Action，同一命名空间只运行一个。
该入口名称保持兼容；它只用于轻量 RViz 轨迹预演，不代表 MuJoCo 物理仿真。

## 对外入口

```text
rebotarm_mujoco_node               -> ROS 2 仿真节点
rebotarm_preview/rebotarm_sim_trajectory_controller -> RViz 轨迹插值预演（无物理引擎）
rebotarm_mujoco_health             -> 模型/物理/渲染健康检查
rebotarm_mujoco_cli                -> mujoco_cli.py
rebotarm_mujoco_viewer             -> 桌面 Viewer
rebotarm_urdf_to_mjcf              -> URDF/MJCF 生成与一致性检查
```

常用命令：

```bash
ros2 launch rebotarm_bringup mujoco_moveit_sim.launch.py use_rviz:=false use_mujoco_viewer:=false
ros2 launch rebotarm_bringup mujoco_moveit_sim.launch.py
ros2 run rebotarm_simulation rebotarm_mujoco_health -- --renderer-timeout 30
ros2 run rebotarm_simulation rebotarm_urdf_to_mjcf -- --repo-root . --check
```

运行 MuJoCo 前需选择包含 `mujoco` 的解释器；launch 支持 `python_executable` 或 `REBOTARM_MUJOCO_PYTHON`。仿真联调必须明确 `use_hardware:=false`，并保证 `/rebotarm/follow_joint_trajectory` 只有一个服务端。

## Python API 与评估契约

Python 导入统一使用上述子包路径；旧顶层模块和 `mujoco_adapter_core` 不再保留。
ROS console script 名称保持不变。示例：

```python
from rebotarm_simulation.core.mujoco_sim import RebotArmMujoco
from rebotarm_simulation.diagnostics.mujoco_runner import run_step_response
```

`run_step_response` 使用正式位置控制器驱动力矩模型，不直接将弧度写入 `ctrl`。
`run_smoke` 仅检查自由步进数值稳定性。
`run_grasp_benchmark` 要求显式提供 `target_body`、两个 `gripper_bodies` 及
`command(sim, elapsed_seconds)` 试验过程；仅统计目标与指定双指接触，成功要求
结束时双指接触且目标高度增加至少 3 cm。它不是抓取规划器，也不保证持续稳定保持。
报告采用 `initial_object_height_m` / `object_height_m`，旧 box 字段已移除。

模型/配置通过 `core.resource_paths` 统一定位：源码资源优先，其次 Python prefix、
AMENT_PREFIX_PATH 与 ament index；显式模型路径优先于发现规则。
控制参数可通过 `RebotArmMujoco(..., motor_parameters=...)` 注入。
RL 仅共享版本化 MJCF 资源，不导入 simulation Python 实现。

仿真状态和接触的只读转换由 `core/observations.py` 提供；`RebotArmMujoco`
仍持有模型、数据与生命周期。诊断通过 `has_body()` 查询模型成员，不再获取 Viewer 原生句柄。
`diagnostics/physics_probe.py` 统一基础健康检查与自由步进 smoke 的模型加载和有限性判定，
逐步检查 qpos、qvel、执行器力与时间；中途出现非有限值不会被后续恢复掩盖。
模型生成一致性检查仍单独比较生成产物，渲染探针仍使用独立子进程隔离 GL 故障。

执行模块已按职责划分为 `trajectory_state.py`（准入状态、停止仲裁、异常清理）、
`goal_policy.py`（收敛判定）、`feedback_timing.py`（反馈节拍）、
`simulation_access.py`（仿真串行访问）。原 `execution/runtime.py` 已移除。
ROS 消息形态转换、时间戳和输入规模检查归 `ros/message_codec.py`。
Viewer 的按键与交互状态归 `apps/viewer_interaction.py`，原生资源关闭保护归
`apps/viewer_lifecycle.py`；`apps/mujoco_viewer.py` 保留命令行与事件循环。


控制目标、模式、控制节拍、积分和力矩记忆统一归
`control/sim_control_runtime.py::SimControlRuntime`，每次调用借用 model/data，
不加载、步进或释放物理引擎。快照校验和物理/控制状态的成套保存恢复归
`core/state_snapshot.py::StateSnapshot`，只接受同一模型实例的内存快照，
全部输入校验通过后才修改运行状态；没有新增文件格式或版本兼容层。
`RebotArmMujoco` 保留原有控制、step、reset、save/restore 入口，并唯一管理模型生命周期。
Viewer 通过 `borrow_viewer_handles()` 借用可变原生句柄，不转移所有权也不隐式加锁；
调用方负责串行同步，并在释放仿真之前完成原生 Viewer 关闭。

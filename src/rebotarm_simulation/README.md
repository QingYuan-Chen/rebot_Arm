# rebotarm_simulation

MuJoCo 离线物理与 ROS 2 仿真后端包。它提供模型、物理步进、仿真控制器、Viewer 和离线指标；不导入真实电机 SDK，不启动 `rebotarmcontroller`，也不依赖 `rebotarm_motion` 的实现。详细安装与运行说明见 [`README_mujoco.md`](README_mujoco.md)。

默认瓶子场景可用 `RebotArmMujoco.randomize_bottle_pose(seed=...)` 做可复现的位姿变化；
`save_state` / `restore_state` 可回放同一仿真状态。接触快照包含力、法向与穿透深度，
ROS 节点在 `/diagnostics` 发布仿真控制与接触告警。

示教轨迹的独立 MuJoCo 预演见 [操作说明](../../docs/reference/commands/mujoco_teach_preview.md)。

## 目录结构

```text
rebotarm_simulation/
├── rebotarm_simulation/
│   ├── mujoco_sim.py                # RebotArmMujoco 物理/API 核心
│   ├── mujoco_runner.py             # 模型加载、步进和运行封装
│   ├── mujoco_adapter_core.py       # ROS/仿真状态适配公共逻辑
│   ├── mujoco_ros_node.py           # ROS 2 MuJoCo 状态、服务和轨迹 Action
│   ├── sim_trajectory_controller_node.py # 轻量仿真 FollowJointTrajectory 服务端
│   ├── sim_gripper.py                # 仿真夹爪状态与开口控制
│   ├── motor_control.py              # 仿真关节/执行器控制
│   ├── trajectory_sampler.py         # 轨迹采样
│   ├── mujoco_types.py               # 仿真状态/对象类型
│   ├── resource_paths.py             # share/model/config 路径定位
│   ├── mujoco_model_profile.py       # 模型关节、执行器和档位校验
│   ├── mujoco_limit_checks.py        # 限位和配置一致性检查
│   ├── mujoco_health.py              # 物理、模型和渲染健康检查
│   ├── mujoco_legacy_health.py       # 兼容健康检查入口
│   ├── mujoco_metrics.py             # 轨迹/响应指标
│   ├── paired_trajectory_analysis.py # 成对轨迹分析
│   ├── mujoco_grasp_quality.py       # 仿真接触/抓取质量判定辅助
│   ├── urdf_to_mjcf.py               # URDF -> MJCF 生成与 --check
│   ├── mujoco_cli.py                 # 无头交互式 CLI
│   ├── mujoco_viewer.py              # 桌面 Viewer 键盘控制
│   └── __init__.py
├── models/rebotarm/                  # 自动生成的 robot.xml、scene.xml 和 STL
├── rebotarm_simulation/assets/       # 随 Python 包安装的 XML 片段
├── config/                           # 仿真和电机标定参数
├── launch/
│   ├── mujoco_sim.launch.py          # 单独 MuJoCo ROS 后端
│   ├── mujoco_headless.launch.py     # 无头入口
│   ├── mujoco_rviz_viewer.launch.py  # MuJoCo + 桌面 Viewer/RViz 组合
│   └── mujoco_moveit_sim.launch.py   # MuJoCo + MoveIt 仿真组合
└── setup.py / package.xml / requirements-mujoco.txt
```

## 对外入口

```text
rebotarm_mujoco_node               -> ROS 2 仿真节点
rebotarm_sim_trajectory_controller -> 轻量轨迹 Action 服务端
rebotarm_mujoco_health             -> 模型/物理/渲染健康检查
rebotarm_mujoco_cli                -> mujoco_cli.py
rebotarm_mujoco                   -> mujoco_cli.py 兼容别名
rebotarm_mujoco_viewer             -> 桌面 Viewer
rebotarm_urdf_to_mjcf              -> URDF/MJCF 生成与一致性检查
```

常用命令：

```bash
ros2 launch rebotarm_simulation mujoco_headless.launch.py
ros2 launch rebotarm_simulation mujoco_moveit_sim.launch.py
ros2 run rebotarm_simulation rebotarm_mujoco_health -- --renderer-timeout 30
ros2 run rebotarm_simulation rebotarm_urdf_to_mjcf -- --repo-root . --check
```

运行 MuJoCo 前需选择包含 `mujoco` 的解释器；launch 支持 `python_executable` 或 `REBOTARM_MUJOCO_PYTHON`。仿真联调必须明确 `use_hardware:=false`，并保证 `/rebotarm/follow_joint_trajectory` 只有一个服务端。

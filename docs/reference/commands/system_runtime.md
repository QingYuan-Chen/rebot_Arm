# 系统运行命令参考

> 状态：REFERENCE；类型：硬件、仿真、构建和运行状态的通用命令入口；适用范围：系统级操作；真机命令不自动授权运动。

本页补齐没有被专题文档覆盖的系统级入口。网页遥操作、视觉、标定、RL 和示教预演的完整流程仍以对应专题文档为准，避免同一条命令在多个文件中漂移。

## 前置条件与构建

```bash
cd /home/huangbin/robotarm_ros2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to rebotarm_bringup
source install/setup.bash
```

只读检查当前工作区是否能发现包：

```bash
ros2 pkg list | grep -E 'rebotarm(controller|_bringup|_motion|_teach|_teleop|_dashboard|_vision|_calibration|_simulation|_moveit_config)$'
```

## 操作步骤：真机控制器（仅连接，不代表已使能）

> 使用前确认串口唯一占用、现场净空、急停和支撑。控制器默认失能；不得把启动成功当作可以运动。

```bash
ros2 launch rebotarm_bringup hardware_controller.launch.py \
  channel:=/dev/ttyACM0
```

启动后先检查新鲜反馈和状态：

```bash
ros2 topic echo --once /rebotarm/arm_status
ros2 topic echo --once /rebotarm/joint_states
ros2 action list | grep follow_joint_trajectory
ros2 service list | grep -E '/rebotarm/(enable|disable|safe_home|trajectory_stop)'
```

只有在本次操作明确授权后，才可另行调用 `/rebotarm/enable`；停止、回位、静止确认和失能顺序见 [功能命令](rebotarm_feature_commands.md)。

如果只需要基础状态发布和可选基础 RViz，不需要 MoveIt，使用基础 bringup 入口：

```bash
ros2 launch rebotarm_bringup bringup.launch.py \
  channel:=/dev/ttyACM0 \
  use_rviz:=true
```

它仍然默认失能；需要规划或执行时改用 `moveit_hardware.launch.py`，不要在同一命名空间
重复启动两套真机入口。

## 操作步骤：仿真后端

无 RViz 的 headless 物理检查：

```bash
ros2 launch rebotarm_simulation mujoco_headless.launch.py
```

带 MoveIt/RViz Plan & Execute 的仿真入口：

```bash
ros2 launch rebotarm_simulation mujoco_moveit_sim.launch.py use_rviz:=true
```

RViz 末端拖动专用入口（包含仿真轨迹控制器）：

```bash
ros2 launch rebotarm_bringup rviz_ee_drag_sim.launch.py
```

MuJoCo Viewer 与单独 ROS 后端也可以分别启动。它们不会打开真实串口：

```bash
# MuJoCo Viewer + RViz 组合
ros2 launch rebotarm_simulation mujoco_rviz_viewer.launch.py

# 只启动 MuJoCo ROS 后端（维护/调试入口）
ros2 launch rebotarm_simulation mujoco_sim.launch.py
```

检查仿真 Action 是否唯一：

```bash
ros2 action list | grep follow_joint_trajectory
ros2 topic echo --once /rebotarm/joint_states
```

## MuJoCo 维护检查

以下命令用于确认模型、渲染器和 URDF→MJCF 产物，不等于真机验收：

```bash
# 模型和物理健康检查；无显示环境可跳过渲染器
ros2 run rebotarm_simulation rebotarm_mujoco_health -- --skip-renderer

# 无头短时运行，检查 CLI 和物理步进
ros2 run rebotarm_simulation rebotarm_mujoco_cli -- --headless --duration 5

# 检查生成的 MJCF 是否与 URDF/资源一致
ros2 run rebotarm_simulation rebotarm_urdf_to_mjcf -- --repo-root . --check
```

`rebotarm_mujoco` 是 `rebotarm_mujoco_cli` 的兼容别名。`rebotarm_mujoco_node` 和
`rebotarm_sim_trajectory_controller` 属于 launch 内部节点，不应和上述入口并行手动
启动，否则可能产生重复仿真后端。

## 验证：纯软件与分层检查

```bash
cd /home/huangbin/robotarm_ros2
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests -q
python3 -m pytest tests/test_package_layering.py -q
python3 -m compileall src/rebotarm_dashboard/rebotarm_dashboard \
  src/rebotarm_teleop/rebotarm_teleop \
  src/rebotarm_teach/rebotarm_teach \
  src/rebotarm_motion/rebotarm_motion -q
python3 -m compileall src/rebotarm_bringup/launch -q
```

测试通过只说明软件/仿真条件满足，不改变真机授权、现场安全或硬件验收状态。

## 相关命令文档

| 主题 | 权威命令文档 |
|---|---|
| 网页/键盘遥操作、示教录制与回放、状态和停机 | [rebotarm_common_commands.md](rebotarm_common_commands.md) |
| RViz MoveIt、网页工作台、真机 Plan/Execute | [rebotarm_feature_commands.md](rebotarm_feature_commands.md) |
| Ubuntu Gemini 2/YOLO/GraspNet 只读链路 | [ubuntu_vision_readonly_test_zh.md](ubuntu_vision_readonly_test_zh.md) |
| 视觉候选、plan-only、仿真检查和真机边界 | [visual_grasp_commands.md](visual_grasp_commands.md) |
| 网页手眼/TCP 标定 | [calibration_web.md](calibration_web.md) |
| MuJoCo 示教轨迹预演 | [mujoco_teach_preview.md](mujoco_teach_preview.md) |
| Gymnasium Reach/RL | [mujoco_rl.md](mujoco_rl.md) |
| 控制器、仿真、构建和通用状态检查 | 本文 |

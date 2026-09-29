# MoveIt 与 RViz 命令

> 状态：REFERENCE；类型：规划与执行命令；适用范围：RViz MotionPlanning、真机 Plan/Execute 和仿真 Plan/Execute。

## 前置条件

旧的 `rviz_ee_drag_real.launch.py` 已移除；当前真机入口是 `moveit_hardware.launch.py`，仿真入口是 `rviz_ee_drag_sim.launch.py`。

```bash
cd /home/huangbin/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

确认已安装 `ros-jazzy-moveit-simple-controller-manager`。Plan 成功而 Execute 失败时，先检查控制器插件和唯一 Action server。

## 仿真 RViz 拖动

```bash
ros2 launch rebotarm_bringup rviz_ee_drag_sim.launch.py
```

在 RViz MotionPlanning 中选择规划组，设置目标姿态，点击 `Plan`，确认轨迹后点击 `Execute`。

检查仿真 Action：

```bash
ros2 action list | grep follow_joint_trajectory
```

## 真机 RViz 拖动

只有在本次操作获得授权、串口无人占用并完成现场安全检查后运行：

```bash
ros2 launch rebotarm_bringup moveit_hardware.launch.py \
  channel:=/dev/ttyACM0 \
  use_rviz:=true
```

操作顺序：

1. 确认 `CONNECTED_DISABLED` 和新鲜 `joint_states`；
2. 在 MotionPlanning 中点击 `Plan`；
3. 检查起点、轨迹和现场净空；
4. 显式调用 `/rebotarm/enable`；
5. 确认当前位置保持；
6. 点击 `Execute`；
7. 结束时停止轨迹、确认静止、受控回位、调用 `/rebotarm/disable`。

使能命令：

```bash
ros2 service call /rebotarm/enable std_srvs/srv/Trigger "{}"
```

停止命令：

```bash
ros2 service call /rebotarm/trajectory_stop std_srvs/srv/Trigger "{}"
```

## 运行检查

```bash
ros2 topic echo --once /rebotarm/arm_status
ros2 topic echo --once /rebotarm/joint_states
ros2 action list | grep follow_joint_trajectory
ros2 service list | grep -E 'plan_kinematic_path|check_state_validity'
```

## 相关文档

- 功能说明：[MoveIt 规划与轨迹执行](../../implemented/features/moveit_planning.md)；
- 系统数据流：[系统节点与数据流](../topology/system_dataflow.md)；
- 网页/键盘/示教：[网页、键盘与示教命令](rebotarm_common_commands.md)。

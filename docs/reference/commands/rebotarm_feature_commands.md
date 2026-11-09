# MoveIt 与 RViz 命令

> 状态：REFERENCE；类型：规划与执行命令；适用范围：RViz MotionPlanning、真机 Plan/Execute 和仿真 Plan/Execute。

## 前置条件

旧的 `rviz_ee_drag_real.launch.py` 已移除；当前真机入口是 `moveit_hardware.launch.py`，仿真入口是 `rviz_ee_drag_sim.launch.py`。

```bash
cd /home/a/project/rebot_Arm
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

确认已安装 `ros-jazzy-moveit-simple-controller-manager`。Plan 成功而 Execute 失败时，先检查控制器插件和唯一 Action server。
使用 continuous 还需要 `ros-jazzy-moveit-py`；准备服务不就绪时明确失败，不回退到旧执行方式。

## 仿真 RViz 拖动

```bash
ros2 launch rebotarm_bringup rviz_ee_drag_sim.launch.py
```

在 RViz MotionPlanning 中选择规划组，设置目标姿态，点击 `Plan`，确认轨迹后点击 `Execute`。

## RViz MoveIt 末端拖动

当前推荐使用 `rebotarm_bringup` 的 MoveIt 组合入口；历史上的
`rviz_ee_drag_real.launch.py` 已移除，真机使用 `moveit_hardware.launch.py`。
网页关节 Preview / Execute / Stop 与 RViz 拖动属于独立操作入口。

## 网页遥操作

网页入口使用 `rebotarm_app.launch.py`，真机可显式设置 `channel:=auto` 或指定串口。

```bash
ros2 launch rebotarm_bringup rebotarm_app.launch.py \
  use_hardware:=true channel:=/dev/ttyACM0 web_execute_enabled:=true
```

检查仿真 Action：

```bash
ros2 action list | grep follow_joint_trajectory
```

## 真机 RViz 拖动

只有在本次操作获得授权、串口无人占用并完成现场安全检查后运行：

```bash
ros2 launch rebotarm_bringup moveit_hardware.launch.py \
  channel:=/dev/ttyACM0 \
  use_rviz:=true trajectory_execution_mode:=continuous
```

这是第三批首次交付的新模式显式选择命令；省略模式参数仍使用 legacy。
与网页工作台/键盘组合互斥，不应在同一命名空间再启动另一控制器。
网页工作台实测命令见 [网页命令](rebotarm_common_commands.md#网页工作台)。

操作顺序：

1. 确认 `CONNECTED_DISABLED` 和新鲜 `joint_states`；
2. 在 MotionPlanning 中点击 `Plan`；
3. 检查起点、轨迹和现场净空；
4. 显式调用 `/rebotarm/enable`；
5. 确认当前位置保持；
6. 点击 `Execute`；
7. 结束时停止轨迹、确认静止、受控回位、调用 `/rebotarm/disable`。

先从小幅运动开始，再逐步覆盖多轴，观察起停、到达与终点保持，以及原有抖动是否改善。
MoveIt 取消后应停止并保持；发生异常停止本次测试，保存界面提示和已有日志。
若准备报告 jerk、速度、加速度超限或碰撞，表示该轨迹被拒绝，不能把 Plan 成功视为 Execute 成功；
不自动改时间或放宽限制，MoveIt 执行预算仍为 `1.2T + 0.5 s`。
用户确认功能与运动表现正常后再切换日常默认，实机定量跟踪精度仍未验收。

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
# 新模式只读检查：后端能力、准备服务、场景版本应存在且模式一致。
ros2 topic echo --once /rebotarm/trajectory_execution_mode std_msgs/msg/String \
  --qos-durability transient_local
ros2 topic echo --once /rebotarm/trajectory_capabilities std_msgs/msg/String \
  --qos-durability transient_local
ros2 service type /rebotarm/prepare_arm_trajectory
ros2 service type /rebotarm/retime_arm_trajectory
ros2 topic echo --once /rebotarm/trajectory_scene_revision std_msgs/msg/String \
  --qos-durability transient_local
ros2 param get /reBotArmController trajectory_execution_mode
```

新模式能力应为 `continuous`，服务类型应为 `rebotarm_msgs/srv/PrepareArmTrajectory`。
网页/键盘组合中的 `trajectory_execution_mode` 也必须相同，准备期间保持当前位置，
普通夹爪运动会因资源占用拒绝；场景变化停止保持，本批不承担动态避障。

新模式分别选择 `arm` 和 `gripper` 两个独立规划组完成 Plan/Execute，不提供 `arm_with_gripper`。
`arm` 下发六轴 FJT；`gripper` 通过独立 GripperCommand 执行，适配器将左指位置转换为双指总开口。
夹爪完成后再规划机械臂，起始状态使用新鲜实际开度。六轴最终计划在规划阶段绑定保持起点并
统一重定时，Execute 不再改节点或时间；旧绑定失效须重新规划。生效夹爪速度、力矩上限查看
上述能力快照。完整契约见 [P005 接口](../../implemented/features/p005_prepared_trajectory.md#修订协同接口)。

## 相关文档

- 功能说明：[MoveIt 规划与轨迹执行](../../implemented/features/moveit_planning.md)；
- 系统数据流：[系统节点与数据流](../topology/system_dataflow.md)；
- 网页/键盘/示教：[网页、键盘与示教命令](rebotarm_common_commands.md)。

## 本地 MuJoCo 扩展

安装与完整说明见 [MuJoCo](../../../src/rebotarm_simulation/README_mujoco.md)。

```bash
ros2 launch rebotarm_bringup mujoco_moveit_sim.launch.py use_rviz:=true use_mujoco_viewer:=true
ros2 launch rebotarm_simulation mujoco_sim.launch.py
rebotarm_mujoco_cli run --duration 5
```

# 网页、键盘与示教命令

> reBotArm 遥操作使用文档

> 状态：REFERENCE；类型：操作命令；适用范围：Dashboard、键盘遥操作、示教录制和回放；真机动作需要本次操作授权。

## 适用范围

本文只处理操作员工作流。RViz MoveIt 使用 [MoveIt 命令](rebotarm_feature_commands.md)，视觉使用 [视觉命令](visual_grasp_commands.md)。

## 前置条件

```bash
cd /home/huangbin/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

真机运行前确认串口唯一占用、反馈新鲜、机械臂有支撑、急停可用，并保持控制器默认失能。

## 网页工作台

网页遥操作通过统一的 `rebotarm_app.launch.py` 入口提供。

仿真或只读模式：

```bash
ros2 launch rebotarm_bringup rebotarm_app.launch.py \
  use_hardware:=false \
  web_execute_enabled:=false
```

已授权的真机工作台：

```bash
ros2 launch rebotarm_bringup rebotarm_app.launch.py \
  use_hardware:=true \
  web_execute_enabled:=true \
  channel:=/dev/ttyACM0
```

未指定设备时也可以使用自动串口选择：`channel:=auto`。该模式只会在候选串口唯一且可用时继续。

打开 `http://127.0.0.1:8088/`。网页功能含 Preview、Execute、Stop、Safe Home、夹爪、示教录制和回放。

## 键盘遥操作

键盘入口不能与另一套占用同一串口的控制器并行运行：

```bash
ros2 launch rebotarm_bringup teleop_keyboard.launch.py \
  use_hardware:=false \
  use_local_rviz:=true
```

需要真机时，在完成现场检查和授权后将 `use_hardware:=true channel:=/dev/ttyACM0` 传入。键盘操作用于小范围调姿，不作为示教数据来源。

需要键盘、Dashboard 和示教录制但不需要 MoveIt 时，使用组合入口：

```bash
ros2 launch rebotarm_bringup teleop_system.launch.py \
  use_hardware:=false \
  panel:=true \
  web_execute_enabled:=false
```

该入口与 `rebotarm_app.launch.py` 互斥；真机使用前仍需唯一串口、现场检查和显式授权。

## 示教录制

这是重力补偿手拖示教录制流程。

1. 启动网页工作台；
2. 打开 `Teach Trajectory`；
3. 输入文件名；
4. 点击 `Start Teach`；
5. 完成手拖示教；
6. 点击 `Stop Teach`；
7. 刷新文件并执行 `Check Trajectory`。

录制文件保存为 `teleop_records/<文件名>.jsonl`。原始记录不能直接执行。

## 示教回放

1. 选择 JSONL 文件；
2. 点击 `Check Trajectory`；
3. 确认 MoveIt 和碰撞检查可用；
4. 先使用 dry-run；
5. 获得本次真机授权后再点击 `Replay`；
6. 异常时点击 `Stop Replay`。

回放使用准备后的轨迹，并执行起点对齐、速度/加速度/jerk 检查、碰撞预检和运行时跟踪门。

## 停止与安全

需要回安全位时使用 `safe_home`，确认静止后再失能。

真机结束顺序：

```text
停止当前轨迹 -> 确认机械臂静止 -> 受控回到已确认位置
-> 调用 /rebotarm/disable -> 确认失能 -> Ctrl+C
```

健康但可恢复的失败不得直接把机械臂失能在未知姿态；控制器故障、通信丢失、反馈过期或明确急停遵循保护策略。

## 相关文档

- 功能说明：[Dashboard、键盘与 RViz 操作](../../implemented/features/operator_control.md)；
- 示教说明：[示教录制、检查与回放](../../implemented/features/teach_replay.md)；
- 命令总索引：[commands/README.md](README.md)。

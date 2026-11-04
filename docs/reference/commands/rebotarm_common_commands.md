# 网页、键盘与示教命令

> reBotArm 遥操作使用文档

> 状态：REFERENCE；类型：操作命令；适用范围：Dashboard、键盘遥操作、示教录制和回放；真机动作需要本次操作授权。

## 适用范围

本文只处理操作员工作流。RViz MoveIt 使用 [MoveIt 命令](rebotarm_feature_commands.md)，视觉使用 [视觉命令](visual_grasp_commands.md)。

## 前置条件

```bash
cd /home/a/project/rebot_Arm
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

真机运行前确认串口唯一占用、反馈新鲜、机械臂有支撑、急停可用，并保持控制器默认失能。

## 网页工作台

网页遥操作通过统一的 `rebotarm_app.launch.py` 入口提供。

该入口专用于真机。无硬件网页联调使用轻量预演组合（legacy），不会连接电机：

```bash
ros2 launch rebotarm_bringup teleop_system.launch.py \
  use_hardware:=false \
  keyboard_prefix:="" \
  web_execute_enabled:=false
```

已授权的真机工作台：

```bash
fuser -v /dev/ttyUSB0 /dev/ttyACM0
ros2 launch rebotarm_bringup rebotarm_app.launch.py \
  use_hardware:=true \
  web_execute_enabled:=true \
  channel:=/dev/ttyACM0
```

未指定设备时也可以使用自动串口选择：`channel:=auto`。该模式只会在候选串口唯一且可用时继续。

打开 `http://127.0.0.1:8088/`。网页功能含 Preview、Execute、Stop、Safe Home、夹爪、示教录制和回放。

P005 新模式实测使用同一工作台，显式增加模式参数：

```bash
ros2 launch rebotarm_bringup rebotarm_app.launch.py \
  use_hardware:=true channel:=/dev/ttyACM0 \
  trajectory_execution_mode:=prepared_v1
```

首次交付的真机默认仍为 `legacy`，启动后仍默认失能。新模式同时传给网页与控制器，
组合自动提供唯一 MoveIt 场景和准备服务。网页的 `web_keyboard` 配置由
`http://127.0.0.1:8088/api/config` 回显；默认步长为 0.02 rad、时长为 0.4 s。
legacy 仍为 0.2 s；自定义 YAML 的显式时长继续生效，过短或超限会拒绝，不自动延时。

可只读核对当前参数（键盘节点仅在键盘组合中存在）：

```bash
ros2 param get /reBotArmController trajectory_execution_mode
ros2 param get /teleop_status_panel_node trajectory_execution_mode
ros2 param get /teleop_status_panel_node web_keyboard_default_duration
# 终端键盘组合：
ros2 param get /teleop_keyboard_node trajectory_duration
```

## 键盘遥操作

键盘入口不能与另一套占用同一串口的控制器并行运行：

```bash
ros2 launch rebotarm_bringup teleop_keyboard.launch.py \
  use_hardware:=false \
  use_local_rviz:=true
```

需要真机时，在完成现场检查和授权后将 `use_hardware:=true channel:=/dev/ttyACM0` 传入。键盘操作用于小范围调姿，不作为示教数据来源。

需要键盘、Dashboard 和示教录制时，使用组合入口：

```bash
ros2 launch rebotarm_bringup teleop_system.launch.py \
  use_hardware:=false \
  panel:=true \
  web_execute_enabled:=false
```

该入口与 `rebotarm_app.launch.py` 互斥；真机使用前仍需唯一串口、现场检查和显式授权。

真机新模式键盘使用以下命令（二选一，不能与工作台同时启动）：

```bash
ros2 launch rebotarm_bringup teleop_keyboard.launch.py \
  use_hardware:=true channel:=/dev/ttyACM0 trajectory_execution_mode:=prepared_v1
# 若需要同时使用网页和示教，将上行入口改成 teleop_system.launch.py。
```

prepared_v1 的真机键盘分支复用已有 MoveIt 组合，提供碰撞场景和准备服务，
不重复启动控制器、状态发布器或 RViz。轻量预演仍只支持 legacy；
需要新模式物理仿真时使用 [MuJoCo 组合](../../implemented/features/p005_prepared_trajectory.md#模式与启动)。

## P005 用户实机功能验证

prepared_v1 的“联合预览”显示六轴重定时后的时长、独立夹爪目标和生效力矩上限。
机械臂保留关键点；夹爪仅单次开合，不再支持延迟或多关键点。组合按钮通过一个
ExecuteTeleopMotion 同周期登记两侧，分别限速和到位，整体结果等两侧完成，不保证同时到达。
网页关节及键盘也使用该 Action；独立夹爪按钮保留，活动组合期间忙时拒绝。
页面“按闭合夹爪模型检查”仅代表本次计算副本，实际张开手指的碰撞范围未检查；
实际开度和公共 MoveIt 场景仍用真实反馈。修改目标后须重新预览；重新进入预览从当前反馈
初始化。界面的“再次执行条件”来自只读诊断，过期为未知，不代表某次 REJECT 的精确原因。
关键点格式和时序约束见 [P005 协同接口](../../implemented/features/p005_prepared_trajectory.md#修订协同接口)。

先完成现场净空、支撑、急停、唯一串口和新鲜反馈检查，记录当次可安全恢复的基线。
确认控制器、网页模式均为 prepared_v1，且准备服务就绪后，再显式 Enable。
本流程由操作者执行，软件交付不包含代理连接或驱动真机。

1. 网页关节点对点从小幅单轴开始：Preview 后 Execute，观察平滑起停、到达目标及结束后保持。
2. 网页键盘逐步覆盖六轴正反向：默认 0.02 rad / 0.4 s，每次独立起停；忙时第二次输入应明确拒绝，不排队累积。
3. 按 [MoveIt 操作](rebotarm_feature_commands.md#真机-rviz-拖动) 先 Plan、审阅，再实际 Execute，逐步覆盖多轴并观察抖动是否改善。
4. 用网页 Stop 或 MoveIt 取消检查停止后保持、结果提示和后续操作。正常结束应保持最终命令目标。
5. 按下文安全流程受控回到已记录基线，确认静止后失能。

能力未知/不匹配、准备服务不可用或超时、超限、碰撞会明确拒绝，不能通过自动回退继续运动。
busy 表示资源正在使用，应等上一操作完成后再次主动操作。异常先停止测试，保留界面提示和已有日志；
健康设备保持使能，不能为退出测试而在基线外失能。
`trajectory mode mismatch` 表示配置与后端能力不一致（`observed None` 为尚未收到能力），
应检查启动参数和唯一后端；准备失败提示中的 `jerk`、`velocity`、`acceleration` 或 `collision`
分别指向限值或碰撞检查。FJT 接纳不表示到位，应核对最终成功/失败提示。
无需固定试验次数、自动 legacy 对照或故障注入。向维护者反馈网页、键盘、MoveIt 和停止保持是否正常，
确认后才切换日常默认；人工通过不代表跟踪误差、速度连续性等实机定量指标已验证。

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

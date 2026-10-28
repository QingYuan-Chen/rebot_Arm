# reBotArm ROS2 SDK

<p align="center">
  <strong>ROS2 Jazzy · 机械臂控制 · 夹爪控制 · JointTrajectory · 自定义单电机调试接口</strong>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/ROS2-Jazzy-blue.svg" alt="ROS2 Jazzy">
  <img src="https://img.shields.io/badge/Python-3.12-blue.svg" alt="Python 3.12">
  <img src="https://img.shields.io/badge/Platform-Ubuntu%2024.04+-orange.svg" alt="Ubuntu 24.04+">
  <img src="https://img.shields.io/badge/Controller-reBotArmController-green.svg" alt="reBotArmController">
</p>

---

## 项目介绍

当前源码版本以 Git 提交为准。项目总览见 [README.md](../../README.md)，
最新操作入口见 [功能命令参考](commands/rebotarm_feature_commands.md)。
启动文件的分层、包含关系和功能边界见
[启动结构与功能](../../src/rebotarm_bringup/launch/README.md)。

`rebotarm_ros2` 是 reBotArm B601 机械臂的 ROS2 工作空间，通过本地设备驱动和运动层
提供 topic、service 和 action，作为二次开发、规划、可视化和单电机调试入口。
厂商 Python SDK 仅保留为参考，生产路径不再导入；不建立第二份通用 SDK。

当前工作空间包含12个 ROS2 包：

| 包 | 作用 |
|---|---|
| `rebotarm_msgs` | 自定义 msg / srv / action 接口 |
| `rebotarmcontroller` | 控制节点包，提供 `reBotArmController` 节点 |
| `rebotarm_bringup` | launch 组合、配置与后端选择 |
| `rebotarm_motion` | 轨迹、MoveIt 适配与运动校验 |
| `rebotarm_teach` | 示教录制、文件与回放流程 |
| `rebotarm_teleop` | 键盘、网页及 RViz 操作者输入适配 |
| `rebotarm_dashboard` | Web UI、HTTP 与状态展示 |
| `rebotarm_moveit_config` | URDF、网格、SRDF 与规划模型 |
| `rebotarm_vision` | 相机、识别与抓取候选 |
| `rebotarm_simulation` | MuJoCo 模型与仿真执行 |
| `rebotarm_preview` | 轻量 FollowJointTrajectory 与 RViz 状态预演 |
| `rebotarm_calibration` | 标定与 TF/TCP 验证 |

---

## 当前路线与安全边界

视觉只维护 Ubuntu 原生 Gemini 2、YOLO、ROS RGB-D/CameraInfo/detections、本机
GraspNet、MoveIt 和 MuJoCo/真机后端。Windows、HTTP、MJPEG、远端 JSON 和独立
GraspNet service 已删除。Dashboard HTTP 仅用于本机网页界面。
语音控制包 `rebotarm_voice_control` 已从当前工作空间移除，不属于现行启动链。

真机启动后默认保持失能，确认反馈和现场安全后显式执行：

```bash
ros2 service call /rebotarm/enable std_srvs/srv/Trigger "{}"
```

每次重启控制器都需要重新 Enable；连接不等于使能。MoveIt Execute 需要：

```bash
sudo apt install ros-jazzy-moveit-simple-controller-manager
```

## 核心功能

- 发布机械臂状态：`/rebotarm/joint_states`、`/rebotarm/arm_status`
- 提供基础服务：`enable`、`disable`、`set_mode`、`set_zero`、`safe_home`
- 支持笛卡尔目标：`MoveToPoseIK` service、`MoveToPose` action
- 支持标准轨迹接口：`control_msgs/action/FollowJointTrajectory`
- 支持夹爪控制：`SetGripper` service、`GripperCommand` action
- 支持 controller 内部重力补偿：`gravity_compensation/start`、`gravity_compensation/stop`
- 支持 per-joint sparse raw command：`JointMotorCmd`
- 本地配置与 MotorBridge 驱动；规范模型 FK/IK 和笛卡尔采样由 Pinocchio 与本地运动实现承接

---

## 环境要求

| 项 | 要求 |
|---|---|
| 操作系统 | Ubuntu 24.04+ |
| ROS2 | Jazzy，安装路径 `/opt/ros/jazzy` |
| Python | 系统 Python 3.12 |
| Python 环境 | 不使用 conda |
| 底层依赖 | 带序号补丁的 MotorBridge；Pinocchio 用于规范模型计算 |
| 默认串口 | `/dev/ttyACM0`，可通过 launch 参数覆盖 |

每次构建或运行前先 source ROS2：

```bash
source /opt/ros/jazzy/setup.bash
source scripts/source_local_environment.bash
```

---

## 配置开发环境

当前仓库的本机安装、硬件配置和视觉地址覆盖方式见
[`docs/setup/ubuntu_ros2_jazzy.md`](../setup/ubuntu_ros2_jazzy.md)。

### Step 1. 安装 ROS2 依赖

请参考[ROS 2 Jazzy 安装文档](https://docs.ros.org/en/jazzy/Installation.html)。
MoveIt Execute 还需要 `ros-jazzy-moveit-simple-controller-manager`。

### Step 2. 安装 motorbridge

先安装固定的 PyPI bootstrap 包：

```bash
python3 scripts/install_python_dependencies.py runtime --user --break-system-packages
```

原始 `motorbridge==0.4.7` 不包含控制器所需的逐电机反馈 sequence，因此单独安装
PyPI 包不够。必须从仓库根目录构建并显式安装已审查的
`0.4.7+rebotarm.1` source patch，再通过启动前检查：

```bash
python3 scripts/setup_motorbridge_fresh_feedback.py --build-only
python3 scripts/setup_motorbridge_fresh_feedback.py --install-user
python3 scripts/setup_motorbridge_fresh_feedback.py --check-installed
```

`--build-only` 只在临时 venv 验证 wheel，不修改用户 Python；
`--check-installed` 不联网、不构建且不访问硬件。详细构建、检查和回退说明见
[`docs/setup/ubuntu_ros2_jazzy.md`](../setup/ubuntu_ros2_jazzy.md)。

### Step 3. 保留参考 SDK（可选）


```bash
# 在当前仓库根目录执行
mkdir -p third_party
vcs import third_party < third_party/rebotarm_dependencies.repos
```

### Step 4. 确认运行依赖

生产无需安装厂商 Python SDK 或设置其 `PYTHONPATH`。构建前检查：

```bash
source /opt/ros/jazzy/setup.bash
python3 -c "import rclpy, motorbridge; print('core imports OK')"
python3 -c "import pinocchio; print('pinocchio', pinocchio.__version__)"
```

## 构建工作空间

```bash
# 在未激活 venv/conda 的新终端，进入当前仓库根目录
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 -m colcon build --base-paths src --executor sequential --symlink-install
source install/setup.bash
```

构建不要求视觉模型或GPU，现有模型可选打包；启用检测时必须提供实际模型路径。
视觉、GraspNet和MuJoCo使用各自的运行解释器，不通过切换构建解释器选择环境。
系统依赖、安装顺序及运行参数以 [本机安装说明](../setup/ubuntu_ros2_jazzy.md) 为准。

验证包和入口：

```bash
ros2 pkg executables rebotarmcontroller
```

期望输出：

```text
rebotarmcontroller reBotArmController
rebotarmcontroller GravityCompensation
rebotarmcontroller GripperControl
rebotarmcontroller MoveTo
rebotarmcontroller MoveToPose
```

---

## 目录结构

```text
rebot_Arm/
├── README.md
├── docs/                  # 架构、环境与操作手册
├── scripts/               # 环境、启动、检查与模型
├── patches/               # MotorBridge 补丁
├── src/                   # 12个活动ROS包
├── tests/
├── third_party/           # 本机参考、运行依赖与固定SDK清单
└── star_arm_102_rebot_b601_follow/  # 独立跟随工具
```

---

## 快速启动

### 启动基础驱动

启动控制节点、`robot_state_publisher`，可选RViz；不包含网页或视觉。
该入口会连接真机，使用前确认本轮授权和串口归属。
完整网页遥操作入口为`rebotarm_app.launch.py`，见
[功能命令参考](commands/rebotarm_feature_commands.md)。

```bash
ros2 launch rebotarm_bringup bringup.launch.py
```

如果机械臂不在默认串口 `/dev/ttyACM0`：

```bash
ros2 launch rebotarm_bringup bringup.launch.py channel:=/dev/ttyACM1
```

启用 RViz：

```bash
ros2 launch rebotarm_bringup bringup.launch.py use_rviz:=true
```

### 只启动控制节点

```bash
ros2 launch rebotarm_bringup hardware_controller.launch.py
```

### 直接运行控制节点

```bash
ros2 run rebotarmcontroller reBotArmController
```

---

## 直接移动到 Pose

不运行 demo 时，可以直接调用 ROS service 和 action 完成一次末端位姿移动。
先在一个终端启动控制节点：

```bash
# 在当前仓库根目录执行
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch rebotarm_bringup bringup.launch.py channel:=/dev/ttyACM0
```

然后在另一个终端执行控制命令：

```bash
# 在当前仓库根目录执行
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

1. 使能机械臂：

```bash
ros2 service call /rebotarm/enable std_srvs/srv/Trigger "{}"
```

2. 移动末端到目标 pose：

```bash
ros2 action send_goal /rebotarm/move_to_pose rebotarm_msgs/action/MoveToPose \
  "{target_pose: {position: {x: 0.30, y: 0.0, z: 0.30}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, duration: 2.0}" \
  --feedback
```

`move_to_pose` 由 motion 的 `PoseCompatNode` 提供，需要已显式使能、新鲜关节/夹爪反馈
及可用的 MoveIt 状态有效性服务；重定时与碰撞检查通过后交唯一 FJT 执行。

3. 回到安全零位：

```bash
ros2 service call /rebotarm/safe_home std_srvs/srv/Trigger "{}"
```

4. 失能并退出：

```bash
ros2 service call /rebotarm/disable std_srvs/srv/Trigger "{}"
```

---

## 示例脚本

所有示例都假设已经启动 `reBotArmController`：

```bash
# 在当前仓库根目录执行
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch rebotarm_bringup bringup.launch.py channel:=/dev/ttyACM0
```

示例已注册为 ROS2 可执行入口，可以直接通过 `ros2 run` 调用。

源文件位于：

```text
src/rebotarmcontroller/rebotarmcontroller/examples/move_to.py
src/rebotarmcontroller/rebotarmcontroller/examples/move_to_pose.py
src/rebotarmcontroller/rebotarmcontroller/examples/gravity_compensation.py
src/rebotarmcontroller/rebotarmcontroller/examples/gripper_control.py
```

### move_to.py

关节空间绝对角移动示例。脚本会等待 `/rebotarm/joint_states`，然后通过
`/rebotarm/follow_joint_trajectory` 发送关节轨迹；控制节点会在 action 内部进入
`pos_vel`，并使用限速和最终到位检查。

这是单次动作 demo，不会在结束后自动 `safe_home` 或 `disable`。

一次性控制 6 个电机，参数为 6 个绝对关节角，单位 rad：

```bash
ros2 run rebotarmcontroller MoveTo -- \
  0.20 -0.20 -0.20 -0.20 0.10 -0.10 \
  --duration 8.0
```

一次性控制 1 个电机，参数为目标关节名和绝对关节角，单位 rad：

```bash
ros2 run rebotarmcontroller MoveTo -- --joint joint3 --position -0.20 --duration 5.0
```

这个入口只作为 joint trajectory 链路验证，不建议用它做大范围规划。轨迹仍需满足
`follow_joint_trajectory` 的首点、速度和最终到位校验。

### move_to_pose.py

末端位姿移动示例。脚本会通过 `/rebotarm/move_to_pose` action 发送
`geometry_msgs/Pose` 目标；运动层负责采样、IK、重定时和碰撞校验，控制器负责 FJT 执行。

这是单次动作 demo，不会在结束后自动 `safe_home` 或 `disable`。

```bash
ros2 run rebotarmcontroller MoveToPose -- --x 0.30 --y 0.0 --z 0.30 --qw 1.0 --duration 2.0
```

Action 等待 FJT 到位；进度到 1.0 或完成规划本身不表示执行成功。

### gravity_compensation.py

重力补偿锁止示例。脚本本身不再在外部节点重写控制循环，而是直接调用
`reBotArmController` 内部的重力补偿服务；真正的重力补偿闭环在 controller
进程内部由唯一运行时采集序号验证反馈、本地重力算法计算有界输出并交 MotorBridge。
进入 MIT 前后均校验反馈，计算或任务失败按已验证保持策略处理。

```bash
ros2 run rebotarmcontroller GravityCompensation
```

脚本启动时会先调用 `/rebotarm/enable`，再启动重力补偿。按 `Ctrl+C` 退出时，
脚本会依次调用 `/rebotarm/safe_home` 和 `/rebotarm/disable`，让机械臂回到安全零位后失能。

对应底层服务：

```bash
ros2 service call /rebotarm/enable std_srvs/srv/Trigger "{}"
ros2 service call /rebotarm/gravity_compensation/start std_srvs/srv/Trigger "{}"
ros2 service call /rebotarm/gravity_compensation/stop std_srvs/srv/Trigger "{}"
ros2 service call /rebotarm/safe_home std_srvs/srv/Trigger "{}"
ros2 service call /rebotarm/disable std_srvs/srv/Trigger "{}"
```

该示例只是这些服务的薄客户端，不会绕过 ROS 节点直接连接硬件。硬件访问仍由
`reBotArmController` 统一持有。

### gripper_control.py

交互式夹爪开闭示例。脚本只调用 `/rebotarm/gripper/set` service，不直接访问硬件。
脚本启动时会先调用 `/rebotarm/enable`，退出时先闭合夹爪，再调用 `/rebotarm/disable`。

```bash
ros2 run rebotarmcontroller GripperControl
```

运行后输入：

```text
o / open    打开夹爪
c / close   闭合夹爪
q / quit    退出
```

`open` 对应夹爪开口 `0.09 m`，`close` 对应 `0.0 m`。

---

## ROS API 速查

默认命名空间为 `/rebotarm`。如果 launch 时设置了 `arm_namespace:=xxx`，下面所有
`/rebotarm/...` 都替换为 `/xxx/...`。

### 状态 Topic

| API | 类型 | 说明 | 简要使用 |
|---|---|---|---|
| `/rebotarm/joint_states` | `sensor_msgs/msg/JointState` | 6 轴关节位置、速度、力矩 | `ros2 topic echo /rebotarm/joint_states --once` |
| `/rebotarm/arm_status` | `rebotarm_msgs/msg/ArmStatus` | 控制模式、使能状态、状态机、关节状态码 | `ros2 topic echo /rebotarm/arm_status --once` |
| `/rebotarm/joints/<joint>/state` | `rebotarm_msgs/msg/JointMotorState` | 单关节电机状态，`<joint>` 为 `joint1` 到 `joint6` | `ros2 topic echo /rebotarm/joints/joint1/state --once` |
| `/rebotarm/gripper/state` | `rebotarm_msgs/msg/JointMotorState` | 夹爪状态，未配置夹爪时不发布 | `ros2 topic echo /rebotarm/gripper/state --once` |

### 服务 Service

| API | 类型 | 说明 | 简要使用 |
|---|---|---|---|
| `/rebotarm/enable` | `std_srvs/srv/Trigger` | 使能机械臂和夹爪 | `ros2 service call /rebotarm/enable std_srvs/srv/Trigger "{}"` |
| `/rebotarm/disable` | `std_srvs/srv/Trigger` | 停止控制循环并失能机械臂 | `ros2 service call /rebotarm/disable std_srvs/srv/Trigger "{}"` |
| `/rebotarm/safe_home` | `std_srvs/srv/Trigger` | 以安全速度回安全姿态 | `ros2 service call /rebotarm/safe_home std_srvs/srv/Trigger "{}"` |
| `/rebotarm/gravity_compensation/start` | `std_srvs/srv/Trigger` | 启动 controller 内部重力补偿闭环 | `ros2 service call /rebotarm/gravity_compensation/start std_srvs/srv/Trigger "{}"` |
| `/rebotarm/gravity_compensation/stop` | `std_srvs/srv/Trigger` | 停止 controller 内部重力补偿闭环 | `ros2 service call /rebotarm/gravity_compensation/stop std_srvs/srv/Trigger "{}"` |
| `/rebotarm/set_mode` | `rebotarm_msgs/srv/SetMode` | 切换 `mit`、`pos_vel`、`vel` | `ros2 service call /rebotarm/set_mode rebotarm_msgs/srv/SetMode "{mode: 'pos_vel'}"` |
| `/rebotarm/set_zero` | `rebotarm_msgs/srv/SetZero` | 设置全部或指定关节零点，空 `joint_name` 表示全部 | `ros2 service call /rebotarm/set_zero rebotarm_msgs/srv/SetZero "{joint_name: ''}"` |
| `/rebotarm/move_to_pose_ik` | `rebotarm_msgs/srv/MoveToPoseIK` | motion 求解、重定时、碰撞检查后提交 FJT | success 仅表示 FJT 接纳，会触发运动 |
| `/rebotarm/gripper/set` | `rebotarm_msgs/srv/SetGripper` | 设置夹爪开合距离和最大力矩 | `ros2 service call /rebotarm/gripper/set rebotarm_msgs/srv/SetGripper "{position: 0.05, max_effort: 0.5}"` |

### 动作 Action

| API | 类型 | 说明 | 简要使用 |
|---|---|---|---|
| `/rebotarm/move_to_pose` | `rebotarm_msgs/action/MoveToPose` | motion 生成并校验笛卡尔轨迹后转 FJT，等待到位 | 输入 `Pose + duration`，重定时可延长时长 |
| `/rebotarm/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | 标准关节轨迹接口，按输入轨迹点定时下发，并做首点/速度/到位校验 | 适合上层规划器或 `move_to.py` |
| `/rebotarm/gripper/command` | `control_msgs/action/GripperCommand` | 标准夹爪 action | 适合行为树、任务编排或 MoveIt 风格接口 |

示例：

```bash
ros2 action send_goal /rebotarm/move_to_pose rebotarm_msgs/action/MoveToPose \
  "{target_pose: {position: {x: 0.30, y: 0.0, z: 0.30}, orientation: {w: 1.0}}, duration: 2.0}" \
  --feedback
```

```bash
ros2 action send_goal /rebotarm/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: ['joint1','joint2','joint3','joint4','joint5','joint6'],
    points: [{positions: [0,0,0,0,0,0], time_from_start: {sec: 2}},
             {positions: [0.1,0,0,0,0,0], time_from_start: {sec: 5}}]}}"
```

`follow_joint_trajectory` 要求首个轨迹点接近当前关节角，最终误差默认需小于
`0.03 rad`。底层 `pos_vel` 速度限制使用 `arm.yaml` 中的电机配置。

### 低层 Command Topic

| API | 类型 | 说明 | 简要使用 |
|---|---|---|---|
| `/rebotarm/joints/<joint>/cmd` | `rebotarm_msgs/msg/JointMotorCmd` | 单关节 sparse raw command，可发 MIT、位置速度、速度模式 | 调试、电机直通、重力补偿 |
| `/rebotarm/gripper/cmd` | `rebotarm_msgs/msg/JointMotorCmd` | 夹爪 sparse raw command | 低层夹爪调试 |

`JointMotorCmd` 使用 sparse-flag 设计：只有 `use_pos/use_vel/use_kp/use_kd/use_tau/use_vlim`
为 `true` 的字段才覆盖默认值。`mode` 可选：

| mode | 常量 | 说明 |
|---|---|---|
| `0` | `MODE_MIT` | MIT 模式，使用 `pos/vel/kp/kd/tau` |
| `1` | `MODE_POS_VEL` | 位置速度模式，使用 `pos/vlim` |
| `2` | `MODE_VEL` | 速度模式，使用 `vel` |

单关节 MIT 示例：

```bash
ros2 topic pub --once /rebotarm/joints/joint1/cmd rebotarm_msgs/msg/JointMotorCmd \
  "{mode: 0, use_pos: true, use_kp: true, use_kd: true, pos: 0.0, kp: 80.0, kd: 4.0}"
```

---

## 预留 API 与上层集成入口

这些接口已经在节点中注册，建议作为后续上层应用、规划器或调试工具的稳定接入点。

| 入口 | 当前用途 | 预留方向 | 使用说明 |
|---|---|---|---|
| `/rebotarm/follow_joint_trajectory` | 关节轨迹 action | MoveIt2、任务规划器、离线轨迹回放 | 发送完整 `joint_names` 和 `points`，首点需接近当前关节角 |
| `/rebotarm/move_to_pose` | 末端位姿 action | 视觉抓取、点击到达、任务级 API | 输入 `Pose + duration`，返回 `final_pose` 和执行结果 |
| `/rebotarm/move_to_pose_ik` | IK 执行兼容服务 | 旧应用接入 | 会触发运动；返回 `q_solution`，success 仅表示接纳，不可作为只读预览 |
| `/rebotarm/joints/<joint>/cmd` | 单电机直通 topic | 调参、重力补偿、低层控制实验 | 轨迹运行时默认 `reject`，可用参数 `cmd_arbitration:=preempt` 改为抢占 |
| `/rebotarm/gripper/command` | 标准夹爪 action | 行为树、抓取 pipeline | 使用 `control_msgs/action/GripperCommand`，位置单位为米 |
| `/rebotarm/arm_status` | latched 状态 topic | UI、健康监控、状态机同步 | 关注 `state_machine`：`IDLE`、`TRAJ_RUNNING`、`LOWLEVEL_STREAMING`、`GRAVITY_COMP` |
| `arm_namespace` 参数 | 命名空间 | 多臂或仿真/真机并行 | launch 时传 `arm_namespace:=left_arm` |
| `frame_id`、`ee_frame_id` 参数 | 坐标帧标识 | 后续 TF、MoveIt2、视觉坐标对齐 | 默认 `base_link`、`end_link` |

---

## 配置说明

`rebotarm_bringup/config/` 提供默认配置：

| 文件 | 说明 |
|---|---|
| `arm.yaml` | 机械臂 6 个关节的电机、反馈 ID、控制参数 |
| `gripper.yaml` | 夹爪电机配置 |
| `operator_common.yaml` | 键盘、网页和示教共同使用的命名空间、关节名称与回退限位 |
| `keyboard_control.yaml` | 终端键盘步长、轮询、输入超时与 `deadman` 参数；网页按钮不读取它 |
| `web_teleop.yaml` | 网页执行、网页关节点动、网页夹爪及本机 HTTP/SSE 参数 |
| `teach_control.yaml` | 示教录制、回放、滤波/重定时、MoveIt 对齐、碰撞与运行期跟踪参数 |

配置按消费者分离：`teleop_keyboard.launch.py` 加载公共配置和键盘配置；网页面板加载
公共、网页和示教配置；示教录制/回放加载公共和示教配置。旧的混合
`teleop_control.yaml` 已移除，`deadman_key` 不影响网页端按钮操作。

常用 launch 参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `arm_config` | bringup 内置 `arm.yaml` | 机械臂配置路径 |
| `gripper_config` | bringup 内置 `gripper.yaml` | 夹爪配置路径 |
| `channel` | 空字符串 | 留空使用 YAML，非空时覆盖串口 |
| `joint_state_rate` | `100.0` | `/rebotarm/joint_states` 发布频率 |
| `cmd_arbitration` | `reject` | 轨迹运行中 per-joint cmd 仲裁，`reject` 或 `preempt` |
| `arm_namespace` | `rebotarm` | ROS 命名空间前缀 |
| `frame_id` | `base_link` | 机械臂基座坐标系，预留给 TF、视觉和规划集成 |
| `ee_frame_id` | `end_link` | 末端坐标系，预留给 TF、视觉和规划集成 |
| `use_rviz` | `false` | 是否启动 RViz |

---

## 排障

### 找不到串口

如果启动时报：

```text
open serial port /dev/ttyACM0 failed: No such file or directory
```

说明默认串口不存在。先查看实际设备：

```bash
ls /dev/ttyACM*
```

然后用 `channel:=...` 覆盖：

```bash
ros2 launch rebotarm_bringup bringup.launch.py channel:=/dev/ttyACM1
```

### 权限不足

如果串口存在但无权限：

```bash
sudo usermod -a -G dialout $USER
```

重新登录后生效。

### RViz 模型不显示

确认 URDF mesh 路径已经是：

```text
package://rebotarm_bringup/description/meshes/...
```

### FastDDS SHM 端口提示

如果终端出现类似：

```text
[RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port7002: open_and_lock_file failed
```

通常是之前的 ROS2 进程异常退出后，FastDDS shared memory 锁文件残留。服务和 action
能正常响应时，这个提示一般不影响控制。需要清理时，先停掉相关 ROS2 进程，再执行：

```bash
pkill -f ros2
pkill -f reBotArmController
rm -f /dev/shm/fastrtps_port*
```

如果希望临时绕开 shared memory transport，可在启动 ROS2 前设置：

```bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
```

---

## 已知状态

`reBotArmController` 启动时会直接连接真实硬件；如果默认 `/dev/ttyACM0` 不存在，
需要通过 `channel:=/dev/ttyACM*` 指定正确串口。当前示例入口均通过 ROS2
service/action/topic 访问 controller，不会绕过 controller 直接占用硬件。

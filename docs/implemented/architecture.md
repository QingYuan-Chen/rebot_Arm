# reBotArm ROS2 架构

> 状态：IMPLEMENTED；类型：当前架构与包边界；依据：当前源码、测试和 `AGENTS.md`。

## 当前部署范围

当前支持的视觉路线是 Ubuntu 24.04 / ROS 2 Jazzy：原生 Gemini 2、本地 YOLO、ROS RGB-D/CameraInfo/检测结果，以及本机进程内 GraspNet。Windows、HTTP、MJPEG、远程 JSON 和独立 GraspNet 服务路线已经废弃，不得恢复。

本仓库由分层 ROS2 包组成。新代码必须遵守以下职责边界。

## 职责边界

### 硬件职责（Hardware ownership）

`rebotarmcontroller` 负责真实硬件通信和最后一道安全防线。

负责：

- 电机 SDK / 串口通道访问
- 机械臂和夹爪状态发布
- `follow_joint_trajectory`
- `trajectory_stop`
- `safe_home`
- enable / disable
- 夹爪执行
- 拒绝不安全或格式错误的底层命令

不得负责：

- Web UI
- 示教文件管理
- 视觉抓取策略
- MoveIt 规划策略
- 面向用户的工作流状态

### 运动职责（Motion ownership）

`rebotarm_motion` 负责运动生成和运动校验。

负责：

- 点到点预览与执行节点
- MoveIt 规划客户端适配器
- 位姿预览 / IK 预览辅助工具
- 轨迹重定时
- 速度 / 加速度 / jerk 检查
- 回放运行时跟踪安全门
- 碰撞预检
- 示教回放起始状态对齐
- `JointTrajectory` 构造工具

可以调用 MoveIt 服务和控制器 Action，但不得直接访问电机 SDK。

### 示教职责

`rebotarm_teach` 负责示教工作流。

负责：

- 重力补偿示教录制
- 示教记录文件格式和文件列表
- 准备轨迹生成
- 示教回放工作流入口
- 示教回放 dry-run / execute 安全门
- 示教回放设置和状态 payload

`TeachReplayWorkflow` 是唯一的回放实现，负责整个回放生命周期，包括准备、dry-run token、对齐、碰撞检查、Action 回调和跟踪状态。它接收明确的快照/状态回调和 ROS 适配器；既不导入 Dashboard 模块，也不控制 HTTP 命令授权。

已退役的 `TeachReplayNode`、`teach_replay.launch.py` 和独立录制 launch 不再作为并行命令行路径维护。录制和回放都从 Dashboard 组合入口发起，以保证始终使用同一套安全门和授权状态。

`TeachRecorderNode` 是唯一的录制服务和文件所有者。控制器发布带稳定接收时间戳的原子验证批次；重复发布同一批次不会生成新的录制样本。录制会检查源数据年龄以及每个电机匹配的批次时间戳。配置的录制频率是上限，不代表反馈一定以该频率到达。

它可以使用 `rebotarm_motion` 进行重定时、对齐、碰撞检查和轨迹验证，但不得实现 Dashboard HTML 或直接编写电机 SDK 逻辑。

### 操作交互职责（Operator interaction ownership）

`rebotarm_teleop` 负责操作员命令适配器。

负责：

- 键盘遥操作
- Web 遥操作命令校验 / 适配逻辑
- 夹爪遥操作适配逻辑
- RViz interactive marker 输入
- RViz 夹爪可视关节状态桥接
- 旧 interactive target 节点兼容

它可以发布目标命令或调用面向控制器的 ROS action/service，但不得负责示教回放质量策略或 Dashboard 渲染。

### Dashboard 职责

`rebotarm_dashboard` 负责 Web 应用边界。

负责：

- HTML / JS / CSS 资源
- HTTP 路由
- SSE 状态流
- Dashboard 状态聚合
- 调用遥操作 / 示教 / 控制器服务
- Dashboard 专用 URDF 和网格资源服务

它不得包含复杂运动规划、重定时、示教回放算法或硬件 SDK 代码。Dashboard 可以显示运动和示教结果，但算法必须位于 `rebotarm_motion` 和 `rebotarm_teach`。

### MoveIt 配置职责

`rebotarm_moveit_config` 仅拥有 MoveIt 模型和规划配置。

规范 URDF 是 `config/rebotarm.urdf`；其中的网格 URI 解析到本包的 `meshes/` 目录。Bringup、Dashboard 和仿真可以使用这些资源，但不能反向依赖 bringup 或兼容包。跨节点 launch 参数 profile 位于 `rebotarm_bringup/config`。仿真拥有自己冻结的固件参考和力矩标定值；修改硬件 profile 不得静默地重新调节仿真。

负责：

- MoveIt 使用的 URDF/SRDF
- 规划组
- 末端执行器配置
- 碰撞几何体
- 规划用关节限位
- RViz MotionPlanning 配置

不得包含可执行的业务逻辑。

### 视觉职责

`rebotarm_vision` 负责感知和抓取候选。

负责：

- 相机 / 深度输入
- 物体检测
- 深度融合
- 抓取候选生成
- 抓取位姿评分
- 视觉抓取执行器集成点

不得绕过运动校验或控制器安全机制。视觉抓取必须经过规划、碰撞检查和执行安全门。

Ready-pose 运动（`visual_ready_node` 及其参数 profile）位于 `rebotarm_motion`。视觉包不再提供旧的顶层 Python 兼容模块；console script 名称保持不变，但统一指向 `nodes/`、`benchmarks/`、`diagnostics/` 等 canonical 子包。bringup 直接启动运动职责包的所有者。

### 仿真职责

`rebotarm_simulation` 负责离线机器人物理和仿真控制器后端。

负责：

- MuJoCo 模型生成和验证
- 仿真 `FollowJointTrajectory` 执行
- 仿真关节和夹爪状态
- headless 物理检查和可选 Viewer 集成
- 离线 Gymnasium 任务、仿真奖励以及可选的 RL 训练/评估
- 轨迹指标、阶跃响应 benchmark 和仿真接触反馈
- 示教轨迹的 MuJoCo 预演执行与 Viewer 生命周期管理

示教预演从 `rebotarm_teach` 读取 JSONL 并复用轨迹准备逻辑，执行入口由 `rebotarm_simulation` 提供。MoveIt 演示及 MuJoCo 后端启动组合由 `rebotarm_bringup` 提供。

不得导入或调用真实电机 SDK。仿真 launch 不得启动 `rebotarmcontroller`、打开硬件通道，或以相同名称暴露第二个活动 `FollowJointTrajectory` server。

### Bringup 职责

`rebotarm_bringup` 负责 launch 时的组合和后端选择。

负责：

- launch 文件和跨包启动组合
- 选择且只选择一个真实或仿真执行后端
- 传递 `use_hardware`、`execution_mode` 和 `use_sim_time`
- 安全 launch 默认值和互斥节点条件

真实控制器通过唯一的 `rebotarm_bringup/launch/hardware_controller.launch.py` 片段组合。其他 bringup launch 文件可以转发公开硬件参数，但不得重复声明 `reBotArmController` 节点。参见 [launch 结构和功能](../../src/rebotarm_bringup/launch/README.md)。

不得在 launch 文件中实现电机控制、运动规划、感知或标定算法。

### 标定职责

`rebotarm_calibration` 是标定工具的指定所有者。

负责：

- 手眼标定
- TCP 标定
- TF 验证工具
- 相机内参 / 外参检查

标定 ROS 节点负责会话文件、同步采集和求解。Dashboard 负责 `/calibration`、HTTP/SSE 和 ROS client；不得读取标定文件或实现标定数学。明确的重力模式操作员请求使用现有控制器服务，不属于标定求解器。

视觉和运动层应通过配置或 TF 使用标定输出，不得把输出复制到 Dashboard 或控制器代码中。

## 依赖方向

允许的依赖方向：

```text
rebotarm_dashboard
  -> rebotarm_teleop
  -> rebotarm_teach
  -> rebotarm_motion
  -> MoveIt / ROS messages / controller actions

rebotarm_teach -> rebotarm_motion
rebotarm_teleop -> 使用旧 interactive preview 辅助工具时依赖 rebotarm_motion
rebotarm_vision -> 为验证和执行依赖 rebotarm_motion / MoveIt 接口
rebotarm_bringup -> 仅依赖各包的 launch 入口和配置
rebotarm_simulation -> rebotarm_teach（仅复用示教记录读取与轨迹准备）、ROS 消息和仿真执行库
```

已退役的 MuJoCo ROS adapter 不再随活动包发布；`rebotarm_simulation` 不得导入或声明对 `rebotarm_motion` 的依赖。Launch 解释器必须通过 launch 参数或环境变量按进程显式选择，不能探测工作区虚拟环境目录，也不能把视觉 site-packages 注入整个 launch group。参见 [launch Python 配置](../setup/launch_python_configuration.md)。

禁止的依赖方向：

```text
rebotarm_motion -> rebotarm_dashboard
rebotarm_motion -> rebotarm_teach
rebotarm_motion -> rebotarm_teleop
rebotarm_teach -> rebotarm_dashboard

rebotarm_teleop -> rebotarm_dashboard

rebotarm_dashboard -> motor SDK
rebotarm_vision -> motor SDK
rebotarm_simulation -> motor SDK
rebotarm_simulation -> rebotarmcontroller implementation
rebotarm_bringup -> 各包的实现内部
```

## 权限矩阵

| 包 | 可直接向硬件下命令 | 可调用控制器 ROS service/action | 可调用 MoveIt | 可拥有文件/UI | 可发布操作员目标 |
| --- | --- | --- | --- | --- | --- |
| `rebotarmcontroller` | 是 | 拥有这些接口 | 否 | 否 | 否 |
| `rebotarm_motion` | 否 | 是 | 是 | 否 | 否 |
| `rebotarm_teach` | 否 | 是，通过回放/录制工作流 | 是，通过运动辅助工具 | 仅示教记录 | 否 |
| `rebotarm_teleop` | 否 | 是 | 仅通过运动辅助工具 | 否 | 是 |
| `rebotarm_dashboard` | 否 | 是 | 无直接规划逻辑 | 仅 Dashboard 资源 | 通过 teleop 适配器 |
| `rebotarm_moveit_config` | 否 | 否 | 仅配置 | 仅模型/配置文件 | 否 |
| `rebotarm_vision` | 否 | 仅通过规划执行接口 | 是，用于验证/执行门控 | 仅感知资源/模型 | 否 |
| `rebotarm_simulation` | 仅仿真后端 | 拥有仿真等价接口 | 无直接规划策略 | 仅生成的仿真产物 | 否 |
| `rebotarm_bringup` | 否 | 无业务逻辑 | 无业务逻辑 | 仅 launch/配置 | 否 |
| `rebotarm_calibration` | 否 | 否，显式验证工具除外 | 否，验证工具除外 | 仅标定输出 | 否 |

如果某个包需要超出自身行定义的权限，应在职责所有者包中创建小型接口并调用该接口，不得跨层复制实现。

## 工作流边界

### 点到点执行

点到点执行是指从当前机器人状态移动到一个目标状态，由 `rebotarm_motion` 负责。

必须满足：

- 执行前校验目标状态；
- 生成的输出是有效的 `JointTrajectory`；
- 最终目标速度为零；
- 控制器停止路径始终可用；
- 硬件执行必须经过 `rebotarmcontroller`。

### 示教回放

示教回放是指安全地复现已记录的示教轨迹，由 `rebotarm_teach` 负责，并使用 `rebotarm_motion` 提供的运动服务。

必须满足：

- 原始记录只作为输入数据，不直接信任为执行命令；
- 回放使用准备后的轨迹；
- 重定时强制执行速度 / 加速度 / jerk 限制；
- 碰撞预检可以阻止回放；
- 运行时跟踪安全门可以停止回放；
- 最终保持使用零速度。

### Web 遥操作

Web 遥操作是指 Dashboard 发送操作员意图对应的关节或夹爪目标。Dashboard 负责 UI；`rebotarm_teleop` 负责命令适配；控制器负责硬件执行。

必须满足：

- Dashboard 不构造底层电机命令；
- stop / safe_home / enable / disable 调用面向控制器的服务；
- 回放状态可以锁定不安全的机械臂命令；
- Web preview 和 execute 是两个独立概念，除非得到明确确认。

### RViz MoveIt 末端拖动

RViz 拖动控制现在采用原生 MoveIt MotionPlanning 工作流。不再使用已退役的
自定义 `ee_target` 标记、`PreviewNode`、`ExecutionNode` 或
`/interactive_control/execute_preview` 服务。

当前分工：

- RViz MotionPlanning 创建目标位姿并请求 MoveIt 规划
- MoveIt `move_group` 计算轨迹
- `rebotarmcontroller` 执行生成的 `FollowJointTrajectory`

## 新代码归属

新增文件前请使用下表确定归属：

| New feature | Package |
| --- | --- |
| New hardware service, motor mode, safe stop behavior | `rebotarmcontroller` |
| New trajectory validator, retimer, planner adapter | `rebotarm_motion` |
| New teach file operation or replay policy | `rebotarm_teach` |
| New keyboard/web/gripper/RViz operator command adapter | `rebotarm_teleop` |
| New web panel, route, SSE payload formatting | `rebotarm_dashboard` |
| New URDF/SRDF/collision/planning group config | `rebotarm_moveit_config` |
| New detection/depth/grasp candidate logic | `rebotarm_vision` |
| New MuJoCo model, simulated controller, physics metric, or contact feedback | `rebotarm_simulation` |
| New launch composition or mutually exclusive backend selection | `rebotarm_bringup` |
| New hand-eye/TCP/TF check tool | `rebotarm_calibration` |

如果一个功能似乎属于多个包，应按职责拆分，而不是让一个大型节点拥有整个工作流。

## 测试规则

架构规则由 `tests/test_package_layering.py` 进行约束和检查。

新增模块时：

- 为纯逻辑添加单元测试
- 变更职责归属时添加包分层测试
- 运行 `python -m pytest tests -q`
- 对变更过的 Python 包运行 `python -m compileall`

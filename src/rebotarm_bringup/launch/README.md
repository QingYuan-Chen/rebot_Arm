# rebotarm_bringup 启动结构与功能

本文描述 `src/rebotarm_bringup/launch` 的当前结构、包含关系和功能边界。所有真机组合共享
同一个硬件底层片段，禁止在其它 bringup 启动文件中再次直接声明
`reBotArmController`。

`driver_only.launch.py` 已删除：它只是无附加行为地包含
`hardware_controller.launch.py`，保留两个名字反而容易让人误以为有两种硬件启动方式。
只启动底层控制器时直接使用：

```bash
ros2 launch rebotarm_bringup hardware_controller.launch.py
```

## 结构总览

```text
src/rebotarm_bringup/launch/
├── hardware_controller.launch.py
├── bringup.launch.py
├── moveit_hardware.launch.py
├── moveit_demo.launch.py
├── mujoco_moveit_sim.launch.py
├── interactive_system.launch.py
├── rebotarm_app.launch.py
├── teleop_keyboard.launch.py
├── teleop_system.launch.py
├── rviz_ee_drag_sim.launch.py
└── visual_grasp_system.launch.py       # 视觉候选入口（选择离线或外部环境）
```

内部组合片段位于 `launch/includes/`，由正式总入口加载，不作为用户入口单独启动：

```text
includes/
├── visual_backend.launch.py
├── visual_lifecycle.launch.py
├── visual_input.launch.py
├── grasp_candidate.launch.py
├── candidate_filter.launch.py
└── grasp_preview.launch.py
```


## 参数归属

- `visual_grasp_interfaces.yaml`：跨节点 topic、frame、MoveIt service 和规划组名称；由
  bringup 维护，避免在多个节点启动字典中重复硬编码。
- `rebotarm_vision/config/`：相机、YOLO、GraspNet、候选评分与视觉策略；由视觉包维护。
- launch 文件：只负责后端选择、生命周期开关和兼容性覆盖；不新增算法默认值。

视觉主入口已统一为 `visual_grasp_system.launch.py`，旧 compact、readonly、plan_only、
execute 包装入口已删除。新接口参数先加到 `visual_grasp_interfaces.yaml`，不要同时复制到
多个 include 文件。策略 profile 与节点 policy YAML 仍有既有默认值覆盖关系；入口收敛
不代表这些策略配置已经全部去重。

## 分层和包含关系

下图从共享底层向使用它的上层展开，表示复用关系；实际 include 调用方向是上层包含底层，
不是 hardware_controller 启动所有下方入口。相机只读诊断使用 scripts/run_ubuntu_vision.sh，不加载此视觉抓取组合。

```text
hardware_controller.launch.py                  唯一真实硬件控制器定义
├── bringup.launch.py                          + 状态发布 + 基础 RViz
├── interactive_system.launch.py               状态源/MoveIt/RViz 的共享实现
│   ├── moveit_hardware.launch.py              固定真机后端的薄入口
│   │   └── rebotarm_app.launch.py             + 示教录制 + Dashboard + 状态 RViz（不含键盘）
│   ├── rviz_ee_drag_sim.launch.py             仿真 MotionPlanning 入口
├── teleop_keyboard.launch.py                  键盘 + 状态发布 + 基础 RViz（控制器点动适配）
│   └── teleop_system.launch.py                + 示教录制 + Dashboard（控制器点动适配）
```

`interactive_system.launch.py` 只有在 `use_hardware=true` 时才包含硬件片段；纯预览和仿真
分支不会打开串口。上层入口必须确保同一命名空间只有一个真实或仿真轨迹执行后端。

## 文件功能

| 文件 | 类型 | 功能和边界 |
|---|---|---|
| `hardware_controller.launch.py` | 底层片段 | 唯一直接启动 `reBotArmController` 的文件；统一串口、反馈频率、夹爪保护、仲裁、命名空间和坐标系参数 |
| `bringup.launch.py` | 基础真机入口 | 硬件、夹爪可视化状态桥、TF 和可选基础 RViz；不启动 MoveIt |
| `moveit_hardware.launch.py` | 唯一真机 MoveIt 用户入口 | 薄包装 `interactive_system.launch.py`，固定真机、MoveIt、真实关节状态和唯一状态源；启动后仍需显式 Enable |
| `moveit_demo.launch.py` | 离线 MoveIt 演示入口 | 使用 `rebotarm_moveit_config` 的模型与规划参数，启动 MoveIt、RViz 和夹爪可视化桥；不接真机 |
| `mujoco_moveit_sim.launch.py` | MuJoCo + MoveIt 组合 | 唯一 MuJoCo 后端、MoveIt 与轨迹准备服务；默认 prepared_v1，可选 Dashboard |
| `interactive_system.launch.py` | 共享实现 | 统一拥有状态发布、MoveIt、RViz 以及真机/无硬件状态源的互斥选择；不建议用户手写其内部组合参数启动真机 |
| `rebotarm_app.launch.py` | 完整真机入口 | 包含真机 MoveIt 组合，再增加示教录制、Dashboard 和状态 RViz；不启动键盘节点 |
| `teleop_keyboard.launch.py` | 遥操作入口 | 可选硬件、键盘关节点动、状态发布和 RViz；默认不接真机。无硬件模式使用 RViz 轨迹插值预演控制器，按键可以改变 RViz 姿态，但不代表物理仿真 |
| `teleop_system.launch.py` | 遥操作组合 | 包含键盘入口，再增加示教录制和默认只读 Dashboard；默认不接真机。真机 prepared_v1 复用 MoveIt 场景/准备服务；legacy 与无硬件轻量预演沿用原状态链 |
| `rviz_ee_drag_sim.launch.py` | 仿真规划入口 | 使用 RViz 轨迹插值预演控制器提供 Plan/Execute，无物理引擎 |
| `visual_grasp_system.launch.py` | 唯一视觉入口 | 感知、筛选及显示；默认 `robot_context=offline`，也可接入外部环境 |

这里的 RViz 轨迹插值预演只验证 ROS/MoveIt 流程和姿态显示，**不代表 MuJoCo
物理验证**。它不计算动力学、执行器力、接触、碰撞或抓取结果；这些内容必须使用
`mujoco_moveit_sim.launch.py` 验证；无窗口时设置
`use_rviz:=false use_mujoco_viewer:=false`。

MuJoCo 组合的 `trajectory_execution_mode` 默认为 `prepared_v1`；真实硬件仍默认为
`legacy`。可选 `use_dashboard:=true` 使用相同轨迹模式；另起终端键盘时也须选择
`trajectory_execution_mode:=prepared_v1`，默认单步为 0.02 rad / 0.4 s。
前后端能力不匹配或未知时拒绝，不自动回退；契约与验收见
[P005 执行接入](../../../docs/implemented/features/p005_prepared_trajectory.md)。

三类日常操作入口均显式转发模式：`moveit_hardware`、`rebotarm_app`、
`teleop_keyboard`（及 `teleop_system`）。
真机首次软件交付需显式传 `trajectory_execution_mode:=prepared_v1`；默认仍 legacy，
等待用户通过现有网页/键盘和 MoveIt 实测后再切换。独立 `hardware_controller`/`bringup`
不提供准备服务，保留底层 legacy 默认。所有真机入口继续默认失能。
随包 YAML 不覆盖键盘时长：prepared_v1 默认 0.02 rad / 0.4 s，legacy 保留 0.2 s；
自定义 YAML 可显式覆盖，超限直接拒绝。

### Jazzy MoveIt 退出兼容

`moveit_demo.launch.py` 同时拥有 move_group 和准备节点的进程环境。配套 MoveIt 2.12.4
的轨迹执行管理器会先卸载控制器插件，再释放仍引用插件内析构代码的 ROS 回调组，
造成两类进程退出段错误。该共用片段仅对这两个进程预加载
`moveit_simple_controller_manager`，使插件映射保留到进程退出；节点、回调、线程和
MoveIt 对象仍执行正常清理，不捕获段错误或改写退出码。
插件由 ament 索引定位，已有 `LD_PRELOAD` 保留，缺失插件在启动前报错；
不修改系统 ROS 库，也不把环境变量传播给硬件控制器或其他节点。
配套 MoveIt 修复析构顺序并通过退出回归后，可移除此兼容处理；诊断及范围见
[退出修复记录](../../../docs/implemented/features/p005_prepared_trajectory.md#moveitmoveitpy-退出段错误修复)。

## 两组容易混淆的入口

| 需求 | 应使用 | 不包含 |
|---|---|---|
| 只需真机 MoveIt 规划/执行 | `moveit_hardware.launch.py` | Dashboard、键盘遥操、示教录制 |
| 需要真机 MoveIt + 示教录制 + Dashboard 完整工作台 | `rebotarm_app.launch.py` | 键盘遥操 |
| 只需键盘点动和基础 RViz | `teleop_keyboard.launch.py` | Dashboard、示教录制；仅真机 prepared_v1 补齐 MoveIt 场景与准备服务 |
| 需要键盘 + 示教录制 + Dashboard | `teleop_system.launch.py` | legacy 不含 MoveIt；真机 prepared_v1 由键盘组合补齐场景与准备服务 |

因此它们共享部分界面，但不是重复实现。`moveit_hardware.launch.py` 和
`teleop_keyboard.launch.py` 是可独立使用的基础组合；各自的上层入口只叠加自己的功能。
`rebotarm_app.launch.py` 与 `teleop_system.launch.py` 不应在同一命名空间同时启动。

视觉入口支持 `robot_context:=offline` 和 `robot_context:=external`。离线模式使用被动关节状态与 MoveIt 查询，外部模式复用已启动的机器人环境。MuJoCo 外部接入时设置 `use_sim_time:=true`；具体命令见[视觉候选命令](../../../docs/reference/commands/visual_grasp_commands.md)。

## includes/ 是什么

`includes/` 是内部 launch 组合片段目录，不是另一组用户入口，也不是 ROS 节点的实现目录。
文件负责选择所属包的节点、传入配置和连线；算法与控制实现仍留在 vision、motion、controller
等属主包。它们依赖主入口准备的 LaunchConfiguration，不建议单独 ros2 launch。

| 内部文件 | 负责的组合 | 不负责 |
|---|---|---|
| `visual_backend.launch.py` | 离线模式装配被动状态、模型、TF 与 MoveIt 查询；按需启动 RViz | 外部环境的公共节点 |
| `visual_lifecycle.launch.py` | 组合输入、候选、筛选和显示 | 运动控制 |
| `visual_input.launch.py` | D405、YOLO 和显式配置的手眼 TF | 运动控制 |
| `grasp_candidate.launch.py` | GraspNet、原始候选 Marker、可选 Open3D | IK 筛选 |
| `candidate_filter.launch.py` | TF、工作空间、IK 和碰撞筛选 | 运动控制 |
| `grasp_preview.launch.py` | 筛选结果 Marker | 运动控制 |

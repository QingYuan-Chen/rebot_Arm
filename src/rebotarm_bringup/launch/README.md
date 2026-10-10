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
└── visual_grasp_system.launch.py       # 唯一视觉入口（use_hardware 选择仿真／真机）
```

内部组合片段位于 `launch/includes/`，由正式总入口加载，不作为用户入口单独启动：

```text
includes/
├── visual_backend.launch.py
├── visual_lifecycle.launch.py
├── visual_input.launch.py
├── grasp_candidate.launch.py
├── candidate_filter.launch.py
├── motion_execution.launch.py
├── grasp_executor.launch.py
└── grasp_preview.launch.py
```


## 参数归属

- `visual_grasp_interfaces.yaml`：跨节点 topic、frame、MoveIt service 和规划组名称；由
  bringup 维护，避免在多个节点启动字典中重复硬编码。
- `rebotarm_vision/config/`：相机、YOLO、GraspNet、候选评分与视觉策略；由视觉包维护。
- `rebotarm_motion/config/`：视觉就绪、运动规划和执行相关策略；由运动包维护。
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
│   └── visual_grasp_system.launch.py          真实感知 + 仿真／真机视觉抓取组合
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
| `mujoco_moveit_sim.launch.py` | MuJoCo + MoveIt 组合 | 启动唯一 MuJoCo 执行后端并包含本包的 MoveIt 演示入口；无头和桌面薄入口位于同目录 |
| `interactive_system.launch.py` | 共享实现 | 统一拥有状态发布、MoveIt、RViz 以及真机/无硬件状态源的互斥选择；不建议用户手写其内部组合参数启动真机 |
| `rebotarm_app.launch.py` | 完整真机入口 | 包含真机 MoveIt 组合，再增加示教录制、Dashboard 和状态 RViz；不启动键盘节点 |
| `teleop_keyboard.launch.py` | 遥操作入口 | 可选硬件、键盘关节点动、状态发布和 RViz；默认不接真机。无硬件模式使用 RViz 轨迹插值预演控制器，按键可以改变 RViz 姿态，但不代表物理仿真 |
| `teleop_system.launch.py` | 遥操作组合 | 包含键盘入口，再增加示教录制和默认只读 Dashboard；不启动 MoveIt，默认不接真机。无硬件模式可驱动 RViz 仿真姿态，真机点动仍由控制器执行 |
| `rviz_ee_drag_sim.launch.py` | 仿真规划入口 | 使用 RViz 轨迹插值预演控制器提供 Plan/Execute，无物理引擎 |
| `visual_grasp_system.launch.py` | 唯一视觉入口 | 固定执行流程；默认 `use_hardware=false`，在 RViz 轨迹插值预演后端执行，无物理引擎 |

这里的 RViz 轨迹插值预演只验证 ROS/MoveIt/视觉流程和姿态显示，**不代表 MuJoCo
物理验证**。它不计算动力学、执行器力、接触、碰撞或抓取结果；这些内容必须使用
`mujoco_moveit_sim.launch.py` 验证；无窗口时设置
`use_rviz:=false use_mujoco_viewer:=false`。

## 两组容易混淆的入口

| 需求 | 应使用 | 不包含 |
|---|---|---|
| 只需真机 MoveIt 规划/执行 | `moveit_hardware.launch.py` | Dashboard、键盘遥操、示教录制 |
| 需要真机 MoveIt + 示教录制 + Dashboard 完整工作台 | `rebotarm_app.launch.py` | 键盘遥操 |
| 只需键盘点动和基础 RViz | `teleop_keyboard.launch.py` | MoveIt、Dashboard、示教录制；无硬件模式使用轻量仿真执行键盘点动 |
| 需要键盘 + 示教录制 + Dashboard，且不需要 MoveIt | `teleop_system.launch.py` | MoveIt |

因此它们共享部分界面，但不是重复实现。`moveit_hardware.launch.py` 和
`teleop_keyboard.launch.py` 是可独立使用的基础组合；各自的上层入口只叠加自己的功能。
`rebotarm_app.launch.py` 与 `teleop_system.launch.py` 不应在同一命名空间同时启动。

视觉主入口固定为执行流程，`use_hardware` 选择仿真或真机。启动后等待操作员调用
`/rebotarm/visual_grasp/execute`，不自动使能或抓取。IK、运动服务和抓取编排自动启动，
无硬件时自动提供 `rebotarm_preview/rebotarm_sim_trajectory_controller`，仅验证 ROS 执行流程和 RViz 姿态；
物理、动力学和接触验证使用 MuJoCo 组合入口。

```bash
# 仿真轨迹执行 + Open3D（真实相机输入，不是物体接触物理仿真）
ros2 launch rebotarm_bringup visual_grasp_system.launch.py \
  use_hardware:=false start_open3d_viewer:=true

# 真机：启动后仍须检查反馈、预览和现场环境，并显式 enable
ros2 launch rebotarm_bringup visual_grasp_system.launch.py \
  use_hardware:=true channel:=auto \
  execute_gripper:=true start_open3d_viewer:=true
```

先在仿真后端调用 execute 检查轨迹，再退出仿真并启动真机后端。真机根据真实反馈和
新鲜目标重新规划，不复用仿真轨迹。只读相机诊断使用 `scripts/run_ubuntu_vision.sh`。
公共 execution_mode 已删除；旧 plan_only/readonly 参数会报错退出，避免误把预览请求
当成执行。节点内部的规划预检能力保留，算法参数和碰撞/新鲜度门槛不变。

## includes/ 是什么

`includes/` 是内部 launch 组合片段目录，不是另一组用户入口，也不是 ROS 节点的实现目录。
文件负责选择所属包的节点、传入配置和连线；算法与控制实现仍留在 vision、motion、controller
等属主包。它们依赖主入口准备的 LaunchConfiguration，不建议单独 ros2 launch。

| 内部文件 | 负责的组合 | 不负责 |
|---|---|---|
| `visual_backend.launch.py` | 包含 interactive_system；无硬件时启动 `rebotarm_preview` 的 RViz 轨迹插值预演控制器 | 不重复定义真实硬件节点，不实现抓取策略 |
| `visual_lifecycle.launch.py` | 顺序列出下面六个视觉处理片段 | 不触发自动摆位；文件名沿用历史命名 |
| `visual_input.launch.py` | 包含 vision.launch，启动 Gemini 2、YOLO、手眼 TF 与 TCP TF | 不执行机械臂动作 |
| `grasp_candidate.launch.py` | GraspNet、原始候选 Marker、可选 Open3D | 不保证候选可达，不执行候选 |
| `candidate_filter.launch.py` | 候选的 TF、工作空间、IK、碰撞过滤节点 | 不下发运动 |
| `motion_execution.launch.py` | motion 包的位姿规划/执行服务与停止确认 | 不决定抓取阶段顺序 |
| `grasp_executor.launch.py` | vision 包的抓取阶段编排，调用运动与夹爪接口 | 不直接访问电机 SDK |
| `grasp_preview.launch.py` | 过滤后抓取 Marker，以及默认关闭的目标预览发送器 | 不承担整条轨迹的规划执行 |

实际视觉组合关系如下；这里只列节点组合依赖，不表示启动后自动开始抓取：

```text
visual_grasp_system.launch.py
├── includes/visual_backend.launch.py
│   ├── interactive_system.launch.py
│   │   └── hardware_controller.launch.py（仅真机）
│   └── RViz 轨迹插值预演控制器（仅无硬件）
└── includes/visual_lifecycle.launch.py
    ├── visual_input.launch.py
    ├── grasp_candidate.launch.py
    ├── candidate_filter.launch.py
    ├── motion_execution.launch.py
    ├── grasp_executor.launch.py
    └── grasp_preview.launch.py
```

Open3D 显示原始候选；过滤结果和机械臂轨迹在 RViz 中检查。
调用 execute 才开始抓取；stop 成功表示停止已确认，不自动回位。

固定观察位工具已移除。抓取从当前姿态规划；safe_home 保留为独立操作员命令。
仿真初始姿态由 motion profile 中 sim_initial_joint_positions 指定，默认与 safe_home 一致（joint3=-1°，其余为零），仅用于初始化仿真状态；不让真机启动时自动回位。

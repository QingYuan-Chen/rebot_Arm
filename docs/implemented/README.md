# 已实现功能

> 状态：IMPLEMENTED；类型：当前功能索引；适用范围：已有源码并有软件/仿真依据的能力。

本目录只描述当前代码中已经实现、并有软件测试或仿真证据的功能。

“已实现”不自动等于真实硬件已验收。每份功能文档必须分别说明软件、仿真和真实设备边界；真实机械臂仍须遵守单独授权和现场安全门。

## 阅读入口

- `architecture.md`：当前实际包职责和架构边界；
- `features/`：已经实现的功能说明；先看 [机械臂系统功能总览](features/system_capabilities.md)，再按硬件、运动、操作、示教、视觉、标定和仿真专题阅读；
- `../reference/`：系统命令、节点拓扑、数据流、参数和接口参考；
- `../setup/`：新用户 clone、安装、构建和第三方环境配置。

## 功能文档边界

| 文档 | 负责说明 |
|---|---|
| `architecture.md` | 包所有权、依赖方向和长期架构约束 |
| `features/system_capabilities.md` | 功能总表和完成/未完成边界 |
| `features/hardware_execution.md` | 真机控制器、反馈、夹爪和执行安全 |
| `features/moveit_planning.md` | MoveIt、轨迹、碰撞和执行适配 |
| `features/operator_control.md` | Dashboard、键盘和 RViz 操作 |
| `features/teach_replay.md` | 示教录制、准备、检查和回放 |
| `features/vision_readonly.md` / `features/visual_grasp.md` | 视觉链路与抓取规划 |
| `features/calibration_acceptance.md` | 网页手眼/TCP 软件验收 |
| `features/mujoco_*.md` | MuJoCo 仿真和 RL 功能 |

本目录不作为命令字典；可复制命令统一放在 `../reference/commands/`。

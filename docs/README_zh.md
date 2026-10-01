# reBotArm 文档索引

> 状态：REFERENCE；类型：文档总索引；适用范围：`docs/` 全部分类入口。

本文档用于区分已实现功能、环境配置、设计方案、参考资料和历史记录。新接手者先看：

1. [`reference/project_status.md`](reference/project_status.md)：唯一项目状态文档、验证范围和安全边界；
2. [`MAINTENANCE.md`](MAINTENANCE.md)：文档分类和长期维护规则；
3. 本页对应类别和具体文档。

## 新用户最快阅读路线

```text
第一次接手 -> setup/ -> implemented/architecture.md
了解功能 -> implemented/features/system_capabilities.md
准备运行 -> reference/topology/system_dataflow.md
复制命令 -> reference/commands/README.md
查看当前验证与限制 -> reference/project_status.md
```

## 文档分类

| 类别 | 入口 | 用途 |
| --- | --- | --- |
| 已实现功能 | [`implemented/`](implemented/) | 当前代码已有并经过软件/仿真验证的功能说明 |
| 环境配置 | [`setup/`](setup/) | clone、第三方依赖、构建和运行环境 |
| 当前设计 | [`design/`](design/) | 尚在讨论或实施中的技术路线和接口 |
| 参考资料 | [`reference/`](reference/) | 参数、拓扑和审计参考 |
| 用户维护 | [`user/`](user/) | 用户本人维护的硬件和工程待办 |

## 当前状态和规则

- 当前保留 MuJoCo 仿真与 CPU Gymnasium Reach 基线；后续训练框架为 MJLab，尚未接入。
- 历史阶段文档已清理；已有实测不自动授权新的真实机械臂动作。
- 当前真实视觉、硬件和 RL 的边界以代码、测试、用户最新决定和 `reference/project_status.md` 为准。
- 文档中的“计划”“有功能”“软件验证”不等于真实硬件验收或任务成功。

## 当前有效文档

### 架构、环境和启动

- [`reference/context.md`](reference/context.md)：部署范围与工程术语。
- [`reference/ros_sdk.md`](reference/ros_sdk.md)：ROS SDK 与控制器接口。
- [`reference/third_party_notices.md`](reference/third_party_notices.md)：第三方来源与许可。
- [`setup/dependencies/README.md`](setup/dependencies/README.md)：按环境分类的依赖版本与安装入口。

- [`implemented/architecture.md`](implemented/architecture.md)：ROS 2 包职责、依赖方向、权限和工作流边界。
- [`setup/ubuntu_ros2_jazzy.md`](setup/ubuntu_ros2_jazzy.md)：Ubuntu 24.04 / ROS 2 Jazzy 环境、构建和安全顺序。
- [`setup/launch_python_configuration.md`](setup/launch_python_configuration.md)：MuJoCo、视觉和 GraspNet 的解释器配置契约。

### 当前操作

- [`implemented/features/hardware_execution.md`](implemented/features/hardware_execution.md)：真机控制器、反馈、夹爪和执行边界。
- [`implemented/features/moveit_planning.md`](implemented/features/moveit_planning.md)：MoveIt 规划、碰撞和轨迹执行。
- [`implemented/features/operator_control.md`](implemented/features/operator_control.md)：网页、键盘和 RViz 操作。
- [`implemented/features/teach_replay.md`](implemented/features/teach_replay.md)：示教录制、准备和回放。
- [`implemented/features/visual_grasp.md`](implemented/features/visual_grasp.md)：视觉候选、规划和阶段编排。
- [`setup/ubuntu_vision_setup_zh.md`](setup/ubuntu_vision_setup_zh.md)：Gemini 2、YOLO、GraspNet 环境和启动。
- [`implemented/features/vision_readonly.md`](implemented/features/vision_readonly.md)：Ubuntu 原生视觉只读链路的功能和边界。
- [`implemented/features/mujoco_rl.md`](implemented/features/mujoco_rl.md)：MuJoCo/Gymnasium Reach 已实现能力和训练边界。
- [`implemented/features/mujoco_teach_preview.md`](implemented/features/mujoco_teach_preview.md)：MuJoCo 示教预演已实现能力和边界。

### 当前软件验收和参数参考

- [`implemented/features/calibration_acceptance.md`](implemented/features/calibration_acceptance.md)：标定网页软件验收边界，不代表物理标定完成。
- [`reference/commands/README.md`](reference/commands/README.md)：按用户任务分类的唯一命令入口。
- [`reference/topology/system_dataflow.md`](reference/topology/system_dataflow.md)：节点与数据流。
- [`reference/parameters/current_parameter_sources.md`](reference/parameters/current_parameter_sources.md)：参数来源和维护边界。

## 设计文档和历史说明

- [`design/calibration_web_plan.md`](design/calibration_web_plan.md)：网页手眼标定完整技术路线；软件链路已实现，真实设备和物理精度仍在后续开发/验收范围。
- 历史迁移快照和已删除文档当前不在工作树；需要追溯时从 Git 历史恢复，不作为当前源码清单。

## 用户维护

- [`user/USER_MAINTAINED_TODO.md`](user/USER_MAINTAINED_TODO.md)：用户维护的待完善事项；代理不得擅自修改。

## 冲突处理顺序

如果文档互相矛盾，按以下顺序判断：

1. 当前代码和测试；
2. `AGENTS.md`、`docs/implemented/architecture.md` 的安全和包边界；
3. 用户最新明确决定；
4. `docs/reference/project_status.md`；
5. 当前操作文档；
6. 历史设计、迁移报告和旧实验记录。

详细分类和修改规则见 [`MAINTENANCE.md`](MAINTENANCE.md)。

# 机械臂系统功能总览

> 状态：IMPLEMENTED；类型：当前源码能力目录；适用范围：全部 ROS 包和主要运行能力；“已实现”不等于真实硬件本轮已验收。

本文是 `implemented/` 的功能总索引。每项都区分“软件能力”和“现场边界”，避免把代码存在、仿真通过误写成真机验收。

专题说明：硬件见 [真机硬件控制与执行](hardware_execution.md)，运动见 [MoveIt 规划与轨迹执行](moveit_planning.md)，操作见 [Dashboard、键盘与 RViz 操作](operator_control.md)，示教见 [示教录制、检查与回放](teach_replay.md)，视觉抓取见 [视觉抓取规划与执行编排](visual_grasp.md)。

## 功能概述与清单

| 功能域 | 当前实现 | 主要所有者 | 当前边界 | 运行入口 |
|---|---|---|---|---|
| 真实硬件通信 | MotorBridge/串口连接、六轴与夹爪反馈、状态发布、失能启动、显式 enable/disable、停止和 safe home | `rebotarmcontroller` | 需要设备、串口唯一占用、现场安全检查；软件测试不等于硬件验收 | [系统运行命令](../../reference/commands/system_runtime.md) |
| 关节轨迹执行 | `FollowJointTrajectory`、轨迹停止、反馈跟踪、超时/通信异常保护 | `rebotarmcontroller` + `rebotarm_motion` | Execute 必须有真实或仿真控制器；Plan 成功不等于 Execute | [功能命令](../../reference/commands/rebotarm_feature_commands.md) |
| 夹爪控制 | 位置控制、夹持服务、反馈新鲜度、超时和有界保持；网页与示教流程可调用 | `rebotarmcontroller`、`rebotarm_teleop` | 接触/夹持成功需要独立物理验证；不能由软件状态单独宣称抓取成功 | [遥操作命令](../../reference/commands/rebotarm_common_commands.md) |
| MoveIt 规划 | URDF/SRDF、规划组、IK、状态有效性/碰撞检查、RViz MotionPlanning Plan/Execute | `rebotarm_moveit_config`、`rebotarm_motion` | 真机 Execute 需要显式授权和 `moveit_simple_controller_manager` | [功能命令](../../reference/commands/rebotarm_feature_commands.md) |
| 网页 Dashboard | 状态面板、SSE、关节 Preview/Execute/Stop、夹爪、Safe Home、Enable/Disable、示教与标定页面 | `rebotarm_dashboard` | Dashboard 是本地 UI/API，不是视觉传输层；不直接访问电机 SDK | [遥操作命令](../../reference/commands/rebotarm_common_commands.md) |
| 键盘/RViz 操作 | 键盘小步调姿、夹爪可视关节状态、RViz 原生 MotionPlanning 拖动 | `rebotarm_teleop` | 不得与另一个真机控制器并行占用串口 | [功能命令](../../reference/commands/rebotarm_feature_commands.md) |
| 示教录制 | 重力补偿录制、原子反馈批次、JSONL 文件、文件列表和记录状态 | `rebotarm_teach` | 录制不自动代表轨迹可执行；真机重力补偿需现场授权 | [遥操作命令](../../reference/commands/rebotarm_common_commands.md) |
| 示教检查与回放 | 滤波、重采样、重定时、速度/加速度/jerk 检查、MoveIt 起点对齐、碰撞预检、dry-run/execute、运行时跟踪门 | `rebotarm_teach` + `rebotarm_motion` | 原始记录不能直接下发；回放通过不等于真实运动安全 | [遥操作命令](../../reference/commands/rebotarm_common_commands.md) |
| 网页手眼/TCP 标定 | 会话保存恢复、同步采样、五方法求解、独立验证、质量门、导出和审计 | `rebotarm_calibration` + `rebotarm_dashboard` | 当前是软件/合成数据闭环；真实相机和物理精度需独立验收 | [网页标定命令](../../reference/commands/calibration_web.md) |
| Ubuntu 原生视觉 | Gemini 2 RGB-D、CameraInfo、YOLO 检测、深度融合、本机 GraspNet 候选、Marker/Open3D 可视化 | `rebotarm_vision` + `rebotarm_bringup` | 相机入口只读；完整抓取入口等待显式 execute，不自动使能或抓取 | [视觉只读命令](../../reference/commands/ubuntu_vision_readonly_test_zh.md) |
| 视觉抓取规划 | 候选过滤、IK、工作空间/姿态/夹爪约束、MoveIt 碰撞门、预抓取/抓取/撤退阶段编排 | `rebotarm_vision` + `rebotarm_motion` | 候选和规划通过不等于抓取成功；真机执行需分级授权 | [视觉抓取命令](../../reference/commands/visual_grasp_commands.md) |
| MuJoCo 仿真 | URDF→MJCF、headless/viewer、仿真关节状态、仿真轨迹控制器、MoveIt Plan/Execute 仿真后端、物理指标 | `rebotarm_simulation` | 不访问真实电机；仿真接触不替代现场碰撞/抓取验收 | [仿真命令](../../reference/commands/system_runtime.md) |
| MuJoCo 示教预演 | 读取示教 JSONL，复用准备/重定时，在独立仿真中报告跟踪、接触和位移 | `rebotarm_teach` + `rebotarm_simulation` | 只覆盖仿真预演，不授权真机回放 | [示教预演命令](../../reference/commands/mujoco_teach_preview.md) |
| 轻量 RViz 预演 | 无物理引擎的轨迹插值、关节/夹爪状态和停止接口 | `rebotarm_preview` | 不验证动力学、碰撞、接触或真实硬件 | [架构说明](../architecture.md) |
| CPU Reach/RL | Gymnasium 无接触 Reach 与可选 SB3/PPO 工具 | `rebotarm_simulation` | 软件基准不代表学习策略收敛或实机适用 | [CPU 命令](../../reference/commands/mujoco_rl.md) |
| 独立 MJLab Reach-and-Hold | 独立环境中的任务、导出、回放和评测工具 | `/home/a/project/rebot_Arm_rl/MJLab/` | 训练与 ROS 分离，当前结果见唯一项目状态文档；本次整合不继续训练 | [MJLab 入口](../../reference/commands/mjlab_rl.md) |

## 本地保留的扩展

- [单瓶抓取](../../single_bottle_grasp_zh.md)：感知与确认执行分开启动，含候选稳定性/高度检查和失败后受控回基线保护。
- [Real2Sim](../../real2sim_bridge_zh.md)：只读反馈镜像、physics/mirror 模式与 Viewer。
- [Sim2Real 仿真工具](../../sim2real_workflow_zh.md)：随机化、记录、确定性回放、对比和批量安全检查，不部署真实策略。
- [Pick 环境](../../mujoco_pick_zh.md)：方块接触、阶段和失败分类；小批次安全通过不代表抓取成功。

## 尚未列为已实现功能的内容

- 稳定收敛的 Reach-and-Hold 策略和 sim-to-real 部署；独立训练与评测结果见[项目状态](../../reference/project_status.md)，软件链路通过不代表任务收敛。
- 自动真机视觉抓取、lift/retreat、放置和抓取结果分类；当前执行器存在，但不能据此宣称现场成功。
- 真实相机/机械臂联合精度、真实接触力和物理抓取成功率；这些属于单独的现场验收。
- 已退役的 Windows、HTTP、MJPEG、远程 JSON、独立 GraspNet 服务和旧 interactive-control 路线。

## 维护规则

新增一个可对外使用的功能时，必须同时更新：

1. 本表中的功能域、所有者和边界；
2. 对应的专用命令文档或 `system_runtime.md`；
3. 相关包的测试/验证依据；
4. 若功能仍在开发或有缺口，则放入 `docs/design/`，不要提前写入本表的“当前实现”。

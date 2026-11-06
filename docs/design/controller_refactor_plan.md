# 控制器结构整理记录

> 状态：REFERENCE；类型：已完成的控制器结构整理记录；适用范围：本仓库 Python 控制器；最后核对：2026-11-05。

本仓库的控制器 Python 结构整理及离线软件验收已完成。根据用户决定，原 C++ 迁移计划已废弃；后续迁移在另一个仓库开展，具体方案由该仓库确定。

## 当前结构

`rebotarmcontroller` 保留单一硬件进程、设备运行时和共享总线。六轴与夹爪分别持有控制、执行和任务状态；节点装配组件，ROS 接口通过职责端口提交请求及读取状态。轨迹生成、重定时、IK 和碰撞检查归 `rebotarm_motion`。

`hardware_manager.py` 负责装配，组件通过明确的端口交接；控制周期、反馈、生命周期和设备写入由运行时协调。目录和接口以[控制器说明](../../src/rebotarmcontroller/README.md)及[工程架构](../implemented/architecture.md)为准。

控制器启动默认失能；控制器或电机错误及明确急停按既有保护策略处理。健康设备的可恢复任务失败按既定停止、保持及基线恢复流程处理。连接和退出过程可能发送失能命令。具体行为与安全约束见[硬件执行说明](../implemented/features/hardware_execution.md)。

## 验证记录

控制器结构收敛、职责端口和设备线程约束已完成软件验证；已记录的实机连接、反馈、使能/失能及夹爪等分项结果与待验范围见[项目状态](../reference/project_status.md)。固定输入、故障决策和线程仲裁样本见[控制器对照样本](../implemented/features/controller_reference_samples.md)。

厂商 SDK 已用功能按职责整合到本地控制器和运动包，设备通信继续使用 MotorBridge；旧 SDK 树保留为参考。ABI 和虚拟串口检查、依赖退出及许可信息见[硬件执行说明](../implemented/features/hardware_execution.md#sdk-退出的离线验收)与[第三方说明](../reference/third_party_notices.md)。

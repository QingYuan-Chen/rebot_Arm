# Dashboard、键盘与 RViz 操作

> 状态：IMPLEMENTED；类型：操作交互功能说明；适用范围：网页、键盘和 RViz 操作适配；操作意图不等于硬件执行授权。

## 功能概述

`rebotarm_dashboard` 提供本地 Web UI/API 和 SSE，`rebotarm_teleop` 将网页、键盘、夹爪和 RViz 输入转换为 ROS 命令。

## 当前实现

- 网页状态、Preview/Execute/Stop、Safe Home、Enable/Disable；
- 网页夹爪控制和实时状态；
- 键盘关节小步调姿；
- RViz 夹爪可视关节状态；
- RViz 原生 MotionPlanning 末端拖动；
- 操作命令校验、状态聚合和错误反馈。

## 验证结果

Dashboard、teleop 和 launch 分层测试覆盖 API、命令适配、状态面板和包所有权。网页完整流程见 [遥操作命令](../../reference/commands/rebotarm_common_commands.md)。

## 边界与未完成

Dashboard 不访问电机 SDK、不实现运动规划或示教算法。不得与另一套真机控制器并行占用串口；真机 Execute 仍需独立授权。

# 真机硬件控制与执行

> 状态：IMPLEMENTED；类型：硬件执行功能说明；适用范围：`rebotarmcontroller`；软件存在不等于本轮真机授权或现场验收。

## 功能概述

`rebotarmcontroller` 是唯一访问 MotorBridge/串口和真实电机 SDK 的 ROS 包，负责反馈、夹爪、轨迹 Action 以及最后一道执行安全门。

## 当前实现

- 连接与反馈状态发布；
- 默认失能启动和显式 `/rebotarm/enable`、`/rebotarm/disable`；
- `FollowJointTrajectory`、`trajectory_stop` 和 `safe_home`；
- 六轴与夹爪状态、反馈新鲜度和错误处理；
- 夹爪位置/抓取服务及有界保持；
- 底层关节限位、格式和通信失败拒绝。

## 验证结果

控制器接口、失能启动、反馈刷新、使能回滚、轨迹停止和状态发布由控制器测试与分层测试覆盖。具体命令见 [系统运行命令](../../reference/commands/system_runtime.md)。

## 边界与未完成

真机启动前必须确认串口唯一占用、fresh feedback、现场净空、支撑和急停。健康但可恢复的任务失败不得把机械臂直接失能在未知姿态；软件测试不替代物理安全验收。

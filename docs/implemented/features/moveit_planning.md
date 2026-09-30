# MoveIt 规划与轨迹执行

> 状态：IMPLEMENTED；类型：运动规划功能说明；适用范围：MoveIt、`rebotarm_motion` 和规划配置；Plan 成功不等于真机 Execute 成功。

## 功能概述

系统使用 `rebotarm_moveit_config` 提供 URDF/SRDF、规划组和限位，由 `rebotarm_motion` 负责规划适配、轨迹检查、重定时、碰撞预检和执行协调。

## 当前实现

- 关节/末端位姿规划和 IK 预览；
- MoveIt 状态有效性与碰撞检查；
- 轨迹速度、加速度和 jerk 约束；
- `FollowJointTrajectory` 执行适配；
- RViz 原生 MotionPlanning Plan/Execute；
- 示教起点对齐和运行时跟踪安全门。

## 验证结果

包分层测试、轨迹安全测试和 MoveIt launch 检查覆盖当前职责边界。真机 Execute 必须存在 `moveit_simple_controller_manager` 和唯一控制器 Action server。

## 边界与未完成

规划结果只代表软件轨迹有效；真机执行仍需显式授权、使能、现场安全检查和受控停机。`rebotarm_motion` 不得直接访问电机 SDK。

## 运行入口

RViz、真机和仿真入口见 [功能启动命令](../../reference/commands/rebotarm_feature_commands.md)。

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

`prepared_v1` 支持 `arm`、`gripper` 和 `arm_with_gripper` Plan/Execute，统一映射到唯一
FJT。复合组用联合关节目标；协同位姿先对 `arm` 求 IK，再构造七轴关节目标，不给复合组
冒用六轴 IK。规划响应在 TOTG/Ruckig 后按执行曲线统一重定时，显示的是最终时间；执行准备
仍复核动态双指碰撞、能力和设备上下文。接口及兼容等级见 [P005 协同接口](p005_prepared_trajectory.md#修订协同接口)。

## 验证结果

包分层测试、轨迹安全测试和 MoveIt launch 检查覆盖当前职责边界。真机 Execute 必须存在 `moveit_simple_controller_manager` 和唯一控制器 Action server。

## 边界与未完成

规划结果只代表软件轨迹有效；真机执行仍需显式授权、使能、现场安全检查和受控停机。`rebotarm_motion` 不得直接访问电机 SDK。

## 运行入口

RViz、真机和仿真入口见 [功能启动命令](../../reference/commands/rebotarm_feature_commands.md)。

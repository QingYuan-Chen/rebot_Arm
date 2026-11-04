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

`prepared_v1` 保留 `arm`、`gripper` 两组独立 Plan/Execute，移除 `arm_with_gripper`。
`arm` 映射完整六轴 FJT；`gripper` 取规划末点，通过
`/rebotarm/gripper/moveit_command`（GripperCommand）转换为独立开合，左指 m ×2 得到
原接口的双指总开口 m，实际反馈和结果再 ÷2。规划时间按生效夹爪限速核定，不向 FJT 发送手指点。
机械臂响应在 TOTG/Ruckig 后取得后端规划快照，先规范化首点再重定时，显示最终时间。
失能可 Plan 预览，使能后须重新 Plan；其他运动、Stop、失能、重连后旧计划拒绝 Execute。
`ompl_reference` 仅供中间段保留虚拟起点，不可直接 Execute。插件匹配 MoveIt 重编码值后
发送服务缓存的权威纳秒轨迹，准备不再替换首点。后续运动带入新鲜真实
夹爪开度，执行准备全段固定它检查完整机器人和附着物。接口及兼容等级见
[P005 接口](p005_prepared_trajectory.md#修订协同接口)。

## 验证结果

包分层测试、轨迹安全测试和 MoveIt launch 检查覆盖当前职责边界。真机 Execute 必须存在 `moveit_simple_controller_manager` 和唯一控制器 Action server。

## 边界与未完成

规划结果只代表软件轨迹有效；真机执行仍需显式授权、使能、现场安全检查和受控停机。`rebotarm_motion` 不得直接访问电机 SDK。

## 运行入口

RViz、真机和仿真入口见 [功能启动命令](../../reference/commands/rebotarm_feature_commands.md)。

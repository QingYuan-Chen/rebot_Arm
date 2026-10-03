# rebotarm_motion

运动生成、规划适配和执行期校验包。它负责把目标变成经过检查的关节轨迹，并通过 ROS 控制器接口执行；不直接访问电机 SDK。

## 目录结构

```text
rebotarm_motion/
├── rebotarm_motion/
│   ├── command_models.py                  # 预览/执行数据模型与状态枚举
│   ├── mode_manager.py                    # sim/real 控制模式解析
│   ├── preview_manager.py                 # 当前状态与最后一次预览缓存
│   ├── message_codec.py                   # 预览命令/状态 JSON 编解码
│   ├── pose_math.py                       # 四元数与 RPY 互转
│   ├── pose_preview_solver.py             # SDK 运动学的无硬件预览
│   ├── moveit_planner.py                  # MoveIt 规划客户端适配
│   ├── execution_coordinator.py           # 预览到执行的唯一门控
│   ├── pose_execution_node.py             # 位姿目标规划与 FollowJointTrajectory 执行
│   ├── paired_trajectory_protocol.py      # 真机/仿真共用轨迹命令协议
│   ├── trajectory_time_parameterization.py# 示教轨迹重定时策略选择
│   ├── trajectory_runtime_limits.py       # 速度/力矩/加速度/jerk 运行时守卫
│   ├── trajectory_safety_monitor.py       # 期望轨迹与实际反馈的跟踪判定
│   ├── replay_runtime_monitor.py          # 回放超差时请求停止
│   ├── collision_precheck.py              # MoveIt 状态有效性/碰撞预检
│   ├── teach_replay_start_align_precheck.py # 回放起点对齐诊断
│   ├── teach_replay_start_alignment.py    # 回放前起点对齐轨迹
│   ├── real_failure_recovery.py           # 真机任务失败后的受保护恢复策略
│   ├── parameter_helpers.py               # 参数结构与 QoS 辅助
│   └── __init__.py
├── setup.py / package.xml / resource/*
```

## 关键脚本

- `moveit_planner.py` 只请求规划并返回轨迹；`pose_execution_node.py` 负责规划/可选执行服务，执行最终走控制器 Action。
- `pose_preview_solver.py` 可在无 ROS/无硬件条件下做 FK/IK 预览；`execution_coordinator.py` 防止绕过预览直接执行。
- `trajectory_time_parameterization.py` 选择 Ruckig 或回退重定时实现；`trajectory_runtime_limits.py` 和 `trajectory_safety_monitor.py` 分别检查运行时物理限值与跟踪误差。
- `collision_precheck.py` 在下发前按采样点调用 MoveIt 状态有效性服务；它只检查，不发送运动。
- `replay_runtime_monitor.py`、`teach_replay_start_alignment.py` 为 `rebotarm_teach` 提供回放安全能力。
- `paired_trajectory_protocol.py` 负责可校验、可哈希、可采样的关节轨迹命令；`real_failure_recovery.py` 遵守健康 enabled 状态下保持或受控回 baseline 的安全原则。

## 对外入口

```bash
ros2 run rebotarm_motion PoseExecutionNode
```


## 依赖边界

```text
目标位姿/示教数据 -> rebotarm_motion -> MoveIt / controller Action
```

本包不能依赖 Dashboard、Teach 或 Teleop 的实现，也不能直接打开串口或调用电机 SDK。新的轨迹验证、重定时和规划适配应继续归入这里。


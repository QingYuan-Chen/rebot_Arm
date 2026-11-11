# rebotarm_motion

运动生成、规划适配和执行期校验包。它负责把目标变成经过检查的关节轨迹，并通过 ROS 控制器接口执行；不直接访问电机 SDK。

## 目录结构

```text
rebotarm_motion/
├── rebotarm_motion/
│   ├── pose_math.py                       # 四元数与 RPY 互转
│   ├── pose_kinematics.py                 # 规范模型 FK/阻尼 IK
│   ├── cartesian_trajectory.py            # SE(3) 采样、连续 IK 与重定时
│   ├── pose_compat_node.py                # move_to_pose Action/IK 服务
│   ├── moveit_planner.py                  # MoveIt 规划客户端适配
│   ├── polynomial_trajectory.py           # 系数生成、极值与碰撞采样时刻
│   ├── coordinated_trajectory.py          # 有效约束与最终曲线重定时
│   ├── trajectory_preparation_node.py     # 带能力/场景绑定的碰撞准备服务
│   ├── trajectory_retiming_node.py        # 规划重定时服务
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

- `pose_kinematics.py` 提供纯计算 FK/IK；`pose_compat_node.py` 完成反馈准入、规划和 FJT 执行协调。
- `trajectory_time_parameterization.py` 按 q/qv/qva 等级重定时，保留合法历史方法名并报告实际算法，未知方法名显式失败。
- `trajectory_runtime_limits.py` 校验 YAML 限位映射；`trajectory_safety_monitor.py` 检查完整有限的反馈位置和速度。
- `collision_precheck.py` 在下发前按采样点调用 MoveIt 状态有效性服务；它只检查，不发送运动。
- `replay_runtime_monitor.py`、`teach_replay_start_alignment.py` 为 `rebotarm_teach` 提供回放安全能力。
- `paired_trajectory_protocol.py` 负责可校验、可哈希、可采样的关节轨迹命令；`real_failure_recovery.py` 遵守健康 enabled 状态下保持或受控回 baseline 的安全原则。
- 准备器必须接收配置目录和有效能力快照，按能力摘要核验实际约束；运行监控将缺字段及非有限反馈纳入现有持续违规停止流程。

## 对外入口

`PoseCompatNode`、`TrajectoryPreparationNode` 和 `TrajectoryRetimingNode` 由相应操作组合启动。


## 依赖边界

```text
目标位姿/示教数据 -> rebotarm_motion -> MoveIt / controller Action
```

本包不能依赖 Dashboard、Teach 或 Teleop 的实现，也不能直接打开串口或调用电机 SDK。新的轨迹验证、重定时和规划适配应继续归入这里。

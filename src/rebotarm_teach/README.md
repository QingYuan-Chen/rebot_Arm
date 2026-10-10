# rebotarm_teach

示教 JSONL 可通过 `rebotarm_simulation` 的 `rebotarm_mujoco_teach_preview` 在独立 MuJoCo 实例中预演；
操作说明见 [MuJoCo 离线预演](../../docs/reference/commands/mujoco_teach_preview.md)。

示教录制、轨迹预处理和安全回放工作流包。它拥有示教文件与回放生命周期，但把重定时、起点对齐、碰撞预检和运行时跟踪能力交给 `rebotarm_motion`；不实现 Dashboard 页面，也不直接访问电机 SDK。

## 目录结构

```text
rebotarm_teach/
├── rebotarm_teach/
│   ├── teach_recorder_node.py          # 唯一示教录制 ROS 节点
│   ├── teach_models.py                 # 不可变数据模型与回放决策类型
│   ├── teach_record_io.py              # JSONL 编解码和文件读写
│   ├── teach_quality.py                # 质量评估与文件巡检
│   ├── teach_preparation.py            # 滤波、重采样与 prepared 轨迹
│   ├── teach_replay_gates.py           # dry-run / Execute 纯门控
│   ├── teach_recording.py              # 原有 Python 导入路径兼容导出
│   ├── recording_feedback.py           # 同批次/新鲜反馈 -> TeachSample
│   ├── teach_record_client.py          # 设置路径、开始、停止录制的客户端
│   ├── teach_replay_workflow.py        # 回放完整生命周期与安全门
│   ├── replay_moveit_precheck.py       # MoveIt 预检适配
│   ├── replay_action_lifecycle.py      # Action 回调与活动目标状态
│   ├── teach_replay_coordinator.py     # 回放状态和协调
│   ├── teach_replay_trajectory_builder.py # 起点/对齐/录制/保持段拼接
│   ├── teach_replay_settings.py        # speed、对齐、末尾保持参数
│   ├── teach_replay_client.py          # 取消 Action + trajectory_stop 门面
│   ├── service_call_helpers.py         # Trigger 服务调用辅助
│   ├── parameter_helpers.py             # 参数与 QoS 辅助
│   └── __init__.py
├── setup.py / package.xml / resource/*
```

## 核心职责

- `teach_recorder_node.py` 订阅已校验的关节/电机反馈，以 JSONL 写入示教样本；重复或跨批次反馈不会伪造新样本。
- `teach_models.py` 只定义示教样本、质量结果、prepared trajectory 和回放门控数据类型。
- `teach_recording.py` 只兼容导出原有 Python API；实现分别位于数据模型、文件、质量、预处理和门控模块。
- `teach_replay_workflow.py` 保持唯一回放编排入口；MoveIt 预检适配与 Action 回调实现已移至独立模块。
- `teach_replay_trajectory_builder.py` 只构造时间轴，`teach_replay_settings.py` 只规范化设置；二者不决定硬件是否可执行。
- `teach_record_client.py` 和 `teach_replay_client.py` 是给 Dashboard 等上层使用的 ROS 适配，不拥有 HTTP 或页面逻辑。

## 对外入口

```bash
ros2 run rebotarm_teach TeachRecorderNode
```

录制和回放通常由 `rebotarm_bringup` 的 Dashboard 组合启动/调用；当前没有并行维护的独立 `TeachReplayNode` 或 `teach_replay.launch.py` 入口。真实回放必须显式通过执行门，默认 dry-run 不能当作真机动作。

## 回放边界

```text
示教 JSONL -> teach_recording 预处理
           -> rebotarm_motion 重定时/对齐/碰撞/跟踪检查
           -> FollowJointTrajectory -> rebotarmcontroller
```

回放失败时，健康的已使能真机不能被任务级异常直接失能；应保持 enabled hold，或受保护返回已捕获 baseline 后再按流程失能。

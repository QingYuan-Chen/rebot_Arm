# 示教录制、检查与回放

> 状态：IMPLEMENTED；类型：示教功能说明；适用范围：`rebotarm_teach` 与 `rebotarm_motion`；回放通过不等于真机安全。

## 功能概述

系统将重力补偿录制的原始 JSONL 作为输入，经过准备、重采样、重定时、起点对齐和碰撞预检后，才允许 dry-run 或 execute。

## 当前实现

- `TeachRecorderNode` 录制和文件所有权；
- 原子反馈批次、样本年龄和批次时间戳校验；
- 示教文件列表、设置和状态 payload；
- 速度/加速度/jerk 检查与轨迹构造；
- MoveIt 起点对齐、碰撞预检和运行时跟踪门；
- 由 `rebotarm_simulation` 提供 MuJoCo 独立示教预演入口；`rebotarm_teach` 只提供记录读取和准备逻辑。

示教包内的文件读写、数据模型、质量检查和轨迹准备分别归
`teach_record_io.py`、`teach_models.py`、`teach_quality.py` 和 `teach_preparation.py`。
MoveIt 预检与 Action 生命周期由专用辅助模块处理，`TeachReplayWorkflow` 继续统一
dry-run token、对齐、执行和停止状态，Dashboard 不复制回放实现。

新录制格式 v2 在原六轴向量之外保存夹爪总开口、源时间和有效性。准备只使用两类反馈的
共同源时间覆盖区间，至少需要两个不同夹爪源帧；无效记录拒绝，不补造夹爪轨道。
派生七轴数据统一预处理、重采样、C2 生成、重定时和碰撞准备，输出
`.prepared.v2.jsonl`，不覆盖原始记录。dry-run 令牌同时绑定执行能力，重连后须重做。
旧六轴记录保留原字段等级，不增加夹爪轨道。七轴运行由正式后端负责发送与到位确认，
不使用旧六轴线性监控近似。详见 [P005 协同接口](p005_prepared_trajectory.md#修订协同接口)。

## 验证结果

录制、轨迹准备、回放协调器、运动安全和 MuJoCo 预演均有软件或仿真测试。网页录制和回放入口见 [遥操作命令](../../reference/commands/rebotarm_common_commands.md)。

## 边界与未完成

原始记录不能直接作为执行命令。真机重力补偿、回放、停止和失能必须遵守现场安全门；仿真预演不替代真实碰撞或抓取验收。

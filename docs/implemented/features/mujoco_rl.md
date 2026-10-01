# MuJoCo / Gymnasium Reach

> 状态：IMPLEMENTED；范围：CPU 无接触位姿环境和可选 SB3/PPO 工具；GPU训练质量未在本地验收。

`RebotArmReachEnv` 提供六维目标增量、24 维观测、目标采样、奖励和成功/超时处理。
物理步进使用 CPU MuJoCo。现有 SB3/PPO 训练、保存/加载、独立种子评估及 Viewer
入口保留作基线；其策略计算使用 CUDA，需要可选 `docs/setup/dependencies/rl.md`。

本地已验证 reset/step、任务契约及 Gymnasium 环境检查。
环境关闭接触/碰撞，不提供夹爪策略、图像观测、避障或实机动作。
本轮没有运行 GPU 训练，不能把上游历史结果当成本机验收。

MJX 专属模块与依赖已移除；后续引入 MJLab，目前尚未接入。
完整命令和任务契约见 [Reach 命令参考](../../reference/commands/mujoco_rl.md)。

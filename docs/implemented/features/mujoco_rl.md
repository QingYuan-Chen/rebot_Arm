# MuJoCo / Gymnasium Reach

> 状态：IMPLEMENTED；类型：纯仿真功能说明；适用范围：MuJoCo/Gymnasium Reach；源码包含 CPU/GPU 训练接口；本地未验收 GPU 训练质量。

## 功能概述

`RebotArmReachEnv` 提供六轴目标增量动作、关节/末端/目标位姿观测、奖励、碰撞与超时处理，以及 Gymnasium、Stable-Baselines3 和 GPU PPO 训练接口。

## 当前实现

- 环境 `reset/step` 和基本任务契约；
- 位置与姿态观测、目标采样和成功门；
- 碰撞、超时和终止处理；
- 模型保存/加载以及 PPO 训练与评估入口。

## 验证结果

本地本轮验证了环境 reset/step、任务契约和 Gymnasium 环境检查。上游报告过 SB3/PPO 训练链路，但本机缺少 PyTorch/SB3/JAX/MJX，尚未运行对应训练或 GPU 验收；多随机种子、固定评估集和策略收敛仍需另行验证。

## 边界与未完成

- 不连接 ROS 控制器、相机、夹爪或真实机械臂；
- 当前策略尚未达到稳定成功率门槛；
- 短训练通过只代表链路可运行，不代表策略收敛或真机可用。

## 运行入口

完整的 clone、环境检查、训练和评估命令见 [MuJoCo/RL 命令参考](../../reference/commands/mujoco_rl.md)。

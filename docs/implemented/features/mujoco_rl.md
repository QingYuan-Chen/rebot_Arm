# MuJoCo / Gymnasium Reach

> 状态：IMPLEMENTED；类型：纯仿真功能说明；适用范围：MuJoCo/Gymnasium Reach；训练链路已验证，策略尚未达到稳定成功率门槛。

## 功能概述

`RebotArmReachEnv` 提供六轴目标增量动作、关节/末端/目标位姿观测、奖励、碰撞与超时处理，以及 Gymnasium、Stable-Baselines3 和 GPU PPO 训练接口。

## 当前实现

- 环境 `reset/step` 和基本任务契约；
- 位置与姿态观测、目标采样和成功门；
- 碰撞、超时和终止处理；
- 模型保存/加载以及 PPO 训练与评估入口。

## 验证结果

软件基线已验证环境 reset/step、模型保存加载、Gymnasium/SB3 检查和短训练链路。多随机种子训练、固定评估集和收敛验收仍属于 `Agent/CURRENT_STATUS.md` 的当前范围。

## 边界与未完成

- 不连接 ROS 控制器、相机、夹爪或真实机械臂；
- 当前策略尚未达到稳定成功率门槛；
- 短训练通过只代表链路可运行，不代表策略收敛或真机可用。

## 运行入口

完整的 clone、环境检查、训练和评估命令见 [MuJoCo/RL 命令参考](../../reference/commands/mujoco_rl.md)。

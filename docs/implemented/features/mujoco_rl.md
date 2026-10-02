# mjlab / MuJoCo Warp Reach

> 状态：EXPERIMENTAL；GPU 训练链路与 CPU MuJoCo 正确性基准隔离。

当前强化学习主线使用现有 MJCF、mjlab manager-based environment、MuJoCo Warp GPU 物理后端和 RSL-RL/PyTorch 训练器。该环境不启动 ROS、不连接真实机械臂，也不构成 sim-to-real 或硬件验收。

CPU MuJoCo 仍用于单环境模型、轨迹和物理正确性检查；训练结果必须用同一 MJCF 和目标集回放复核。

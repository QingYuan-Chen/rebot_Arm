# 独立 MJLab 工程入口

> 状态：REFERENCE；范围：现有独立强化学习工程的入口与边界；不授权新的训练或实机动作。

本地 MJLab 工程位于 `/home/a/project/rebot_Arm_rl/MJLab/`，不属于本 ROS 工作区，
主工程不保留 `DRL/` 目录。任务、依赖锁、模型快照、导出、评测与远端部署说明以
[MJLab README](../../../../rebot_Arm_rl/MJLab/README.md) 为准。
本次上游整合不迁移该工程、不改变依赖或任务参数，也不启动训练。

本 ROS 工程保留 CPU MuJoCo 模型、轨迹和物理基准，以及
[CPU Gymnasium Reach / SB3 工具](mujoco_rl.md)。ROS 仿真默认使用系统 Python 3.12
的用户级 MuJoCo 3.3.0 / NumPy 1.26.4 依赖；MJLab 使用自己的独立环境，二者不混装。

训练项目不启动 ROS、不连接真实机械臂；策略导出或仿真成功不构成实机验收。
规范模型由 `rebotarm_simulation` 维护，独立工程通过 `provenance.json` 记录来源提交和 SHA-256 摘要，使用显式模型快照。
本机仅导出、测试和回放；新的训练改动或远端运行需另行明确授权。

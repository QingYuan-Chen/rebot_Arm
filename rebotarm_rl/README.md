# rebotarm_rl

同仓库独立 Python 项目：mjlab Reach 任务、PPO 配置和 CPU/GPU 配对评估。
不依赖 ROS、preview 或 simulation Python 实现，不控制真实硬件。

```text
rebotarm_rl/
  tasks/reach.py             # 任务、命令、观测、奖励、终止条件
  agents/reach_ppo.py        # 网络/PPO/runner配置，算法由RSL-RL实现
  evaluation/paired_eval.py # 固定目标CPU/GPU配对评估
  resources.py             # 共享MJCF定位
```

在仓库根目录执行（使用已有统一环境，不另建CPU环境）：

```bash
third_party/rebotarm_mjlab_venv/bin/python -m pip install -r requirements/requirements-mjlab.txt
third_party/rebotarm_mjlab_venv/bin/python -m pip install -e rebotarm_rl --no-deps
third_party/rebotarm_mjlab_venv/bin/python -m mjlab.scripts.list_envs
third_party/rebotarm_mjlab_venv/bin/python -m mjlab.scripts.train RebotArm-Reach-Mjlab --env.scene.num-envs 128 --agent.max-iterations 1000 --log-root runs/mjlab
```

训练、回放、评估的完整说明见 [命令参考](../docs/reference/commands/mjlab_rl.md)。
任务名 `RebotArm-Reach-Mjlab`、奖励、观测、力矩动作和网络参数保持迁移前语义。
checkpoint 和训练输出留在仓库 `runs/`，不要提交到普通 Git。

机器人模型只有一份：`src/rebotarm_simulation/models/rebotarm/`。
本次保留 reach_scene.xml 及其 robot.xml 相对引用，不复制模型。
同仓库 editable 安装可自动定位；wheel/服务器部署必须显式设置：

```bash
export REBOTARM_MJLAB_SCENE=/absolute/path/to/model-bundle/reach_scene.xml
```

该目录须同时包含 robot.xml 与其网格资源，并记录代码、模型版本和依赖版本。
不能只拷贝一个 XML 文件。CPU/GPU配对评估仍使用 mjlab 编译的同一模型。

迁移后 simulation 不再注册 mjlab 插件；已有环境需重新安装或重建 simulation 元数据，
避免旧插件重复注册。ROS 包仍按工作区 colcon 流程构建；本项目不需要 colcon。

当前动作是六轴力矩，不等价于 ROS 位置轨迹。真实策略部署需要另外的动作/观测契约、
安全执行适配和独立授权。训练 smoke 不代表策略收敛或 sim-to-real 验收。

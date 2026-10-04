# mjlab GPU Reach 训练（可选实验线）

> 状态：EXPERIMENTAL；这条线与 CPU MuJoCo 基准和 ROS 仿真后端隔离。

## 运行边界

该入口使用 `mjlab + MuJoCo Warp + PyTorch/RSL-RL` 在 GPU 上运行并行 Reach
环境。它不启动 ROS，不读取电机 SDK，不连接真实机械臂。
任务、PPO 配置和配对评估属于根目录独立 Python 项目 `rebotarm_rl/`。
共享 MJCF 仍由 `rebotarm_simulation/models/rebotarm/` 提供。

当前任务使用六轴力矩动作；CPU/GPU 配对评估使用同一个 mjlab 编译模型、
观测和动作契约。ROS 的关节位置轨迹接口不能直接接收这类策略输出。

## 隔离安装

使用统一的 mjlab Python 环境，同时提供 CPU MuJoCo 基准和 GPU Warp/RSL-RL 训练。
已有环境直接复用；首次创建时执行：

```bash
cd /home/huangbin/robotarm_ros2
uv venv third_party/rebotarm_mjlab_venv --python 3.12
uv pip install --python third_party/rebotarm_mjlab_venv/bin/python -r requirements/requirements-mjlab.txt
```

安装独立训练项目：

```bash
third_party/rebotarm_mjlab_venv/bin/python -m pip install -e rebotarm_rl --no-deps
```

该安装注册 `mjlab.tasks` 插件，不需要 source ROS 或 colcon。
从旧版迁移且环境曾安装 simulation 时，还需刷新旧包元数据以移除旧插件：

```bash
third_party/rebotarm_mjlab_venv/bin/python -m pip install -e src/rebotarm_simulation --no-deps
```

ROS 工作区另按 colcon 流程重建受影响包。wheel/服务器安装需通过
`REBOTARM_MJLAB_SCENE` 指定完整模型资源中的 Reach 场景，详见
[项目说明](../../../rebotarm_rl/README.md)。

## 检查、训练和回放

从仓库根目录运行，使任务插件能定位源码 MJCF：

```bash
export REBOTARM_MJLAB_SCENE="$PWD/src/rebotarm_simulation/models/rebotarm/reach_scene.xml"
export MUJOCO_GL=egl
export MJLAB_WARP_QUIET=0
source third_party/rebotarm_mjlab_venv/bin/activate

python -m mjlab.scripts.list_envs | grep RebotArm
python -m mjlab.scripts.train RebotArm-Reach-Mjlab \
  --env.scene.num-envs 128 \
  --agent.max-iterations 10 \
  --log-root runs/mjlab
python -m mjlab.scripts.play RebotArm-Reach-Mjlab \
  --checkpoint-file runs/mjlab/rebotarm_mjlab_reach/<run>/model_*.pt \
  --env.scene.num-envs 1
```

## CPU/GPU 配对评估

训练 checkpoint 需要先用相同的固定目标集分别跑 CPU MuJoCo 和 GPU Warp。
评估入口不启动 ROS，也不连接真实机械臂；它会记录每个目标的末端位置/姿态误差、
成功门、关节轨迹差异和 checkpoint SHA-256：

```bash
export REBOTARM_MJLAB_SCENE="$PWD/src/rebotarm_simulation/models/rebotarm/reach_scene.xml"
MUJOCO_GL=egl third_party/rebotarm_mjlab_venv/bin/python \
  -m rebotarm_rl.evaluation.paired_eval \
  --checkpoint runs/mjlab/rebotarm_mjlab_reach/<run>/model_*.pt \
  --episodes 100 \
  --steps 250 \
  --seed 20000 \
  --output runs/mjlab/rebotarm_mjlab_reach/<run>/paired_eval.json
```

`1 iteration` 或短 smoke checkpoint 只能验证接口和物理后端一致性，不能作为收敛、
泛化或 sim-to-real 证据。正式比较前应使用完成训练的 checkpoint，并保持目标集、种子、
控制周期和成功门不变。

先从 128 个环境开始，再根据显存逐步测试 256 和 512；不要直接假设 8 GB
显存可以运行示例中的 4096 个环境。训练日志、checkpoint 和视频放在
`runs/mjlab/`，不要把它们作为 ROS 安装资源提交。

## 证据要求

- 训练 smoke 通过只证明 mjlab/Warp/PyTorch 链路可运行；
- 必须使用固定目标、同一随机种子和 CPU MuJoCo 做 paired evaluation；
- GPU 训练成功不等于 MuJoCo 模型、接触行为或 sim-to-real 已校准；
- 当前仍保持真实硬件 disabled，训练入口没有任何硬件 action client。

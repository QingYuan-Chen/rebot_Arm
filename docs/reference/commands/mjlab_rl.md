# mjlab GPU Reach 训练（可选实验线）

> 状态：EXPERIMENTAL；这条线与 CPU MuJoCo 基准和 ROS 仿真后端隔离。

## 运行边界

该入口使用 `mjlab + MuJoCo Warp + PyTorch/RSL-RL` 在 GPU 上运行并行 Reach
环境。它不启动 ROS，不读取电机 SDK，不连接真实机械臂，也不会修改现有 CPU
mjlab 训练入口。

当前任务显式使用 `robot.xml` 已有的六个 torque actuator。它与 CPU Reach
环境的关节位置增量 action 不是同一个控制契约；训练结果必须先回到 CPU
MuJoCo 用同一目标集复评，才能进入后续 sim-to-real 分析。

## 隔离安装

当前 mjlab 版本需要单独的 Python 环境、Linux/NVIDIA GPU，并建议 CUDA 12.4
或更新版本。该环境同时提供 CPU MuJoCo 3.11 正确性基准和 GPU Warp/RSL-RL 训练依赖。

```bash
cd /home/huangbin/robotarm_ros2
uv venv third_party/rebotarm_mjlab_venv --python 3.12
uv pip install --python third_party/rebotarm_mjlab_venv/bin/python \
  --extra-index-url https://download.pytorch.org/whl/cu130 \
  "mjlab[cu130]==1.6.0"
third_party/rebotarm_mjlab_venv/bin/python -m pip install -e src/rebotarm_simulation --no-deps
colcon build --packages-select rebotarm_simulation --symlink-install
```

The editable install registers this package's `mjlab.tasks` entry point inside
the isolated environment. The `colcon` build is still required for the normal
ROS package/install verification, but the mjlab interpreter discovers the task
from its own Python environment.

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
  -m rebotarm_simulation.mjlab_paired_eval \
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

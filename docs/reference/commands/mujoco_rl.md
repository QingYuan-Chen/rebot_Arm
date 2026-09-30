# MuJoCo / Gymnasium 强化学习上手（Ubuntu 24.04）

> 状态：REFERENCE；类型：仿真/RL 操作与验收边界；适用范围：MuJoCo/Gymnasium；当前不连接真实机械臂。

## 适用范围与安全边界

提供纯 headless 的 `RebotArmReachEnv` 位姿到达任务、Gymnasium/SB3 接口检查、GPU PPO 训练和
独立种子评估。无需相机、ROS 节点、MoveIt 或真实机械臂。短训练通过只能证明链路可运行，
不代表策略收敛、抓取成功或真机验收。首版没有夹爪策略、图像观测和渲染。

## 前置条件与安装

在 Ubuntu 24.04 x86_64 / Python 3.12 下执行。本配置需要 NVIDIA GPU 与可用驱动；本机 RTX 4060 Laptop / driver 535.288.01 已实测。
下面创建隔离虚拟环境，不继承用户已有 ROS/视觉依赖；完整 ROS 工作区另见
[本机部署说明](../../setup/ubuntu_ros2_jazzy.md)。不复制他人的 venv/build/install。

```bash
sudo apt update
sudo apt install git python3-venv libgl1 libegl1 libglfw3
nvidia-smi  # 先确认 NVIDIA 驱动和 GPU 可见
git clone https://github.com/huangbinai/robotarm_ros2.git
cd robotarm_ros2
python3 -m venv third_party/rebotarm_mujoco_venv
third_party/rebotarm_mujoco_venv/bin/python -m pip install -r requirements-rl.txt
```

`requirements-rl.txt` 包含物理依赖，固定 MuJoCo 3.3.0、NumPy 1.26.4、
Gymnasium 1.2.3、imageio 2.37.4、Stable-Baselines3 2.8.0、PyTorch 2.5.1+cu121。
从 PyPI 和 PyTorch 官方 CUDA 12.1 索引下载 wheel。PyTorch wheel 随带所需 CUDA 运行库；
此流程不要求单独安装 CUDA Toolkit，但必须有兼容的 NVIDIA 驱动。
这是关键依赖版本基线，不是全部传递依赖的哈希锁文件。升级后须重新验证。
现有 MuJoCo venv 用户只需执行同一条 `pip install -r requirements-rl.txt`。
训练依赖不强制加入所有 ROS 节点的依赖。

源码方式运行时，在每个新终端的仓库根目录设置：

```bash
export REBOTARM_MUJOCO_PYTHON="$PWD/third_party/rebotarm_mujoco_venv/bin/python"
export PYTHONPATH="$PWD/src/rebotarm_simulation${PYTHONPATH:+:$PYTHONPATH}"
"$REBOTARM_MUJOCO_PYTHON" -m pip check
"$REBOTARM_MUJOCO_PYTHON" -c "import torch; print(torch.__version__, torch.version.cuda, torch.cuda.is_available(), torch.cuda.get_device_name(0) if torch.cuda.is_available() else None)"
"$REBOTARM_MUJOCO_PYTHON" -m rebotarm_simulation.mujoco_health --skip-renderer
"$REBOTARM_MUJOCO_PYTHON" -m rebotarm_simulation.rl_reach check
```

`check` 运行 Gymnasium 和 SB3 两套检查器；`train`/`eval` 显式使用 CUDA，GPU 不可用即失败。无限观测范围警告是当前接口的已知提示；
实际观测仍检查有限值。现有 `--system-site-packages` venv 可能继承其它应用的依赖冲突，
应与新建隔离环境的检查结果区分，不要为本任务修改视觉/OCR 环境。

## 执行命令：训练与评估

```bash
# 仅验证采样、更新参数、保存和加载链路，实际采样按 256 步批次向上取整。
"$REBOTARM_MUJOCO_PYTHON" -m rebotarm_simulation.rl_reach train --steps 512 --model runs/reach/smoke.zip
"$REBOTARM_MUJOCO_PYTHON" -m rebotarm_simulation.rl_reach eval --model runs/reach/smoke.zip --seed 10000 --episodes 5

# 首次较长实验；步数是实验预算，不保证收敛。
"$REBOTARM_MUJOCO_PYTHON" -m rebotarm_simulation.rl_reach train --curriculum-stage fixed --steps 500000 --seed 7 --model runs/reach/fixed_seed7.zip
"$REBOTARM_MUJOCO_PYTHON" -m rebotarm_simulation.rl_reach eval --curriculum-stage fixed --model runs/reach/fixed_seed7.zip --seed 10000 --episodes 100

# 固定目标稳定后，再切换到每回合随机的局部目标。
"$REBOTARM_MUJOCO_PYTHON" -m rebotarm_simulation.rl_reach train --curriculum-stage local --steps 1000000 --seed 7 --model runs/reach/local_seed7.zip
```

当前 MuJoCo 物理步进仍在 CPU，PPO MLP 网络参数更新在 GPU。SB3 会提示小型 MLP PPO 在 GPU 上
利用率可能很低，甚至比 CPU 慢；GPU 版安装和 CUDA 执行并不保证此任务加速。

这里的“物理在 CPU”是当前 `mujoco.mj_step()` 后端的实现边界，不是 CUDA 安装失败。
MuJoCo 另有基于 JAX 的 MJX 后端，可把批量仿真放到 GPU/TPU，但需要迁移模型和数据结构、
控制器接口，并重新验证接触与数值行为；不能给现有 `RebotArmMujoco` 加一个开关就切换。
当前也提供下文的实验性 MJX 批量后端；它和 CPU Reach 使用相同的专用场景。

模型路径请以 `.zip` 结尾，已有文件拒绝覆盖；每次实验换路径。评估输出 success_rate、
collision_rate、mean_final_distance_m 和 mean_final_orientation_error_deg；当前 Reach 关闭碰撞，
所以 collision_rate 恒为 0，不是避障证据。评估种子与训练种子分开。
模型为本机输出，不提交 Git。
保留命令、种子、Git commit 和 `pip freeze`，才能追溯实验。

课程顺序是 `fixed → local`：固定阶段只验证策略是否能学会一个稳定目标，局部阶段再扩大到
home 附近随机目标。达到固定目标的稳定成功率后再切换；不要把固定目标的结果当作随机目标泛化结果。

新训练每 25,000 步保存检查点，并用独立环境的种子 10000 起连续 20 回合做确定性评估。
优先按成功率、同成功率时按最终位置误差选择 `<model名>_best.zip`；训练末尾模型仍保存为
`--model` 路径。对应检查点在 `<model名>_checkpoints/`。可用 `--eval-freq` 和
`--eval-episodes` 调整。中途终端显示的训练回合统计并非固定评估集成功率；
应使用相同阶段、相同评估种子比较 best 与 final，另用未参与选模的种子做最终检验。
这项保存机制只对启动新脚本的训练生效，已在运行中的 Python 进程不会自动获得检查点。

## 环境契约与输入输出

### 用 MuJoCo Viewer 看已训练策略

在 Ubuntu 桌面终端执行（需要可用 DISPLAY 和 OpenGL）：

```bash
cd /home/a/project/rebot_Arm
source tools/source_local_environment.bash
"$REBOTARM_MUJOCO_PYTHON" -m rebotarm_simulation.rl_reach_viewer \
  --model runs/reach/mjx_reach_only_fixed_seed7.zip \
  --curriculum-stage fixed --episodes 5 --speed 0.25
```

同一策略在单个 CPU MuJoCo Reach 环境中执行，策略推理仍使用 CUDA；这是 CPU 后端
可视化复评，不是显示全部 128 个 MJX 环境。黄色球表示目标 TCP，红/绿/蓝轴表示
目标 X/Y/Z 方向，均为无物理作用的绘制标记。默认在每回合开始和结束暂停 1 秒，
`--speed 0.25` 为四分之一速度，`--speed 1` 为实时；不修改物理步长或策略。
关闭窗口或 Ctrl+C 可退出，终端逐回合输出成功与误差，不更新模型。

`episodes` 是回合数：每回合 reset 到 home，执行策略，达到成功条件或 5 秒超时后结束。
观看可用 `--episodes 1`，不写默认观看 5 回合；数值 `eval` 的默认回合数由对应入口决定
（MJX 默认100，CPU默认20）。固定目标确定性评估的回合通常重复相同轨迹，不能证明
随机目标泛化；local 阶段多回合才用于覆盖不同目标。

| 项目 | 定义 |
| --- | --- |
| 起始状态 | `reach_scene.xml` 的 home 关键帧；夹爪保持初始开口；没有桌子、瓶子和地面 |
| 目标位姿 | `fixed` 阶段使用一个稳定的 home 附近 FK 目标；CPU/MJX `local` 阶段在 home 各关节 ±0.12 rad 内采样构型，位置距起点 2–10 cm；两后端分布规则相同，但随机数序列不逐项相同 |
| 动作 | float32 六维 [-1,1]，对应 joint1..joint6 目标增量，每步最多 0.01 rad |
| 局部限制 | 目标角在 home ±0.25 rad 内，再由物理 API 裁剪到模型关节限位 |
| 观测 | 24 维：q-home(6)、dq(6)、目标位置误差(3)、目标姿态旋转向量(3)、控制目标-home(6) |
| 时间 | 物理步 2 ms；策略每 10 个物理步决策一次，50 Hz |
| 奖励 | TCP 四个 keypoint 的平均误差、指数精度项、误差进展项、动作 L2、动作变化率 L2，另加成功奖励 |
| 成功 | 位置距离 <1 cm、姿态误差 <3°、所有关节速度绝对值 <0.1 rad/s，连续10次决策（0.2 s） |
| 终止 | 达到成功条件；超出时长则截断 |
| 截断 | 默认250个策略步（5 s）；回合结束后必须 reset |

关节构型可达不保证全路径可达。CPU 与 MJX Reach 均关闭全部碰撞，只用于无接触位姿训练；
成功率不能证明避障、自碰撞安全或真机可执行。姿态误差是两个 XYZW 单位四元数的最短旋转角，不替代 MoveIt 检查，也不是实机碰撞保护。低层 PI 控制器内部积分和成功保持计数
未全部包含在观测中，首版属于部分可观测任务，不宣称严格完整 Markov 状态。
reset 用 Gymnasium `np_random` 采样；同版本同种子可复现目标与动作响应。

## 功能说明

它是本项目已有的 Python 物理后端类，源码为
`src/rebotarm_simulation/rebotarm_simulation/mujoco_sim.py`。
它加载 `models/rebotarm/scene.xml` 和 robot.xml，维护 MuJoCo 模型/运行数据，
执行仿真电机控制律（位置/速度串级 PI、hold、重力补偿），并提供：

- `reset_home()`：恢复 home、同步控制目标及控制器状态。
- `set_joint_position_targets()`：设置六轴位置目标（rad），不是瞬移。
- `set_gripper_width()`：设置夹爪开口目标（m）。
- `step(n)`：真正推进 n 个物理步。
- `get_state()` / `get_contacts()`：读取关节、TCP、物体和接触。
- `save_state()` / `restore_state()`：同模型实例内保存/恢复物理和控制器状态。

它不负责奖励、回合和算法更新。新的 `gym_reach.py` 在它上面定义 Reach 任务，
`rl_reach.py` 用 Stable-Baselines3 PPO 学习策略。三层均不发送真实 ROS 动作。
仿真控制参数是固件参考模型，不等于全部真实动力学/接触参数已标定。

## 验证与维护

```bash
# 专项测试：pytest 是开发工具，不是运行依赖。
"$REBOTARM_MUJOCO_PYTHON" -m pip install pytest
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 "$REBOTARM_MUJOCO_PYTHON" -m pytest tests/test_gym_reach.py -q
```

ROS 用户修改仿真源码后还需用系统 Python 重建仿真包并重新 source overlay：

```bash
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 -m colcon build --base-paths src --packages-select rebotarm_simulation --symlink-install
source install/setup.bash
```

参考：[Gymnasium环境](https://gymnasium.farama.org/api/env/)、
[SB3](https://stable-baselines3.readthedocs.io/)、[PyTorch](https://pytorch.org/get-started/locally/)。

### 本次软件验证（2026-09-28）

新建隔离 Python 3.12 venv 安装成功，pip check 无冲突；MuJoCo health 和
Gymnasium/SB3 checker 通过。Reach 专项7项通过，ROS overlay 下全量744 passed/15 skipped，
分层18项通过，simulation 重建通过。位姿版环境加入后，GPU 512 步 PPO 训练、保存和加载完成；
CUDA 版在另一个隔离 venv 中按 `requirements-rl.txt` 重装后 `pip check`、GPU 识别及两套环境检查也通过。
GPU 版已在本机 RTX 4060 上完成 512 步 PPO 训练、保存与加载；独立种子 10000 起 5 回合
成功率 0%、碰撞率 0%、平均最终距离 0.24631 m。它只证明训练链路可运行，尚未学会位姿 Reach，
更不是抓取或真机验收。无硬件操作。

## GPU 批量 MJX Reach（实验性）

标准单环境 MuJoCo 用于相同 Reach 场景的 CPU 数值复评。MJX Reach 在同一个虚拟环境中额外安装
JAX/CUDA，不需要再建第二个虚拟环境。驱动 535 系列使用本仓库固定的 CUDA 12 依赖：

```bash
cd /home/a/project/rebot_Arm
third_party/rebotarm_mujoco_venv/bin/python -m pip install -r requirements-mjx.txt
source tools/source_local_environment.bash
"$REBOTARM_MUJOCO_PYTHON" -c 'import jax, torch; print(jax.devices(), torch.cuda.is_available())'
"$REBOTARM_MUJOCO_PYTHON" -m rebotarm_simulation.rl_reach_mjx check --num-envs 128
"$REBOTARM_MUJOCO_PYTHON" -m rebotarm_simulation.rl_reach_mjx train --curriculum-stage fixed --num-envs 128 --steps 500000 --seed 7 --model runs/reach/mjx_reach_only_fixed_seed7.zip
"$REBOTARM_MUJOCO_PYTHON" -m rebotarm_simulation.rl_reach_mjx eval --curriculum-stage fixed --num-envs 128 --model runs/reach/mjx_reach_only_fixed_seed7_best.zip --seed 20000 --episodes 100
```

`MJXReachVecEnv` 默认把 128 个环境同时放到 GPU，保留六维动作、24 维观测、50 Hz 策略、
500 Hz 物理与当前 PI 控制参数。CPU 与 MJX 都加载无桌面物体的 `reach_scene.xml`，
并在各自模型实例中关闭所有接触；原 `scene.xml` 留给其他仿真任务。
旧模型是不同场景/碰撞定义下训练的，指标不能与新任务直接对比，应重新训练。
可用 CPU MuJoCo 对候选策略做相同无接触 Reach 任务的独立复评：

```bash
"$REBOTARM_MUJOCO_PYTHON" -m rebotarm_simulation.rl_reach eval --curriculum-stage fixed --model runs/reach/mjx_reach_only_fixed_seed7_best.zip --seed 20000 --episodes 100
```

MJX 第一次运行会 JIT 编译，可能花几十秒；之后才比较稳态吞吐。固定目标下的模型
默认每 25,000 步保存检查点，用 20 回合独立评估选 best；训练结束还保存 final。
MJX 与原 MuJoCo 的动力学结果是近似而非逐位一致，目前仅验证了固定目标下零动作和
50 步随机动作的关节/位置误差量级。`local` 阶段已与 CPU Reach 对齐：从 home 各关节
±0.12 rad 均匀采样，取前 100 次中首个关节合法、FK 位置距 home 2–10 cm 的目标；
极少数 100 次均失败时，MJX 退回已知合法的 fixed 目标，CPU 则报错。
两后端随机数实现不同，不能要求同一个 seed 逐个目标相同。

### 第二阶段：附近随机目标

以第一阶段的 fixed 模型为初始化，使用新模型路径保存 local 训练：

```bash
cd /home/a/project/rebot_Arm
source tools/source_local_environment.bash
"$REBOTARM_MUJOCO_PYTHON" -m rebotarm_simulation.rl_reach_mjx train \
  --curriculum-stage local --num-envs 128 --steps 300000 --seed 7 \
  --eval-freq 50000 \
  --init-model runs/reach/mjx_reach_only_fixed_seed7.zip \
  --model runs/reach/mjx_reach_local_stage2_seed7.zip
```

`--init-model` 只用于训练开始时加载权重和优化器状态，不覆盖 fixed 模型；
`--steps` 是本轮增加的策略交互步数，SB3 会按 `128×256` 的 rollout 大小向上取整。
检查点、best、final 均使用新模型名。评估时，每个并行环境预先分配固定回合数，
避免只取最先结束的回合而偏向较容易的目标。先用 20 个独立目标选 best，训练后
再用另一组 100 个目标确认，并用标准 MuJoCo CPU 后端复评。

```bash
# 完成训练后，用未参与训练内选模的目标评估 final：
"$REBOTARM_MUJOCO_PYTHON" -m rebotarm_simulation.rl_reach_mjx eval \
  --curriculum-stage local --num-envs 128 --episodes 100 --seed 20000 \
  --model runs/reach/mjx_reach_local_stage2_seed7.zip

# 同模型在标准 MuJoCo CPU 物理下复评：
"$REBOTARM_MUJOCO_PYTHON" -m rebotarm_simulation.rl_reach eval \
  --curriculum-stage local --episodes 100 --seed 20000 \
  --model runs/reach/mjx_reach_local_stage2_seed7.zip

# 观看其中 5 个随机目标：
"$REBOTARM_MUJOCO_PYTHON" -m rebotarm_simulation.rl_reach_viewer \
  --curriculum-stage local --episodes 5 --seed 20000 \
  --model runs/reach/mjx_reach_local_stage2_seed7.zip
```

2026-09-29 首轮 local 训练实际执行 327,680 步。独立评估 100 回合：MJX
初始 fixed 模型 / local best / local final 成功率分别为 1% / 2% / 2%，平均最终
位置误差 50.1 / 40.8 / 40.2 mm；标准 MuJoCo CPU 对另一组按相同规则采样的
100 目标，三者成功率均为 0%，位置误差 58.5 / 47.2 / 46.2 mm。两种后端的
同 seed 不产生相同的逐项目标；不能把上述差异全部归因于物理误差。位置改善同时
姿态误差变大，随机目标任务尚未学会。20 回合的训练内 best 选模有较大采样噪声。

### 同目标诊断与分级课程

`rl_reach_diagnose` 先用 CPU Reach 生成并保存 `goal_q`，再让 CPU MuJoCo 与
MJX 执行同一批目标。逐目标报告位置、姿态、最大关节速度和各成功门槛：

```bash
"$REBOTARM_MUJOCO_PYTHON" -m rebotarm_simulation.rl_reach_diagnose \
  --model runs/reach/mjx_reach_local_stage2_seed7.zip \
  --curriculum-stage local --episodes 100 --seed 20000 --backend both \
  --output runs/reach/mjx_reach_local_stage2_paired_diagnosis.json
```

同目标测试中，上一轮 local final 在两后端成功率均为 0%，最终位置合格率 2%，
姿态合格率 0%；两后端平均误差基本一致。因此优先改训练信号。

新增 `local_easy`：关节在 home ±0.06 rad 采样，目标 TCP 距 home 2–5 cm，
初始姿态误差不超过 8°。原 `local` 2–10 cm 范围和旧 `baseline` 奖励仍可选。
`pose_v2` 奖励分别使用 `exp(-位置误差/0.03)` 和
`0.5×exp(-姿态角误差rad/0.1)`，其余 progress、动作惩罚、成功奖励不变。

从上一轮 local final 初始化 `local_easy + pose_v2`，实际训练 327,680 步；
在未参与选模的同 100 目标上，起始模型 / best / final 的 CPU 与 MJX 成功率
分别为 15% / 45% / 47%。final 平均姿态误差约 0.99°，姿态门槛 100% 通过；
位置门槛 47% 通过。把同模型放回完整 `local` 目标范围时，两后端成功率均为
5%、姿态门槛约 98%、位置门槛约 5%。较远随机目标的位置精度是下一瓶颈。

`pose_v3` 是位置精度对照：保留独立姿态项，把位置项改为
`exp(-位置误差/0.015)`，让学习信号集中在 1 cm 成功门槛附近。从 easy final
初始化并使用新的模型路径在完整 `local` 范围训练：

```bash
"$REBOTARM_MUJOCO_PYTHON" -m rebotarm_simulation.rl_reach_mjx train \
  --curriculum-stage local --reward-profile pose_v3 --num-envs 128 \
  --steps 300000 --seed 13 --eval-freq 50000 \
  --init-model runs/reach/mjx_reach_local_easy_pose_v2_seed11.zip \
  --model runs/reach/mjx_reach_local_pose_v3_seed13.zip
```

这些成功率只属于无碰撞纯仿真任务；用独立固定目标集选择模型，
不以 `rollout/success_rate` 代替确定性评估。

本轮 `pose_v3` 实际训练 327,680 步。未参与选模的同 100 个完整 `local` 目标上，
起始 easy final / 本轮 best / 本轮 final 成功率为 5% / 24% / 28%；CPU MuJoCo
与 MJX 分别执行同一批 `goal_q`，成功率完全一致。final 平均最终位置误差约
21.4 mm、姿态误差约 0.92°；100% 达到姿态门槛，28% 达到 1 cm 位置门槛。
结果仍未达到随机目标任务的可用成功率。报告保存在
`runs/reach/mjx_reach_local_pose_v3_seed13_paired_comparison.json`，final 模型为
`runs/reach/mjx_reach_local_pose_v3_seed13.zip`。下一步应针对剩余位置误差分析
目标距离分桶、动作目标限幅及回合终点行为，再决定进一步训练参数；不要只凭增加
步数或训练内 20 回合的 success_rate 宣称收敛。

本机 RTX 4060 Laptop（8 GB）已完成 128 环境检查和 32,768 步短训练。一次带
25,000 步评估的短训练报告约 554 policy steps/s（含 JIT 冷启动与评估）；按进程每
0.25 秒采样 `nvidia-smi`，显存最高读到 388 MiB。采样不能保证抓到瞬时峰值。
需设置 `XLA_PYTHON_CLIENT_PREALLOCATE=false`，脚本会在导入 JAX 前默认设置。
这验证可运行和短时吞吐，尚不证明长时间训练不会显存溢出或策略收敛。
旧桌面场景下记录的吞吐与本次批量、JIT 和任务定义不同，不作为严格速度对照。
首次 JIT 编译约 30 秒，短于几万步的任务可能得不到总耗时收益。

### pose_v3 位置失败分析（2026-09-29）

对未参与选模的同一组 100 个完整 local 目标，额外记录 CPU MuJoCo 中每步的
末端距离、动作、关节控制目标、实际关节角、速度和限幅次数；失败回合从 250 步
延长到 500 步作诊断。原模型与训练参数未修改。原始数据：
`runs/reach/mjx_reach_local_pose_v3_position_analysis.json`。

| 初始目标距离 | 目标数 | 250 步成功数 | 平均最终误差 |
| --- | ---: | ---: | ---: |
| 2–3.5 cm | 26 | 9 | 14.4 mm |
| 3.5–5 cm | 27 | 9 | 18.6 mm |
| 5–7 cm | 29 | 9 | 21.8 mm |
| 7–10 cm | 18 | 1 | 35.1 mm |

按目标相对 home 的 Z 方向分组：向下 53 目标成功 28 个，平均最终误差
11.8 mm；向上 47 目标成功 0 个，平均最终误差 32.2 mm。两组平均初始
目标距离分别约 48.5 和 52.7 mm，故不能只用“向上组更远”解释差异。
按 X 方向，X 减小的 52 目标成功 9 个，X 增大的 48 目标成功 19 个；
Y 正负组差异较小。分组只是相关性，不等于单独证明某个坐标方向的动力学原因。

100 个目标在 250 步成功 28 个，延长到 500 步仍为 28 个；失败目标延长阶段
平均位置误差没有继续下降。只有 3 个目标触及 home±0.25 rad 的策略控制目标限幅，
全部发生在关节 4；限幅解释不了大多数失败。向上目标结束时关节速度很小，
实际关节角跟随了策略的控制目标，但该目标与采样生成目标位姿的关节构型有较大差异，
尤其关节 3/4。因而当前更需要分析策略的方向映射与训练目标覆盖，而不是仅放宽
回合时长。额外使用“直接命令到采样关节构型”的底层控制对照时，仅 39/100
在 5 秒后位置误差小于 1 cm；该对照不是策略可达性的上界，因为策略有 12 个
目标在这一对照未达标时仍成功。它提示底层控制对部分目标有跟踪误差，不能把
全部剩余问题归于策略。此分析仍是无碰撞仿真证据。

### 自适应初始距离奖励试验（2026-09-29）

`adaptive_position` 在每回合 reset 时固定初始 TCP 位置距离 `d0`，定义
`s = clip(0.4×d0, 0.015, 0.04)`（单位米）；每步位置项为
`0.5×exp(-当前距离/s) + 0.5×exp(-当前距离/0.015)`。前半项让远目标在
初始阶段仍有可分辨的奖励梯度，后半项保留靠近 1 cm 成功门槛的精度信号。
姿态项 `0.5×exp(-姿态角rad/0.1)`、四点误差 progress、动作惩罚和成功奖励
沿用 pose_v3。`d0` 不随动作更新，也未加入观测；初始 home 固定，初始目标
位置误差已在 24 维观测中，因此策略可从观测推知 d0。旧 `pose_v3` 保留。

从 pose_v3 final 初始化，在相同完整 local 目标分布训练 327,680 步、seed17，
使用另一模型路径 `runs/reach/mjx_reach_local_adaptive_seed17.zip`。训练内
20 目标最终检查为 60% 成功。未参与选模的同 100 个 `goal_q` 上：

| 模型 | CPU/MJX 成功率 | 平均最终位置误差 | 平均姿态误差 |
| --- | ---: | ---: | ---: |
| 起始 pose_v3 final | 28% | 21.4 mm | 0.92° |
| adaptive best | 52% | 13.1 mm | 0.90° |
| adaptive final | 54% | 12.3 mm | 0.88° |

按原目标方向，向下 53 个目标由 28 个成功提高到 50 个，向上 47 个由 0 个
提高到 4 个。初始距离≥7 cm 的 18 个目标由 1 个成功提高到 4 个。
相对起始模型有改善，但这一结果不能单独归因于奖励改动；向上与较远目标
仍有明显缺口。报告：
`runs/reach/mjx_reach_local_adaptive_seed17_paired_comparison.json`。

### 同步数旧奖励对照（2026-09-29）

从同一个 `pose_v3` final 模型初始化，固定 `local` 目标分布、128 个 MJX 环境、
seed 17、请求训练 300,000 步（两组都实际完成 327,680 步）、每 50,000 步
训练内评估一次。对照组只将 `--reward-profile` 设回 `pose_v3`：

```bash
source tools/source_local_environment.bash
"$REBOTARM_MUJOCO_PYTHON" -m rebotarm_simulation.rl_reach_mjx train \
  --curriculum-stage local --reward-profile pose_v3 --num-envs 128 \
  --steps 300000 --seed 17 --eval-freq 50000 \
  --init-model runs/reach/mjx_reach_local_pose_v3_seed13.zip \
  --model runs/reach/mjx_reach_local_pose_v3_control_seed17.zip
```

两组在相同的独立 100 个 `goal_q`（seed 20000）上以确定性动作评估，
CPU MuJoCo 与 MJX 分别运行；报告中的目标关节数组已核对完全相同。
比较 final 是主要结论，best 是训练内 20 目标选模的辅助结果：

| 模型 | CPU 成功数 / 100 | MJX 成功数 / 100 | CPU 平均最终位置误差 | CPU 平均姿态误差 |
| --- | ---: | ---: | ---: | ---: |
| 共同起始 `pose_v3` final | 28 | 28 | 21.4 mm | 0.92° |
| 旧奖励 `pose_v3` best | 56 | 56 | 12.0 mm | 0.85° |
| 自适应奖励 best | 52 | 52 | 13.1 mm | 0.90° |
| 旧奖励 `pose_v3` final | **61** | **60** | **11.1 mm** | **0.82°** |
| 自适应奖励 final | 54 | 54 | 12.3 mm | 0.88° |

同目标逐项比较 CPU final：自适应奖励成功的 54 个目标，旧奖励全部成功；
旧奖励另成功 7 个。向下 53 个目标旧奖励/自适应分别成功 53/50，向上
47 个分别成功 8/4；初始距离≥7 cm 的 18 个分别成功 7/4。旧奖励的
这些分组平均最终位置误差也更低。CPU 与 MJX 的旧奖励 final 仅有一个
目标跨越成功门槛，属于后端数值差异；主要比较采用 CPU MuJoCo 结果。

因此，在这一次匹配初始化、seed 和训练步数的对照中，**没有证据表明
自适应位置奖励优于继续使用旧奖励**；观察到的改善主要可以由继续训练解释。
这只是一组训练 seed 和一批评估目标，不能推断所有随机种子上的优劣。
对照报告：`runs/reach/mjx_reach_local_pose_v3_control_seed17_paired_comparison.json`；
对应模型：`runs/reach/mjx_reach_local_pose_v3_control_seed17.zip` 与 `_best.zip`。

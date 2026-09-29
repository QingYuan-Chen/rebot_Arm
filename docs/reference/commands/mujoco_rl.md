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
"$REBOTARM_MUJOCO_PYTHON" -m rebotarm_simulation.rl_reach train --steps 100000 --seed 7 --model runs/reach/ppo.zip
"$REBOTARM_MUJOCO_PYTHON" -m rebotarm_simulation.rl_reach eval --model runs/reach/ppo.zip --seed 10000 --episodes 100
```

当前 MuJoCo 物理步进仍在 CPU，PPO MLP 网络参数更新在 GPU。SB3 会提示小型 MLP PPO 在 GPU 上
利用率可能很低，甚至比 CPU 慢；GPU 版安装和 CUDA 执行并不保证此任务加速。

模型路径请以 `.zip` 结尾，已有文件拒绝覆盖；每次实验换路径。评估输出 success_rate、
collision_rate、mean_final_distance_m；评估种子与训练种子分开。模型为本机输出，不提交 Git。
保留命令、种子、Git commit 和 `pip freeze`，才能追溯实验。

## 环境契约与输入输出

| 项目 | 定义 |
| --- | --- |
| 起始状态 | canonical `scene.xml` 的 home 关键帧；夹爪保持初始开口 |
| 目标位姿 | home 各关节 ±0.12 rad 内采样构型，FK 求 TCP 位置和 XYZW 姿态，位置距起点 2–10 cm；拒绝机器人接触 |
| 动作 | float32 六维 [-1,1]，对应 joint1..joint6 目标增量，每步最多 0.01 rad |
| 局部限制 | 目标角在 home ±0.25 rad 内，再由物理 API 裁剪到模型关节限位 |
| 观测 | 32 维：q(6)、dq(6)、控制目标角(6)、TCP位置/姿态(3+4)、目标位置/姿态(3+4) |
| 时间 | 物理步 2 ms；策略每 10 个物理步决策一次，50 Hz；碰撞可提前结束物理循环 |
| 奖励 | -10 × 位置距离(m) - 0.25 × 姿态误差(rad) - 0.001 × 动作平方和 + 成功5 - 碰撞5 |
| 成功 | 位置距离 <1 cm、姿态误差 <3°、所有关节速度绝对值 <0.1 rad/s，连续10次决策（0.2 s） |
| 终止 | 成功或机器人接触；允许瓶子与桌面/地面接触 |
| 截断 | 默认250个策略步（5 s）；回合结束后必须 reset |

关节构型可达不保证全路径可达。接触只依据 MuJoCo 已启用的碰撞几何和过滤；
姿态误差是两个 XYZW 单位四元数的最短旋转角，不替代 MoveIt 检查，也不是实机碰撞保护。低层 PI 控制器内部积分、瓶子状态和成功保持计数
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

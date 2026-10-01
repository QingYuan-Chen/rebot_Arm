# MuJoCo / Gymnasium Reach 基线

> 状态：REFERENCE；适用范围：保留的 CPU 物理 Reach 环境与可选 SB3/PPO 工具。

## 当前范围

Reach 是无接触的六轴末端位姿到达任务，不连接 ROS、相机或真实机械臂。
原 MJX 批量训练与 CPU/MJX 对比工具已剥离；后续训练框架采用 MJLab，尚未接入。
保留这套环境用于目标采样、动作/观测契约和迁移后的行为对照。
物理步进在 CPU；现有 PPO 训练/推理显式要求 CUDA。
本地已验证 Gymnasium reset/step 与契约，未验收 SB3/CUDA 训练质量或策略收敛。

## 可选依赖与环境检查

从主目录操作；只在需要保留的 SB3 工具时安装这份独立依赖。
安装步骤不属于机器人启动命令，不会修改电机参数。

```bash
cd /home/a/project/rebot_Arm
python3 -m venv third_party/rebotarm_rl_venv
python3 scripts/install_python_dependencies.py rl --python third_party/rebotarm_rl_venv/bin/python
export REBOTARM_MUJOCO_PYTHON="$PWD/third_party/rebotarm_rl_venv/bin/python"
export PYTHONPATH="$PWD/src/rebotarm_simulation${PYTHONPATH:+:$PYTHONPATH}"
"$REBOTARM_MUJOCO_PYTHON" -m pip check
"$REBOTARM_MUJOCO_PYTHON" -m rebotarm_simulation.mujoco_health --skip-renderer
"$REBOTARM_MUJOCO_PYTHON" -m rebotarm_simulation.rl_reach check
```

`docs/setup/dependencies/rl.md` 的依赖块相对包含 `-r mujoco.md`，
由安装工具解析后交给 pip，包含固定的 MuJoCo、
Gymnasium、SB3 和 CUDA PyTorch 版本，与视觉/GraspNet 环境隔离。
`check` 使用 Gymnasium 和 SB3 检查器；无限观测范围提示不表示真实观测可以非有限。

## 保留的训练与评估入口

```bash
# fixed 只验证单个 home 附近的目标；预算不保证收敛。
"$REBOTARM_MUJOCO_PYTHON" -m rebotarm_simulation.rl_reach train \
  --curriculum-stage fixed --steps 500000 --seed 7 --model runs/reach/fixed_seed7.zip
"$REBOTARM_MUJOCO_PYTHON" -m rebotarm_simulation.rl_reach eval \
  --curriculum-stage fixed --model runs/reach/fixed_seed7_best.zip --seed 20000 --episodes 100

# local 在 home 附近采样可达构型，需要独立评估泛化。
"$REBOTARM_MUJOCO_PYTHON" -m rebotarm_simulation.rl_reach train \
  --curriculum-stage local --steps 1000000 --seed 7 --model runs/reach/local_seed7.zip
```

已有模型/检查点目录拒绝覆盖。默认每 25,000 个采样步保存检查点，用独立环境
seed 10000 起的 20 回合选 best；最终模型另存。用未参与选模的种子做最终评估。
报告成功率、最终位置和姿态误差；当前 collision_rate 恒为零，不构成避障证据。
当前单环境物理采样在 CPU，小型 PPO 网络使用 GPU 不保证训练加速。

## Viewer 复评

在有 DISPLAY/OpenGL 的桌面终端中使用上述解释器和 PYTHONPATH：

```bash
"$REBOTARM_MUJOCO_PYTHON" -m rebotarm_simulation.rl_reach_viewer \
  --model runs/reach/fixed_seed7_best.zip --curriculum-stage fixed --episodes 5 --speed 0.25
```

黄色球和坐标轴表示目标位姿；关闭窗口或 Ctrl+C 退出，不更新策略。

## 环境契约

| 项目 | 定义 |
| --- | --- |
| 场景 | `reach_scene.xml`；无桌子、瓶子和地面，夹爪保持初始状态 |
| 目标 | `fixed` 为 home 附近 FK 目标；`local` 在各关节 home ±0.12 rad 内采样，TCP 距起点 2–10 cm |
| 动作 | 六维 float32 [-1,1]；joint1..joint6 目标每步增量最多 0.01 rad |
| 局部限制 | 控制目标限制在 home ±0.25 rad，再按模型限位裁剪 |
| 观测 | 24 维：q-home(6)、dq(6)、目标位置误差(3)、目标姿态旋转向量(3)、控制目标-home(6) |
| 周期 | 物理步 2 ms；策略周期 20 ms，即 50 Hz |
| 成功 | 位置 <1 cm、姿态 <3°、各关节速度绝对值 <0.1 rad/s，连续10次决策 |
| 结束 | 成功则终止；默认250步/5 s则截断；随后必须 reset |

接触和碰撞关闭，成功率不证明抓取、自碰撞安全或真机可执行。
低层 PI 积分等内部状态未全部进入观测，不宣称严格完整 Markov 状态。
后续 MJLab 迁移需明确关节顺序、单位、动作缩放、控制周期、观测与成功门；
保持相同评估目标，分别比较误差、稳定性、成功率和实际训练时间。

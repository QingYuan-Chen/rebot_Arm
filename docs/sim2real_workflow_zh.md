# reBotArm Sim2Real / Real2Sim 仿真工作流

## 当前边界

本工作流只连接 MuJoCo，不连接机械臂实机，不启动 `rebotarmcontroller`，
不打开 CAN 或串口设备。

当前已经实现：

- 按 YAML profile 和 seed 对质量、阻尼、摩擦、力矩比例、控制延迟和噪声采样；
- 运行 Reach 随机动作并记录标准 JSONL 轨迹；
- 使用相同 seed、profile 和动作进行确定性回放；
- 比较关节位置、速度、末端位置、夹爪宽度、执行器力矩和接触力；
- 检查关节限位、执行器力矩限位、接触穿透、非有限数据和同 seed 复现性；
- 输出机器可读 JSON 报告，失败时返回非零退出码。

当前没有进行强化学习训练，也没有采集实机数据。Real2Sim 参数辨识要等实机日志具备
时间戳、`q`、`dq`、目标值、电流或力矩、夹爪宽度或力、接触状态后再开始。

## 环境准备

在 Ubuntu VM 中执行：

```bash
cd /home/a/project/rebot_Arm
export PYTHONPATH="$PWD/src/rebotarm_simulation:${PYTHONPATH:-}"
mkdir -p logs/sim2real
```

如果已经执行过 `colcon build` 并 source 了工作区，也可以直接使用
`rebotarm_sim2real`；以下命令使用 `python3 -m`，不依赖是否重新安装入口。

## 1. 随机化运行并记录

```bash
python3 -m rebotarm_simulation.sim2real_cli rollout \
  --randomization-profile training_profile \
  --seed 7 \
  --steps 100 \
  --record logs/sim2real/seed-7-reference.jsonl \
  --report logs/sim2real/seed-7-rollout-report.json
```

验收：终端 JSON 中 `ok` 为 `true`，`safety.violation_count` 为 `0`，
并生成轨迹和报告两个文件。

## 2. 确定性回放

```bash
python3 -m rebotarm_simulation.sim2real_cli replay \
  logs/sim2real/seed-7-reference.jsonl \
  --randomization-profile training_profile \
  --seed 7 \
  --record logs/sim2real/seed-7-replay.jsonl \
  --report logs/sim2real/seed-7-replay-report.json
```

验收：`ok` 和 `comparison.ok` 为 `true`。同一 MuJoCo 版本、模型、profile、
seed 和动作下，各误差应为 `0` 或在设置的阈值内。

## 3. 单独比较两条轨迹

```bash
python3 -m rebotarm_simulation.sim2real_cli compare \
  logs/sim2real/seed-7-reference.jsonl \
  logs/sim2real/seed-7-replay.jsonl \
  --joint-position-max 0.001 \
  --joint-velocity-max 0.01 \
  --ee-position-max 0.001 \
  --gripper-width-max 0.001 \
  --actuator-torque-max 0.1 \
  --contact-force-max 1.0 \
  --report logs/sim2real/seed-7-compare-report.json
```

这些阈值表示两条轨迹之间允许的最大误差，不是机械臂自身的物理限位。

## 4. 批量安全与复现性验收

```bash
python3 -m rebotarm_simulation.sim2real_cli batch-check \
  --randomization-profile training_profile \
  --seed 7 \
  --episodes 20 \
  --steps 200 \
  --max-contact-penetration 0.01 \
  --log-dir logs/sim2real/batch \
  --report logs/sim2real/batch-report.json
```

验收：总报告 `ok=true`，每个 episode 的 `safety.ok=true`、
`replay_safety.ok=true`、`seed_reproducible=true`。

如果已经根据实际抓取任务确定了合理的接触力上限，可追加：

```bash
--max-contact-force 50
```

在没有标定依据前，不应把任意接触力数值写成实机安全结论。默认仅报告最大接触力，
接触穿透默认上限为 `0.01 m`。

## 失败定位

命令返回码：

- `0`：运行和验收通过；
- `1`：参数、文件、模型或运行时错误；
- `2`：命令完成，但安全检查或误差阈值未通过。

报告中常见 `kind`：

- `joint_limit`：关节位置超出 MJCF 声明限位；
- `actuator_torque`：执行器力矩超过 MJCF actuator 控制范围；
- `contact_force`：接触力超过显式给定阈值；
- `contact_penetration`：接触穿透超过阈值。

## Real2Sim 后续入口

拿到实机日志后，先把实机数据转换为同一 `TrajectorySample` JSONL schema，
其中 `source` 设置为 `real`。随后使用 `compare` 对齐仿真与实机轨迹，先辨识
控制延迟、阻尼、摩擦和力矩比例，再扩大随机化范围。没有实机日志时不进行参数拟合，
避免把无证据的参数当成真实机械臂动力学。

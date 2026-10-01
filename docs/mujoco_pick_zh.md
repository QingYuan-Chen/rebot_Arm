# reBotArm MuJoCo Pick 精细抓取环境

## 当前目标

当前实现的是可训练、可测试的 Pick 任务环境，不进行强化学习训练，也不连接实机。
环境使用现有 STL 机械臂、夹爪和方块模型，复用 POS_VEL、夹爪 MIT 仿真控制、
场景随机化和 Sim2Real 轨迹记录接口。

## 任务定义

动作是 7 维连续向量，输入范围为 `[-1, 1]`：

```text
[joint1, joint2, joint3, joint4, joint5, joint6, gripper]
```

前 6 维表示关节位置目标增量，第 7 维表示夹爪宽度增量。动作最终仍经过项目现有
POS_VEL 和夹爪控制器转换为 MuJoCo actuator 力矩或力。

主要观测包括：

- 六关节位置和速度；
- 夹爪宽度；
- 末端位置；
- 方块位置、姿态和相对末端位置；
- 抬升目标位置；
- 左右手指接触状态和接触力；
- 双指接触、接触法向夹角和 `force_closure_candidate` 力闭合候选；
- 接触数量、最大接触力和最大穿透深度。

## 阶段状态

环境明确报告以下 `stage`：

- `approach`：尚未进入有效接触；
- `contact`：接近方块或只有单指接触；
- `grasp`：双指接触法向近似相对，正在累计抓稳步数；
- `lift`：已经抓稳，正在抬升；
- `success`：达到抬升高度并持续稳定；
- `failure`：命中明确失败条件。

单纯“两根手指都碰到方块”不会直接判定成功。环境还要求两侧主要接触法向近似相对，
并持续满足抓稳步数，防止方块同时碰到两根手指或夹爪基座时被误判为稳定抓取。

## 成功与失败

默认成功条件：

```text
双指形成力闭合候选
+ 夹爪宽度不超过 0.065 m
+ ��续抓稳至少 5 个动作步
+ 方块相对初始稳定高度抬升至少 0.05 m
+ 抬升状态连续保持至少 5 个动作步
```

失败分类：

- `excessive_contact_force`：方块接触力超过上限；
- `excessive_penetration`：接触穿透超过上限；
- `cube_fell`：方块跌落到场景下方；
- `cube_out_of_workspace`：方块离开有效工作空间；
- `dropped_after_grasp`：已经抓稳后持续丢失双指接触；
- `none`：没有任务失败，可能仍在执行或到达评估时域。

`terminated` 只表示成功或明确失败，`truncated` 表示达到环境最大步数。

## Python API

```python
from rebotarm_simulation.mujoco_pick_env import RebotArmPickEnv

with RebotArmPickEnv() as env:
    obs, info = env.reset(seed=7)
    for _ in range(400):
        action = [0.0] * 7
        obs, reward, terminated, truncated, info = env.step(action)
        if terminated or truncated:
            break

print(info["stage"], info["failure_reason"], info["lift_height_m"])
```

## 无界面批量验收

Ubuntu VM 中执行：

```bash
cd /home/a/project/rebot_Arm
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH="$PWD/src/rebotarm_simulation:$PYTHONPATH"

python3 -m rebotarm_simulation.mujoco_pick_batch \
  --episodes 20 \
  --steps 400 \
  --seed 7 \
  --action-magnitude 0.25 \
  --randomization-profile training_profile \
  --log-dir /tmp/rebotarm-pick-logs
```

批量命令使用有界随机动作作为环境验收基线，不是抓取策略。报告字段含：

- `ok`：物理状态、关节限位、力矩、接触力和穿透安全检查是否通过；
- `success_rate`：该动作策略的任务成功率；
- `stage_counts`：各任务阶段停留步数；
- `failure_counts`：失败分类统计；
- `randomization`：每个 seed 的动力学随机化样本；
- `safety`：每条轨迹的安全检查明细。

随机策略 `success_rate=0` 不代表环境验收失败。只有 `ok=false` 才表示运行或安全合同失败。

基础 MuJoCo 总验收现在也包含 Pick 环境 smoke test：

```bash
python3 -m rebotarm_simulation.mujoco_acceptance --skip-renderer
```

## 当前限制与下一步

当前已完成 Pick 任务合同、观测、动作、奖励、成功判定、失败分类、随机化和批量安全验收。
尚未完成可稳定成功的确定性专家抓取轨迹，因此当前不能声称抓取任务已经由控制策略解决。

下一步应实现一个可重复成功的脚本专家或 IK 抓取基线，并用多 seed 验证任务可达性；
通过后再把同一环境部署到云服务器进行强化学习训练。

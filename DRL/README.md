# reBotArm 强化学习工作区

本目录集中管理 MJLab 环境、模型导出、评测、基线回放和远端发布包。
本机仅做导出、测试和回放；训练在独立的远端服务器执行。
首个强化学习任务已选定为 Reach-and-Hold（末端到达并稳定保持）。
已包含任务环境、远端训练入口、策略导出/回放、固定目标评测和传统控制基线。
这些是首版仿真配置，不是实机控制参数；运行与验收状态统一见
[项目状态](../docs/reference/project_status.md)，短训练通过不代表策略收敛。

## 首个任务：Reach-and-Hold

任务目标：机械臂末端到达目标区域后，降低运动速度，并连续保持在合格范围内。
仅瞬时经过目标不算成功；保持过程中离开合格范围或速度超限，连续保持计时应重新开始。
成功判定与累计奖励分别记录，不能只用训练奖励判断策略质量。

任务注册名 `RebotArm-Reach-Hold-v1`，无接触位置目标，不约束朝向。
TCP 为快照模型的 `ee_site`，即 `end_link` 的 `[-0.04,0,0] m`，使用基座坐标系。
夹爪固定在 40 mm 开口：任务加载时移除两手指自由度及对应执行器/传感器/耦合约束，
保留几何与质量；增加地面和不参与碰撞的绿色目标球。主项目模型和快照文件不改。

| 项目 | 首版设定 |
| --- | --- |
| 物理 / 策略周期 | 2 ms / 20 ms（50 Hz） |
| 动作 | 六轴输出裁剪至 ±1；每策略步将命令累加 `0.01 × action rad`，命令距位置限位留 0.05 rad |
| 控制层 | 每物理步执行 PD＋仿真模型偏置力矩补偿，前三轴限 ±27 Nm，后三轴限 ±7 Nm |
| PD 增益 | Kp `[100,100,100,40,40,20]`，Kd `[4,4,4,1.5,1.5,0.5]`；模型原有被动阻尼仍保留 |
| 成功 | 位置误差 ≤1 cm、TCP 速度 ≤0.02 m/s、各轴速度 ≤0.05 rad/s，连续50步（1 s） |
| 终止 | 碰撞、关节超范围超过0.005 rad、轴速度超过2 rad/s、数值失效；10 s超时独立统计 |
| 观测 | 34维：q(6)、dq(6)、目标−TCP(3)、TCP速度(3)、命令−q(6)、前次裁剪动作(6)、目标位置(3)、连续保持比例(1) |
| PPO | 256并行环境、每次32步、初始2000轮、种子42、actor/critic各128×128、学习率3e-4；未调优 |

`reach_model.py` 是任务常量来源；训练保存完整环境/网络配置和部署契约。
导出包含 actor 的观测归一化；动作积分和 PD 仍由环境执行，ONNX 不直接输出力矩。
奖励包含宽/窄距离奖励、合格保持奖励、关节速度和动作变化惩罚，以及成功/违规终止奖励；
成功指标独立于奖励。碰撞与速度门在物理子步检查，保持在每个策略步检查一次。

固定种子生成128组训练起点/目标和16组评测起点/目标，评测目标距所有训练目标至少1 cm。
以 `[0,-1.2,-1.5,0,0,0] rad` 附近采样，目标与起点TCP距离4–22 cm、TCP高度至少10 cm；
每组含可达性FK见证，起终点及61点关节插值路径经过几何碰撞/限位预检。
训练只抽取训练集，评测只用未见集；这是有限区域的入门任务，不代表全工作空间泛化。
离散几何预检不能代替动态碰撞检查，模型接触精度也不代表实机。

评测使用固定的未见目标集和相同初始条件，对比学习策略与
逆运动学＋平滑轨迹＋位置控制基线。主要指标包括到达并保持成功率、末端位置误差、
稳定保持误差、到达/稳定时间、超调、动作平滑程度以及约束违规次数。
对照运行应保持模型、执行器限制、目标和评测时限一致，并记录随机种子。

训练产物需保留整个运行目录。环境/动作/模型契约变化后，旧策略不得未经核对直接复用。

## 目录与版本

- `.venv/`：独立 Python 3.12 环境；不注入 ROS、视觉或系统 Python。
- `src/rebotarm_drl/`：通用命令、接口契约和机器人模型快照。
- `scripts/`：环境重建与显式模型同步。
- `wheelhouse/`：Linux x86_64 / CPython 3.12 的离线 wheels。
- `.cache/`、`outputs/`、`runs/`、`dist/`：缓存、导出结果、完整训练记录和部署包，均不入 Git。

固定 MJLab 1.6.0、MuJoCo / MuJoCo Warp 3.11.0、Warp 1.14.0、
PyTorch 2.9.1+cu128、RSL-RL 5.4.2。`uv.lock` 是解析锁；
`requirements.lock` 是带摘要的离线安装清单，两者来自同一次解析。
锁文件是机器读取的部署输入，不转换为 Markdown。
模型权威来源仍是 `src/rebotarm_simulation/models/rebotarm`；DRL 内仅保留
`robot.xml` 及其依赖资产的可迁移快照，`provenance.json` 记录来源提交和摘要。
`scripts/sync_assets.py` 只在主模型有经确认的修改后显式运行；独立部署包不运行它。

## 本机检查

无需 source ROS 或激活其他虚拟环境：

```bash
cd /home/a/project/rebot_Arm/DRL
.venv/bin/rebot-drl doctor --gpu
.venv/bin/rebot-drl reach-smoke
REBOT_DRL_GPU_TESTS=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest tests -q
.venv/bin/python -m pip check
```

`doctor --gpu` 检查资产摘要、CPU 模型加载、GPU 算术和两个并行模型各 10 步的
物理步进。这些是环境冒烟检查，不是学习任务，也不代表策略质量或实机验收。
本机标识的 SHA256 保存在 `configs/execution_policy.json`；训练主机检查会拒绝本机。
`reach-smoke` 构造真实环境/RSL网络，执行前向和少量保持步进，不更新网络。
CLI训练入口及注册Runner的 `learn()` 均检查主机并拒绝本机训练；这不是操作系统级封锁。
GPU测试可通过省略 `REBOT_DRL_GPU_TESTS=1` 跳过。

## 远端训练

服务器完成下方部署与GPU检查后，在远端执行。当前部署目录为
`/root/autodl-tmp/rebot_Arm/DRL`；无需 source ROS。先运行短训练检查：

```bash
cd /root/autodl-tmp/rebot_Arm/DRL
.venv/bin/rebot-drl doctor --gpu
.venv/bin/rebot-drl reach-smoke --num-envs 2 --steps 100
.venv/bin/rebot-drl train --num-envs 8192 --iterations 20 --seed 42 --confirm-remote-training
```

短训练结束后，正式训练可使用以下后台命令；SSH断开不会终止该进程：

```bash
cd /root/autodl-tmp/rebot_Arm/DRL || exit 1
mkdir -p outputs
REBOT_TRAIN_LOG="outputs/train_8192_$(date +%Y%m%d_%H%M%S).log"
nohup env PYTHONUNBUFFERED=1 .venv/bin/rebot-drl train \
  --num-envs 8192 --iterations 2000 --seed 42 --confirm-remote-training \
  > "$REBOT_TRAIN_LOG" 2>&1 < /dev/null &
REBOT_TRAIN_PID=$!
printf '%s\n' "$REBOT_TRAIN_PID" > "${REBOT_TRAIN_LOG%.log}.pid"
printf '训练 PID：%s\n日志：%s\n' "$REBOT_TRAIN_PID" "$REBOT_TRAIN_LOG"
tail -n 80 -f "$REBOT_TRAIN_LOG"
```

`Ctrl+C`只退出上述日志查看。此命令从随机初始化开始，不加载冒烟检查点；
8192环境×每轮32步×2000轮，共524288000个环境步。CLI默认并行数仍为256，
8192由命令显式覆盖；改变并行数也会改变每轮PPO样本数，不能只按轮数比较训练效果。
中断后先检查现有进程与日志，避免重复启动；恢复可信检查点时使用：

```bash
# 恢复可信检查点；iterations 表示额外训练轮数：
.venv/bin/rebot-drl train --num-envs 8192 --resume runs/reach_hold_v1/RUN/model_1000.pt \
  --iterations 1000 --confirm-remote-training
```

日志、检查点、`params/agent.yaml`、`params/env.yaml`、目标集和
`deployment_contract.json` 保存至 `runs/reach_hold_v1/时间戳/`。
默认使用本地TensorBoard日志，不上传W&B。保留整个运行目录，并通过固定未见目标评测选择策略。

## 模型导出和校验

任务实现并完成远端训练后，保留整个运行目录（检查点、配置、日志、评测协议）。
导出只接受显式提供的策略和 JSON 契约，不推测观测、动作、单位或控制语义。
契约必须包含 `schema_version: 1`、正整数 `input_dim` / `output_dim` 和非空
`semantics` 对象。后者应记录真实的输入顺序、归一化、单位、输出缩放、控制频率、
模型/配置版本。Reach训练会自动保存上述契约，导出使用同一份文件。

```bash
.venv/bin/rebot-drl export --format torchscript \
  --source runs/SELECTED_RUN/policy.pt \
  --contract runs/SELECTED_RUN/deployment_contract.json \
  --output outputs/SELECTED_EXPORT
.venv/bin/rebot-drl verify-export --output outputs/SELECTED_EXPORT \
  --contract runs/SELECTED_RUN/deployment_contract.json
```

RSL-RL 5.4.2 的 `MLPModel` 检查点可使用 `--format rsl`、
`--source runs/SELECTED_RUN/model.pt` 和 `--agent-config runs/SELECTED_RUN/params/agent.yaml`。
还必须在契约中提供有序 `input_groups`（每组 `name`、`size`）；顺序必须匹配 agent.yaml。
包含 actor 的 observation normalizer（观测归一化器），严格加载权重；不支持的
循环/卷积网络请先用对应训练框架导出 TorchScript。只处理可信训练产物。

输出为 `policy.onnx` 和 `manifest.json`，包含源摘要、契约摘要和数值误差。
检查 batch 1/7/32 共 40 个合成输入，容差 `rtol=atol=1e-5`；
此验证只证明抽样输入上的算术一致性，不证明任务成功率或真实控制行为。
输出目录非空时拒绝覆盖。

## 记录回放与基线比较

Reach任务专用评测（同一MJLab物理、16组固定未见目标、相同控制限制）：

```bash
# IK＋五次平滑轨迹＋位置控制基线
.venv/bin/rebot-drl reach-eval --output outputs/reach_baseline
# 先导出训练产物；SELECTED_RUN替换为真实运行目录
.venv/bin/rebot-drl export --format rsl \
  --source runs/reach_hold_v1/SELECTED_RUN/model_1999.pt \
  --agent-config runs/reach_hold_v1/SELECTED_RUN/params/agent.yaml \
  --contract runs/reach_hold_v1/SELECTED_RUN/deployment_contract.json \
  --output outputs/reach_policy
.venv/bin/rebot-drl reach-eval --policy outputs/reach_policy --output outputs/reach_policy_eval
.venv/bin/rebot-drl compare --candidate outputs/reach_policy_eval/report.json \
  --baseline outputs/reach_baseline/report.json
# 加载原始检查点交互式回放；远端网页查看可改为 --viewer viser
.venv/bin/rebot-drl reach-play --checkpoint runs/reach_hold_v1/SELECTED_RUN/model_1999.pt
# 基线/策略记录的离线运动学回放
.venv/bin/rebot-drl replay --model outputs/reach_baseline/model/scene.xml \
  --recording outputs/reach_baseline/eval-000.npz
```

输出含逐目标报告、NPZ轨迹和可迁移场景模型。IK独立求解目标，不使用目标集的FK见证作为答案。
记录成功/超时/违规/规划失败、末端误差、最后1秒误差、到达/成功时间、沿接近方向超调及动作变化率。
无到达或无成功时对应时间记10 s，必须结合成功标志解读；最后窗口误差不单独代表保持成功。
初版不包含域随机化、真实延迟/摩擦拟合、接触抓取或朝向任务。

2026-10-04 本机软件验证：DRL 22项测试通过（含CUDA物理边界及实际训练配置导出测试）；
16组IK基线全部成功，成功时间3.40–3.78 s，最终位置误差最大0.877 mm，零违规。
未训练的零输出ONNX测试策略16组全部超时、零成功，确认评测不会把静止或低奖励误算为成功；
它仅用于接口检查，不随包提供为训练策略。记录模型可迁移加载及运动学回放通过。
上述本机检查没有训练更新；远端部署、训练冒烟和用户实测状态见[项目状态](../docs/reference/project_status.md)。
主项目完整回归1203项通过、2项跳过，分层19项通过，必需编译与依赖检查通过。
训练保护测试使用模拟的本机身份，在服务器运行测试也不会启动训练。

回放输入是已有 NPZ，必须包含 `qpos`（帧数 × 模型 nq）和严格递增的 `time_s`。
模型 XML 由调用者显式指定，记录的关节顺序须与该模型一致。

```bash
.venv/bin/rebot-drl replay --model PATH_TO_MODEL.xml --recording PATH_TO_RECORDING.npz
# 无窗口的数据与运动学校验：追加 --headless
.venv/bin/rebot-drl compare --candidate PATH_TO_CANDIDATE.json --baseline PATH_TO_BASELINE.json
```

回放直接设置记录的状态并计算运动学，不重新执行控制器，不访问 ROS 或硬件。
通用比较工具读取已有报告；`reach-eval` 负责生成本任务的评测报告。
报告结构为 `{"protocol": {...}, "trials": [{"id": "...", "metrics": {...}}]}`。
`protocol` 应包含场景、随机种子、指令、时长和指标定义；两份报告必须一致，
trial ID 必须配对，指标名称必须一致且数值有限。输出均值、P95、配对差值；
不预设指标好坏方向或合格门限。

## 打包与远端部署

含首个任务的最终部署包为 `dist/rebotarm-drl-reach-hold-offline-20261004-r2.tar.gz`，
对应同名 `.sha256`；源码版为 `dist/rebotarm-drl-reach-hold-source-20261004-r2.tar.gz`。
20261003 包仅是此前通用环境快照，不含任务；上传时使用上述 Reach-and-Hold 修订包。
既有r2包保持原样，其中README是打包时的文档快照；仓库README已补充远端操作说明。

```bash
.venv/bin/rebot-drl bundle --with-wheels --output dist/rebotarm-drl-offline.tar.gz
.venv/bin/rebot-drl verify-bundle dist/rebotarm-drl-offline.tar.gz
```

省略 `--with-wheels` 可制作小型源码包。包采用白名单，只带源码、模型、测试、
说明、锁文件及可选 wheels，不带 `.venv`、缓存、训练记录或远端凭据。
同时生成 `.sha256`；包内 `BUNDLE_MANIFEST.json` 列出逐文件摘要。
打包拒绝覆盖同名文件；再次生成请选择新名称。

服务器需要 Linux x86_64、Python 3.12（包含 venv/ensurepip）、兼容 NVIDIA 驱动，
以及运行时所需系统图形库。离线包包含 Python/CUDA 用户态依赖，
不包含操作系统、显卡驱动或 Python 解释器。无窗口渲染需要系统EGL库；
Ubuntu缺失 `libEGL.so.1` 时，先通过系统包管理器安装 `libegl1` 及其依赖。
更换服务器后仍需核验其系统、驱动和GPU兼容性。
上传后在空的目标父目录中执行（包展开成一个 `DRL/`）：

```bash
sha256sum -c rebotarm-drl-offline.tar.gz.sha256
tar -xzf rebotarm-drl-offline.tar.gz
cd DRL
python3 scripts/setup_env.py
.venv/bin/rebot-drl doctor --gpu
.venv/bin/rebot-drl reach-smoke
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest tests -q
```

`setup_env.py` 默认离线，按摘要锁安装，再以 editable 方式安装本项目；
不复制不可迁移的虚拟环境，不覆盖已有 `.venv`。只有小型源码包时可显式加 `--online`。
所有依赖下载和编译缓存留在 DRL；SSH地址和凭据不写入仓库，部署操作不自动启动训练。

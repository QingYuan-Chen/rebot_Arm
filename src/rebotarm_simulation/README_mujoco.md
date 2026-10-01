# reBotArm MuJoCo 仿真底座

当前项目基线是 [`huangbinai/robotarm_ros2`](https://github.com/huangbinai/robotarm_ros2)
`main@19f2939e27c52e07f73e7ff1ab8f723c01852ce0`。本包在该新基线上选择性整合
Viewer、Reach/Pick、Sim2Real、Real2Sim、诊断与批量验收能力，同时保留上游的
包结构、启动组合、MoveIt 权威 URDF及安全限制；虚拟 RGB-D 发布已随上游退役。

实机状态驱动 MuJoCo 的只读 Real2Sim Bridge、mirror/physics 模式和 Viewer 使用方法见
[`../../docs/real2sim_bridge_zh.md`](../../docs/real2sim_bridge_zh.md)。

Pick 精细抓取环境、成功/失败判定和批量验收见
[`../../docs/mujoco_pick_zh.md`](../../docs/mujoco_pick_zh.md)。

Sim2Real/Real2Sim 的仿真侧随机化、JSONL 记录、确定性回放、轨迹比较和批量安全
检查见 [`../../docs/sim2real_workflow_zh.md`](../../docs/sim2real_workflow_zh.md)。该流程
不连接实机。

本目录提供可独立使用的 MuJoCo 物理仿真核心、桌面 Viewer 和 ROS 2
适配层。已验证的目标环境是 Ubuntu 24.04、ROS 2 Jazzy、Python 3.12。
已提供基于现有物理核心的 Gymnasium 末端位姿 Reach 环境与 GPU PPO 训练入口。
克隆后配置、环境契约和验证步骤见 [强化学习命令参考](../../docs/reference/commands/mujoco_rl.md)。

## 安装与构建

ROS launch 默认使用 PATH 中的 `python3`，本机部署使用系统 Python 3.12 和用户级
MuJoCo 依赖，不需要 MuJoCo 专属虚拟环境、激活脚本或解释器环境变量。
外部部署可用 `python_executable` 参数或 `REBOTARM_MUJOCO_PYTHON` 显式覆盖；
启动目录不会影响解释器选择。混合视觉入口见
[启动解释器说明](../../docs/setup/launch_python_configuration.md)。

`mujoco_moveit_sim.launch.py` 默认同时打开原生 MuJoCo viewer，便于桌面联调
观察物理模型；ROS 适配节点仍是独立的 headless 节点，负责 ROS action 和状态
发布。关闭 viewer 可传 `use_mujoco_viewer:=false`，适用于无图形界面或 CI。

日常桌面联调可直接使用专用入口：

```bash
ros2 launch rebotarm_simulation mujoco_rviz_viewer.launch.py
```

服务器、CI 或只做后台物理测试使用：

```bash
ros2 launch rebotarm_simulation mujoco_headless.launch.py
```

在当前工作区根目录、未激活venv的新终端执行。系统依赖按
[安装说明](../../docs/setup/ubuntu_ros2_jazzy.md)准备；源码和仿真统一使用系统 Python，
不需要相机、视觉权重或历史上游副本。

```bash
cd /home/a/project/rebot_Arm
source /opt/ros/jazzy/setup.bash
python3 scripts/install_python_dependencies.py mujoco --user --break-system-packages
/usr/bin/python3 -m colcon build --symlink-install --packages-select rebotarm_simulation
source install/setup.bash
```

依赖只需部署一次。`--user` 安装到当前用户包目录，不使用 sudo 或覆盖系统包文件；
用户级包优先于同名系统包，固定版本见[依赖清单](../../docs/setup/dependencies/mujoco.md)。
每个新终端加载 ROS 和 `install/setup.bash` 即可；旧终端若曾设置专属解释器路径，
迁移后先执行 `unset REBOTARM_MUJOCO_PYTHON`。

colcon 安装的 console script 使用构建时的解释器 shebang；本机统一由
`/usr/bin/python3` 构建，因此 ROS launch 和 `ros2 run` 都能读取同一组用户级依赖。
若选择其他解释器运行独立工具，使用 `ros2 run --prefix /绝对路径/python ...`。
以下是从工作区源码直接运行的完整等价命令：

```bash
MUJOCO_GL=egl PYTHONPATH=src/rebotarm_simulation python3 -m rebotarm_simulation.mujoco_health --renderer-timeout 30
PYTHONPATH=src/rebotarm_simulation python3 -m rebotarm_simulation.mujoco_cli run --duration 5
PYTHONPATH=src/rebotarm_simulation python3 -m rebotarm_simulation.mujoco_viewer --duration 30
```

## 健康检查与无界面运行

EGL 渲染检查在隔离子进程中运行，并有超时保护，避免驱动挂起拖死主进程：

```bash
MUJOCO_GL=egl rebotarm_mujoco_health --renderer-timeout 30
```

预期 JSON 关键字段如下；版本和路径允许不同，但关节/执行器均应为 8，
`ok`、模型加载和有限步进应为 true。无 EGL 设备时物理健康仍可为 true，
同时 `renderer_available` 为 false 并给出 `renderer_error`。

```json
{"ok": true, "model_loaded": true, "physics_step_finite": true,
 "joint_count": 8, "expected_joint_count": 8,
 "actuator_count": 8, "expected_actuator_count": 8,
 "headless": true, "renderer_available": true}
```

不依赖窗口的定时运行：

```bash
rebotarm_mujoco_cli run --duration 5
```

输出同时含 `"requested_duration"` 和 `"achieved_duration"`；后者会按物理
时间步向上取整。交互模式使用 `rebotarm_mujoco_cli shell`，支持 `state`、
`control`、`joint`、`joints`、`jog`、`gripper`、`mode`、`step`、`contacts`、
`reset`、`pause`、`resume` 和 `quit`。限时原始力矩诊断使用：

```bash
rebotarm_mujoco_cli torque --values 1 0 0 0 0 0 --timeout 0.1 --observe 0.2
```

Raw torque 只用于无实机诊断，按关节峰值限幅；看门狗超时后清零并进入 Hold。

无界面 Reach 随机策略批量 rollout：

```bash
rebotarm_mujoco_batch --episodes 3 --steps 100 --seed 0
```

该命令输出 JSON 摘要，包含每回合步数、累计奖励、末端到目标距离和成功标志；
它只验证 API/物理层可无界面运行，不代表已经完成强化学习训练。

夹爪接触稳定性检查：

```bash
rebotarm_mujoco_contact_check
```

该命令在无界面 MuJoCo 中移动到固定接触验收姿态，打开夹爪、放置 `test_cube`、
闭合夹爪并输出 JSON。`ok=true` 表示仿真中出现持续的手指-方块接触，方块位姿
保持有限，单步跳变、接触力和最低高度都在阈值内；它用于排查抖动、穿模或接触力
爆炸，不连接实机。

基础总验收入口会串行运行 health、headless Reach batch 和接触检查：

```bash
rebotarm_mujoco_acceptance --skip-renderer
```

在已经 source ROS 2 与 `install/setup.bash` 的 Ubuntu VM 中，可追加 ROS 2 接口验收：

```bash
rebotarm_mujoco_acceptance --skip-renderer --include-ros --timeout 30
```

MoveIt 联动验收需要先按下文启动 MuJoCo 后端和 MoveIt，再单独运行
`rebotarm_mujoco_moveit_acceptance --timeout 30`，（入口归属 `rebotarm_motion`），或对总验收追加 `--include-moveit`，由兼容入口转发至同一运动包实现。

## 当前仿真核验与已知问题

2026-10-01 在 `develop@03a32a0` 完成无实机功能核验。正常功能路径可运行，
但以下两处控制行为尚未修复，不能把完整回归通过记为全部仿真验收通过。

| 功能 | 核验结果与范围 |
| --- | --- |
| 模型、物理步进与渲染 | 8 关节、8 执行器；MuJoCo 3.3.0、EGL 离屏渲染及有限数值步进通过 |
| ROS 六轴轨迹与夹爪服务 | 轨迹返回成功，最终最大关节误差 0.015438 rad；夹爪到达 0.05 m |
| 夹爪接触物理 | 600 个接触步中 440 步有手指接触；最大方块接触力 2.329 N |
| MoveIt 规划并执行 | 实际生成、下发 37 个轨迹点并完成，最终最大关节误差 0.010004 rad |
| 示教离线预演 | 合成六轴记录 201 个样本、整形后 401 个点，4 s / 2000 物理步，最大跟踪误差 0.000353 rad；重复时间戳被拒绝 |
| Real2Sim | 合成反馈 100 Hz / 200 样本；mirror 关节误差为 0，physics 最大误差 0.067447 rad |
| Reach / Pick | 基础运行通过；Pick 三个 seed 各 100 步安全违规为 0，随机动作抓取成功率为 0，未验证策略成功抓取 |
| 视觉多阶段规划 | 经 FK 与碰撞有效性校验的模拟计划完成 pregrasp、approach、safe_retreat 三段规划；plan_only 没有下发 Action；缺少计划、时间戳无效、夹爪宽度超限均被拒绝 |

完整回归为 `1167 passed / 14 skipped`，两条既有 Gymnasium 观测范围提示；
分层测试 `18 passed`，必需编译检查通过。视觉测试没有覆盖真实相机、YOLO/GraspNet
推理；示教和 Real2Sim 输入是合成数据，没有访问真实控制器或串口。

### 越界轨迹可能返回成功

独立仿真中，J3 的模型上限为 `+0.02 rad`，提交 `+0.03 rad` 轨迹终点仍被
Action 接受并返回 `SUCCEEDED / SUCCESSFUL`。最后 desired 为 `+0.03 rad`，
actual 为 `+0.020412 rad`，误差约 `0.009588 rad`。

`mujoco_ros_node._goal_callback` 未在接纳前校验轨迹点的位置范围；底层
`command_joint_positions` 裁剪目标后，执行层仍按原始目标和 `0.02 rad`
完成容差判断，掩盖了这次越界请求。应在 Action 接纳前依据模型逐轴检查全部
轨迹点并拒绝越界；保持合法轨迹的现有完成容差，不能扩大 J3 限位来处理。

### 切换保持模式存在支撑力矩断续

小幅预弯和 HOME 两个独立取消测试均返回 CANCELED，并在随后 5 s 观察中进入
稳定 hold；但取消后 J3 峰值偏移分别为 `0.047163 / 0.044613 rad`，峰值速度为
`0.594690 / 0.582318 rad/s`。最后 1 s 最大速度低于 `0.000137 rad/s`，说明
最终保持稳定，不能据此忽略切换过程中的抖动。

静止姿态、零运动命令的模式切换也复现了扰动：J3 支撑力矩从 `-8.536169 N·m`
降到 `-0.630000 N·m`，随后偏移 `0.039644 rad`。`set_mode('hold')` 捕获姿态
后重置控制器，将 applied_torque 清零，再通过变化率与低通限制重新爬升。
后续应实现连续的支撑力矩切换，并保留取消时的姿态锚点与稳定验收；不改成直接
失能。该结论只针对仿真实现，未证明真实控制器有相同行为。

上述两处实现文件在 `dc7e067..03a32a0` 没有代码变化，属于本次核验发现的已有
问题。另观察到一次 MoveGroup 在规划结束后未于默认 5 s 内响应 SIGINT，
被 launch 升级 SIGTERM 后退出；退出原因尚未定位，运行阶段没有因此失败。

原始报告、JSON、逐步反馈和测试日志保留在本机
`log/functional_sim_20261001/`，不随文档提交。测试使用预先检查为空闲的 ROS 域
119/176、localhost 通信及唯一 MuJoCo Action/反馈源检查；收尾未发现测试进程或
DDS 端口残留，51 份运行/模型配置摘要一致。再次测试时需重新检查空闲域。
当前问题状态见[项目状态](../../docs/reference/project_status.md)。

## 桌面 Viewer

在 Ubuntu 图形桌面的终端运行，而不是普通无显示 SSH 会话：

```bash
rebotarm_mujoco_viewer --duration 30
```

按 `Z/X` 选择上一个/下一个关节，按一次 `J/K` 开始连续反/正向移动当前关节，按一次
`C/O` 开始连续闭合/打开夹爪；连续运动不依赖虚拟机的键盘连发，按 `S` 停止并保持
当前位置。按 `-` 降低速度档、按 `+`（主键盘上通常为 `Shift+=`）提高速度档，窗口左上
会显示当前档位和实际速度：

| 档位 | 关节速度 | 夹爪宽度速度 | 用途 |
| --- | ---: | ---: | --- |
| Precision | `0.05 rad/s` | `0.005 m/s` | 接近目标、精细调整 |
| Normal | `0.20 rad/s` | `0.020 m/s` | 默认手动操作 |
| Fast | `0.50 rad/s` | `0.050 m/s` | 大范围快速移动 |
| Turbo | `1.00 rad/s` | `0.100 m/s` | 无接触区快速定位；接近物体前降档 |
| Max | `1.50 rad/s` | `0.150 m/s` | 仅用于宽阔无接触区的最快定位 |

`J/K/C/O` 采用锁存式连续控制：按一次开始连续运动，不需要一直按键或反复点击；按
`S` 才停止并进入 Hold。若仍觉得慢，连续按 `+` 可升到 Max；靠近桌面、方块或关节
限位时应退回 Precision/Normal，避免大步目标造成碰撞。

按 `M` 可在三类手动控制间循环：`JOINT`（单关节）、`XYZ`（末端平移）、`RPY`
（末端转动）。`Z/X` 在当前类别中选择关节或轴，`J/K` 沿负/正方向连续移动；在
XYZ/RPY 中按 `F8` 切换世界坐标与工具坐标。末端控制使用阻尼最小二乘 IK，并保持
关节限位；不可达目标不会写入关节命令，窗口会显示 IK 状态。项目不使用 `Tab`
切换输入模式，因为该键由 MuJoCo 原生 Viewer 用于开关侧栏，会改变渲染视口。

绿色球和红/绿/蓝 XYZ 姿态轴是末端目标，仅在 XYZ/RPY 模式显示；JOINT 模式自动
隐藏目标和蓝色幽灵，保持模型视野整洁。用 MuJoCo 原生方式双击选中目标，按住 `Ctrl` 配合鼠标拖动即可
平移/旋转目标，求解成功后蓝色半透明幽灵机械臂显示目标姿态，实体机械臂通过
POS_VEL 控制跟踪。绿色目标本身没有质量和碰撞，不会给物理世界施加额外外力。

Viewer 默认进入简洁的 `Overview` 页面。`F6` 按 Overview → Joints 循环切换：
Joints 显示六轴实际/目标/误差/速度；
`F7` 单独显示/隐藏完整按键帮助。`F9` 按 OFF → TRACKING → EFFORT 循环，一次仅显示
一张当前关节曲线；Tracking 包含实际/目标/误差/速度，Effort 包含请求/施加力矩和
最大接触力。打开曲线时会隐藏右侧文字页，避免相互遮挡。项目不复用 MuJoCo 原生
`F1–F5`（帮助、信息、性能、传感器、全屏），从而避免控制操作弹出原生面板或改变窗口。

人工示教录制、轨迹文件管理、检查、滤波、重定时和安全回放统一使用项目网页中的
`Teach Trajectory` 工作流，不在 MuJoCo Viewer 中维护第二套格式和快捷键。MuJoCo
继续通过标准 `/rebotarm/joint_states` 和 `/rebotarm/follow_joint_trajectory` 接口支持
网页录制与仿真回放。用于强化学习及 Sim2Real 批量实验的 JSONL 数据记录、确定性回放
和比较工具保持独立，它们是实验数据管线，不是人工示教界面。

`G` 进入重力补偿，`H` 捕获并保持当前位置，`P` 进入
Position（内部仍为 POS_VEL→motor 力矩），`V` 显示/隐藏碰撞代理，空格暂停，
`.` 单步，`R` 重置，`T` 回 Home，`Q` 退出。数字键和 `[/]` 保留给 MuJoCo
原生几何组与相机快捷键，不再用于选关节。

已经处于 Hold 时再次按 `H` 是幂等操作：不会重复捕获目标、重置控制器或引起力矩跳变。
从 Gravity/Position 切换到 Hold 时才会捕获切换瞬间的当前位置。Wayland 不支持窗口
坐标查询属于 GLFW 能力差异，Viewer 会定向隐藏这一条无害警告，其他 GLFW 错误仍会输出。

MuJoCo 原生还复用了 `T/H/P/Z/X/J/C/O/V` 等显示快捷键；项目 Viewer 会在每帧
恢复固定的正常显示标志，防止机器人控制按键同时触发透明、凸包、纹理、灯光或接触
调试效果。需要查看本项目碰撞代理时只使用 `V`，不依赖原生 Rendering 面板开关。

窗口叠加层采用分页驾驶舱：左上固定显示运行、控制、输入和速度；右上只显示当前页面；
左下仅在力矩饱和、IK 失败、回放超差或 Raw torque 看门狗临界时出现告警；右下仅保留
四行核心提示。终端默认只显示一条
启动提示、用户主动输入命令的结果和错误，不再逐帧刷状态；诊断时可加
`--verbose-status` 恢复实时文本输出。Viewer 默认隐藏 MuJoCo 原生左右侧栏，避免占用
模型视野；其中原始 Control 面板显示的是底层 actuator 力矩而不是位置目标。如需原生
侧栏可在窗口菜单中手动打开。若从 SSH 启动，需正确配置 X11 转发，且
`DISPLAY` 与 `XAUTHORITY` 必须指向当前桌面会话；否则请改用 headless CLI。

Viewer 运行时也可以直接在启动它的终端输入目标命令，命令会排队到仿真主线程执行，
不会从键盘回调线程直接改 MuJoCo 状态：

```text
joints 0.0 -0.8 -1.0 0.3 0.0 0.0
joint joint2 -0.6
jog joint3 -0.1
gripper 0.05
mode hold
mode position
home
reset
state
contacts
```

其中 `joints` 一次设置六个关节目标角，`joint` 设置单个关节目标角，`jog` 是以当前
实际关节角为基准加一个增量，`gripper` 设置夹爪目标开口宽度。若只想看 Viewer 而不
读取终端命令，可加 `--no-command-input`。

模型始终使用完整原始 STL 作为 visual。机械臂主体和夹爪基座使用配置化稳定碰撞
代理，左右手指保留精细凸 mesh 接触；碰撞层仅在按 `V` 调试时显示。场景只 include
`models/rebotarm/robot.xml`，没有叠加旧的手写模型。

## ROS 2 适配层

只启动 MuJoCo 后端：

```bash
ros2 launch rebotarm_simulation mujoco_sim.launch.py
```

节点默认从 Home+Hold 启动，提供标准控制接口、仿真模式服务、诊断和仿真时钟：

- Action：`/rebotarm/follow_joint_trajectory`
- Topic：`/rebotarm/joint_states`
- Service：`/rebotarm/gripper/set`
- Topic：`/rebotarm/gripper/state`
- Service：`/rebotarm/trajectory_stop`
- Service：`/rebotarm/sim/set_mode`（只接受 `position/hold/gravity_comp`）
- Topic：`/diagnostics`
- Topic：`/clock`

MuJoCo 物理步进定时器固定使用 steady clock，不依赖节点的 ROS 时钟；消费仿真状态的
MoveIt、RViz 和状态发布节点使用 `use_sim_time:=true` 跟随 `/clock`。这样 `/clock` 的
发布者不会反过来等待自己尚未发布的仿真时间。

另开已 source 的终端发送一个六关节目标：

```bash
ros2 action send_goal /rebotarm/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [joint1, joint2, joint3, joint4, joint5, joint6], points: [{positions: [0.1, -0.2, -0.2, 0.2, 0.0, 0.0], time_from_start: {sec: 3}}]}}" --feedback

ros2 service call /rebotarm/gripper/set rebotarm_msgs/srv/SetGripper \
  "{position: 0.05, max_effort: 10.0}"

ros2 service call /rebotarm/trajectory_stop std_srvs/srv/Trigger "{}"
```

也可以在 Ubuntu VM 中直接跑一次无实机 ROS 2 接口验收。它在同一进程里启动
MuJoCo ROS 2 节点和探针节点，检查 `/rebotarm/joint_states`、`/clock`、
`/rebotarm/gripper/set` 和 `/rebotarm/follow_joint_trajectory`：

```bash
rebotarm_mujoco_ros_acceptance --timeout 15
```

输出 JSON 中 `ok=true` 才表示 topic、service 和 action 均通过；该命令不启动
`rebotarmcontroller`，也不使用 `use_hardware:=true`。

MoveIt 的控制器映射保持为 `/rebotarm/follow_joint_trajectory`。联调时所有
bringup 必须显式使用 `use_hardware:=false`，且只保留一个该 Action 的服务端。

### MuJoCo 与 MoveIt 联动

使用主项目保留的组合入口启动唯一 MuJoCo 轨迹服务端和 MoveIt：

```bash
ros2 launch rebotarm_simulation mujoco_moveit_sim.launch.py use_sim_time:=true
```

如不需要 RViz，可追加 `use_rviz:=false`。该入口固定关闭 fake joint state publisher，
让 `/rebotarm/joint_states` 只有 MuJoCo 这一份权威物理状态。
MoveIt 控制器映射到 `/rebotarm/follow_joint_trajectory`，由同一入口中的
MuJoCo Action 服务端执行。

两端启动后，可在第三个终端运行 MoveIt-MuJoCo 联动验收：

```bash
rebotarm_mujoco_moveit_acceptance --timeout 30
```

该命令等待 `/plan_kinematic_path`、`/rebotarm/follow_joint_trajectory`、
`/rebotarm/joint_states` 和 `/clock`，向 MoveIt 请求一条六关节规划，再把 MoveIt
返回的 `JointTrajectory` 发给 MuJoCo action server 执行。`ok=true` 表示规划、
执行和最终关节误差均通过；它不启动 RViz、不连接实机，但验证的就是 RViz 规划执行
会走的同一条 MoveIt planning service 和 MuJoCo trajectory action 链路。

## Python API（面向后续训练/推理复用）

核心类不依赖 ROS 2：

```python
from rebotarm_simulation.mujoco_sim import RebotArmMujoco

with RebotArmMujoco() as sim:
    sim.reset(seed=7)
    scene = sim.randomize_scene(seed=7)
    sim.set_mode("position")
    sim.command_joint_positions([0.1, -0.2, -0.2, 0.2, 0.0, 0.0])
    sim.command_gripper_width(0.05, max_force_n=10.0)
    control = sim.get_control_status()
    sim.set_object_pose("test_cube", [0.45, 0.0, 0.44], [0.0, 0.0, 0.0, 1.0])
    saved = sim.save_state()
    state = sim.step(10)
    sim.restore_state(saved)
    contacts = sim.get_contacts()
```

`get_state()` 返回关节位置/速度、执行器力、末端位姿、夹爪宽度、物体位姿
和仿真时间。公开的末端与物体四元数顺序都是 XYZW；MuJoCo 内部 WXYZ 已在
API 边界转换。`save_state()`/`restore_state()` 只允许同一模型实例，
`set_object_pose()` 只接受带 free joint 的物体，四元数也使用 XYZW。
`randomize_scene()` 会按给定 seed 随机 `test_cube` 位置和 Reach 目标点，
返回 `RandomizedScene(cube_pose, reach_target_position, seed)`，用于后续
Reach/Pick 场景随机化。

轻量 Gym 风格 Reach 环境不依赖 `gymnasium`：

```python
from rebotarm_simulation.mujoco_env import RebotArmReachEnv

with RebotArmReachEnv() as env:
    obs, info = env.reset(seed=7)
    obs, reward, terminated, truncated, info = env.step([0.0] * 7)
    obs, reward, done, info = env.step_done([0.0] * 7)
```

`obs` 包含 `joint_positions`、`joint_velocities`、`gripper_width`、
`ee_position`、`target_position`、`cube_pose` 和 `max_contact_force`。
`step(action)` 接受 6 维关节增量或 7 维关节+夹爪增量，内部裁剪到 `[-1, 1]`。
默认 `step()` 使用 Gymnasium 的 `terminated/truncated` 形状，同时在 `info` 里提供
`done`；旧 Gym 风格代码可直接调用 `step_done()` 获取 `(obs, reward, done, info)`。

## Sim2Real / Real2Sim 基础层

当前先实现不连接实机的基础设施：随机化、统一轨迹日志、仿真回放和误差比较。
默认 profile 不改变现有 MuJoCo 行为；`training_profile` 只在显式传入时启用。

```python
from pathlib import Path

from rebotarm_simulation.mujoco_env import RebotArmReachEnv
from rebotarm_simulation.sim2real import (
    RandomizationConfig,
    TrajectoryRecorder,
)

config = RandomizationConfig.from_yaml(
    Path("src/rebotarm_simulation/config/sim2real_randomization.yaml"),
    profile="training_profile",
)
sample = config.sample(seed=7)
recorder = TrajectoryRecorder(episode_id="reach-0007", source="sim")

with RebotArmReachEnv() as env:
    env.reset(seed=7, randomization=sample)
    for step_index in range(100):
        action = [0.0] * 7
        obs, reward, terminated, truncated, info = env.step(action)
        recorder.append(
            env.sample_from_last_step(
                action,
                episode_id="reach-0007",
                step_index=step_index,
            )
        )
        if terminated or truncated:
            break

recorder.to_jsonl("logs/reach-0007.jsonl")
```

相同 seed 会得到相同随机化 sample。`RandomizationSession` 会显式修改并在退出时恢复
MuJoCo 的质量、阻尼、摩擦和 arm torque scale。日志格式不依赖 ROS 2，未来真实机械臂
只需要把数据转换成同一个 `TrajectorySample(source="real")`。`replay_actions()` 和
`compare_trajectories()` 可用于仿真回放和误差报告；当前阶段不宣称已经完成真实系统辨识
或 sim-to-real 精度标定。

后续架构是云服务器运行无界面 MuJoCo 并行训练，本地 Ubuntu VM 做模型验证、
ROS 2 联调和策略推理。现已提供本地 GPU Reach 训练基线；云端并行训练尚未验证。

模型在 `end_link` 下提供命名坐标系 `wrist_camera_mount`，作为后续手眼/末端 RGB-D
相机的稳定安装基准。上游新增 Gemini 2/支架仿真负载（含质量、惯量与网格），不表示实机内参或手眼标定已验证。

## 排障

- **重复节点**：若 Action 提示重复服务端，先用 `ros2 node list` 检查重复节点。
  仅结束 launch 父进程可能留下 `rebotarm_mujoco_node` 子进程；先用
  `pgrep -af rebotarm_mujoco_node` 定向查找候选 PID，再用
  `ps -fp <confirmed-pid>` 核对完整命令。只在确认它是本次仿真的遗留进程后，执行
  `kill -TERM <confirmed-pid>`，再复查节点列表。禁止宽泛 kill，
  也禁止同时运行旧 RViz-only 控制器。
- **SSH 无窗口**：检查 `DISPLAY`、`XAUTHORITY` 和 X11 转发；物理验证优先用
  `--headless`。
- **EGL 超时/崩溃**：健康检查的 renderer 子进程会在 `--renderer-timeout`
  后报告失败；可先用 `--skip-renderer` 单独确认物理模型。
- **轨迹未成功**：检查配置中的 goal tolerance（位置、速度和时间），当前采用
  节点级容差，不接受每个 goal 的覆盖值。
- **规划或接触异常**：MoveIt self-collision 矩阵与 MuJoCo 接触过滤是不同层，
  分别检查 SRDF self-collision 与 MJCF 的 `contype/conaffinity`。

## 无硬件安全边界

本流程不得设置 `use_hardware:=true`，不得启动 `rebotarmcontroller`，不得打开
CAN、串口或真实夹爪，不得发送实机使能、回零或轨迹命令。发现任何硬件节点、
CAN 设备或串口已被占用时，停止联调并确认环境；MuJoCo 测试不需要连接实机。

## 从 URDF 生成本地 MJCF

机器人结构的唯一权威来源是
`src/rebotarm_moveit_config/config/rebotarm.urdf`，MuJoCo 日常运行读取本地文件
`src/rebotarm_simulation/models/rebotarm/robot.xml`。后者是自动生成并随仓库提交的
MJCF，不需要联网加载。当前支持范围固定为 `mujoco>=3.3,<4`。

URDF 或其引用的原始 STL 更新后，在仓库根目录重新生成：

```bash
rebotarm_urdf_to_mjcf --repo-root .
```

只检查仓库中的 MJCF 是否仍与 URDF 一致，不写文件：

```bash
rebotarm_urdf_to_mjcf --repo-root . --check
```

转换使用 MuJoCo 官方 URDF 解析器。完整原始 STL 保留为视觉网格；主体碰撞代理由
`config/mujoco_collision.yaml` 明确定义，手指使用精细凸 mesh。执行器、
双指联动、传感器、末端 site 和接触过滤由确定性后处理补入。不要手工修改
`robot.xml`；应修改 URDF、碰撞配置或生成器后重新生成。

Windows 与 Ubuntu VM 同步后，可分别检查受控文件哈希：

```powershell
Get-FileHash src/rebotarm_simulation/models/rebotarm/robot.xml -Algorithm SHA256
```

```bash
sha256sum src/rebotarm_simulation/models/rebotarm/robot.xml
```

生成、检查和运行均不连接实机，不探测 CAN、串口或机械臂控制器；ROS 2 联调继续明确
使用 `use_hardware:=false`。

## 本地合并兼容说明

默认 `scene.xml` 保留 cube 接触/Pick 与 Viewer 拖拽目标；瓶体随机化使用
`scene_bottle.xml`，上游 Gymnasium CPU Reach 使用独立无碰撞 `reach_scene.xml`。
三类场景共享本轮更新的 `robot.xml` 和 Gemini 2 仿真负载。旧虚拟 RGB-D
发布流程已退役。MoveIt 仿真验收统一由 `rebotarm_motion` 实现。

本地软件回归不代表 GPU PPO 训练质量或实机验收。

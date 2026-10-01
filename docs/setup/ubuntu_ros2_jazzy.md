# reBotArm 本机环境与配置

> 状态：SETUP；类型：环境与构建操作；适用范围：Ubuntu 24.04 / ROS 2 Jazzy；最后核对：2026-09。

本文记录本仓库在 Ubuntu 24.04 / ROS 2 Jazzy 上的本地运行基线。

> 当前状态：P0-P6 已按用户确认的工程范围关闭，未执行项不冒充通过。
> `HardwareManager.connect()` 只连接并保持失能，任何新机器或新一轮真机动作仍需
> 独立确认现场条件和授权。安装、构建、导入检查不会授权或自动启动硬件。

下列命令均从当前仓库根目录执行。推荐顺序：系统依赖 → 源码构建 →
按需安装控制器/视觉/GraspNet/MuJoCo依赖 → 配置模型和逐进程解释器 → 软件检查。
源码构建与设备、模型准备分离，不再要求新克隆预先具有机器专属TensorRT engine。

## 1. 系统依赖

根目录五份 `requirements-*.txt` 按运行环境和安装方式拆分，不应全部安装到
同一个 Python 环境，也不是五份重复清单：

| 清单 | 用途 | 环境 / 安装入口 |
| --- | --- | --- |
| `requirements-runtime.txt` | 控制器 MotorBridge 基础依赖，随后必须安装本地审查补丁 | 系统 Python；本节命令 |
| `requirements-vision.txt` | 相机、YOLO、OpenCV，固定 NumPy 1.x 兼容 ROS cv_bridge | `.venv-vision`；`tools/setup_ubuntu_vision.sh` |
| `requirements-graspnet.txt` | GraspNet 抓取候选、点云及固定 PyTorch/CUDA 依赖 | `.venv-graspnet`；`tools/setup_ubuntu_graspnet.sh` |
| `requirements-mujoco.txt` | MuJoCo 仿真与模型工具 | `third_party/rebotarm_mujoco_venv`；见仿真包 README |
| `requirements-rl.txt` | MuJoCo + Gymnasium + GPU PPO（包含物理依赖） | 同一 MuJoCo venv；见 `../reference/commands/mujoco_rl.md` |
| `requirements-tensorrt.txt` | TensorRT 推理扩展，单独以 `--no-deps` 安装以避免拉取额外 CUDA 工具链 | 与 vision 共用环境；由视觉安装脚本调用 |

GraspNet 与 MuJoCo 的 PyYAML 固定版本分别为6.0.1和6.0.3；直接合并会产生
版本冲突。TensorRT 是视觉环境的独立安装步骤，不是第五个虚拟环境。
视觉和GraspNet使用相同的固定PyTorch/CUDA wheel版本，但仍保留独立环境。

先按ROS官方说明安装ROS 2 Jazzy，再准备构建和运行所需系统包：

```bash
sudo apt update
sudo apt install build-essential git python3-venv python3-pip \
  python3-colcon-common-extensions python3-vcstool python3-rosdep python3-pytest \
  ros-jazzy-moveit ros-jazzy-moveit-simple-controller-manager \
  ros-jazzy-pinocchio ros-jazzy-cv-bridge
source /opt/ros/jazzy/setup.bash
# 仅在本机从未初始化rosdep时执行 sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
```

### MoveIt 执行插件：能 Plan 但不能 Execute

`src/rebotarm_moveit_config/config/moveit_controllers.yaml` 指定
`moveit_simple_controller_manager/MoveItSimpleControllerManager`，由它把
MoveIt 轨迹交给 `/rebotarm/follow_joint_trajectory`。仿真和真机的 MoveIt
执行链都需要这个插件；它不是机械臂 SDK，也不要求额外启动 ros2_control
的 controller_manager。只安装部分 MoveIt 组件可能遗漏它。
当前 `rebotarm_moveit_config/package.xml` 尚未显式声明该插件依赖，
不能仅依靠现有 manifest 的 rosdep 安装来保证它存在；本次仅补文档。

典型日志组合：

```text
Exception while loading controller manager 'moveit_simple_controller_manager/MoveItSimpleControllerManager'
... class ... does not exist. Declared types are
Failed to reload controllers: `controller_manager_` does not exist.
Unable to identify any set of controllers that can actuate the specified joints
CONTROL_FAILED
```

先确认启动日志中的插件加载错误，不要只凭最后一行 `CONTROL_FAILED`
推断原因。`base_link` 根惯量的 KDL 警告不是上述插件缺失的原因，
不应为解决这类 Execute 错误而先改 URDF。

只读检查（ROS 命令需先加载当前环境）：

```bash
dpkg-query -W ros-jazzy-moveit-simple-controller-manager
ros2 pkg prefix moveit_simple_controller_manager
ros2 param get /move_group moveit_controller_manager
ros2 action info /rebotarm/follow_joint_trajectory
```

检查终端与启动终端必须使用相同的 `ROS_DOMAIN_ID`。如果 MuJoCo Action
server 为 1，但插件类不存在、MoveIt 的 Known controllers 为空，故障在
MoveIt 插件加载侧，而非仅因模拟器未启动。若包已安装仍报错，继续检查
当前 ROS 环境、插件发现与启动日志，不通过禁用安全检查来绕过故障。

### 启动时反馈分批到达

控制器启动会为每个电机保存独立的反馈 sequence；六个 sequence 不要求相等。
串口桥可能在不同轮询中返回不同电机的帧。控制器会跨轮询收集这些帧，只要每个
电机自己的 sequence 在响应窗口内推进且状态/数值有效，就会进入健康反馈状态。
不要为了“对齐”而修改电机 ID、sequence 或放宽 `status_code` 检查。

若仍看到 `verified feedback pending: ...`，先用 SDK 只读测试确认每个电机的
`get_state_with_sequence()` 都推进，再重新启动 ROS 控制器；两者不能同时占用串口。
只有某个电机 sequence 长时间不变、持续 `NO_FEEDBACK` 或 `status_code != 0` 时，
才按底层电机反馈链路排查。

缺包时手动安装：

```bash
sudo apt update
sudo apt install ros-jazzy-moveit-simple-controller-manager
```

安装后需重新启动 `move_group`，已运行的进程不会自动恢复插件实例。
以下重启流程仅用于仿真：在原仿真启动终端 `Ctrl+C` 退出，再从新终端执行：

```bash
cd /home/huangbin/robotarm_ros2
source tools/source_local_environment.bash
export ROS_DOMAIN_ID=173
ros2 launch rebotarm_bringup mujoco_moveit_sim.launch.py
```

应不再出现插件加载失败或空控制器列表；确认唯一 Action server 后，
重新 Plan 并检查小目标轨迹，再手动 Execute，以实际执行结果和关节反馈验收。
包安装成功、源码 build、pytest 或 `--show-args` 通过都不能替代该运行验证。
真机不能直接套用上述 Ctrl+C 重启流程：先按本轮已验证基线受控停机，
确认可安全失能后再退出；健康但回位失败时保持 enabled hold、等待人工处置。

### 控制器依赖（真机或完整软件回归需要）

先在同一个终端加载 ROS 和当前工作区，再检查已经安装的控制器运行时：

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 tools/setup_motorbridge_fresh_feedback.py --check-installed
```

`requirements-runtime.txt` 中的普通 `motorbridge==0.4.7` 只用于 bootstrap，原始
PyPI 包没有本控制器所需的逐电机反馈 sequence，不能作为真机运行版本。不要在
patched 版本已经安装后再次执行该 requirements 文件，否则可能覆盖已审查版本。

如果检查失败，优先使用仓库已有的已构建 wheel：

```bash
python3 -m pip install --user --break-system-packages --force-reinstall --no-deps \
  build_motorbridge_fresh_feedback/wheel/motorbridge-0.4.7+rebotarm.1-*.whl
python3 tools/setup_motorbridge_fresh_feedback.py --check-installed
```

如果仓库没有 wheel，或需要从源码重建，先安装 Rust/Cargo、Git、Python venv 等
构建工具，然后执行：

```bash
# 只构建并在临时 venv 验证，不改用户 Python
python3 tools/setup_motorbridge_fresh_feedback.py --build-only

# 构建并显式安装已验证的 0.4.7+rebotarm.1 用户包
python3 tools/setup_motorbridge_fresh_feedback.py --install-user

# 启动 controller 前的 fail-closed 检查；不联网、不构建、不访问硬件
python3 tools/setup_motorbridge_fresh_feedback.py --check-installed
```

脚本固定上游 commit
`2b7b350914ace47ba06e85fcad333143de2b057b`（上游 `v0.4.7`），只接受仓库内已审查的 source patch，
并同时构建 `motor_abi`、`ws_gateway` 和 wheel。`--check-installed` 必须报告
`version=0.4.7+rebotarm.1 feedback_sequence=true`；否则不要启动真机 controller。
此版本还校验达妙反馈的 CAN ID、电机 ID 和完整 DLC，并要求置零前主动查询到
新的 status0 反馈；调用 disable 或新建句柄都不能代替状态确认。置零 API 不会
自动失能，调用方仍需完成置零后的新帧验收。上游 `v0.4.7` 的 `dm-serial`
单次底层读写超时为 10 ms（旧 `v0.4.6` 为 1 ms），并包含模式切换总预算和寄存器
写 ACK 校验改进；反馈陈旧和通信失败后的
保护失能策略不变。构建保留旧版本 wheel 以便回退。
重复安装 bootstrap 清单后也必须重新安装审查补丁并检查；不要把原始 0.4.7 用作真机运行版本。

厂商 SDK 使用仓库根目录的 `rebotarm_dependencies.repos` 固定版本：

```bash
mkdir -p third_party
vcs import third_party < rebotarm_dependencies.repos
```

也可以直接克隆：

```bash
git clone https://github.com/huangbinai/rebotarm_control.git \
  third_party/reBotArm_control_py
git -C third_party/reBotArm_control_py checkout \
  6a49302804f25e624995e771acb6d61896d1856d
```

`third_party/` 是本机依赖目录，不提交到主仓库。
仓库保留 `third_party/COLCON_IGNORE`，避免 `colcon` 扫描厂商 SDK 内部附带的
ROS 1 `catkin` 示例包。

## 2. 构建

在未激活venv/conda、未source旧工作区的全新终端执行。统一使用系统Python，
视觉安装脚本只准备依赖，不再以视觉venv重写控制器或ROS入口。

```bash
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 -m colcon build --base-paths src --executor sequential --symlink-install
source install/setup.bash
```

不需要连接设备，也不需要先提供YOLO/GraspNet权重。构建时存在的
`tools/yolo26s-seg.pt` 和旧默认engine会按原路径打包；不存在时不阻塞构建。
运行时显式传入外部模型路径无需重建。不要把“构建通过”等同于“推理资产已准备”。

安装控制器依赖并构建后可运行软件回归（不会授权真机动作）：

```bash
python3 tools/setup_motorbridge_fresh_feedback.py --check-installed
python3 -m pytest tests -q
```

历史材料及其专属测试仅本机保留，已有开发目录若要只验证发布范围，可用
`git ls-files -z tests | xargs -0 python3 -m pytest -q`。

### 可选MuJoCo运行环境

```bash
python3 -m venv --system-site-packages third_party/rebotarm_mujoco_venv
third_party/rebotarm_mujoco_venv/bin/python -m pip install -r requirements-mujoco.txt
export REBOTARM_MUJOCO_PYTHON="$PWD/third_party/rebotarm_mujoco_venv/bin/python"
```

不需要激活该venv或重新构建ROS包。仿真入口和模型见
`src/rebotarm_simulation/README_mujoco.md`。

## 3. 硬件配置

接入真机前必须按实际设备确认：

- `src/rebotarm_bringup/config/arm.yaml`：通信通道、电机 ID、型号和控制增益
- `src/rebotarm_bringup/config/gripper.yaml`：夹爪 ID、型号和控制增益
- `src/rebotarm_moveit_config/config/joint_limits.yaml`：规划速度、加速度和 jerk 限制
- `src/rebotarm_moveit_config/config/rebotarm.urdf`：几何、关节方向和物理模型
- `src/rebotarm_moveit_config/config/rebotarm.srdf`：规划组、命名姿态和碰撞矩阵

不要在没有核对电机型号和机械限位时修改或提高控制增益。

夹爪闭合端允许等效 `1 mm` 的反馈偏差：按 `5 rad ↔ 90 mm` 换算为
`0..0.0555555556 rad`（约3.18°），原始角度保留，显示开口为0；这不是
软件零点补偿，也不扩大运动目标（仍不大于0）。超过该容差仍拒绝；张开端
只允许既有量化容差。显式置零验收仍为两个反馈刻度约0.000763rad，不能复用
1 mm的闭合反馈容差。

## 4. 视觉配置

Ubuntu 物理机直连 Gemini2 并在本机运行 CUDA YOLO 时，使用：

```bash
./tools/setup_ubuntu_vision.sh
./tools/install_orbbec_udev_rules.sh
./tools/run_ubuntu_vision.sh yolo_model_path:="$PWD/tools/yolo26s-seg.pt" yolo_device:=0
```

上述命令只启动相机和YOLO，不启动机械臂。CPU可改为`yolo_device:=cpu`。
使用TensorRT前应在目标机器准备兼容engine并显式传入路径，安装脚本不会下载或导出模型。
GraspNet独立环境、模型输入和完整视觉launch见 `docs/setup/ubuntu_vision_setup_zh.md`。
逐进程解释器设置见 `docs/setup/launch_python_configuration.md`。

视觉相机配置使用 `src/rebotarm_vision/config/camera_ubuntu.yaml`，通过本机
Gemini 2 SDK 获取 RGB-D；不再配置远端 HTTP/MJPEG/JSON 服务。

手眼标定文件也可以独立覆盖：

```bash
ros2 launch rebotarm_vision vision.launch.py \
  handeye_config:=/absolute/path/to/my_handeye.yaml
```

旧的 `ordinary_grasp` 节点及其 launch 参数已退役；当前候选由本地 GraspNet 链路生成。

## 5. 安全顺序

1. 先运行 MoveIt/RViz 或 MuJoCo 无硬件演示，确认没有打开串口或 CAN。
2. 在真机旁验证已完成的 `connect()` / `enable()` 解耦：启动驱动不得自动上力。
3. 检查串口或 CAN、关节方向、零位、软限位和 enable 失败回滚。
4. 在失能状态验证 `joint_states`，再显式 enable 并保持当前位置。
5. 使用低速度验证停止服务、单关节小角度和单个安全姿态。
6. 依次开放网页执行、示教回放、视觉 plan-only 和视觉低速执行。

完整视觉抓取的目标安全默认值是：

```text
use_hardware:=false
execution_mode:=plan_only
move_to_visual_ready_on_start:=false
shutdown_safe_home:=false
auto_enable:=false
```

这些默认值已经落地；实机执行仍必须显式选择硬件后端并调用 `/rebotarm/enable`，
且针对当前机器完成分级预检。旧阶段的验收结果不自动适用于新设备。

## 6. 历史 P0 验收工具

P0 Gate B/C 自动使能、保持与失能验收工具已于 2026-09-19 移除，不再作为日常测试步骤。
历史报告已从工作树清理；如需追溯，可从 Git 历史恢复。它们仅用于历史审计，不代表当前硬件通过验收。
控制器的显式使能、反馈校验、失败回滚和停止保护继续保留。
真机操作与停机步骤见 [功能命令参考](../reference/commands/rebotarm_feature_commands.md#moveit-实机操作与停机)。

## 可选：MuJoCo 强化学习

按 [强化学习命令参考](../reference/commands/mujoco_rl.md) 安装根目录 `requirements-rl.txt`，并检查 Reach 环境。纯离线训练无需启动 ROS 或连接真机。

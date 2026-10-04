# 启动解释器显式配置

> 状态：SETUP；类型：启动解释器和环境配置契约；适用范围：MuJoCo、视觉和 GraspNet 进程。

2026-09-06 起，MuJoCo、视觉、GraspNet 的 ROS launch 不再向上搜索工作区内的
`.venv-*` 或 `third_party/rebotarm_mujoco_venv`，也不向整组节点注入视觉
`site-packages/PYTHONPATH`。每类 Python 节点使用自己的解释器 prefix。

选择优先级：显式 launch 参数 > 对应环境变量 > `PATH` 中的 `python3`。
解释器必须具有该节点所需依赖，并能读取 source ROS/工作区后提供的 ROS 包路径。

| 节点 | 环境变量 | launch 参数 |
| --- | --- | --- |
| MuJoCo | `REBOTARM_MUJOCO_PYTHON` | 仿真包入口 `python_executable`；bringup 混合入口 `mujoco_python_executable` |
| 相机、普通候选、TCP frame、离线 YOLO | `REBOTARM_VISION_PYTHON` | `vision_python_executable` |
| in-process GraspNet | `GRASPNET_PYTHON` | `graspnet_python_executable` |

这些参数分别传递给对应进程；不会修改 MoveIt、控制器或其他节点的 Python 环境。
环境变量设置在启动前完成，路径允许放在工作区外。未设置时，依赖需安装在当前
`PATH` 所选 Python 中；不再自动切换到恰好在邻近目录找到的虚拟环境。

本机现有环境可在终端中明确选择（仅配置环境，不启动节点）：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export REBOTARM_MUJOCO_PYTHON="$PWD/third_party/rebotarm_mujoco_venv/bin/python"
export REBOTARM_VISION_PYTHON="$PWD/.venv-vision/bin/python"
export GRASPNET_PYTHON="$PWD/.venv-graspnet/bin/python"
```

日常使用也可以直接执行：

```bash
source tools/source_local_environment.bash
```

该脚本还会默认设置本工作区的 `GRASPNET_MODEL_ROOT` 和
`GRASPNET_CHECKPOINT_PATH`；如果当前 shell 已经显式设置过这两个变量，则保留已有值。

整合前的工作树分别使用 `install_decoupling` 和 `install_coupling_audit` 验证。
整合后主目录独立构建全部包，不再依赖工作树overlay。主目录重建命令为：

```bash
# 从当前仓库根目录、未激活虚拟环境的新终端执行
unset PYTHONPATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 -m colcon build --base-paths src --executor sequential --symlink-install
```

模型不再是构建前置条件。构建时存在的旧默认engine和PT模型会可选打包，
缺失则不打包；运行时启用检测仍必须提供可读且兼容的模型。
独立视觉使用`yolo_model_path`，完整视觉使用`vision_yolo_model_path`覆盖路径。
模型准备和PT启动示例见 [视觉环境说明](ubuntu_vision_setup_zh.md)。
`tools/setup_ubuntu_vision.sh`只安装依赖；`tools/run_ubuntu_vision.sh`只为视觉
设置解释器环境变量，不再激活venv或注入全局PYTHONPATH。

部署到其他机器时，改为那台机器实际安装的解释器路径即可。也可以在每次 launch
中设置表中的参数覆盖环境变量。GraspNet 模型根目录、checkpoint、相机配置和
MuJoCo 模型资源的选择方式保持原有契约；本次只解除解释器的目录布局耦合。

覆盖入口：

- `rebotarm_simulation`: `mujoco_sim.launch.py`
- `rebotarm_bringup`: `visual_grasp_system.launch.py`、`mujoco_moveit_sim.launch.py`
- `rebotarm_vision`: `vision.launch.py`、`vision_ubuntu.launch.py`

旧 `mujoco_ros_adapter_node.py` 和快照进程转发器 `upstream_backend.py` 已退出
活动源码包，`rebotarm_simulation` 不再依赖 `rebotarm_motion`。正式 ROS 入口仍是
`rebotarm_mujoco_node = rebotarm_simulation.ros.mujoco_ros_node:main`，ROS action/service
接口不变。历史源码可从 Git 历史追溯，本机参考归档不再随源码发布。
模型分析、指标和独立离线工具仍使用的公共模块继续保留。

验证结果：全量测试 `761 passed, 8 skipped`，分层20通过；从当前活动源码导入
正式 MuJoCo 后端执行轨迹/停止/取消等测试55通过。三个包独立构建成功，8个已安装
launch均通过 `--show-args`；搬迁目录测试覆盖默认、环境变量和显式参数优先级。
旧模块在新overlay中不可导入。以上均为软件验证，未启动硬件或发送运动命令。

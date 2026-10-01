# Python 依赖版本与安装

> 状态：SETUP；类型：依赖文档索引；适用范围：Ubuntu 24.04 / ROS 2 Jazzy / Python 3.12。

## 当前范围

六份依赖清单已从独立 TXT 文件转换为本目录的 Markdown 文档。
每份文档的单个 `requirements` 代码块是依赖版本的唯一来源，正文说明不参与安装。
安装工具直接将解析后的参数传给指定 Python 的 pip，无需另存临时清单。

| 文档 | 用途 | 环境/入口 |
| --- | --- | --- |
| [runtime.md](runtime.md) | MotorBridge 控制器基础依赖 | 系统 Python，安装后仍需反馈序号补丁 |
| [vision.md](vision.md) | Gemini 2、YOLO、OpenCV | `.venv-vision`；`scripts/setup_ubuntu_vision.sh` |
| [graspnet.md](graspnet.md) | GraspNet、点云及 CUDA PyTorch | `.venv-graspnet`；`scripts/setup_ubuntu_graspnet.sh` |
| [mujoco.md](mujoco.md) | MuJoCo 仿真与模型工具 | `third_party/rebotarm_mujoco_venv` |
| [rl.md](rl.md) | CPU Reach 和可选 CUDA SB3/PPO | `third_party/rebotarm_rl_venv` |
| [tensorrt.md](tensorrt.md) | TensorRT 推理运行库 | `.venv-vision`；单独使用 `--no-deps` |

## 使用方式

从仓库根目录执行；`--show` 只读显示实际命令，不启动 pip：

```bash
python3 scripts/install_python_dependencies.py mujoco --show
```

实际安装时显式指定目标环境，其他选项见 `--help`：

```bash
python3 scripts/install_python_dependencies.py mujoco --python third_party/rebotarm_mujoco_venv/bin/python
```

不要直接将带标题、说明和代码围栏的 Markdown 交给 `pip -r`。
`rl.md` 的 `-r mujoco.md` 由工具按当前文档目录解析；PyTorch 索引和 TensorRT
`--no-deps` 语义保留。各功能环境版本不同，不合并安装到同一环境。
已有 MotorBridge 补丁版本时不要再次执行 runtime bootstrap；依赖安装不构成实机验收。

MJX 专属依赖已删除；MJLab 尚未引入。
`src/rebotarm_simulation/requirements-mujoco.txt` 是仿真包独立发布的最低兼容声明，
本机复现使用本目录固定版本。

系统/ROS、虚拟环境和构建步骤见 [部署说明](../ubuntu_ros2_jazzy.md)。
厂商源码版本见 `third_party/rebotarm_dependencies.repos`；第三方来源/许可见[第三方说明](../../reference/third_party_notices.md)。

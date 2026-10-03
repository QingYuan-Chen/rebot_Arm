# Ubuntu 原生 Gemini2 与 YOLO 环境

> 状态：SETUP；类型：Ubuntu 视觉环境和启动说明；适用范围：Gemini 2、YOLO、GraspNet；真实设备结论需看最新证据。

> 当前状态：Ubuntu原生Gemini2、YOLO和同进程GraspNet已集成到
> `visual_grasp_system.launch.py` 的 `vision_profile:=ubuntu_native`。
> P0-P6已按当前范围关闭；新设备仍需复核相机、模型、手眼标定和现场条件，
> 历史验收不构成新一轮真机动作授权。

本机方案不再依赖 Windows HTTP 服务，数据链路为：

```text
Gemini2 USB -> pyorbbecsdk -> rebotarm_vision_node
             -> Ultralytics YOLO/CUDA -> ROS 2 topics
```

Windows 服务及远端 HTTP 输入已删除。Ubuntu 原生链路使用 `camera_ubuntu.yaml` 和
`vision_ubuntu.launch.py`。

## 1. 安装

先按 [本机安装说明](ubuntu_ros2_jazzy.md) 安装系统依赖并构建ROS工作区，
再从仓库根目录安装视觉运行环境：

```bash
./tools/setup_ubuntu_vision.sh
```

脚本创建 `.venv-vision`，固定安装 PyTorch `2.11.0+cu128`、torchvision
`0.26.0+cu128`、Ultralytics、TensorRT和Orbbec Python SDK，并检查导入。
它不构建ROS包、不下载模型、不导出engine、不连接相机或控制器。
所有ROS包统一用系统Python构建，运行时再按节点指定解释器。

这里固定 `numpy==1.26.4` 和 `pyorbbecsdk2==2.0.18`。不要直接升级到
`pyorbbecsdk2==2.1.1`，它在 Python 3.12 上要求 NumPy 2，与 ROS 2 Jazzy
系统包中的 `cv_bridge` ABI 不兼容。

## 2. 相机权限

接入 Gemini2 后执行一次：

```bash
./tools/install_orbbec_udev_rules.sh
```

该步骤需要输入 sudo 密码。完成后拔插相机，并检查：

```bash
lsusb | grep -i -E "orbbec|2bc5"
```

## 3. 启动

### 新部署：直接使用已有PT模型

```bash
./tools/run_ubuntu_vision.sh \
  yolo_model_path:="$PWD/tools/yolo26s-seg.pt" yolo_device:=0
```

无需TensorRT engine即可启动相机/YOLO；它不是已验证YOLO26m engine的等效性能或精度验收。
无CUDA时可用同一PT模型配合`yolo_device:=cpu`，速率需重新评估。
包装脚本默认通过`REBOTARM_VISION_PYTHON`选择`.venv-vision/bin/python`，
不激活venv、不向整组ROS节点注入视觉site-packages；外部环境可设置该变量覆盖。

当前相机配置：

- Gemini2 彩色流 `640x480 @ 30 FPS`
- Gemini2 深度原始流 `640x400 @ 30 FPS`，硬件对齐到彩色图
- 硬件深度对齐
- NVIDIA GPU `device=0`
- 检测发布 `/grasp/detections`
- 标注图发布 `/camera/color/annotated`

### 可选：部署目标机器的TensorRT engine

使用来源已确认、适配目标GPU及CUDA/TensorRT版本的engine，通过绝对路径指定：

```bash
./tools/run_ubuntu_vision.sh \
  yolo_model_path:=/absolute/path/to/reviewed-model.engine yolo_device:=0
```

engine不能用于CPU，也不能假定可跨机器复用。需要重新导出时，必须使用
对应的原始权重及目标环境；不能将YOLO26s导出结果改名冒充YOLO26m。
仓库不提供YOLO26m源权重的自动下载或engine导出流程，未准备好时使用上面的PT路径。
当前本机默认使用包内 `models/yolo26s-seg.pt`，因为仓库实际提供该模型。
如果以后准备好已审核的 TensorRT engine，可以通过 launch 参数显式覆盖；
外部模型无需重建，运行时模型缺失将报错，不静默换模型。

## 4. 验证

```bash
.venv-vision/bin/python -c \
  "import torch; print(torch.cuda.is_available(), torch.cuda.get_device_name(0))"

ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
ros2 topic echo /grasp/detections --once
```

当前验证还必须补充：

```bash
ros2 topic echo /camera/color/camera_info --once
ros2 topic echo /camera/depth/camera_info --once
```

验收时记录而不是估算：

- 设备名称、序列号、固件版本和 USB 类型；
- 实际 RGB/depth profile 与格式；
- SDK intrinsics、distortion、extrinsics 和 depth scale；
- 深度中心点及多个已知距离样本的误差；
- 硬件对齐后 RGB 与 depth 是否对应同一像素坐标系。

在驱动尚未提供 SDK 标定信息前，`CameraInfo` 出现并不等于标定正确。原始深度也不能默认视为毫米，必须先确认设备 `depth_scale`。

如果启动时报找不到相机，优先检查 USB 枚举和 udev 规则。如果 CUDA 不可用，
检查 `nvidia-smi`，不要先安装系统级 CUDA Toolkit；PyTorch wheel 已包含运行时。

该入口只覆盖 Gemini 2 + YOLO，不代表 Ubuntu 本地 GraspNet、hand-eye 标定或完整抓取执行已经完成。

## 5. 查看 Gemini 2 点云

首次使用安装 ROS 2 深度图转点云组件：

```bash
sudo apt install ros-jazzy-depth-image-proc
```

保持 `./tools/run_ubuntu_vision.sh` 运行，在第二个终端启动转换节点：

```bash
source /opt/ros/jazzy/setup.bash
source ~/robotarm_ros2/install/setup.bash
ros2 run depth_image_proc point_cloud_xyz_node --ros-args \
  -r image_rect:=/camera/depth/image_raw \
  -r camera_info:=/camera/depth/camera_info \
  -r points:=/camera/depth/points
```

第三个终端运行 `rviz2`，将 `Fixed Frame` 设为 `camera_depth_frame`，添加
`PointCloud2` display 并选择 `/camera/depth/points`。视觉节点让 Image 与对应的
CameraInfo 使用相同 QoS，避免 `depth_image_proc` 因 reliability 不兼容而收不到
同步内参。该路径只读取相机，不启动机械臂控制器。

如需查看候选，请直接订阅 `/grasp/graspnet_candidates` 或使用 RViz 的 ROS
可视化节点；该路径不属于 Ubuntu 抓取执行链路。

## 6. P4 Ubuntu 本地 GraspNet 环境

P4 使用与 `.venv-vision` 隔离的 `.venv-graspnet`，仅安装经过固定的
PyTorch/CUDA 和 GraspNet 推理依赖。安装和健康检查均为本地操作，不会连接机械臂、
打开串口、enable 电机或发送轨迹：

```bash
./tools/setup_ubuntu_graspnet.sh
PYTHONPATH= ./.venv-graspnet/bin/python tools/check_ubuntu_graspnet_env.py
PYTHONPATH= ./.venv-graspnet/bin/python -m pip check
```

当前仓库没有随附 GraspNet 模型代码或 checkpoint。安装脚本不会自动下载第三方
模型；只有在确认来源、版本和 license 后，才设置：

```bash
export GRASPNET_MODEL_ROOT=/path/to/reviewed/graspnet
export GRASPNET_CHECKPOINT_PATH=/path/to/reviewed/checkpoint.pth
```

Ubuntu生产路径不再单独启动 HTTP service。相机/YOLO topics 已启动后，可在
ROS 2环境中单独验证整合后的in-process node：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export GRASPNET_PYTHON="$PWD/.venv-graspnet/bin/python"
ros2 run --prefix "$GRASPNET_PYTHON" rebotarm_vision rebotarm_graspnet_baseline_node --ros-args \
  --params-file install/rebotarm_vision/share/rebotarm_vision/config/graspnet_ubuntu.yaml
```

该node直接将时间匹配的aligned RGB、米制depth、CameraInfo intrinsics、YOLO mask/bbox、
timestamp和`camera_depth_frame`交给同进程GraspNet runner，再把结果发布到
`/grasp/graspnet_candidates`。它保留完整场景collision cloud、目标mask/depth分离、
确定性20,000点采样、projection filter与jaw-width filter。RGB/depth skew超限、模型未配置
或推理异常时只发布空候选，不复用旧结果。

新部署的依赖导入或旧service liveness通过不等于真实推理通过；必须核实自己的模型资产，
并验证当前输入下的候选输出。模型代码、pointnet2扩展、graspnetAPI和checkpoint需
与所选GraspNet版本匹配，环境安装脚本不负责提供这些第三方资产。

## 7. 完整视觉链路（无真机、仿真执行）

先设置上节的GraspNet模型根目录和checkpoint。
在没有其他相机/仿真节点运行时执行；不要与第3节的相机入口重复启动：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export REBOTARM_VISION_PYTHON="$PWD/.venv-vision/bin/python"
export GRASPNET_PYTHON="$PWD/.venv-graspnet/bin/python"
ros2 launch rebotarm_bringup visual_grasp_system.launch.py \
  vision_profile:=ubuntu_native use_hardware:=false \
  vision_yolo_model_path:="$PWD/tools/yolo26s-seg.pt"
```

这会使用真实相机和RViz-only运动学后端，不启动MuJoCo物理仿真或真机控制器；
真实相机相对模拟机器人TF是否有物理意义需自行核验，候选规划不代表实机可执行。
独立的“真实感知 + MuJoCo 执行”组合入口已删除；当前只保留本节的
真实感知与仿真轨迹执行验证以及独立的 MuJoCo 离线仿真入口。
真实执行另见功能手册和现场授权边界。

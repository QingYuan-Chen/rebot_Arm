# Ubuntu 原生 D405 与 YOLO 环境

> 状态：SETUP；类型：Ubuntu 视觉环境和启动说明；适用范围：D405、YOLO、GraspNet；真实设备结论需看最新证据。

> 2026-11-05：腕部相机切换为 D405，视觉总入口保留感知、筛选与显示。
> 默认手眼外参为空，完成 D405 重标定后显式传入。真实设备已完成出图检查，深度精度和标定待现场验收，
> 软件检查结果统一记录于[项目状态](../reference/project_status.md)。

本机方案不再依赖 Windows HTTP 服务，数据链路为：

```text
D405 USB -> pyrealsense2 -> rebotarm_vision_node
             -> Ultralytics YOLO/CUDA -> ROS 2 topics
```

Windows 服务及远端 HTTP 输入已删除。Ubuntu 原生链路使用 `camera_ubuntu.yaml` 和
`vision_ubuntu.launch.py`。

## 1. 安装

先按 [本机安装说明](ubuntu_ros2_jazzy.md) 安装系统依赖并构建ROS工作区，
再从仓库根目录安装视觉运行环境：

```bash
./scripts/setup_ubuntu_vision.sh
```

脚本创建 `.venv-vision`，固定安装 PyTorch `2.11.0+cu128`、torchvision
`0.26.0+cu128`、Ultralytics、TensorRT 和 RealSense Python SDK，并检查导入。
它不构建ROS包、不下载模型、不导出engine、不连接相机或控制器。
所有ROS包统一用系统Python构建，运行时再按节点指定解释器。

这里固定 `numpy==1.26.4` 以兼容 ROS 2 Jazzy 的 `cv_bridge`，
相机 SDK 为 `pyrealsense2==2.59.1.11053`。完整版本由[视觉依赖清单](dependencies/vision.md)管理。
已有视觉环境仅补装相机 SDK 时可执行：

```bash
.venv-vision/bin/python -m pip install --no-deps pyrealsense2==2.59.1.11053
```

## 2. 相机权限

按 [RealSense Linux 安装说明](https://github.com/realsenseai/librealsense/blob/master/doc/distribution_linux.md)
配置 SDK apt 源后，安装 USB 规则与枚举工具：

```bash
sudo apt install librealsense2-udev-rules librealsense2-utils
```

该步骤需要输入 sudo 密码。完成后拔插相机，并检查：

```bash
rs-enumerate-devices -s
```

## 3. 启动

### 新部署：直接使用已有PT模型

```bash
./scripts/run_ubuntu_vision.sh \
  yolo_model_path:="$PWD/src/rebotarm_vision/models/yolo26s-seg.pt" yolo_device:=0
```

无需TensorRT engine即可启动相机/YOLO；它不是已验证YOLO26m engine的等效性能或精度验收。
无CUDA时可用同一PT模型配合`yolo_device:=cpu`，速率需重新评估。
包装脚本默认通过`REBOTARM_VISION_PYTHON`选择`.venv-vision/bin/python`，
不激活venv、不向整组ROS节点注入视觉site-packages；外部环境可设置该变量覆盖。

当前相机配置：

- D405 彩色流 `640x480 @ 30 FPS`，SDK 输出 BGR8
- D405 深度原始流 `640x480 @ 30 FPS`，Z16 按实际比例尺转为毫米 uint16
- RealSense SDK 将深度对齐到彩色光学系，对齐深度使用彩色内参
- NVIDIA GPU `device=0`
- 检测发布 `/grasp/detections`
- 标注图发布 `/camera/color/annotated`

相机档位是请求值，SDK 在设备不支持时直接报错。驱动使用同步 frameset，
缺流或超时保留空检测；实际 USB 带宽、精度和帧率由现场核查确认。

### D405 手眼重标定

独立相机入口默认只发布相机内部光学帧及 TCP 帧，机器人到相机的外参需显式提供。
`config/handeye.yaml` 保留旧 Gemini2 原始数值用于追溯。用现有
[标定入口](../reference/commands/calibration_web.md)重新采集 D405 数据：
彩色图与内参的话题保持默认，`camera_frame` 选 `camera_color_frame`。
将核验后的求解结果整理为以下已有格式（数值使用实际结果）：

```yaml
handeye:
  parent_frame: end_link
  child_frame: camera_color_frame
  translation: {x: <米>, y: <米>, z: <米>}
  rotation: {x: <qx>, y: <qy>, z: <qz>, w: <qw>}
```

独立相机入口通过 `handeye_config:=<路径>/d405_handeye.yaml` 加载；
总入口使用 `vision_handeye_config:=<路径>/d405_handeye.yaml`。
本次上游方法结果已写入包配置 `src/rebotarm_vision/config/d405_handeye.yaml`，
安装后位于 `install/rebotarm_vision/share/rebotarm_vision/config/d405_handeye.yaml`；
默认仍为空，需显式加载，避免未完成现场精度验收的外参自动生效。
对齐后的 `camera_color_frame` 与 `camera_depth_frame` 表示同一光学系，
launch 将两者连接为同位姿；手眼文件也可使用 `camera_depth_frame`。
URDF 的 `d405_link` 为 CAD 装配帧，光学外参使用实测标定。
手眼参数为空时保留相机、YOLO 和相机系候选显示，机器人系筛选因缺 TF 拒绝候选。

### 可选：部署目标机器的TensorRT engine

使用来源已确认、适配目标GPU及CUDA/TensorRT版本的engine，通过绝对路径指定：

```bash
./scripts/run_ubuntu_vision.sh \
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
- SDK 对齐后 RGB 与 depth 是否对应同一像素坐标系。

驱动从实际流 profile 读取内参、畸变及出厂外参，深度比例尺读取设备和每帧 SDK 数据。
现场仍须核对 CameraInfo 尺寸、已知距离误差与对齐效果。

深度探针 `rebotarm_grasp_depth_probe` 默认订阅 `/grasp/graspnet_candidates`，
诊断区间按 [D405 产品规格](https://www.intel.com/content/www/us/en/products/sku/229218/intel-realsense-depth-camera-d405/specifications.html)
取 0.07–0.50 m；现场按测量结果调整诊断阈值。正式 GraspNet 深度裁剪和候选策略保留现有参数，
该统计结果用于诊断查看。

如果启动时报找不到相机，优先检查 USB 枚举和 udev 规则。如果 CUDA 不可用，
检查 `nvidia-smi`，不要先安装系统级 CUDA Toolkit；PyTorch wheel 已包含运行时。

该入口覆盖 D405 + YOLO；本地 GraspNet 与机器人系候选筛选按下节及总入口分别验证。

## 5. 查看 D405 点云

首次使用安装 ROS 2 深度图转点云组件：

```bash
sudo apt install ros-jazzy-depth-image-proc
```

保持 `./scripts/run_ubuntu_vision.sh` 运行，在第二个终端启动转换节点：

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
可视化节点；该路径用于独立可视化诊断。

## 6. P4 Ubuntu 本地 GraspNet 环境

P4 使用与 `.venv-vision` 隔离的 `.venv-graspnet`，仅安装经过固定的
PyTorch/CUDA 和 GraspNet 推理依赖。安装和健康检查均为本地操作，不会连接机械臂、
打开串口、enable 电机或发送轨迹：

```bash
./scripts/setup_ubuntu_graspnet.sh
PYTHONPATH= ./.venv-graspnet/bin/python scripts/check_ubuntu_graspnet_env.py
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

## 7. 完整视觉候选链路（离线筛选）

先设置上节的GraspNet模型根目录和checkpoint。
在没有其他相机/仿真节点运行时执行；不要与第3节的相机入口重复启动：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export REBOTARM_VISION_PYTHON="$PWD/.venv-vision/bin/python"
export GRASPNET_PYTHON="$PWD/.venv-graspnet/bin/python"
ros2 launch rebotarm_bringup visual_grasp_system.launch.py \
  robot_context:=offline \
  vision_handeye_config:="$PWD/install/rebotarm_vision/share/rebotarm_vision/config/d405_handeye.yaml" \
  vision_yolo_model_path:="$PWD/src/rebotarm_vision/models/yolo26s-seg.pt"
```

这会使用真实相机、被动关节状态和 MoveIt 查询；先核验 D405 手眼结果。接入已有 MuJoCo 环境时使用 `robot_context:=external use_sim_time:=true`。详细命令见[视觉候选命令](../reference/commands/visual_grasp_commands.md)。

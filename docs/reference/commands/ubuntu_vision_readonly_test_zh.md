# Ubuntu 视觉只读测试手册

> 状态：REFERENCE；类型：只读视觉验证；适用范围：Gemini 2/YOLO/GraspNet；不授权机械臂运动。

本手册只启动 Gemini 2、YOLO、ROS RGB-D/CameraInfo/detection 和本地 GraspNet
候选发布，不启动机械臂控制器，不调用 enable，不发送轨迹。

## 前置条件与环境准备

```bash
cd /home/a/project/rebot_Arm
source /opt/ros/jazzy/setup.bash
source install/setup.bash

./scripts/setup_ubuntu_vision.sh
./scripts/setup_ubuntu_graspnet.sh
```

安装脚本输出 `CUDA available: True`、显卡名称、`GraspNet environment imports: OK`
和 `No broken requirements found.` 即表示 Python 环境检查通过。它不会下载
GraspNet 模型；必须先准备经过确认的模型资产：

```bash
export GRASPNET_MODEL_ROOT=/absolute/path/to/graspnet-model-root
export GRASPNET_CHECKPOINT_PATH=/absolute/path/to/checkpoint-rs.tar
```

## 操作步骤：启动视觉只读链路

模型路径和推理设备使用仓库默认值，不需要再输入参数：

```bash
./scripts/run_ubuntu_vision.sh
```

默认值为：

- YOLO：安装包内的 `yolo26s-seg.pt`
- YOLO 设备：`0`，即 CUDA 第一张 GPU
- YOLO 抓取白名单：`bottle`、`cup`；其他类别不会发布到 `/grasp/detections`
- GraspNet 设备：`cuda:0`
- RGB-D：`/camera/color/image_raw`、`/camera/depth/image_raw`
- YOLO 检测：`/grasp/detections`
- GraspNet 候选：`/grasp/graspnet_candidates`

如果安装包内没有模型，启动会明确报模型路径错误；不要通过静默切换到
未知模型继续验收。更换模型或机器时，使用完整 launch 参数覆盖即可，但不属于
日常测试命令。

## 检查：YOLO 分割画面

另开终端：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run rqt_image_view rqt_image_view /camera/color/annotated
```

同时检查话题是否持续发布：

```bash
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
ros2 topic hz /camera/color/annotated
ros2 topic echo /grasp/detections --once
ros2 topic echo /camera/color/camera_info --once
ros2 topic echo /camera/depth/camera_info --once
```

正常现象是 RGB、depth、annotated 频率持续更新，分割图上出现检测框/掩码，
检测消息的时间戳和 frame_id 有效。此步骤不代表 GraspNet 已成功。

## 检查：GraspNet 候选和夹爪位姿

检查候选消息：

```bash
ros2 topic echo /grasp/graspnet_candidates --once
ros2 topic hz /grasp/graspnet_candidates
```

相机只读诊断使用已有的视觉专用入口，不启动控制器、MoveIt 或抓取执行器：

```bash
./scripts/run_ubuntu_vision.sh
```

上述入口负责相机、YOLO 和 TF；GraspNet/Open3D 与仿真执行组合参见
[视觉抓取命令](visual_grasp_commands.md)，不能把该执行组合当作相机只读入口。

## 验证顺序与判定

```text
相机 RGB/depth 持续发布
-> CameraInfo 有效
-> YOLO annotated 图像正常
-> Detection2DArray 有检测
-> GraspNet backend_available=True
-> graspnet_candidates 有非空候选
-> RViz 中夹爪位姿和 TF 方向合理
```

如果 YOLO 图像正常但候选为空，优先检查：

```bash
echo "$GRASPNET_MODEL_ROOT"
echo "$GRASPNET_CHECKPOINT_PATH"
ros2 topic echo /grasp/detections --once
```

候选为空时仍属于 fail-closed，不得因此直接开放真机执行。

## 停止与安全

使用 `Ctrl-C` 停止视觉终端和 RViz。整个手册流程不需要连接机械臂，
也不需要调用 `/rebotarm/enable`。

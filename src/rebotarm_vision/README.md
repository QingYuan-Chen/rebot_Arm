# rebotarm_vision

Ubuntu 原生视觉包提供 D405 RGB-D 与 YOLO 检测、本地 GraspNet 候选、基于 TF/IK/碰撞和工作空间的筛选，以及 RViz 和 Open3D 显示。操作与参数见[视觉候选命令](../../docs/reference/commands/visual_grasp_commands.md)。

D405 驱动使用 `pyrealsense2`，请求彩色 BGR8 与深度 Z16 `640x480@30`，按设备比例尺输出毫米深度并对齐彩色。图像、CameraInfo 和候选话题沿用现有接口。默认 `handeye_config` 为空；完成新手眼标定后显式加载，旧 `config/handeye.yaml` 仅保留作 Gemini2 历史记录。2026-11-06 的上游方法 D405 候选外参保存在 `config/d405_handeye.yaml`，仍需显式传入。安装与现场核查见[视觉环境](../../docs/setup/ubuntu_vision_setup_zh.md)。

- `camera/`、`detector/`、`converters/`：图像、深度与检测消息。
- `backends/`、`graspnet_baseline_adapter.py`：本机 GraspNet 推理。
- `nodes/candidate_ik_filter_node.py`、`candidate_ik_*`、`policies/candidate_*`：候选校验、姿态变体、MoveIt 查询和排序。
- `candidate_pose.py`、`utils/tf_message_adapter.py`：候选姿态值及共享 TF 几何。
- `nodes/grasp_candidate_marker_node.py`、`nodes/visual_grasp_marker_node.py`、`nodes/grasp_tcp_frame_node.py`：候选、筛选结果和 TCP 显示。
- `diagnostics/`、`offline_yolo.py`：相机、深度、Open3D 和离线检测工具。
- `config/`：相机、GraspNet、手眼、候选策略配置。

视觉总入口 `visual_grasp_system.launch.py` 支持独立离线筛选和接入已有机器人环境。`vision_ubuntu.launch.py`、`vision.launch.py` 提供感知入口。模型或输入依赖不可用时按节点现有校验规则报错或拒绝候选。

检测模型未提供分割掩膜时保留框检测；已提供的掩膜须包含至少三个有限的二维顶点。
缺失、索引错误、形状或数值损坏会由共享转换器明确拒绝，节点发布同帧空检测并记录原因，
GraspNet 收到后清空候选。在线按类别白名单筛选后转换，离线保留完整 raw 诊断输出，
任一参与转换的掩膜损坏都会拒绝本次转换；后续有效帧可恢复发布。

`offline_yolo_node` 订阅外部发布或回放的 RGB 图像，默认输出标注图。解码、推理或检测转换
失败时 raw/filtered 两路同帧清空；标注绘制或发布故障向执行器传播，已发布检测保留该帧语义。

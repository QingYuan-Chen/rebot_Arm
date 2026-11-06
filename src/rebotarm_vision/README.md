# rebotarm_vision

Ubuntu 原生视觉包提供 D405 RGB-D 与 YOLO 检测、本地 GraspNet 候选、基于 TF/IK/碰撞和工作空间的筛选，以及 RViz 和 Open3D 显示。操作与参数见[视觉候选命令](../../docs/reference/commands/visual_grasp_commands.md)。

D405 驱动使用 `pyrealsense2`，请求彩色 BGR8 与深度 Z16 `640x480@30`，按设备比例尺输出毫米深度并对齐彩色。图像、CameraInfo 和候选话题沿用现有接口。默认 `handeye_config` 为空；完成新手眼标定后显式加载，旧 `config/handeye.yaml` 仅保留作 Gemini2 历史记录。安装与现场核查见[视觉环境](../../docs/setup/ubuntu_vision_setup_zh.md)。

- `camera/`、`detector/`、`converters/`：图像、深度与检测消息。
- `backends/`、`graspnet_baseline_adapter.py`：本机 GraspNet 推理。
- `nodes/candidate_ik_filter_node.py`、`candidate_ik_*`、`policies/candidate_*`：候选校验、姿态变体、MoveIt 查询和排序。
- `candidate_pose.py`、`utils/tf_message_adapter.py`：候选姿态值及共享 TF 几何。
- `nodes/grasp_candidate_marker_node.py`、`nodes/visual_grasp_marker_node.py`、`nodes/grasp_tcp_frame_node.py`：候选、筛选结果和 TCP 显示。
- `diagnostics/`、`offline_yolo.py`：相机、深度、Open3D 和离线检测工具。
- `config/`：相机、GraspNet、手眼、候选策略配置。

视觉总入口 `visual_grasp_system.launch.py` 支持独立离线筛选和接入已有机器人环境。`vision_ubuntu.launch.py`、`vision.launch.py` 提供感知入口。模型或输入依赖不可用时按节点现有校验规则报错或拒绝候选。

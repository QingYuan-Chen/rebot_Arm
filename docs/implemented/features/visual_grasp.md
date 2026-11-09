# 视觉候选生成与筛选

> 状态：IMPLEMENTED；类型：视觉候选功能说明；适用范围：候选生成、筛选和显示。

D405 的 RGB-D/CameraInfo 与 YOLO 检测结果输入本地 GraspNet。候选经过置信度、开度、传感器时间 TF、工作空间、IK、碰撞和关节行程检查，发布 `/grasp/filtered_candidates`、`/grasp/filtered_plan` 和 RViz Marker。缺失有效输入或查询依赖时按筛选规则拒绝候选。

显式启用的 YOLO 开放词表及 GraspNet 模型、权重、NMS、启用的碰撞过滤依赖在启动时加载；加载失败时报告原因。推理产物损坏时清空该帧候选及显示，正常空候选保持原有语义。

D405 通过 RealSense SDK 采集，深度按实际比例尺转为毫米并对齐彩色图。手眼外参默认为空；机器人坐标下的筛选需显式传入新标定文件，配置和步骤见[视觉环境](../../setup/ubuntu_vision_setup_zh.md)。

`visual_grasp_system.launch.py` 的 `robot_context:=offline` 使用被动关节状态与 MoveIt 查询服务；`robot_context:=external` 接入已有的模型、TF、关节状态和 MoveIt 环境。使用 MuJoCo 时需设置 `use_sim_time:=true`。两种组合均用于感知、筛选与显示。

筛选继续使用现有策略 profile 的有效值。独立夹爪位置控制、通用运动、轻量预演和 MuJoCo 入口保持各自职责。操作命令见 [视觉候选命令](../../reference/commands/visual_grasp_commands.md)，项目验证进度见[项目状态](../../reference/project_status.md)。

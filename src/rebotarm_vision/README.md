# rebotarm_vision

Ubuntu 原生视觉与抓取候选包。维护链路为 Gemini 2/RGB-D/CameraInfo → YOLO 检测 → 本机 GraspNet → 候选筛选/IK/工作空间/碰撞门 → MoveIt 规划接口。旧 Windows、HTTP、MJPEG、远程 JSON 和独立 GraspNet 服务路线不属于当前包。已有运行说明见 [`README_zh.md`](README_zh.md)。

## 目录结构

```text
rebotarm_vision/
├── rebotarm_vision/
│   ├── camera/
│   │   ├── base.py                  # 相机抽象
│   │   └── gemini2_driver.py        # Gemini 2 图像/深度/CameraInfo 发布
│   ├── detector/
│   │   └── yolo_detector.py         # YOLO 推理和检测结果
│   ├── converters/
│   │   ├── detection_msgs.py        # 检测消息转换
│   │   ├── image_msgs.py            # 图像消息转换
│   │   └── ordinary_grasp_adapter.py# 普通抓取输入适配
│   ├── vision_node.py                # 主相机/YOLO 视觉节点
│   ├── offline_yolo.py               # 离线检测纯逻辑
│   ├── offline_yolo_node.py          # 离线图像检测 ROS 节点
│   ├── graspnet_baseline_adapter.py  # 本机 GraspNet backend 适配
│   ├── graspnet_baseline_node.py     # RGB-D/检测 -> GraspCandidate
│   ├── ordinary_grasp_node.py        # OBB/mask/depth 普通候选
│   ├── candidate_target_policy.py    # 目标标签和候选目标策略
│   ├── candidate_precheck_policy.py  # 候选预检
│   ├── candidate_workspace_gate.py   # 工作空间门
│   ├── candidate_filter_policy.py    # 候选过滤
│   ├── candidate_gate_policy.py      # 候选总门控
│   ├── candidate_scoring_policy.py   # 候选评分
│   ├── candidate_motion_policy.py    # 候选运动约束
│   ├── candidate_tf_adapter.py       # 候选坐标变换适配
│   ├── candidate_ik_filter_node.py   # MoveIt IK/碰撞/可执行性过滤
│   ├── grasp_candidate_policy.py     # 候选生成策略
│   ├── grasp_depth_probe_node.py     # 深度采样诊断
│   ├── grasp_preview_sender_node.py  # 抓取计划预览发送
│   ├── visual_grasp_marker_node.py   # RViz 抓取标记
│   ├── visual_grasp_executor_node.py # 视觉抓取分阶段执行编排
│   ├── visual_grasp_pose_policy.py   # pregrasp/grasp 位姿策略
│   ├── visual_grasp_sequence.py      # 开爪/接近/闭合/抬升/回撤阶段构造
│   ├── grasp_tcp_frame_node.py       # TCP/抓取坐标系广播
│   ├── grasp_verification_policy.py  # 抓取结果验证策略
│   ├── gripper_policy.py / gripper_quality.py # 夹爪开口与质量门
│   ├── approach_policy.py / retreat_policy.py # 接近与回撤策略
│   ├── place_task_policy.py           # 放置任务策略
│   ├── pose_variant_policy.py        # 候选姿态变体
│   ├── visual_servo_policy.py        # 单步视觉伺服限幅
│   ├── grasp_retry_policy.py         # 抓取重试策略
│   ├── retry_policy.py               # 通用重试策略
│   ├── motion_feasibility_policy.py # 运动可行性策略
│   ├── trajectory_recovery_policy.py # 轨迹恢复策略
│   ├── depth_quality.py              # 深度质量
│   ├── timestamp_policy.py / message_freshness.py / latest_only_work_queue.py # 时间与队列门
│   ├── transform_points.py / candidate_tf_adapter.py # 坐标变换
│   ├── aruco_reference.py / handeye_config.py # 参考物与手眼配置
│   ├── visual_grasp_benchmark.py      # 视觉抓取软件 benchmark
│   ├── hybrid_grasp_sim_benchmark.py  # 视觉 + 仿真 benchmark
│   ├── debug_camera_preview.py        # 相机调试预览
│   ├── graspnet_open3d_viewer.py      # GraspNet 候选可视化
│   ├── utils/visualization.py         # 可视化辅助
│   ├── models/README.md               # 模型文件放置说明
│   └── __init__.py
├── config/                            # 相机、YOLO/GraspNet、hand-eye、候选/夹爪/安全参数
├── launch/vision.launch.py            # 通用视觉入口
├── launch/vision_ubuntu.launch.py     # Ubuntu 原生相机入口
├── setup.py / package.xml / resource/*
└── ../../scripts/graspnet_baseline_inference.py # 构建时安装到 share/.../graspnet_backend/
```

`candidate_*.py` 是一组独立策略模块，负责把候选逐层变成可执行计划；不要把这些门绕过后直接调用 controller。`visual_ready_node` 属于 `rebotarm_motion`，视觉包只保留其兼容/调用接口。

## 对外入口

```text
rebotarm_vision_node                  # Gemini 2/YOLO 主节点
rebotarm_ordinary_grasp_node          # 普通深度/几何抓取
rebotarm_graspnet_baseline_node       # 本机 GraspNet ROS 节点
rebotarm_send_grasp_preview            # 发送抓取预览
rebotarm_visual_grasp_markers          # RViz 抓取标记
rebotarm_visual_grasp_executor         # 视觉抓取执行编排
rebotarm_grasp_candidate_ik_filter     # 候选 IK/碰撞过滤
rebotarm_grasp_tcp_frame               # TCP 坐标系
rebotarm_visual_grasp_benchmark        # 软件 benchmark
rebotarm_hybrid_grasp_sim_benchmark    # 混合仿真 benchmark
rebotarm_debug_camera_preview           # 相机调试
rebotarm_grasp_depth_probe              # 深度探针
rebotarm_graspnet_open3d_viewer         # Open3D 候选查看器
rebotarm_offline_yolo_node              # 离线 YOLO 节点
```

启动示例：

```bash
ros2 launch rebotarm_vision vision_ubuntu.launch.py
ros2 launch rebotarm_vision vision.launch.py
```

`vision_ubuntu.launch.py` 面向 Ubuntu 原生相机；模型、设备和是否启动普通抓取由 launch 参数和 `config/` profile 控制。缺少模型、相机或 GraspNet checkpoint 时应 fail closed。

## 运行边界

```text
RGB-D + CameraInfo + detections
 -> graspnet/ordinary candidate
 -> freshness/depth/TF/workspace/IK/collision/trajectory gates
 -> MoveIt / explicit executor
```

视觉包不得直接 import 电机 SDK，也不能把“检测到目标”或“生成候选”当作真实抓取成功；真实执行还需要控制器反馈、现场安全检查和显式授权。

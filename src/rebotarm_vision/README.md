# rebotarm_vision

Ubuntu 原生视觉与抓取候选包。维护链路为 Gemini 2/RGB-D/CameraInfo → YOLO 检测 → 本机 GraspNet → 候选筛选/IK/工作空间/碰撞门 → MoveIt 规划接口。旧 Windows、HTTP、MJPEG、远程 JSON 和独立 GraspNet 服务路线不属于当前包。运行说明见 [视觉抓取命令](../../docs/reference/commands/visual_grasp_commands.md)。

## 目录结构

```text
rebotarm_vision/
├── rebotarm_vision/
│   ├── camera/
│   │   └── gemini2_driver.py        # Gemini 2 图像/深度/CameraInfo 发布
│   ├── detector/
│   │   └── yolo_detector.py         # YOLO 推理和检测结果
│   ├── backends/
│   │   └── graspnet_baseline_inference.py # GraspNet 进程内推理实现
│   ├── converters/
│   │   ├── detection_msgs.py        # 检测消息转换
│   │   ├── image_msgs.py            # 图像消息转换
│   ├── diagnostics/                 # 只读诊断与开发可视化工具
│   │   ├── debug_camera_preview.py  # 相机调试预览
│   │   ├── grasp_depth_probe_node.py # 深度采样诊断
│   │   └── graspnet_open3d_viewer.py # GraspNet 候选可视化
│   ├── nodes/                        # 正式 ROS 适配节点
│   │   ├── vision_node.py            # Gemini 2/YOLO 主节点
│   │   ├── graspnet_baseline_node.py # RGB-D/检测 -> GraspCandidate
│   │   ├── candidate_ik_filter_node.py # MoveIt IK/碰撞过滤
│   │   ├── visual_grasp_executor_node.py # 抓取执行编排
│   │   └── ...                       # 其余 ROS 节点适配器
│   ├── policies/                     # 可测试的纯策略与门控函数
│   │   ├── candidate_*_policy.py     # 候选预检、评分、门控与目标策略
│   │   ├── visual_grasp_*_policy.py  # 抓取位姿、运行阶段与伺服策略
│   │   └── ...                       # 夹爪、回撤、恢复和时间策略
│   ├── utils/                        # 公共纯函数与消息适配
│   │   ├── transform_points.py       # 坐标变换数学函数
│   │   ├── tf_message_adapter.py     # TF/Pose 消息适配
│   │   ├── visual_grasp_messages.py   # 位姿消息转换
│   │   ├── parameter_validation.py    # 参数校验
│   │   └── message_freshness.py / latest_only_work_queue.py
│   ├── offline_yolo.py               # 离线检测纯逻辑
│   ├── graspnet_baseline_adapter.py  # 本机 GraspNet backend 适配
│   ├── candidate_tf_adapter.py       # 候选坐标变换适配
│   ├── visual_grasp_sequence.py      # 开爪/接近/闭合/抬升/回撤阶段构造
│   ├── gripper_quality.py             # 夹爪质量判定（策略在 policies/）
│   ├── handeye_config.py           # 手眼配置读取（ArUco 算法归标定包）
│   ├── utils/visualization.py         # 可视化辅助
│   └── __init__.py
├── config/                            # 相机、YOLO/GraspNet、hand-eye、候选/夹爪/安全参数
├── models/                            # YOLO 权重，安装到 share/rebotarm_vision/models/
├── launch/vision.launch.py            # 通用视觉入口
├── launch/vision_ubuntu.launch.py     # Ubuntu 原生相机入口
└── setup.py / package.xml / resource/*
```

`candidate_*.py` 是一组独立策略模块，负责把候选逐层变成可执行计划；不要把这些门绕过后直接调用 controller。抓取从当前姿态规划，无固定观察位调用接口。

## 对外入口分类

入口注册是为了保留可复制的命令名，不代表每个入口都是生产启动路径：

| 类别 | 入口 | 说明 |
| --- | --- | --- |
| 正式运行 | `rebotarm_vision_node`, `rebotarm_graspnet_baseline_node`, `rebotarm_grasp_candidate_ik_filter`, `rebotarm_visual_grasp_executor`, `rebotarm_grasp_tcp_frame` | Ubuntu 主链路；节点裸启动默认 plan_only，正式视觉 launch 固定 execute，启动后仍等待服务触发且不自动使能 |
| 只读诊断 | `rebotarm_debug_camera_preview`, `rebotarm_grasp_depth_probe`, `rebotarm_grasp_candidate_markers`, `rebotarm_visual_grasp_markers` | 观察图像、深度或候选，不应下发运动 |
| 开发调试 | `rebotarm_send_grasp_preview`, `rebotarm_graspnet_open3d_viewer`, `rebotarm_offline_yolo_node` | 离线或可视化工具，需单独准备输入 |
| legacy 兼容 | （已退役）ordinary grasp 入口 | 历史路线已从安装包和 launch 移除 |

正式 Ubuntu GraspNet 节点使用 `InProcessGraspNetBackend`。外部研究代码只作为该后端加载的
推理引擎模块，不再在视觉包内维护第二个后端类。

## 对外入口（命令名兼容表）

```text
rebotarm_vision_node                  # Gemini 2/YOLO 主节点
rebotarm_graspnet_baseline_node       # 本机 GraspNet ROS 节点
rebotarm_send_grasp_preview            # 发送抓取预览
rebotarm_visual_grasp_markers          # RViz 抓取标记
rebotarm_visual_grasp_executor         # 视觉抓取执行编排
rebotarm_grasp_candidate_ik_filter     # 候选 IK/碰撞过滤
rebotarm_grasp_tcp_frame               # TCP 坐标系
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

`vision_ubuntu.launch.py` 面向 Ubuntu 原生相机；模型、设备和 GraspNet 参数由 launch 参数与 `config/` profile 控制。缺少模型、相机或 GraspNet checkpoint 时应 fail closed。

## 运行边界

```text
RGB-D + CameraInfo + detections
 -> GraspNet candidate
 -> freshness/depth/TF/workspace/IK/collision/trajectory gates
 -> MoveIt / explicit executor
```

视觉包不得直接 import 电机 SDK，也不能把“检测到目标”或“生成候选”当作真实抓取成功；真实执行还需要控制器反馈、现场安全检查和显式授权。

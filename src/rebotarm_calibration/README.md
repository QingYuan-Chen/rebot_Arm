# rebotarm_calibration

手眼、TCP 和 TF 验证工具包。它负责同步采集、ArUco/PnP、手眼与 TCP 数学求解、
质量门和会话数据，不访问电机 SDK、不规划轨迹、不使能机械臂，也不自动部署外参。

## 目录结构

```text
rebotarm_calibration/
├── rebotarm_calibration/
│   ├── aruco_pose.py              # ArUco 检测与 camera->marker PnP
│   ├── calibration_quality.py     # 覆盖度、可观测性、残差和 bootstrap
│   ├── handeye_capture_node.py    # ROS Image/CameraInfo/TF 采集服务
│   ├── handeye_calibration_cli.py # 离线手眼求解入口
│   ├── handeye_residual.py        # 手眼残差、离群和一致性分析
│   ├── handeye_residual_cli.py    # 手眼残差 JSON 入口
│   ├── handeye_solver.py          # 五种 eye-in-hand 求解器
│   ├── handeye_workflow.py        # 训练选择、独立验证和报告
│   ├── provenance.py              # 数据集和源码来源指纹
│   ├── session_store.py           # 原子 JSON 会话、版本和幂等
│   ├── stability_window.py        # 连续稳定姿态窗口
│   ├── tcp_calibration.py         # TCP pivot 数学计算
│   ├── tcp_calibration_node.py    # 传统交互式 TCP 节点
│   ├── tcp_workflow.py            # 网页 TCP 训练/验证流程
│   └── __init__.py
├── setup.py                       # ament_python 与 console_scripts
├── package.xml                    # ROS 2 依赖
└── resource/rebotarm_calibration  # ament 包索引
```

## 数据流和职责边界

```text
Image + CameraInfo + TF
        │
        └── HandeyeCaptureNode
              ├── ArUco/PnP -> camera_to_marker
              ├── TF        -> base_to_end
              └── SessionStore -> training/validation JSON
                                  │
                    handeye_workflow / tcp_workflow
                                  │
                         candidate report (deployed=false)
```

网页由现有 `rebotarm_dashboard` 提供；Dashboard 只负责 HTTP/SSE、页面和
`CalibrationCommand` ROS 客户端。本包不依赖 Dashboard，也不读取控制器命令。

## 对外入口

```bash
# ROS 采集服务，网页 /calibration 使用
ros2 run rebotarm_calibration rebotarm_handeye_capture

# 离线手眼求解
ros2 run rebotarm_calibration rebotarm_handeye_calibration \
  --input session.json --output report.json

# 离线残差分析
ros2 run rebotarm_calibration rebotarm_handeye_residual \
  --input dataset.json --output residual_report.json

# 传统交互式 TCP 节点
ros2 run rebotarm_calibration rebotarm_tcp_calibration
```

入口参数以 `--help` 和当前源码为准。网页采集节点提供
`/rebotarm_handeye_capture/command`，支持 `create/status/preflight/preview/tf_check/`
`capture/solve/reopen/accept/abort`。同一会话目录只能由一个采集节点写入。
另提供 `/rebotarm_handeye_capture/observation`，复用 `CalibrationCommand` 类型且只接受
`observe`。它独立读取匹配会话的新鲜相机快照及 ArUco 位姿，返回光学中心到标记中心
距离和光学坐标 XYZ；无需 TF/稳定窗口，采集等待期间仍可查询，保留订阅和会话内容。
网页每 0.5 秒查询，过期或质量失败后清除旧值，详细口径见[操作说明](../../docs/reference/commands/calibration_web.md#实时距离与采样反馈)。

## 会话和求解规则

- 手眼模式使用训练集选择 TSAI、PARK、HORAUD、ANDREFF、DANIILIDIS 之一，验证集只做独立评估。
- TCP 模式只求 TCP 位置和固定 pivot，不求工具旋转；验证集使用训练得到的 pivot。
- 每次写操作携带 `request_id` 和 `expected_revision`，重复请求不会重复追加样本。
- 首个样本会冻结 topic、CameraInfo、阈值和源码来源；条件变化必须新建会话。
- 报告包含观测性、覆盖度、残差、离群诊断、bootstrap 和完整数据集哈希。
- `passed`、`accepted` 和 `deployed` 是不同状态；本包不会自动写入视觉配置。

## 采集质量门

ArUco 模式要求 Image、CameraInfo 的 frame 和分辨率匹配，使用新鲜且时间戳推进的
图像/TF；默认还检查重投影 RMSE、标记面积、正深度、距离和连续稳定窗口。
图像与内参使用 RELIABLE 订阅，与正式 D405 发布端匹配。手眼稳定窗口同时检查
`base_to_end` 与 `camera_to_marker`，沿用 0.4 秒、最大间隔 0.2 秒、平移 1 mm、旋转
0.5° 的门限；从窗口中选择总位姿距离最小的实测配对帧，保留机器人与视觉的同期关系。
跨训练/验证的重复机器人姿态在保存前拒绝，判定精度与求解器一致；同一请求重试保持幂等。
相对旋转约束必须满秩且条件数满足阈值，单轴重复运动会被拒绝。

新手眼样本保存角点、图像/内参/TF 时间及 frame、窗口逐帧观测、所选帧无损 PNG/base64
和 SHA256。`arm_namespace` 默认 `rebotarm`，用于订阅原始 JointState；各观测附带图像
时刻两侧最近的关节源消息及时间差，缺失时明确留空。TF2 仍负责历史变换查询。
这些字段随现有会话 JSON 原子保存、导出和重试；旧会话可继续只读求解，代码版本变化后新建采集会话。

TCP/手眼采样失败只返回结构化错误并保留已持久化状态，不会自动退出重力补偿、失能或回位。
硬件 Enable 和 Gravity Start/Stop 由 Dashboard 的显式操作管理。

## 测试和操作文档

软件回归主要位于 `tests/test_handeye_*.py`、`tests/test_calibration_*.py` 和
`tests/test_tcp_*.py`。网页操作、安全顺序、真实相机预检和现场验收边界见：

- [网页操作命令参考](../../docs/reference/commands/calibration_web.md)
- [软件验收清单](../../docs/implemented/features/calibration_acceptance.md)
- [技术路线与后续缺口](../../docs/design/calibration_web_plan.md)

软件测试或 MuJoCo 结果不等于真实相机、重力补偿、末端方向或物理精度验收；部署前必须
进行独立多轴留出验证和现场测量。

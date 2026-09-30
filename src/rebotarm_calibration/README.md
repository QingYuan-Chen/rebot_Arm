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
相对旋转约束必须满秩且条件数满足阈值，单轴重复运动会被拒绝。

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

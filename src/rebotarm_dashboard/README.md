# rebotarm_dashboard

## 标定工作台操作状态

- `file://` 页面仅供界面预览，设备与会话按钮不可用；实际操作使用 Dashboard 的 `/calibration`。
- 创建/恢复会话后参数锁定；“复制配置，准备新会话”只复制配置，不修改原会话。
- 试采检查不保存样本，失败时显示观测失败原因，不把未完成的项目标为通过。
- 机械臂请求期间锁住冲突操作；超时结果未知时必须核对新鲜反馈与现场状态。状态流在线不等于控制器反馈新鲜；网页不替代控制器安全门。
- 样本排除保留原始记录和旧报告。修改排除选择，或查看验证结果后新增训练数据，必须补采至少五个新的最终验证姿态。验证样本的排除/恢复还需填写采集问题证据，不能只因误差大而删点。
- 结果同时展示训练、有效验证、完整验证及适用时的新增最终验证误差；任何数值通过均不自动部署外参。

本机 Web 面板与状态 API 包。它聚合 ROS 状态、提供页面/HTTP/SSE 接口，并把浏览器意图转交给 `rebotarm_teleop`、`rebotarm_teach` 和控制器服务；不实现运动规划、示教算法或电机 SDK 访问。

## 目录结构

```text
rebotarm_dashboard/
├── rebotarm_dashboard/
│   ├── teleop_status_panel_node.py # 唯一常驻 ROS 节点：状态聚合 + HTTP 服务
│   ├── status_panel_http.py        # GET、SSE、POST 路由和本机 HTTP server
│   ├── status_panel_page.py        # 从包资源读取 index.html
│   ├── status_panel_assets/
│   │   ├── index.html              # 页面、JavaScript 和样式资源
│   │   ├── calibration.html        # /calibration 手眼/TCP 标定向导
│   │   └── __init__.py
│   ├── status_panel_state.py       # 快照、状态存储和 SSE 编码
│   ├── status_panel_api.py         # POST 白名单与 handler 分发
│   ├── calibration_client.py       # CalibrationCommand ROS 客户端
│   ├── calibration_gravity.py      # 标定重力补偿状态机和安全门
│   ├── arm_command_api.py          # enable/disable/safe_home 命令门控
│   ├── arm_control_client.py       # 控制器 Trigger 服务客户端
│   ├── web_command_gateway.py      # dry_run/execute 执行模式安全门
│   ├── web_robot_assets.py         # URDF、网格路径和夹爪显示换算
│   ├── parameter_helpers.py        # 参数数组与传感器 QoS 整形
│   └── __init__.py
├── setup.py                         # 安装 HTML 资源并注册节点
├── package.xml
└── resource/rebotarm_dashboard
```

## 脚本职责

- `TeleopStatusPanelNode` 订阅关节/电机/整臂/遥操作/示教状态，把快照交给 HTTP 层，并创建 ROS 服务与动作客户端。
- `status_panel_http.py` 提供 `/`、`/calibration`、`/api/status`、`/api/config`、
  `/api/calibration/export`、示教记录查询、`/events` SSE、`/robot/urdf` 和网格资源。
- `status_panel_api.py` 集中列出可写 POST 路径，包括普通遥操作和
  `/api/calibration/command`、`/api/calibration/gravity`。
- `calibration_client.py` 只负责把网页请求转换为 `CalibrationCommand` ROS service 请求，
  不实现 PnP、手眼求解或会话文件读写。
- `calibration_gravity.py` 管理标定期间的显式 Enable、Gravity Start/Stop、状态新鲜度、
  超时 future 和控制权占用；它不实现标定数学。
- `status_panel_page.py` 用 `importlib.resources` 读取安装后的 `status_panel_assets/index.html`，不依赖源码树相对路径。
- `arm_command_api.py` 与 `web_command_gateway.py` 在调用 ROS 之前检查命令白名单、回放锁定和执行模式。
- `status_panel_state.py` 只做状态聚合/序列化；`web_robot_assets.py` 只做模型资源、安全路径和夹爪显示数据处理。
- `arm_control_client.py`、`parameter_helpers.py` 是客户端和参数辅助层，避免把 ROS 调用散落在 HTTP 路由中。

## 对外入口和启动

```bash
ros2 run rebotarm_dashboard TeleopStatusPanelNode
```

生产组合通常由 `rebotarm_bringup` 启动：

```text
浏览器 -> rebotarm_dashboard -> rebotarm_teleop / rebotarm_teach / controller ROS 接口
```

Dashboard HTTP 只绑定本机 UI/API，不是视觉传输链路；当前视觉路径使用 ROS RGB-D、CameraInfo 和 detection 话题。

## 边界

- 不在这里生成 `JointTrajectory`、做 MoveIt 规划或实现示教回放质量策略。
- 不从这里直接 import 电机 SDK。
- Web 预览和真实执行由请求中的执行模式及下游安全门区分；默认 dry-run/预览不能当作真机执行证据。

## 标定页面

同端口 `/calibration` 提供手眼/TCP 向导，HTTP 请求通过 `CalibrationCommand` 调用
独立的 `rebotarm_handeye_capture` 节点。Dashboard 只拥有网页、HTTP/SSE、ROS 客户端和
标定期间的操作者互斥；标定包拥有 Image/CameraInfo/TF 采集、SessionStore、PnP 和求解。

相关接口：

```text
GET  /calibration
GET  /api/calibration/export?session_id=<32位hex>
POST /api/calibration/command
POST /api/calibration/gravity
```

默认不会自动启动相机、标定节点或机械臂控制器，也不会自动应用外参。网页关闭不会
改变机器人模式；Gravity Stop 后还必须等待新鲜 IDLE 反馈，才能解除标定控制占用。
操作顺序、安全边界和真实硬件测试见 [网页标定命令参考](../../docs/reference/commands/calibration_web.md)。

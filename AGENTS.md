# 编码助手仓库工作规范

## 支持范围

支持的部署环境为 Ubuntu 24.04 / ROS 2 Jazzy：

```text
Gemini 2 -> YOLO -> ROS RGB-D/CameraInfo/检测结果 -> 本地 GraspNet
```

不得恢复已退役的 Windows、HTTP、MJPEG、远程 JSON 或独立 GraspNet 服务视觉路线。
Dashboard 的 HTTP 仅用于本地界面和 API。

Before changing code, read `docs/implemented/architecture.md`, `docs/reference/context.md`,
`tests/test_package_layering.py`, and `docs/reference/project_status.md`.

## 必须遵守的规则

- 健康真机发生可恢复任务失败时，不得在远离已验证基线的位置自动失能。应停止运动并保持使能以维持当前位置，或在安全监控下返回已记录基线，验证后再失能。仅控制器/电机错误、通信丢失、反馈过期或明确急停允许在基线外保护性失能。
- 真机启动默认失能。必须具备新鲜反馈、完成现场安全检查，并显式调用 `/rebotarm/enable`；不得恢复自动使能。
- MoveIt 执行需要 `moveit_simple_controller_manager`；规划成功不能作为执行成功的证据。
- 硬件访问归 `rebotarmcontroller`。
- 轨迹生成、MoveIt 适配、重定时、碰撞检查和运行时安全检查归 `rebotarm_motion`。
- 示教录制、轨迹准备和回放编排归 `rebotarm_teach`。
- 键盘、网页和 RViz 操作适配归 `rebotarm_teleop`。
- 网页界面、HTTP 路由、SSE 和 Dashboard 资源归 `rebotarm_dashboard`。
- URDF、SRDF 和规划配置归 `rebotarm_moveit_config`。
- 感知、深度、检测和抓取候选归 `rebotarm_vision`。
- MuJoCo 模型、物理仿真执行和物理指标归 `rebotarm_simulation`。
- 不加载物理引擎的 ROS/RViz 轻量预演归 `rebotarm_preview`，不得访问真实电机 SDK。
- 启动组合和后端选择归 `rebotarm_bringup`。
- 手眼标定、TCP 和 TF 验证归 `rebotarm_calibration`。

## Project State

`docs/reference/project_status.md` is the only project-state document. Read it
before changes and update it directly when important decisions, blockers, or
verification results change. Keep feature details and commands in their existing
docs pages and link to them instead of copying them into state notes. Do not
recreate the retired state directory, generated JSON, activity logs, or phase
documents. Do not mark hardware acceptance from software tests alone.

## 本地目录与依赖约定

- 环境工具放在 `scripts/`；依赖版本保留于 `docs/setup/dependencies/` 的 Markdown `requirements` 代码块，由现有安装工具读取。
- MuJoCo 默认使用系统 Python 3.12 的用户级依赖；不恢复强制 `.venv-mujoco` 或屏蔽用户包的默认值。
- 厂商 SDK 保留在 `third_party/reBotArm_control_py`，固定版本清单归 `third_party/rebotarm_dependencies.repos`。
- 主工程不保留 `DRL/`；现有独立 MJLab 工程位于 `/home/a/project/rebot_Arm_rl/MJLab/`，不随 ROS 整合迁移或启动训练。

## 必须执行的检查

```bash
python3 -m pytest tests/test_package_layering.py -q
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests -q
python3 -m compileall src/rebotarm_dashboard/rebotarm_dashboard src/rebotarm_teleop/rebotarm_teleop src/rebotarm_teach/rebotarm_teach src/rebotarm_motion/rebotarm_motion -q
python3 -m compileall src/rebotarm_bringup/launch -q
```

功能跨越职责边界时，应拆分为由相应包拥有的接口，不得跨层复制实现。

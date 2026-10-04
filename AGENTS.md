# 编码助手仓库工作规范

## 支持范围

支持的部署环境为 Ubuntu 24.04 / ROS 2 Jazzy：

```text
Gemini 2 -> YOLO -> ROS RGB-D/CameraInfo/检测结果 -> 本地 GraspNet
```

不得恢复已退役的 Windows、HTTP、MJPEG、远程 JSON 或独立 GraspNet 服务视觉路线。
Dashboard 的 HTTP 仅用于本地界面和 API。

修改代码前，阅读 `docs/implemented/architecture.md`、`docs/reference/context.md`、
`tests/test_package_layering.py`、`Agent/README.md`、`Agent/MEMORY.md`、
`Agent/PROJECT_STATUS.md` 和 `Agent/STATE.json`。

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

## 助手状态记录

`Agent/` 是项目实时状态来源。任务开始时运行：

```bash
python3 Agent/update_state.py --event start --actor <agent> --note "<task>"
```

事实或阻塞变化时更新 `Agent/MEMORY.md`。只能通过 `Agent/update_state.py` 更新
`STATE.json`；不得改写历史活动记录，也不得仅凭软件测试标记真机验收完成。

## 必须执行的检查

```bash
python3 -m pytest tests/test_package_layering.py -q
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests -q
python3 -m compileall src/rebotarm_dashboard/rebotarm_dashboard src/rebotarm_teleop/rebotarm_teleop src/rebotarm_teach/rebotarm_teach src/rebotarm_motion/rebotarm_motion -q
python3 -m compileall src/rebotarm_bringup/launch -q
```

功能跨越职责边界时，应拆分为由相应包拥有的接口，不得跨层复制实现。

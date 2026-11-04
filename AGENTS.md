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

- 仅在控制器/电机错误、明确急停允许在基线外保护性失能。
- 真机启动默认失能。
- 新加入和修改的功能遵循现在的包分类原则，避免功能包的职责不清晰
- 单个节点尽量只负责单一功能，不要让一个节点承载过多的功能
- 控制器新增及调整的代码块需增添必要的注释、文档字符串使用中文，在并发、生命周期和安全约束等需要解释之处补充说明，并且在文件开头声明职责，对外接口，安全约束以及文件结构。

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

## 检查要求

根据改动范围和风险选择相关的定向检查，覆盖受影响的功能及安全边界，并在交付时说明实际执行的检查和未验证范围。非必要不运行完整回归；用户明确要求、重大跨包或安全关键改动需要整体验收，以及发布前需要确认整体兼容性时，再运行完整回归。

以下分层和编译检查保持必需：

```bash
python3 -m pytest tests/test_package_layering.py -q
python3 -m compileall src/rebotarm_dashboard/rebotarm_dashboard src/rebotarm_teleop/rebotarm_teleop src/rebotarm_teach/rebotarm_teach src/rebotarm_motion/rebotarm_motion -q
python3 -m compileall src/rebotarm_bringup/launch -q
```

需要完整回归时执行：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests -q
```

功能跨越职责边界时，应拆分为由相应包拥有的接口，不得跨层复制实现。

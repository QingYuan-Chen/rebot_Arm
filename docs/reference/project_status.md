# 当前项目状态与关键决定

> 状态：REFERENCE；类型：项目状态；适用范围：当前本地工作空间；最后核对：2026-10-01。

本文件是项目状态的唯一文档，保留当前有效决定、验证范围和未解决问题。
功能、架构、参数与启动命令分别链接到对应手册；不维护历史任务阶段、进度百分比或重复流水。
状态变化时直接更新本文，不再生成独立状态 JSON 或事件日志。

## 当前基线

- 主目录：`/home/a/project/rebot_Arm`；工作与发布分支为 `develop`，远端仓库为 `QingYuan-Chen/rebot_Arm`；保留 `main` 分支。
- 本地上游整合提交：`dc7e067`，来源 `huangbinai/robotarm_ros2 main@19f2939`；这是核对快照，不代表上游永远最新。
- Ubuntu 24.04 / ROS 2 Jazzy，11 个活动 ROS 包；职责和依赖边界见[架构说明](../implemented/architecture.md)。
- 视觉只维护 Gemini 2 → 本地 YOLO → ROS RGB-D/CameraInfo/detections → 本机进程内 GraspNet。
- 分环境依赖由[依赖索引](../setup/dependencies/README.md)统一维护，安装工具为 `scripts/install_python_dependencies.py`。
- 视觉、GraspNet 与 MuJoCo 分别使用 `.venv-vision`、`.venv-graspnet`、`third_party/rebotarm_mujoco_venv`；解释器选择见[配置契约](../setup/launch_python_configuration.md)。
- 主控制器要求配套的 MotorBridge 反馈序号补丁；启动前用 `scripts/setup_motorbridge_fresh_feedback.py --check-installed` 只读检查，版本以当前工具和依赖清单为准。

## 保留的关键决定

- 网页/键盘/RViz 操作、统一示教、MoveIt、视觉候选及正式单瓶抓取入口已保留；使用[功能命令索引](commands/README.md)。
- 真实硬件由 `hardware_controller.launch.py` 唯一组合；串口只能有一个控制器占用。
- 反馈按每个电机的序号和新鲜度检查；各关节序号不要求相等。
- 夹爪接触保持、限力夹持和张开释放已经实现并经用户历史实测；当前整合基线仍需独立实机复验。
- J2/J3 软件位置范围保留 `[-3.14,+0.02] rad`，目标和反馈同样检查；不改零位或对反馈加偏移。
- MuJoCo Viewer、Reach/Pick、示教离线预演、Sim2Real/Real2Sim 和诊断保留。
- MJX 专属训练、依赖和对比工具已剥离；CPU Gymnasium Reach 与可选 SB3/PPO 保留；后续采用 MJLab，尚未接入。
- 参数以当前包内配置和显式 launch 覆盖为准，见[参数来源](parameters/current_parameter_sources.md)，不从旧任务记录复制。

## 当前限制与待解决问题

- 2026-10-01 只读检查：本机 MotorBridge 为 `0.4.6+rebotarm.2`，当前源码要求 `0.4.7+rebotarm.1`，启动前版本检查未通过；目录整理未升级该依赖。实机启动前需另行处理并复核。
- MJLab 尚未接入；GPU 训练质量、真实相机标定和新基线实机验收需分别验证。
- MoveIt Plan 成功不等于 Execute；真实执行需要 `moveit_simple_controller_manager` 和正确的 Action 链。
- Star Arm 联合跟随的 J5 动态滞后根因未查清：已知触发是整形命令与新鲜反馈误差约 `0.262 rad` 持续超限。不能归因于 J3 限位、夹爪故障或已证实的旧反馈缓存问题。
- 六轴 `0.25 rad / 0.3 s` 保护未获准统一放宽；逐轴边界、动态滞后和减速策略须先讨论并确认。
- 软件/仿真通过不能代替真实夹取、物理精度或控制安全验收。

## 实机与测试边界

- 实机默认失能；运动需要本窗口明确授权、新鲜反馈、现场安全检查和显式 Enable。
- 引导端口 `/dev/ttyUSB0`、从臂端口 `/dev/ttyACM0`；端口变化时显式指定。每次串口操作前执行 `fuser -v /dev/ttyUSB0 /dev/ttyACM0`。
- 健康硬件的可恢复失败：停止并保持，或受控回到本轮捕获的安全基线，稳定验收后才失能。
- 回位失败但硬件健康：保持使能并等待人工恢复；通信丢失、反馈失效、电机错误或急停才允许保护性失能。
- 安全位验收：位置误差 ≤`0.02 rad`、速度 ≤`0.05 rad/s`，稳定 `1 s`，总超时 `30 s`。
- 非硬件 ROS 测试先检查活动进程、域与 DDS 端口，选空闲隔离域并设置 `ROS_LOCALHOST_ONLY=1`，只清理本轮资源。
- 不强杀其他任务，不覆盖或回滚已有未提交改动；提交和推送仅在用户明确要求时执行。

## 数据与维护

- 旧阶段文档、状态生成脚本、状态 JSON、事件流水和历史实验数据已清理，原 `Agent/` 目录移除。
- 当前运行标定与控制配置仍在所属功能包内；清理历史数据不改动配置或重新校准。
- 新运行报告由命令显式指定输出，推荐 `log/visual_grasp/` 或 `log/calibration/`；运行数据不作为项目状态文档，也不加入例行提交。
- `scripts/` 只保留环境安装/启动/检查脚本、GraspNet 推理后端及 YOLO/TensorRT 模型资产；旧 P5 阶段工具和 P6 兼容入口已删除。
- 目录结构以用户手动整理后的布局为准；补充文档在 `docs/reference/`，SDK 清单在 `third_party/`，MotorBridge 补丁在根目录的 `patches/`。
- 独立跟随工具在根目录的 `star_arm_102_rebot_b601_follow/`；不再使用此前的 `相关资料/` 布局，不自动恢复或重新移动用户整理的文件。
- 单瓶抓取使用正式 `rebotarm_single_bottle_grasp` 入口；标定使用所属功能包的网页与 ROS 工具，见[标定操作说明](commands/calibration_web.md)。
- 当前源码的历史变更从 Git 历史追溯；已删除的本机未跟踪实验数据不保留。

## 最近软件验证

- 用户手动布局的引用同步完成：工具位于 `scripts/`，补丁位于 `patches/`，独立跟随工具位于根目录；仅同步引用与说明，未移动目录。
- 完整回归 `1167 passed / 14 skipped`，两条既有 Gymnasium 观测范围提示；本次补丁与分层专项 `41 passed`，其中分层测试 `18 passed`。
- 本次必需编译检查、54 份文档的本地链接检查和 `git diff --check` 通过；此前目录整理的 11 包 `symlink-install` 构建与脚本语法检查通过，未改变构建资源路径。
- 安装后的 YOLO/TensorRT 模型与 GraspNet 后端均解析至新 `scripts/` 路径，后端模块加载通过；当前 MuJoCo 和视觉 launch 的 `--show-args` 参数解析通过，未启动节点。
- 51 份运行/模型配置校验一致，未修改标定或控制参数；位于 `patches/` 的补丁文件摘要符合固定版本契约。
- 完整回归日志：`/tmp/rebotarm-user-layout-20261001-full.log`；此前构建日志：`/tmp/rebotarm-root-layout-20261001-build.log`。测试使用预先确认空闲的 ROS 域 119 和本机通信隔离，与既有 domain66 分离。
- 软件验证未访问串口、启动实机、安装依赖或执行训练。

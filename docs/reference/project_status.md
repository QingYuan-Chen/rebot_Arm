# 当前项目状态与关键决定

> 状态：REFERENCE；类型：项目状态；适用范围：当前本地工作空间；最后核对：2026-10-03。

本文件是项目状态的唯一文档，保留当前有效决定、验证范围和未解决问题。
功能、架构、参数与启动命令分别链接到对应手册；不维护历史任务阶段、进度百分比或重复流水。
状态变化时直接更新本文，不再生成独立状态 JSON 或事件日志。

## 当前基线

- 主目录：`/home/a/project/rebot_Arm`；工作与发布分支为 `develop`，远端仓库为 `QingYuan-Chen/rebot_Arm`；保留 `main` 分支。
- 本地上游整合提交：`dc7e067`，来源 `huangbinai/robotarm_ros2 main@19f2939`；这是核对快照，不代表上游永远最新。
- Ubuntu 24.04 / ROS 2 Jazzy，11 个活动 ROS 包；职责和依赖边界见[架构说明](../implemented/architecture.md)。
- 视觉只维护 Gemini 2 → 本地 YOLO → ROS RGB-D/CameraInfo/detections → 本机进程内 GraspNet。
- 分环境依赖由[依赖索引](../setup/dependencies/README.md)统一维护，安装工具为 `scripts/install_python_dependencies.py`。
- 视觉、GraspNet 分别使用 `.venv-vision`、`.venv-graspnet`；MuJoCo 默认使用系统 Python 3.12 的用户级依赖；解释器选择见[配置契约](../setup/launch_python_configuration.md)。
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

- 2026-10-03 按用户授权构建并安装 MotorBridge `0.4.7+rebotarm.1`，版本/反馈序号检查通过；构建使用仓库固定提交与摘要校验的补丁，无硬件连接。
- 重力补偿已补齐时间积分、模式检查、严格发送、力矩边界、周期诊断与权威模型来源，详见[硬件执行说明](../implemented/features/hardware_execution.md#重力补偿控制)。实际控制时序、实物附加载荷/质心、零位/方向及模式切换连续性仍需本窗口另行实机授权后采样验收；Python 控制线程不提供硬实时保证，笛卡尔阻抗尚未实现。
- 2026-10-01 仿真功能核验发现：MuJoCo 轨迹 Action 未在接纳前检查位置范围；J3 `+0.03 rad` 请求超过 `+0.02 rad` 上限，底层裁剪后仍返回成功。需补充逐轴轨迹入口校验，不能扩大限位或放宽完成容差。
- MuJoCo 从 position 切换到 hold 会清零控制器支撑力矩；独立取消测试出现约 `0.047 rad` 短时偏移，静止零运动切换也复现。取消终态和最终保持正常，但切换连续性待修复；此结论限于仿真，未验证真实控制器有相同行为。
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
- 按用户最新决定保留功能回归测试：两轮删除的 127 个测试文件已全部恢复；加入重力补偿专项后，`tests/` 共 130 个测试模块和 `conftest.py`。功能已实现不作为删除测试的依据；测试缓存可清理。
- 目录结构以用户手动整理后的布局为准；补充文档在 `docs/reference/`，SDK 清单在 `third_party/`，MotorBridge 补丁在根目录的 `patches/`。
- 按用户授权删除 `third_party/robotarm_ros2_upstream`、`robotarm_ros2_mujoco_snapshot` 和 `reBotArm_develop_hjx` 三个旧参考目录，包含其中未接入主项目的历史实验。`third_party/` 当前保留厂商 SDK、固定版本清单和 `COLCON_IGNORE`；正式源码与第三方归属说明继续保留。
- 独立跟随工具在根目录的 `star_arm_102_rebot_b601_follow/`；不再使用此前的 `相关资料/` 布局，不自动恢复或重新移动用户整理的文件。
- 单瓶抓取使用正式 `rebotarm_single_bottle_grasp` 入口；标定使用所属功能包的网页与 ROS 工具，见[标定操作说明](commands/calibration_web.md)。
- 当前源码的历史变更从 Git 历史追溯；已删除的本机未跟踪实验数据不保留。

## 最近软件验证

- 2026-10-03 重力补偿四项软件修复：相关专项 `201 passed`，完整回归 `1202 passed / 2 skipped`（两条既有 Gymnasium 提示），分层检查、必需 compileall、11 包构建与安装资源核验通过。显式对齐 MuJoCo 附加载荷后，三姿态各 2 s 保持最大漂移小于 `0.00005 rad`；未使用仿真负载值修改真机默认负载。离线控制计算中位数约 `0.032 ms`，不含串口和调度时延。空闲域 110、固定域 176/177/181 隔离，收尾进程与 DDS 端口无残留；MotorBridge 协议测试仅使用 `/dev/pts/` 虚拟串口，未访问实机串口、使能或发送实机命令。证据在本机 `log/gravity_control_20261003/`，新增边界与物理补偿效果尚待实机验收。
- 2026-10-01 恢复全部功能测试后，完整回归 `1167 passed / 14 skipped`（两条既有 Gymnasium 观测范围提示），分层专项 `18 passed`，必需编译检查及 `git diff --check` 通过。测试文件与删除前 `2bd5451` 完全一致，随删除修改的文档和源码注释已撤回。使用经核对空闲的域 110，固定测试域 176/177/181；未访问实机。证据保留在本机 `log/test_restore_20261001/`。
- 2026-10-01 解除 MuJoCo 专属虚拟环境依赖：launch 默认使用 PATH 中的 `python3`，本机部署到系统 Python 3.12 的用户包目录；显式解释器覆盖仍可用，环境脚本不会自动设置或覆盖 MuJoCo 解释器。MuJoCo 3.3.0、Gymnasium 1.2.3 和原依赖版本保留，NumPy 仍为系统 1.26.4；`pip check` 通过，MotorBridge 未升级或覆盖。旧 MuJoCo 环境已在验证后删除。
- 本轮完整回归 `1167 passed / 14 skipped`（两条既有 Gymnasium 观测范围提示），分层专项 `18 passed`，11 包构建及必需编译检查通过。从 `/tmp` 使用默认 Python 验证模型、有限物理步进、EGL 渲染、原生 Viewer 和 MoveIt 实际执行；规划 37 点，终点最大误差 `0.012408 rad`。Goal 前确认唯一 MuJoCo Action server 和关节反馈源；域 119/176 隔离，本轮未访问实机。54 份模型/配置文件摘要未变；不代表两项既有仿真控制问题已修复。证据保留在本机 `log/mujoco_python_cleanup_20261001/`。

- 用户手动布局的引用同步完成：工具位于 `scripts/`，补丁位于 `patches/`，独立跟随工具位于根目录；仅同步引用与说明，未移动目录。
- 2026-10-01 在 `develop@03a32a0` 完成主要功能仿真核验：基础物理与 ROS 接口、夹爪服务与接触、MoveIt 实际规划执行、合成示教预演、Real2Sim 镜像/物理跟随、Reach/Pick 环境和视觉多阶段 plan_only 规划均可运行；额外边界探针发现上述两项仿真控制问题，不能记为全部验收通过。
- MoveIt 实际执行 37 个轨迹点，最终最大误差 `0.010004 rad`；4 s 示教预演最大误差 `0.000353 rad`。取消后最终 hold 稳定，但不代表瞬态切换已合格。
- Pick 三个 seed 各 100 步安全违规为 0，随机动作抓取成功率为 0；本轮未验证训练策略成功抓取。视觉输入为经过 FK/碰撞校验的合成计划，未验证真实相机、YOLO/GraspNet 推理或单瓶实机执行。
- 删除三个旧参考目录后重跑完整回归，仍为 `1167 passed / 14 skipped`，两条既有 Gymnasium 观测范围提示；分层专项 `18 passed`，必需编译检查及 MuJoCo 模型/物理步进/EGL 渲染检查通过，保留的 72 份依赖文件摘要未变。此前目录整理的 11 包 `symlink-install` 构建通过；安装后的视觉模型与后端路径已同步至 `scripts/`。
- 51 份运行/模型配置摘要一致，本轮未修改标定或控制参数。空闲隔离域 119/176、`ROS_LOCALHOST_ONLY=1`；Goal 前验证唯一 MuJoCo Action 和反馈源；收尾测试进程及 DDS 端口无残留，既有 domain66 未动。
- 视觉规划结束后的 MoveGroup 有一次超过默认 5 s 退出期限、由 launch 升级 SIGTERM 的观察记录；功能执行阶段正常，退出原因待另行核对。
- 核验结果、问题原因和修复方向已写入[MuJoCo 使用说明](../../src/rebotarm_simulation/README_mujoco.md#当前仿真核验与已知问题)。原始证据保留在本机 `log/functional_sim_20261001/`，不随文档提交。软件验证未访问串口、启动实机、安装依赖或执行训练；两处问题尚未修复。

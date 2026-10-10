# 当前项目状态与关键决定

> 状态：REFERENCE；类型：项目状态；适用范围：当前工作空间及强化学习远端部署；最后核对：2026-10-10（整合上游 main@8c02470，保留本地控制与独立 RL 布局）。

本文件是项目状态的唯一文档，保留当前有效决定、验证范围和未解决问题。
功能、架构、参数与启动命令分别链接到对应手册；不维护历史任务阶段、进度百分比或重复流水。
状态变化时直接更新本文，不再生成独立状态 JSON 或事件日志。

## 当前基线

- 主目录：`/home/a/project/rebot_Arm`；开发分支为 `develop`、发布分支为 `main`，远端仓库为 `QingYuan-Chen/rebot_Arm`。
- 当前源码已整合 `huangbinai/robotarm_ros2 main@8c02470f594cfa0180e2ebfb86d1e14a640b4db3`，相对上次 `19f2939` 纳入 25 个上游提交。本次发布内容包括上游整合及已有 RL 迁出改动；原整合提交为 `dc7e067`，本轮整合前开发基线为 `53f9802`。此处是核对快照，不代表上游永远最新。
- Ubuntu 24.04 / ROS 2 Jazzy，12 个活动 ROS 包；职责和依赖边界见[架构说明](../implemented/architecture.md)。
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
- MuJoCo Viewer、Reach/Pick、示教离线预演、Sim2Real/Real2Sim 和诊断保留；采用上游 `core/control/execution/ros/apps/diagnostics` 分层。无物理引擎的 RViz 轻量预演归新包 `rebotarm_preview`，MuJoCo 物理执行仍归 `rebotarm_simulation`。
- 视觉节点、策略、推理后端及工具采用上游子包布局；launch 按输入、候选、过滤、执行等职责拆分，默认配置集中到 backend/strategy/motion profile。接入确认停止接口，保留采集时间戳检查和本地正式单瓶入口。示教录制、准备及回放模块拆分后继续保留原停止与碰撞门。
- MJX 专属训练、依赖和对比工具已剥离；CPU Gymnasium Reach 与可选 SB3/PPO 保留。新的 MJLab 独立工作区位于 `/home/a/project/rebot_Arm_rl/MJLab/`，后续 IsaacLab 使用独立同级目录（尚未引入），不改变现有 ROS 仿真环境，见 [DRL 使用与部署](../../../rebot_Arm_rl/MJLab/README.md)。
- 2026-10-04 用户选定并授权实现、打包 Reach-and-Hold（末端到达并稳定保持）；到达后须停稳并连续保持，不能将瞬时经过目标算作成功。任务范围与初版仿真契约见 [DRL 任务说明](../../../rebot_Arm_rl/MJLab/README.md#首个任务reach-and-hold)。
- 参数以当前包内配置和显式 launch 覆盖为准，见[参数来源](parameters/current_parameter_sources.md)，不从旧任务记录复制。

## 当前限制与待解决问题

- 2026-10-03 按用户授权构建并安装 MotorBridge `0.4.7+rebotarm.1`，版本/反馈序号检查通过；构建使用仓库固定提交与摘要校验的补丁，无硬件连接。
- 重力补偿已补齐时间积分、模式检查、严格发送、力矩边界、周期诊断与权威模型来源，详见[硬件执行说明](../implemented/features/hardware_execution.md#重力补偿控制)。实际控制时序、实物附加载荷/质心、零位/方向及模式切换连续性仍需本窗口另行实机授权后采样验收；Python 控制线程不提供硬实时保证，笛卡尔阻抗尚未实现。
- 2026-10-01 仿真功能核验发现：MuJoCo 轨迹 Action 未在接纳前检查位置范围；J3 `+0.03 rad` 请求超过 `+0.02 rad` 上限，底层裁剪后仍返回成功。需补充逐轴轨迹入口校验，不能扩大限位或放宽完成容差。
- MuJoCo 从 position 切换到 hold 会清零控制器支撑力矩；独立取消测试出现约 `0.047 rad` 短时偏移，静止零运动切换也复现。取消终态和最终保持正常，但切换连续性待修复；此结论限于仿真，未验证真实控制器有相同行为。
- MJLab Reach-and-Hold、位置增量/受限力矩PD控制、远端PPO入口、导出契约、策略回放与未见目标成对评测已实现于 `/home/a/project/rebot_Arm_rl/MJLab/`。本机仅导出/测试/回放，CLI和Runner均拒绝本机训练；远端工作区仍为 `/root/autodl-tmp/rebot_Arm/DRL`。8192环境、2000轮、seed 42的正式训练已完成，完整运行已下载本地；全部21个检查点在固定16组未见目标上成功率均为0，原策略到位后关节速度短时超出保持门限并重置计时，尚未按完整任务目标收敛。奖励与成功终止的组合存在拖延结束的激励，需先讨论改进再另行授权实施/训练。详见 [DRL说明](../../../rebot_Arm_rl/MJLab/README.md)及下方评测记录。
- MuJoCo Warp 3.11.0 加载现有模型时提示 capsule/cylinder、capsule/mesh、cylinder/box 组合不支持多接触点，每对最多生成一个接触点。基础GPU物理与Reach碰撞门检查通过，但未来接触类任务须单独评估该限制；主项目权威模型和接触参数未修改，DRL任务固定夹爪/增加地面的派生模型见其README。
- MoveIt Plan 成功不等于 Execute；真实执行需要 `moveit_simple_controller_manager` 和正确的 Action 链。
- Star Arm 联合跟随的 J5 动态滞后根因未查清：已知触发是整形命令与新鲜反馈误差约 `0.262 rad` 持续超限。不能归因于 J3 限位、夹爪故障或已证实的旧反馈缓存问题。
- 六轴 `0.25 rad / 0.3 s` 保护未获准统一放宽；逐轴边界、动态滞后和减速策略须先讨论并确认。
- 部分 launch 对布尔参数的解析仍存在既有差异；`use_hardware` 必须使用 `true/false`，不用 `1/0`，以免硬件和预演后端选择不一致。
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
- `scripts/` 保留环境安装/启动/检查脚本；GraspNet 推理后端归 `rebotarm_vision.backends`，YOLO/TensorRT 模型资产归 `src/rebotarm_vision/models/`。旧 P5 阶段工具和 P6 兼容入口已删除。
- 按用户最新决定保留功能回归测试：两轮删除的 127 个测试文件已全部恢复；本次上游整合后，`tests/` 共 141 个测试模块和 `conftest.py`。功能已实现不作为删除测试的依据；测试缓存可清理。
- 目录结构以用户手动整理后的布局为准；补充文档在 `docs/reference/`，SDK 清单在 `third_party/`，MotorBridge 补丁在根目录的 `patches/`。
- 按用户授权删除 `third_party/robotarm_ros2_upstream`、`robotarm_ros2_mujoco_snapshot` 和 `reBotArm_develop_hjx` 三个旧参考目录，包含其中未接入主项目的历史实验。`third_party/` 当前保留厂商 SDK、固定版本清单和 `COLCON_IGNORE`；正式源码与第三方归属说明继续保留。
- 独立跟随工具在根目录的 `star_arm_102_rebot_b601_follow/`；不再使用此前的 `相关资料/` 布局，不自动恢复或重新移动用户整理的文件。
- 单瓶抓取使用正式 `rebotarm_single_bottle_grasp` 入口；标定使用所属功能包的网页与 ROS 工具，见[标定操作说明](commands/calibration_web.md)。
- 当前源码的历史变更从 Git 历史追溯；已删除的本机未跟踪实验数据不保留。

## 最近软件验证

- 2026-10-10 上游 `main@8c02470` 内容已整合到主目录，采用独立工作树解决冲突和验证后再叠加补丁；落地前完整核对原工作区，已有 RL 迁出改动保留。硬件控制器、权威 URDF、手眼与单瓶抓取配置内容未变；正式单瓶入口仅调整反馈工具导入路径，保留受控回基线与健康失败保持规则。整合验证阶段未操作实机、独立 RL 工程、远端或系统依赖。2026-10-10 用户已授权将已验证代码提交并推送至自有仓库 `main`；保留上游提交的合并关系，发布范围不包含日志、安装目录、虚拟环境或本机 TensorRT engine。
- 本次主目录独立构建 12 包通过；最终完整回归 **1244 passed / 2 skipped**、分层 **26 passed**，必需及扩展编译、Shell 语法检查通过。MuJoCo 仍为 3.3.0 / NumPy 1.26.4；MotorBridge 只读契约核验为 `0.4.7+rebotarm.1`、反馈序号可用。修复重构中的路径引用、夹爪重置目标与 MuJoCo 3.3.0 快照 warmstart 精确恢复兼容问题，没有放宽物理或恢复断言。
- 本次无硬件验证：MuJoCo 模型加载、有限物理步进、EGL 离屏渲染、2 秒 CLI 运行通过；ROS 仿真夹爪服务、状态/时钟与轨迹执行通过，最大关节终点误差 `0.006106 rad`；MoveIt 生成 37 点轨迹并成功驱动 MuJoCo 执行，终点最大误差 `0.006904 rad`。使用 localhost 与隔离 ROS 域，仅清理本轮测试进程；MoveIt 关闭时 `move_group` 在 SIGINT 后 5 秒未退出，由 launch 对该测试进程发送 SIGTERM，最终无残留。上述结果不覆盖实机验收，也不关闭前述两项既有仿真待办。
- 证据与合并前备份在 `log/upstream_merge_20261010/`；原 `build/`、`install/` 保留为该目录内的 `build_before_integration/`、`install_before_integration/`。已确认新安装区没有退役 ready/benchmark/旧 simulation 轻量控制器入口。已有 TensorRT engine 复制到视觉包 `models/` 后 SHA256 一致，原文件保留；没有重新生成模型或修改 CUDA 依赖。
- 2026-10-10 按用户要求取消强化学习工程的 `DRL/` 层级，当前入口为 `/home/a/project/rebot_Arm_rl/MJLab/`；原总览合并到工程根 README，旧目录已删除。新环境在最终路径离线重建，294 个依赖版本一致；失效虚拟环境备份及测试缓存已清理。新包直接展开为 `MJLab/`，同时兼容历史两种 DRL 布局；既有离线包、模型、训练记录及评测证据摘要不变。MJLab 29 项（含 GPU）、主工程 1203 项通过/2 项跳过、分层 19 项通过，必需编译、依赖及文档链接检查通过；本机训练保护有效，无训练更新、无实机操作。验证证据在 `/home/a/project/rebot_Arm_rl/MJLab/outputs/flatten_layout_20261009/`。远端既有部署未改动，未提交或推送。
- 2026-10-09 按用户要求，将整个 DRL 迁至同级独立工程 `/home/a/project/rebot_Arm_rl/`，主工程旧目录已移除。源码、21 个正式训练检查点、离线依赖、部署包和评测产物完整保留；迁移清单 1295 份文件无缺失，除目录说明、忽略规则和模型同步脚本的路径修复外，内容摘要一致。新路径离线重建环境，294 个依赖版本不变；清除已失效的旧虚拟环境备份（约 8 GiB）及测试缓存，环境可由保留的锁文件和 wheelhouse 重建。模型同步默认定位同级 ROS 主工程，并支持显式 `--source-repo`；未重写模型快照或修改任务/控制参数。
- 本次迁移验证：MJLab 28 项（含 GPU）、主工程 1203 项通过/2 项跳过、分层 19 项通过；依赖、必需编译、路径和文档链接检查通过。GPU 算术、两环境物理步进、Reach 25 步和网络前向通过，本机禁止训练保护有效，零训练更新。使用已核验空闲的 ROS 域 110/176/177/181 和 localhost 隔离，未操作实机或远端部署。证据在 `/home/a/project/rebot_Arm_rl/MJLab/outputs/migration_rebot_Arm_rl_20261009/`；未提交或推送。
- 2026-10-04 检查点评测：完整读取2000轮TensorBoard曲线，21个检查点各完成40组ONNX/PyTorch数值对比，并用同一16组固定未见目标、10秒时限进行确定性仿真评测；全部模型成功0/16、违规0/16，除model_0外均曾16/16进入10 mm区域。最终model_1999终点误差均值0.540 mm/最大1.385 mm，最长连续合格保持均值0.689 s/最大0.86 s；model_800终点均值最佳0.347 mm，model_900单条最长保持最佳0.98 s，均未达1 s要求。同协议IK基线16/16成功，终点均值0.320 mm。最终模型249次保持计时归零均伴随关节速度超过0.05 rad/s，位置及末端速度未触发这些归零；J4此类速度超限最多。
- 保持诊断：仅在独立仿真对照中，将最终策略连续合格100 ms后的目标增量置零，按原门限获得16/16成功、零违规，平均成功时间2.295 s。这是加入保持接管后的诊断成绩，不是原策略成绩。原策略超时平均总回报62.53，成功诊断14.53，IK成功基线15.74；结合目标附近持续奖励约7分/秒和成功后5分并终止，强烈提示奖励鼓励延长回合，但学习因果仍需独立对照训练验证。最后200轮训练回报均值62.324、回合长度500步，成功指标为0；到达误差和回报趋稳不等于Reach-and-Hold收敛。未修改源码、任务参数或门限，零训练更新，21个原检查点SHA256不变。完整指标、图表、逐目标轨迹和诊断在 `/home/a/project/rebot_Arm_rl/MJLab/outputs/checkpoint_review_20261004_210919/report.md`；结果限于16组目标，不构成全工作空间或实机验收。
- 2026-10-04 正式训练下载：核验远端运行 `20261004_154811_868200` 已完成第1999/2000轮、总524288000环境步，日志耗时25分21秒；核验时无活动训练或GPU计算进程。完整运行保存至 `/home/a/project/rebot_Arm_rl/MJLab/runs/reach_hold_v1/20261004_154811_868200/`，含21个检查点、环境/网络配置、目标集、部署契约和TensorBoard记录；另保留完整终端日志。共27文件、15439642字节，传输前后远端摘要稳定，下载文件全部SHA256一致。最终 `model_1999.pt` 在本机以CPU、weights_only模式加载通过，60个张量均有限，迭代号1999；模型契约和目标集与当前本地一致，训练源码摘要一致（仅打包布局逻辑有已知差异）。末轮日志TCP误差指标为0.0066 m、成功指标为0；后续固定未见目标评测见上文。下载及核验记录在 `/home/a/project/rebot_Arm_rl/MJLab/outputs/model_download_20261004_173803/`；远端原件保留，下载阶段未训练、导出ONNX或操作实机；后续评测阶段另行导出ONNX并保留验证记录。
- 2026-10-04 本地 DRL 按框架整理：MJLab 源码、任务、锁文件、离线依赖、缓存、包及完整运行产物迁入 `/home/a/project/rebot_Arm_rl/MJLab/`，`DRL/` 仅保留总览与 ROS 构建排除标记；后续 IsaacLab 使用同级独立目录，当前未安装。虚拟环境在新路径离线重建，294个已安装包版本与原环境一致；入口、激活路径、缓存路径、模型同步来源及本机禁止训练保护均核验。未改任务参数、主模型或 ROS/视觉环境，也未连接或迁移远端既有 r2 部署。
- 迁移验证：MJLab完整25项（含GPU）通过，主项目1203通过/2跳过、分层19项通过，必需编译及依赖检查通过。GPU算术、两个模型各10步物理、Reach两个环境各25步及RSL前向通过，零训练更新。空闲ROS域110与固定域176/177/181使用localhost隔离，既有域66未动。验证证据见 `/home/a/project/rebot_Arm_rl/MJLab/outputs/mjlab_layout_20261004/`。
- 迁移前后384份既有部署包、wheel、模型/配置及运行产物的大小和SHA256一致，四个旧包均通过新验证器。新源码包 `/home/a/project/rebot_Arm_rl/MJLab/dist/rebotarm-mjlab-source-20261004-layout.tar.gz` 已按新结构打包、校验52个文件并在独立解压路径完成16份资产和CPU步进检查。既有r2完整包仍保留旧解压结构；本轮未重打完整离线包。使用与部署路径见 [MJLab说明](../../../rebot_Arm_rl/MJLab/README.md)。
- 2026-10-04 develop提交前复验：加载ROS Jazzy及本工作区现有安装后，主项目完整回归1203通过/2跳过（两条既有Gymnasium提示）、分层19通过；DRL完整22项通过（含GPU物理边界、实际网络导出和训练主机保护），pip check及全部必需compileall通过。首次测试终端未加载工作区消息包导致收集失败，补齐环境后重跑通过，没有为此改动代码。ROS域110与固定测试域176/177/181检查为空闲后使用localhost隔离，既有实机域66未动，测试结束无残留。仅提交源码、模型快照、依赖锁、测试及文档；虚拟环境、缓存、离线包和完整训练产物保留本机并排除提交。验证证据：`/home/a/project/rebot_Arm_rl/MJLab/outputs/publish_develop_20261004/`。
- 用户反馈8192环境运行无压力后，使用8192环境、2000轮、seed 42完成正式训练，完整运行与下载验证见上文；命令见[远端训练](../../../rebot_Arm_rl/MJLab/README.md#远端训练)。尚未取得该档位完整训练峰值显存和收敛评测证据，不把此前采样吞吐记作训练吞吐。默认256环境和任务参数未修改，既有r2离线包保持原样。
- 2026-10-04 远端训练冒烟已核验：用户运行 `runs/reach_hold_v1/20261004_152429_567896`，256环境、每轮32步、20轮、seed 42，总163840环境步；完成至第19轮并保存 `model_19.pt`、配置及TensorBoard日志，后几轮含学习更新的吞吐约1.5万环境步/秒。该日志未记录训练显存峰值，结束后GPU为空闲，不能用空闲显存推断训练占用。完整运行副本及日志保留在本机 `/home/a/project/rebot_Arm_rl/MJLab/outputs/concurrency_probe_20261004/`。
- 同机并行采样测量：固定上述 `model_19.pt`，逐档独立进程，2轮预热、6轮测量，每轮32步，包含物理、actor/critic前向、采样动作及rollout存储，加载已有优化器状态；没有反向传播或优化器更新，网络权重与归一化状态摘要前后相同。256/512/1024/2048/4096/8192环境的采样阶段峰值显存分别约0.71/0.84/1.12/1.67/2.74/4.89 GiB，吞吐分别约2.07/4.05/7.86/14.79/26.71/44.96万环境步/秒。显存由0.3秒间隔的整卡采样得到，不能视为完整PPO训练峰值；该吞吐也不含学习更新。建议下一轮4096环境做完整短训练验证，再对照8192；尚未修改默认256环境配置。并行数同时改变PPO每轮样本数，不能用相同轮数直接比较训练效果。原运行保留，本轮没有新增训练运行，测量结束GPU无残留进程。原始结果和采样记录：`/home/a/project/rebot_Arm_rl/MJLab/outputs/concurrency_probe_20261004/`。
- 2026-10-04 DRL远端部署：AutoDL Ubuntu 22.04.5 / Python 3.12.3 / RTX 4090 24 GiB，驱动580.76.05；独立工作区 `/root/autodl-tmp/rebot_Arm/DRL`，不部署ROS。SSH上传r2源码包，复制46个与原离线包摘要一致的旧wheels，其余依赖按锁文件下载；安装前后194个文件均与r2完整包清单一致，142个wheels齐备。本地完整包保留；远端保留源码压缩包及完整wheelhouse，没有保留未传完的完整压缩包。默认离线安装创建独立 `.venv`，MJLab 1.6.0、MuJoCo/mujoco-warp 3.11.0、Warp 1.14.0、PyTorch 2.9.1+cu128、RSL-RL 5.4.2；补齐远端缺失的系统EGL运行库及其Mesa依赖，NVIDIA驱动不变。
- 远端 `pip check`、源码编译、模型资产摘要、CUDA算术、两个模型各10步GPU物理、Reach两个环境各25步与RSL网络前向均通过；DRL完整22项测试通过，EGL无窗口渲染通过。16组IK基线全部成功、零违规，最大最终误差0.877 mm，成功时间3.40–3.78 s；与本机相同协议的基线成功/违规/时间一致，最终误差最大差异小于 `3.6e-8 m`。该次安装验收没有学习更新或训练运行目录，GPU检查进程已退出；这不证明PPO收敛或实机适用性。安装验收时远端50 GiB数据盘剩余约21.13 GiB。旧MicroDuck源码及完整训练产物491个文件摘要仍一致。证据已同步至本机 `/home/a/project/rebot_Arm_rl/MJLab/outputs/remote_install_20261004/`，基线报告与轨迹在 `/home/a/project/rebot_Arm_rl/MJLab/outputs/reach_baseline_remote_20261004/`；部署包README中的远端待核验表述属于打包时快照，当前部署状态以本文为准。
- 2026-10-04 Reach-and-Hold：DRL专项22项（含CUDA物理边界、实际训练配置到ONNX导出），主项目完整1203通过/2跳过、分层19通过，必需编译、pip check与diff检查通过。128组训练/16组未见目标经FK、限位和61点路径碰撞预检；IK基线16/16到达并保持、零违规，最大最终误差0.877 mm。零输出ONNX测试策略16组全部超时，未误报成功；模型记录回放通过。测试使用空闲ROS域110/176/177/181及localhost隔离，既有实机域66未动，测试域无残留。未训练或访问实机，仿真基线结果不是学习策略性能。说明及命令见 [DRL README](../../../rebot_Arm_rl/MJLab/README.md)，验证日志归档在 `/home/a/project/rebot_Arm_rl/MJLab/outputs/verification_20261004/`，评测报告与轨迹保留在 `/home/a/project/rebot_Arm_rl/MJLab/outputs/`。
- 含任务的最终部署包为 `/home/a/project/rebot_Arm_rl/MJLab/dist/rebotarm-drl-reach-hold-offline-20261004-r2.tar.gz`（约4.2 GiB、194个校验文件），另附源码版（52文件）和SHA256。最终包已实际解压到新路径、离线重建环境，pip check、CPU检查、19项常规测试（3项可选GPU测试跳过）、两个环境各10步GPU任务检查及RSL前向通过；主环境22项包含全部GPU边界测试。补齐安全YAML导出和跨机器测试保护；本机没有训练更新。源码包在打包时与源码摘要一致；本次目录调整后仍保留为旧结构快照，不覆盖重打；按用户授权清理下载/编译/测试缓存及回收站内本任务的临时安装、废弃打包副本，释放约41.1 GiB。36份验证日志/报告已归档，保留最终包、校验文件、`.venv`、`wheelhouse`和评测产物；清理后包摘要不变、pip check通过。远端部署及验证见上文。
- 2026-10-04 远端准备：按授权清除两处MicroDuck uv下载缓存及用户级编译缓存，50 GiB数据盘释放约11.35 GiB，清理后、部署前 `/root/autodl-tmp`剩余约33.58 GiB。MicroDuck源码和完整训练产物491个文件清理前后SHA256一致，原环境发行包文件无缺失、无缓存路径依赖；旧环境和安装介质保留。清理证据：`/home/a/project/rebot_Arm_rl/MJLab/outputs/remote_cleanup_20261004.log`。连接密码未写入项目文件。
- 此前通用环境包 `/home/a/project/rebot_Arm_rl/MJLab/dist/rebotarm-drl-offline-20261003.tar.gz` 及源码版仍保留；首任务部署使用上述20261004-r2包。此前新路径离线安装、CPU检查及12项测试通过的日志已归档至 `/home/a/project/rebot_Arm_rl/MJLab/outputs/verification_20261004/`，临时验证副本按用户授权清除；通用旧包未用于本次远端部署。
- 2026-10-03 MJLab 独立工作区：DRL 专项 `12 passed`，主项目完整回归 `1203 passed / 2 skipped`，分层 `19 passed`，必需 compileall 与 `git diff --check` 通过。原 ROS/视觉/系统 Python 依赖未改动；域 110 与固定测试域 176/177/181 经扫描为空闲后使用 localhost 隔离，既有域 66 未动。新环境的依赖一致性、16 份模型资产摘要、CPU 模型步进、RTX 5060 上 PyTorch 算术及 MuJoCo Warp 两个并行模型各 10 步检查通过；ONNX 测试只使用未训练算术 fixture。当时首个任务未配置，未训练、未访问实机；通用报告比较不等于任务性能评测。操作说明见 [DRL README](../../../rebot_Arm_rl/MJLab/README.md)，本机检查输出已归档至 `/home/a/project/rebot_Arm_rl/MJLab/outputs/verification_20261004/`。
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

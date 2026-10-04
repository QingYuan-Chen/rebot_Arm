# 第三方来源与许可说明

本文件记录项目使用的第三方代码和资源来源，不替代各上游的许可条款。
根目录 `LICENSE` 保留 Apache License 2.0 正式英文文本；第三方材料应分别核对其许可。

## 当前本机依赖

### reBotArm 控制 SDK

- 来源：<https://github.com/huangbinai/rebotarm_control.git>。
- 固定版本：`6a49302804f25e624995e771acb6d61896d1856d`。
- 版本清单：`third_party/rebotarm_dependencies.repos`。
- 本机目录：`third_party/reBotArm_control_py`，不纳入主仓库版本管理。
- 用途：厂商适配、运动学、动力学和配置；按上游许可获取和使用。

### MotorBridge

- 来源：<https://github.com/motorbridge/motorbridge.git>。
- 固定版本：`2b7b350914ace47ba06e85fcad333143de2b057b`。
- 本地补丁：`patches/motorbridge/0001-add-feedback-sequence-api.patch`。
- 所需运行版本：`0.4.7+rebotarm.1`，必须支持 `feedback_sequence=true`。
- 版本与构建依据：`tools/setup_motorbridge_fresh_feedback.py`；未打补丁的版本不能替代当前真机运行依赖。
- 本地源码、工具链与安装包位于 `build_motorbridge_fresh_feedback/`，不在 `third_party/` 下。

### 视觉资源

仓库跟踪的 YOLO 权重为 `src/rebotarm_vision/models/yolo26s-seg.pt`。
其上游来源和许可仍需在再分发前核实，文件路径迁移不改变许可状态。

GraspNet 源码、检查点、PointNet2/KNN 扩展和 TensorRT 引擎是外部运行资源，
不随主仓库提供。本机使用 `third_party/graspnet-baseline` 与 `third_party/graspnetAPI`；
本机 `graspnet-baseline/LICENSE` 写明非商业研究用途限制，`graspnetAPI/LICENSE` 为 MIT 文本。
应保留各自的 `LICENSE` 以及内部依赖的许可文件。本机目录和用途见
[third_party 目录说明](third_party/README.md)。

## MuJoCo 历史快照来源

- 来源项目：`huangbinai/robotarm_ros2`。
- 来源地址：<https://github.com/huangbinai/robotarm_ros2>。
- 固定版本：`fb28dcdd358b45de79eb47adfb333e2e94e9d5b4`。
- 范围：历史上用于仿真对比的 57 个文件，包括代码、配置、MJCF/XML、网格、文档和测试。
- 当前实现：由该来源演化的代码仍在 `src/rebotarm_simulation`，本地修改由本仓库记录。
- 此处是来源归属记录，不是 Git 远程配置；开发远程以本仓库 `origin` 为准。
- 固定快照的包元数据声明 Apache-2.0，但快照根目录没有 LICENSE、COPYING 或 NOTICE。
  维护者曾于 2026-08-13 记录上游所有者对该确切快照公开再分发的许可。
  此历史记录不扩展到其他版本或无关第三方材料。

## 不随主仓库发布的本机材料

未被 Git 跟踪的模型与权重、Python 虚拟环境、构建/安装/日志目录、被忽略的 SDK
和 MuJoCo 本机依赖、实验原始证据以及历史实施计划，不属于主仓库发布内容。
Windows、HTTP、MJPEG 视觉工具和相关协议测试已退出支持范围。
仓库跟踪的 YOLO 权重仍需要单独核对来源与许可。

# 本机第三方依赖目录

本目录用于厂商 SDK 和视觉推理所需的第三方源码。
主仓库保留本说明、`COLCON_IGNORE` 和 `rebotarm_dependencies.repos`；
本机依赖源码由部署准备，不随主仓库发布。

## 目录与用途

| 目录或文件 | 用途与引用 | 维护方式 |
| --- | --- | --- |
| `reBotArm_control_py/` | 控制器厂商适配、动力学和配置；运动层通过所需接口使用运动学 | 保留；固定版本由 `rebotarm_dependencies.repos` 声明 |
| `graspnet-baseline/` | 视觉后端加载网络、工具模块及 PointNet2 扩展 | 保留部署实际使用的源码、模型和许可证 |
| `graspnetAPI/` | 视觉后端使用的 `GraspGroup` 数据结构 | 与所选 GraspNet 版本配套 |
| `rebotarm_dependencies.repos` | 固定 reBotArm SDK 地址和提交 | 变更前核对兼容性，不随目录整理升级 |
| `COLCON_IGNORE` | 阻止 colcon 发现第三方内部附带的 ROS 包 | 保留 |

实际存在的目录、版本和大小以当前机器只读检查为准；上游机器的审查结果不作为本机证据。
CPU MuJoCo 使用系统 Python 3.12 的用户级 MuJoCo 3.3.0 / NumPy 1.26.4，
不要求 `.venv-mujoco`。视觉和 GraspNet 分别使用 `.venv-vision/` 和
`.venv-graspnet/`；安装说明见[环境部署](../docs/setup/ubuntu_ros2_jazzy.md)。
独立 MJLab 工程及其环境位于 `/home/a/project/rebot_Arm_rl/MJLab/`。

## 构建产物与推理资源

第三方项目中的 `build/` 与 `__pycache__/` 可能是生成产物，但本次整合不清理本机依赖。
清理前须确认没有构建进程，也没有安装引用依赖这些路径。
不要误删推理所需的 `pointnet2/_ext*.so` 或其他已编译扩展。
视觉后端对部分训练辅助代码提供兼容处理，不代表整个 GraspNet 源码目录可以删除。

## 许可文件

应保留第三方源码自带的 `LICENSE`、`NOTICE` 和内部依赖许可；主项目 Apache-2.0
许可不覆盖这些资源。具体版本和再分发条件应按部署时获取的来源核实，
来源与历史归属记录见[第三方说明](../docs/reference/third_party_notices.md)。

# 本机第三方依赖目录

本目录用于第三方源码，不是待清理的缓存集合。
主仓库只跟踪本说明、`COLCON_IGNORE` 和 `rebotarm_dependencies.repos`；
下面三个依赖目录由本机部署准备，不随主仓库发布。

## 当前目录与用途

2026-10-04 本机审查结果（大小为近似值）：

| 目录或文件 | 用途与引用 | 结论 |
| --- | --- | --- |
| `reBotArm_control_py/`（41 MiB） | 控制器使用厂商适配、动力学和配置；`rebotarm_motion` 的预览求解使用运动学 | 保留；本地 HEAD 与 repos 清单固定版本一致，工作树无修改 |
| `graspnet-baseline/`（196 MiB） | 当前视觉后端加载网络、工具模块和 PointNet2 扩展；环境脚本设置为默认模型源码目录 | 保留；不是已退役的独立 GraspNet 服务 |
| `graspnetAPI/`（7 MiB） | 当前视觉后端从相邻目录加载 `GraspGroup` 数据结构 | 保留 |
| `rebotarm_dependencies.repos` | 固定 reBotArm SDK 的地址和提交 | 保留 |
| `COLCON_IGNORE` | 阻止 colcon 递归发现第三方内部附带的 ROS 包 | 保留 |

CPU MuJoCo 环境已统一放在仓库根目录 `.venv-mujoco/`，与 `.venv-vision/`、
`.venv-graspnet/` 并列；创建步骤见 [环境安装说明](../docs/setup/ubuntu_ros2_jazzy.md)。

## 可再审查的生成产物

- `graspnet-baseline/pointnet2/build/`：约 124 MiB 编译产物。
- `graspnet-baseline/knn/build/`：约 39 MiB 编译产物。
- `graspnetAPI/build/` 与 `graspnet-baseline/build/`：合计不足 1 MiB。
- 各目录中的 `__pycache__/`：Python 生成缓存。

这些是后续清理候选，本轮未删除。清理前应确认没有构建进程和指向这些目录的安装引用。
不要连同 `pointnet2/_ext*.so` 一起删除，该文件是推理所需的已编译扩展。
当前视觉后端为训练辅助代码的 KNN 导入提供替代实现，不能据此把整个 GraspNet 源码目录视为无用。

本轮未发现旧 MuJoCo 比较快照或旧 RL 环境目录；`.gitignore` 中历史目录名称不代表它们仍存在。
“当前使用”不等于“上游最新”。本轮检查本地内容、固定版本及调用关系，未执行依赖升级。
GraspNet 两个目录没有 `.git` 元数据，无法仅从本地目录确认其确切上游提交。

## 许可文件

本机 `graspnet-baseline/LICENSE` 写明非商业研究用途限制；`graspnetAPI/LICENSE`
为 MIT 文本，内部还包含其他依赖的许可文件。不要用主仓库 Apache-2.0 许可证覆盖这些文件，
也不要翻译后替换上游正式许可。来源与历史授权记录见 [第三方说明](../THIRD_PARTY_NOTICES.md)。

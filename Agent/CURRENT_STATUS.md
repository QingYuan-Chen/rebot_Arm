# 当前项目状态

> 更新时间：2026-09-30。本文件描述本地整合状态；上游记录和历史 P0-P6 不等于本轮验收。

## 当前阶段

- 当前阶段：上游 main@19f2939 已在当前主目录整合，软件验证通过，等待另行授权的实机/GPU 验收。
- 当前范围：上游 10 个新提交、MuJoCo/Gemini 2 负载、示教预演、Gymnasium/MJX Reach、文档重组。
- 当前安全状态：没有新的真实机械臂或相机操作授权，不访问串口，不启动运动链路。

## 当前验收清单

- [x] 主目录原始 HEAD、未提交项目记录和两边历史已保留。
- [x] 本地单瓶抓取、候选过滤和失败后受控回基线恢复代码已保留。
- [x] 仿真核心合并冲突已解决，旧虚拟 RGB-D 发布已退役。
- [x] 完整回归 1164 passed/15 skipped；分层 18 passed；编译和 11 包构建通过。
- [x] MuJoCo 8 关节/8 执行器、物理步进与 EGL 渲染通过；Pick 1 回合/10 步无安全违规；launch 参数与安装入口检查通过。

## 当前阻塞与风险

- 本地 MuJoCo venv 已安装 Gymnasium 1.2.3，但尚无 SB3/PyTorch/JAX/MJX，GPU 训练及收敛不在本轮验收范围。
- 自动测试不构成新基线的实机验收。

## 当前下一步

1. 根据用户后续要求使用更新后的仿真/感知入口。
2. GPU 训练和实机验收另行执行，不自动推送。

## 证据入口

- 详细事实和历史：`MEMORY.md`、`PROJECT_STATUS.md`。
- 操作日志：`ACTIVITY_LOG.md`；生成状态：`STATE.json`。

## 本轮软件证据

- `/tmp/rebotarm-upstream-20260930-full-2.log`：完整回归，ROS_DOMAIN_ID=119、ROS_LOCALHOST_ONLY=1。
- `/tmp/rebotarm-upstream-20260930-build.log`：11 包构建。
- `/tmp/rebotarm-upstream-20260930-health.json`、`/tmp/rebotarm-upstream-20260930-pick.json`：物理/渲染和小批次安全检查，不是抓取成功率验收。

# 网页手眼/TCP 标定软件

> 状态：IMPLEMENTED；类型：功能说明；适用范围：Dashboard `/calibration`、手眼采样和 TCP 采样；不代表真实设备精度验收。

## 功能是什么

网页提供可恢复的手眼和 TCP 标定软件流程：采集同步数据、检查质量、运行求解、比较训练/验证结果、导出报告并记录接受状态。

## 当前已经实现

- Dashboard `/calibration` 页面、HTTP 和 SSE 状态流；
- Image、CameraInfo 和 TF 同步采样；
- 手眼模式和 TCP 模式；
- 会话保存、恢复、版本和幂等控制；
- 多算法求解、训练/验证拆分和质量门；
- 残差、覆盖度和 bootstrap 诊断；
- 原子报告、失败报告保留和候选数据包导出；
- 浏览器刷新恢复、SSE 重连和接受记录；
- 软件模式下的请求互斥和 fail-closed 保护。

## 当前验证结论

| 验证层级 | 结论 |
|---|---|
| 纯算法和会话存储 | 已验证 |
| Dashboard HTTP/SSE/ROS 软件链 | 已验证 |
| 合成图像多姿态手眼流程 | 已验证 |
| MuJoCo FK/投影图像组合流程 | 已验证 |
| TCP 多姿态软件流程 | 已验证 |
| 真实相机、真实机械臂和物理精度 | 尚未作为当前软件文档的完成结论 |

## 边界

- 合成图像和控制器替身不能证明真实相机安装精度；
- 求解结果被接受不等于自动部署外参；
- 软件流程通过不等于真实机械臂运动或 TCP 物理精度通过；
- 采集失败不会自动使能或失能真实机械臂；
- 上游软件审计记录未作为本机原始证据保存；当前本地验证边界见 [项目状态](../../reference/project_status.md)。

## 使用入口

- 操作命令：[网页标定操作说明](../../reference/commands/calibration_web.md)；
- 技术路线：[网页手眼标定技术路线](../../design/calibration_web_plan.md)；
- 上游状态来源：[固定提交的项目记忆](https://github.com/huangbinai/robotarm_ros2/blob/19f2939e27c52e07f73e7ff1ab8f723c01852ce0/Agent/MEMORY.md)；上游软件审计记录不构成本地的新实机验收。

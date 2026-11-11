# 当前设计文档

> 状态：DESIGN；类型：技术路线与实施缺口索引；适用范围：仍在讨论或实施中的方案。

这里放仍在讨论或实施中的技术路线、接口合同和设计决策。

当前设计路线：

- [`engineering_audit_scope.md`](engineering_audit_scope.md)：工程规范审核与分类消融范围；附三项错误契约整改及工程中文注释完善方案。
- [`visual_grasp_cleanup_scope.md`](visual_grasp_cleanup_scope.md)：视觉执行与通用夹爪力夹取/握持剔除范围，区分整文件删除、共享定义迁移和保留项。
- [`controller_refactor_plan.md`](controller_refactor_plan.md)：Python 控制器结构整理记录；原 C++ 迁移计划已废弃，后续迁移由另一个仓库承担。
- [`calibration_web_plan.md`](calibration_web_plan.md)：网页手眼标定的完整路线。软件链路已实现，但真实相机、真实机械臂和物理精度验收仍是后续工作。

技术路线完成后，应将已采用内容同步到当前架构/操作文档，未采用内容标为历史，而不是继续伪装成当前计划。

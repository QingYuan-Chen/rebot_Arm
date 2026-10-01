# 参考文档

> 状态：REFERENCE；类型：参数、接口和命令索引；适用范围：当前可查阅资料。

这里放当前系统的可复制命令、节点拓扑、数据流、参数来源和接口参考。

当前内容包括：

- [`context.md`](context.md)：部署范围、术语和运行契约；
- [`ros_sdk.md`](ros_sdk.md)：ROS SDK、控制器接口及开发说明；
- [`third_party_notices.md`](third_party_notices.md)：第三方来源与许可说明；
- [`project_status.md`](project_status.md)：唯一项目状态文档，维护关键决定、验证范围和未解决问题；
- `commands/`：唯一的可复制功能命令来源，见 [`commands/README.md`](commands/README.md)；
- `topology/`：节点所有权、launch 组合和数据流；
- `parameters/`：参数文件、launch 参数、Topic/Service/Action 的来源索引；

建议阅读顺序：先看 [节点与数据流](topology/system_dataflow.md) 理解系统，再看 [命令索引](commands/README.md) 执行操作，最后按需查阅参数来源。

这里的“参考”表示集中查阅入口，不自动代表功能已经实现或硬件已经验收；执行真机命令前仍须结合当前代码、测试和 [项目状态](project_status.md)。

# 视觉抓取规划与执行编排

> 状态：IMPLEMENTED；类型：视觉抓取功能说明；适用范围：候选过滤、规划门和阶段编排；不宣称真实抓取成功。

## 功能概述

在 Ubuntu 原生 RGB-D/YOLO/GraspNet 链路上，系统生成候选并依次执行坐标、工作空间、IK、碰撞、夹爪和新鲜度检查，再交给运动层规划。

## 当前实现

- 候选深度、TF、时间戳和置信度检查；
- 工作空间、姿态、夹爪宽度和关节变化约束；
- IK 与 MoveIt 状态有效性/碰撞门；
- pregrasp、approach、grasp、verification、retreat 阶段编排；
- 主入口固定执行流程，由 use_hardware 选择仿真或真机；启动不自动使能或抓取；
- 节点内部保留规划预检和纯规划能力；公共 launch 不再提供 plan-only 切换；
- 失败、过期候选和无候选时 fail-closed。
- 停止确认门控：`/visual_grasp/stop` 停止并保持当前位置，只有旧执行请求退出、动作终态及新鲜静止反馈确认后才重新开放 execute；确认失败继续阻断，排查后重试 stop。普通流程不再提供 reset，也不自动回 safe_home。

## 验证结果

视觉只读、候选过滤、Marker/Open3D、规划预检和仿真 benchmark 已有软件或仿真验证。完整数据流和命令见 [视觉抓取命令](../../reference/commands/visual_grasp_commands.md)。

## 边界与未完成

候选、规划或仿真接触通过不等于真实抓取成功；真实 approach、lift、retreat、放置和结果分类仍需单独分级验收和授权。

重复抓取从当前反馈姿态规划，不强制回观察位。新过期输入会撤销旧缓存，execute 在缓存缺失/过期时有界等待有效计划并保留具体拒收原因。

# 视觉抓取规划与执行编排

> 状态：IMPLEMENTED；类型：视觉抓取功能说明；适用范围：候选过滤、规划门和阶段编排；不宣称真实抓取成功。

## 功能概述

在 Ubuntu 原生 RGB-D/YOLO/GraspNet 链路上，系统生成候选并依次执行坐标、工作空间、IK、碰撞、夹爪和新鲜度检查，再交给运动层规划。

## 当前实现

- 候选深度、TF、时间戳和置信度检查；
- 工作空间、姿态、夹爪宽度和关节变化约束；
- IK 与 MoveIt 状态有效性/碰撞门；
- pregrasp、approach、grasp、verification、retreat 阶段编排；
- plan-only、仿真执行和可选真机执行接口；
- 失败、过期候选和无候选时 fail-closed。

## 验证结果

视觉只读、候选过滤、Marker/Open3D、规划预检和仿真 benchmark 已有软件或仿真验证。完整数据流和命令见 [视觉抓取命令](../../reference/commands/visual_grasp_commands.md)。

## 边界与未完成

候选、规划或仿真接触通过不等于真实抓取成功；真实 approach、lift、retreat、放置和结果分类仍需单独分级验收和授权。

# MuJoCo 示教轨迹预演

> 状态：REFERENCE；类型：纯仿真示教预演操作说明；适用范围：六轴示教 JSONL 预演；不连接真实机械臂。

## 适用范围与安全边界

本命令读取已有示教 JSONL，在独立 MuJoCo 模型中预演准备后的六轴轨迹。原文件只读，不连接控制器、不访问串口，也不构成真机回放授权。

## 前置条件

- 已完成 ROS 2 Jazzy 环境加载；
- 已按[仿真依赖说明](../../setup/dependencies/mujoco.md)为系统 Python 安装依赖；
- 已准备可读的示教 JSONL 文件；
- 有桌面时可使用 Viewer，无桌面时省略 `--viewer`。

## 执行命令

```bash
cd /home/a/project/rebot_Arm
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run rebotarm_simulation rebotarm_mujoco_teach_preview \
  /你的/示教记录.jsonl --viewer
```

预演执行与 viewer 生命周期由 `rebotarm_simulation` 管理。

## 输出与失败条件

报告包含原始/准备后的质量分级、轨迹时长、最大跟踪误差、接触及瓶子起终位置。

以下情况会拒绝预演：

- 轨迹质量为红色；
- 关节名不匹配；
- 时间戳倒退；
- 起点超出模型限位。

如果示教记录没有夹爪命令，程序不会猜测夹爪开合。MuJoCo 接触不代替 MoveIt 完整碰撞预检。

## 停止与后续

Viewer 关闭或终端按 `Ctrl+C` 即可结束。需要真实回放时，必须回到 [遥操作命令](rebotarm_common_commands.md) 的完整安全流程。

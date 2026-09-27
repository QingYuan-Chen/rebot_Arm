# MuJoCo 示教轨迹预演

使用已有示教 JSONL 文件；原文件只读，程序复用示教包的平滑、滤波、重采样与
重定时流水线，再在独立 MuJoCo 模型中按重定时路径播放：

```bash
cd /home/huangbin/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
third_party/rebotarm_mujoco_venv/bin/python -m rebotarm_teach.mujoco_preview \
  /你的/示教记录.jsonl --viewer
```

无桌面环境时去掉 `--viewer`。报告包含原始/准备后的质量分级、轨迹时长、
最大跟踪误差、接触及瓶子起终位置。红色质量、关节名不匹配、时间戳倒退或
模型限位外起点都会拒绝预演。当前功能只预演手臂六轴；示教记录若没有夹爪
命令，程序不会猜测夹爪开合。MuJoCo 接触并不代替 MoveIt 完整碰撞预检，
预演通过也不构成真机回放授权。

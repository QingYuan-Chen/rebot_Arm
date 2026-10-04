# 当前项目状态

> 这是接手项目时优先阅读的文件。更新时间：2026-10-04。
> 本文件只描述当前范围；历史 P0-P6 见 `PROJECT_STATUS.md`，过程记录见 `MEMORY.md` 和 `ACTIVITY_LOG.md`。

## 当前阶段

- 当前阶段：RL 已迁移至独立 rebotarm_rl 仓库；本仓库维护 ROS 应用、CPU MuJoCo 集成和模型源。
- 当前范围：ROS 应用、CPU MuJoCo 仿真及策略部署接口；RL 训练由独立仓库维护。
- ROS 仿真环境：.venv-mujoco，约333 MiB；旧ROS内训练环境已退役。
- 当前安全状态：没有新的真实机械臂 enable、trajectory、gripper、approach、lift、retreat 或接触授权。

## 当前验收清单

- [x] Reach 环境可启动并完成基本 reset/step。
- [x] 位置与姿态观测、奖励、成功门、碰撞和超时处理已实现。
- [x] mjlab manager-based 环境插件已注册，Warp/RSL-RL/PyTorch CUDA 导入和 GPU smoke 通过。
- [x] 独立 RL 环境已完成 GPU 短训练迭代；结果仅证明链路可运行。
- [ ] 多随机种子训练和固定评估集尚未完成。
- [ ] Reach 成功率尚未达到预设验收门槛；当前策略不能宣称已经学会。
- [ ] sim-to-real 的 action、observation、控制周期、关节顺序和限位合同尚未冻结。

## 当前阻塞与风险

- Reach 策略目前只是证明训练链路可运行，尚未稳定收敛。
- 真实 Gemini 2 的最新 K/D、畸变和 RGB-D 一致性需要设备连接后重新核对；不能使用 MuJoCo 参考内参代替实机证据。

## 当前下一步

1. 在独立仓库 https://github.com/huangbinai/rebotarm_rl 继续训练与固定评估集验证。
2. 冻结 mjlab 的 action、observation、奖励、控制周期和目标采样合同。
3. 在仿真侧冻结 sim-to-real 合同，再决定是否设计 LeRobot 数据转换或真实控制桥接。
4. 只有在用户明确授权后，另行进行真实相机或机械臂验证。

## 证据入口

- 当前任务队列：`EXECUTION_FLOW.md` 的“当前执行队列”。
- 当前/历史证据分类：`evidence/README.md`，再按 `evidence/current/` 或 `evidence/archive/` 阅读。
- 历史 P0-P6：`PROJECT_STATUS.md`。
- 详细事实、决策和历史上下文：`MEMORY.md`。
- 追加式操作日志：`ACTIVITY_LOG.md`。
- 机器可读快照：`STATE.json`，由 `update_state.py` 生成。

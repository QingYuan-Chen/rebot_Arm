# mjlab 强化学习：独立仓库

训练代码、配置、任务测试和配对评估已迁移至 [https://github.com/huangbinai/rebotarm_rl](https://github.com/huangbinai/rebotarm_rl)。
本工作区保留 CPU MuJoCo 基准、ROS 仿真后端和模型源文件。
训练项目不启动 ROS，不连接真实机械臂；当前六轴力矩动作不能直接作为 ROS 位置轨迹执行。

## 本地与云端入口

~~~bash
git clone https://github.com/huangbinai/rebotarm_rl.git
cd rebotarm_rl
python3.12 -m venv .venv
source .venv/bin/activate
# Keep ROS Python paths out of the independent training environment.
unset PYTHONPATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH
python -m pip install -r requirements/mjlab-cu130.txt
python -m pip install -e '.[test]'
python -m rebotarm_rl.resources
python -m mjlab.scripts.list_envs
MUJOCO_GL=egl python -m mjlab.scripts.train RebotArm-Reach-Mjlab --env.scene.num-envs 128 --agent.max-iterations 1000 --log-root runs/mjlab
~~~

模型通过训练仓库的 model_manifest.json 固定 ROS 仓库提交与 SHA-256，
下载到用户缓存。不需要本 ROS 工作区、colcon 或 source ROS。
自定义模型需显式设置 REBOTARM_MJLAB_SCENE，并保留 XML 引用和网格资源。

训练/回放/CPU-GPU 配对评估及云端记录要求以
[训练仓库说明](https://github.com/huangbinai/rebotarm_rl#readme) 为准。短 smoke 不代表策略收敛或实机验收。

ROS 仿真使用专用轻量环境 .venv-mujoco；
新安装只需 requirements/requirements-mujoco.txt。训练使用独立仓库的 .venv。
Isaac Lab 仍为计划中的后端，本次迁移没有实现其训练任务。

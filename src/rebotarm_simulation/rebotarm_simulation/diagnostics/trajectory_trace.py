"""MuJoCo 轨迹验收的逐物理步 JSONL 记录器。

文件职责
    记录参考 q/v/a/jerk、实际 q/v、参考减实际的误差、执行力矩和模式，供离线分析。
对外接口
    TrajectoryTrace(path) 开启记录；空路径禁用；record 接收同一物理步的 state、
    status 和可选 active={curve, start}；close 在仿真退出时关闭文件。
安全约束
    使用仿真时间求参考，不改变模型、控制器状态或 Action 成功条件。
    文件以写模式打开，同名文件会覆盖，应为每次验收指定独立路径。
    写入同步执行且无内部锁，由仿真所有者串行调用；不能用于实时硬件周期。
文件结构
    记录流初始化 → 单步参考/实际值编码 → 关闭。
"""

import json
from pathlib import Path


class TrajectoryTrace:
    def __init__(self, path):
        self.stream = None
        if path:
            target = Path(path)
            target.parent.mkdir(parents=True, exist_ok=True)
            self.stream = target.open('w', encoding='utf-8')

    def record(self, state, status, active):
        """有活动曲线时按仿真经过时间采样；否则记录保持目标和零参考导数。"""
        if self.stream is None:
            return
        elapsed = state.simulation_time-active['start'] if active else None
        reference = active['curve'].sample(elapsed) if active else (
            tuple(status.joint_targets), (0.,)*6, (0.,)*6, (0.,)*6)
        self.stream.write(json.dumps(dict(time=state.simulation_time, elapsed=elapsed,
            reference=reference, q=state.joint_positions, v=state.joint_velocities,
            joint_names=list(active['curve'].joint_names) if active else list(state.joint_names[:6]),
            error=tuple(a-b for a,b in zip(reference[0],state.joint_positions)),
            effort=state.actuator_forces, mode=status.mode), allow_nan=False)+'\n')

    def close(self):
        """冲刷并关闭记录流；禁用记录时无需额外处理。"""
        if self.stream is not None:
            self.stream.close()

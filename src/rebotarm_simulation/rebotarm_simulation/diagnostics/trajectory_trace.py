"""可选验收记录，只观察物理步，不改变模型、增益或 Action 判定。"""

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
        if self.stream is None:
            return
        elapsed = state.simulation_time-active['start'] if active else None
        reference = active['curve'].sample(elapsed) if active else (
            tuple(status.joint_targets), (0.,)*6, (0.,)*6, (0.,)*6)
        self.stream.write(json.dumps(dict(time=state.simulation_time, elapsed=elapsed,
            reference=reference, q=state.joint_positions, v=state.joint_velocities,
            error=tuple(a-b for a,b in zip(reference[0],state.joint_positions[:6])),
            effort=state.actuator_forces, mode=status.mode), allow_nan=False)+'\n')

    def close(self):
        if self.stream is not None:
            self.stream.close()

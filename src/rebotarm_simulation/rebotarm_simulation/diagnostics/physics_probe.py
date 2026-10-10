"""Shared raw-MJCF probes; independent of the closed-loop arm controller."""
from dataclasses import dataclass
from pathlib import Path
import operator


def positive_steps(steps):
    try:
        value = operator.index(steps)
    except TypeError as exc:
        raise ValueError("steps must be a positive integer") from exc
    if isinstance(steps, bool) or value < 1:
        raise ValueError("steps must be a positive integer")
    return value


def data_is_finite(data):
    import numpy as np
    return all(bool(np.isfinite(value).all()) for value in
               (data.qpos, data.qvel, data.actuator_force, data.time))


@dataclass
class PhysicsProbe:
    """Own one raw model/data pair for a diagnostic, not simulated execution."""
    mj: object
    model: object
    data: object
    path: Path

    @classmethod
    def load(cls, path):
        try:
            import mujoco
        except ImportError as exc:
            raise RuntimeError("MuJoCo probes require requirements-mujoco.txt") from exc
        path = Path(path).expanduser().resolve(strict=True)
        model = mujoco.MjModel.from_xml_path(str(path))
        return cls(mujoco, model, mujoco.MjData(model), path)

    def advance(self, steps):
        finite = data_is_finite(self.data)
        for _ in range(positive_steps(steps)):
            self.mj.mj_step(self.model, self.data)
            finite = data_is_finite(self.data) and finite
        return finite

"""Copied numeric values across controller component boundaries (no ROS/SDK objects)."""
from dataclasses import dataclass

@dataclass(frozen=True)
class MotorFeedback:
    pos: float
    vel: float
    torq: float
    status_code: int

    @classmethod
    def copy_from(cls, state):
        return cls(float(state.pos),float(state.vel),float(state.torq),int(state.status_code))

@dataclass(frozen=True)
class VerifiedFeedbackSample:
    state: MotorFeedback
    sequence: int
    observed_at: float

@dataclass(frozen=True)
class CachedMotorFeedback:
    values: tuple[float,float,float] | None
    raw_status_code: int | None
    failure_reason: str | None
    sample: VerifiedFeedbackSample | None

    @property
    def status_code(self):
        return 255 if self.failure_reason is not None or self.raw_status_code is None else self.raw_status_code


@dataclass(frozen=True)
class CommandReceipt:
    sent: tuple[str, ...]
    failed: str | None = None
    not_attempted: tuple[str, ...] = ()
    detail: str | None = None

    @property
    def success(self):
        return self.failed is None

@dataclass(frozen=True)
class PositionCommand:
    positions: tuple[float, ...]
    velocity_limits: tuple[float, ...]
    generation: int

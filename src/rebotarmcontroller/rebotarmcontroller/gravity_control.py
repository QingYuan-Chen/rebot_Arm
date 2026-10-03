"""Offline gravity model and bounded MIT commands; never opens hardware."""
from __future__ import annotations

from dataclasses import dataclass, fields
import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml

JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 7))
POSITION_LIMITS = ((-2.8, 2.8), (-3.14, .02), (-3.14, .02),
                   (-1.87, 1.57), (-1.57, 1.57), (-3.14, 3.14))


def vector(value, size, label):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f"{label} must contain {size} finite values")
    return result


@dataclass(frozen=True)
class GravitySettings:
    # Existing stiffness/damping, now with time-based integral in Nm/(rad*s).
    kp: float = 7.
    kd: float = .8
    integral_gain: float = 1.
    integral_limit_nm: float = .5
    torque_rate_limits_nm_s: tuple = (108., 108., 108., 28., 28., 28.)
    gravity_xyz_m_s2: tuple = (0., 0., -9.81)
    payload_mass_kg: float = 0.
    payload_com_xyz_m: tuple = (0., 0., 0.)

    def __post_init__(self):
        scalars = (self.kp, self.kd, self.integral_gain, self.integral_limit_nm,
                   self.payload_mass_kg)
        if not np.isfinite(scalars).all() or min(scalars) < 0 or self.kp <= 0:
            raise ValueError("gravity settings must be finite and nonnegative; kp > 0")
        if self.kp > 500 or self.kd > 5:
            raise ValueError('MIT gains exceed protocol range: kp <= 500, kd <= 5')
        if (vector(self.torque_rate_limits_nm_s, 6, 'torque rate') <= 0).any():
            raise ValueError('torque rate limits must be positive')
        gravity = vector(self.gravity_xyz_m_s2, 3, 'gravity')
        if not 9. < np.linalg.norm(gravity) < 10.:
            raise ValueError('gravity vector must describe Earth gravity in base_link')
        vector(self.payload_com_xyz_m, 3, 'payload COM')

    @classmethod
    def from_yaml(cls, path):
        values = yaml.safe_load(Path(path).read_text())
        if not isinstance(values, dict):
            raise ValueError('gravity settings must be a mapping')
        unknown = set(values) - {f.name for f in fields(cls)}
        if unknown:
            raise ValueError(f'unknown gravity settings: {sorted(unknown)}')
        return cls(**values)


def bounded_mit_command(target, q, qd, kp, kd, feedforward, limits, rates,
                        previous, dt):
    """Bound estimated TOTAL torque and feedforward, including motor-side PD.

    Slew uses measured dt. Under saturation PD is reduced only as necessary to
    keep the compensating feedforward representable. Actual torque between fresh
    feedback samples still requires hardware validation.
    """
    target, q, qd, kp, kd, feedforward, limits, rates, previous = [
        vector(v, 6, label) for v, label in zip(
            (target, q, qd, kp, kd, feedforward, limits, rates, previous),
            ('target', 'position', 'velocity', 'kp', 'kd', 'feedforward',
             'torque limits', 'torque rates', 'previous torque'))]
    if not np.isfinite(dt) or dt <= 0 or (limits <= 0).any() or (rates <= 0).any():
        raise ValueError('dt, torque limits and rates must be finite and positive')
    if (kp < 0).any() or (kd < 0).any() or (np.abs(previous) > limits + 1e-9).any():
        raise ValueError('invalid gains or previous torque outside limit')
    pd = vector(kp * (target - q) - kd * qd, 6, 'PD torque')
    desired = np.clip(pd + feedforward, -limits, limits)
    total = np.clip(desired, previous - rates * dt, previous + rates * dt)
    # Reserve protocol headroom for PD cancellation; no oversized tau_ff.
    scale = np.minimum(1., np.maximum(0., limits - np.abs(total)) /
                       np.maximum(np.abs(pd), 1e-12))
    return kp * scale, kd * scale, total - pd * scale, total


class LoopTiming:
    def __init__(self, rate_hz):
        if not np.isfinite(rate_hz) or rate_hz <= 0:
            raise ValueError('control rate must be finite and positive')
        self.period = 1. / rate_hz
        self.last_start = None
        self.start = None
        self.consecutive_misses = 0
        self.cycles = self.misses = 0
        self.max_period = self.max_execution = 0.
        self.last_dt = self.period

    def begin(self, now):
        dt = self.period if self.last_start is None else now - self.last_start
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError('control clock must advance monotonically')
        self.last_start = self.start = now
        self.last_dt = dt
        self.max_period = max(self.max_period, dt)
        return dt

    def finish(self, now):
        execution = now - self.start
        missed = execution > self.period or self.last_dt > 1.5 * self.period
        self.cycles += 1
        self.misses += int(missed)
        self.consecutive_misses = self.consecutive_misses + 1 if missed else 0
        self.max_execution = max(self.max_execution, execution)

    @property
    def faulted(self):
        return self.consecutive_misses >= 3

    def snapshot(self):
        return dict(configured_rate_hz=1. / self.period, cycles=self.cycles,
                    missed_cycles=self.misses, consecutive_misses=self.consecutive_misses,
                    max_period_sec=self.max_period, max_execution_sec=self.max_execution)


class GravityModel:
    def __init__(self, pin, model, path, settings, effort):
        self.pin, self.model = pin, model
        self.data = model.createData()
        self.frame_id = model.getFrameId('end_link')
        self.effort = effort
        self.path = str(path.resolve())
        self.sha256 = hashlib.sha256(path.read_bytes()).hexdigest()
        self.settings = settings

    def gravity(self, q):
        q = vector(q, 6, 'position')
        return self.pin.computeGeneralizedGravity(self.model, self.data, q).copy()

    def jacobian(self, q):
        self.pin.computeJointJacobians(self.model, self.data, q)
        self.pin.updateFramePlacements(self.model, self.data)
        return self.pin.getFrameJacobian(self.model, self.data, self.frame_id,
                                        self.pin.LOCAL_WORLD_ALIGNED).copy()


def load_gravity_model(path, settings):
    import pinocchio as pin
    path = Path(path)
    root = ET.parse(path).getroot()
    joints = {j.get('name'): j for j in root.findall('joint')}
    effort = []
    for name, bounds in zip(JOINT_NAMES, POSITION_LIMITS):
        joint = joints.get(name)
        if joint is None or joint.get('type') != 'revolute':
            raise ValueError(f'{name} must be a bounded revolute joint')
        limit = joint.find('limit')
        actual = (float(limit.get('lower')), float(limit.get('upper')))
        if not np.allclose(actual, bounds, atol=1e-9, rtol=0):
            raise ValueError(f'{name} model limit {actual} differs from hardware {bounds}')
        effort.append(float(limit.get('effort')))
    effort = vector(effort, 6, 'effort')
    if (effort <= 0).any():
        raise ValueError('model effort must be positive')
    if (effort > np.array([27., 27., 27., 7., 7., 7.])).any():
        raise ValueError('model effort exceeds the reviewed arm torque ceilings')
    full = pin.buildModelFromUrdf(str(path))
    # Only gripper joints may be locked. No silent loss of an arm coordinate.
    extras = list(full.names)[1:]
    lock_names = [n for n in extras if n not in JOINT_NAMES]
    if set(lock_names) - {'left_finger_joint', 'right_finger_joint'}:
        raise ValueError(f'unexpected model joints: {lock_names}')
    model = pin.buildReducedModel(full, [full.getJointId(n) for n in lock_names],
                                  pin.neutral(full)) if lock_names else full
    if model.nq != 6 or model.nv != 6 or tuple(model.names)[1:] != JOINT_NAMES:
        raise ValueError('model must preserve joint1..joint6 order and coordinates')
    if not model.existFrame('end_link'):
        raise ValueError('model is missing end_link')
    model.gravity.linear = vector(settings.gravity_xyz_m_s2, 3, 'gravity')
    if settings.payload_mass_kg:
        frame = model.frames[model.getFrameId('end_link')]
        payload = pin.Inertia(settings.payload_mass_kg,
                              vector(settings.payload_com_xyz_m, 3, 'payload COM'),
                              np.zeros((3, 3)))
        model.inertias[frame.parentJoint] += frame.placement.act(payload)
    return GravityModel(pin, model, path, settings, effort)


def package_file(package, relative):
    """Use the installed resource when sourced, otherwise the source checkout."""
    try:
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory(package)) / relative
        if path.is_file():
            return path
    except (ImportError, LookupError):
        pass
    path = Path(__file__).resolve().parents[2] / package / relative
    if not path.is_file():
        raise FileNotFoundError(f'missing {package}/{relative}')
    return path

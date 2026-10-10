"""Jacobian-based Cartesian commands for the reBotArm MuJoCo simulation.

The solver owns a scratch ``MjData`` instance.  It never mutates the live
simulation state or actuator controls; only :meth:`command_delta` hands a
successful six-joint solution to ``RebotArmMujoco.command_joint_positions``.
"""

from __future__ import annotations

from dataclasses import dataclass
import importlib
import math
from typing import Literal, Protocol, Sequence

import numpy as np


CartesianFrame = Literal["world", "tool"]
IkStatus = Literal["converged", "max_iterations", "unreachable", "numerical_failure"]
_ARM_JOINT_NAMES = tuple(f"joint{index}" for index in range(1, 7))


def _vector3(values: Sequence[float], label: str) -> tuple[float, float, float]:
    if isinstance(values, (str, bytes)):
        raise TypeError(f"{label} must be a numeric sequence")
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError) as exc:
        raise TypeError(f"{label} must be a numeric sequence") from exc
    if len(result) != 3:
        raise ValueError(f"{label} must contain exactly 3 values")
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{label} values must be finite")
    return result  # type: ignore[return-value]


def _unit_quaternion_wxyz(values: Sequence[float]) -> tuple[float, float, float, float]:
    if isinstance(values, (str, bytes)):
        raise TypeError("quaternion_wxyz must be a numeric sequence")
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError) as exc:
        raise TypeError("quaternion_wxyz must be a numeric sequence") from exc
    if len(result) != 4:
        raise ValueError("quaternion_wxyz must contain exactly 4 values")
    if not all(math.isfinite(value) for value in result):
        raise ValueError("quaternion_wxyz values must be finite")
    norm = float(np.linalg.norm(result))
    if norm <= 1e-12:
        raise ValueError("quaternion_wxyz must be non-zero")
    if not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1e-6):
        raise ValueError("quaternion_wxyz must have unit length")
    return tuple(value / norm for value in result)  # type: ignore[return-value]


@dataclass(frozen=True)
class CartesianDelta:
    xyz_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rpy_rad: tuple[float, float, float] = (0.0, 0.0, 0.0)
    frame: CartesianFrame = "world"

    def __post_init__(self) -> None:
        object.__setattr__(self, "xyz_m", _vector3(self.xyz_m, "xyz_m"))
        object.__setattr__(self, "rpy_rad", _vector3(self.rpy_rad, "rpy_rad"))
        if self.frame not in ("world", "tool"):
            raise ValueError("frame must be 'world' or 'tool'")


@dataclass(frozen=True)
class IkOptions:
    damping: float = 0.03
    max_iterations: int = 160
    position_tolerance_m: float = 5e-4
    orientation_tolerance_rad: float = 2e-3
    max_joint_step_rad: float = 0.04
    max_position_step_m: float = 0.01
    max_orientation_step_rad: float = 0.04
    joint_limit_margin_rad: float = 1e-5
    stagnation_iterations: int = 15
    minimum_progress: float = 1e-7

    def __post_init__(self) -> None:
        positive = (
            "damping",
            "position_tolerance_m",
            "orientation_tolerance_rad",
            "max_joint_step_rad",
            "max_position_step_m",
            "max_orientation_step_rad",
            "minimum_progress",
        )
        for name in positive:
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
            object.__setattr__(self, name, value)
        margin = float(self.joint_limit_margin_rad)
        if not math.isfinite(margin) or margin < 0.0:
            raise ValueError("joint_limit_margin_rad must be finite and non-negative")
        object.__setattr__(self, "joint_limit_margin_rad", margin)
        for name in ("max_iterations", "stagnation_iterations"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError(f"{name} must be a positive integer")


@dataclass(frozen=True)
class IkResult:
    success: bool
    status: IkStatus
    message: str
    joint_positions: tuple[float, float, float, float, float, float]
    iterations: int
    position_error_m: float
    orientation_error_rad: float
    target_position_m: tuple[float, float, float]
    reached_position_m: tuple[float, float, float]
    hit_joint_limits: tuple[bool, bool, bool, bool, bool, bool]
    target_rpy_rad: tuple[float, float, float]
    reached_rpy_rad: tuple[float, float, float]


class _Simulation(Protocol):
    def _unsafe_viewer_handles(self): ...
    def command_joint_positions(self, values: Sequence[float]) -> tuple[float, ...]: ...


class MujocoCartesianController:
    """Damped-least-squares IK adapter for a ``RebotArmMujoco`` instance."""

    def __init__(self, simulation: _Simulation, *, options: IkOptions | None = None) -> None:
        self._simulation = simulation
        self.options = options or IkOptions()
        self._mj = importlib.import_module("mujoco")
        self._model, live_data = simulation._unsafe_viewer_handles()
        self._data = self._mj.MjData(self._model)
        self._site_id = self._required_id(self._mj.mjtObj.mjOBJ_SITE, "ee_site")
        self._joint_ids = tuple(
            self._required_id(self._mj.mjtObj.mjOBJ_JOINT, name)
            for name in _ARM_JOINT_NAMES
        )
        self._qpos_addresses = np.asarray(
            [int(self._model.jnt_qposadr[joint_id]) for joint_id in self._joint_ids], dtype=int
        )
        self._dof_addresses = np.asarray(
            [int(self._model.jnt_dofadr[joint_id]) for joint_id in self._joint_ids], dtype=int
        )
        margin = self.options.joint_limit_margin_rad
        ranges = np.asarray([self._model.jnt_range[joint_id] for joint_id in self._joint_ids])
        self._lower = ranges[:, 0] + margin
        self._upper = ranges[:, 1] - margin
        if np.any(self._lower > self._upper):
            raise ValueError("joint_limit_margin_rad leaves an empty joint range")
        self._copy_live_kinematics(live_data)

    def solve_delta(self, delta: CartesianDelta) -> IkResult:
        """Solve an incremental pose command without changing live simulation state."""
        if not isinstance(delta, CartesianDelta):
            raise TypeError("delta must be a CartesianDelta")
        _model, live_data = self._simulation._unsafe_viewer_handles()
        if _model is not self._model:
            raise RuntimeError("simulation model changed after Cartesian controller creation")
        self._copy_live_kinematics(live_data)
        current_position, current_rotation = self._site_pose()
        translation = np.asarray(delta.xyz_m, dtype=float)
        increment_rotation = _rotation_from_rpy(delta.rpy_rad)
        if delta.frame == "tool":
            target_position = current_position + current_rotation @ translation
            target_rotation = current_rotation @ increment_rotation
        else:
            target_position = current_position + translation
            target_rotation = increment_rotation @ current_rotation
        return self._solve_target_pose(target_position, target_rotation)

    def solve_pose(
        self,
        position_m: Sequence[float],
        quaternion_wxyz: Sequence[float],
    ) -> IkResult:
        """Solve an absolute world-frame pose without changing live state.

        ``quaternion_wxyz`` deliberately follows MuJoCo's native ordering.
        Non-unit quaternions are rejected instead of silently normalized.
        """
        target_position = np.asarray(_vector3(position_m, "position_m"), dtype=float)
        quaternion = _unit_quaternion_wxyz(quaternion_wxyz)
        target_rotation = _rotation_from_quaternion_wxyz(quaternion)
        _model, live_data = self._simulation._unsafe_viewer_handles()
        if _model is not self._model:
            raise RuntimeError("simulation model changed after Cartesian controller creation")
        self._copy_live_kinematics(live_data)
        return self._solve_target_pose(target_position, target_rotation)

    def command_delta(self, delta: CartesianDelta) -> IkResult:
        """Solve ``delta`` and submit a position target only when it converges."""
        result = self.solve_delta(delta)
        if result.success:
            self._simulation.command_joint_positions(result.joint_positions)
        return result

    def command_pose(
        self,
        position_m: Sequence[float],
        quaternion_wxyz: Sequence[float],
    ) -> IkResult:
        """Solve an absolute pose and submit it only when IK converges."""
        result = self.solve_pose(position_m, quaternion_wxyz)
        if result.success:
            self._simulation.command_joint_positions(result.joint_positions)
        return result

    def _solve_target_pose(self, target_position: np.ndarray, target_rotation: np.ndarray) -> IkResult:
        options = self.options
        hit_limits = np.zeros(6, dtype=bool)
        previous_cost = math.inf
        stagnation = 0
        position_error_norm = math.inf
        orientation_error_norm = math.inf
        status: IkStatus = "max_iterations"
        message = "IK iteration limit reached"

        for iteration in range(options.max_iterations + 1):
            position, rotation = self._site_pose()
            position_error = target_position - position
            orientation_error = _rotation_log(target_rotation @ rotation.T)
            position_error_norm = float(np.linalg.norm(position_error))
            orientation_error_norm = float(np.linalg.norm(orientation_error))
            if (
                position_error_norm <= options.position_tolerance_m
                and orientation_error_norm <= options.orientation_tolerance_rad
            ):
                return self._result(
                    True, "converged", "IK converged", iteration,
                    position_error_norm, orientation_error_norm,
                    target_position, position, target_rotation, rotation, hit_limits,
                )
            if iteration == options.max_iterations:
                break

            task_error = np.concatenate((
                _limited_norm(position_error, options.max_position_step_m),
                _limited_norm(orientation_error, options.max_orientation_step_rad),
            ))
            jacobian_position = np.zeros((3, int(self._model.nv)), dtype=float)
            jacobian_rotation = np.zeros((3, int(self._model.nv)), dtype=float)
            self._mj.mj_jacSite(
                self._model, self._data, jacobian_position, jacobian_rotation, self._site_id
            )
            jacobian = np.vstack((
                jacobian_position[:, self._dof_addresses],
                jacobian_rotation[:, self._dof_addresses],
            ))
            try:
                regularized = jacobian @ jacobian.T + (options.damping ** 2) * np.eye(6)
                joint_delta = jacobian.T @ np.linalg.solve(regularized, task_error)
            except np.linalg.LinAlgError:
                status = "numerical_failure"
                message = "IK linear solve failed"
                break
            if not np.all(np.isfinite(joint_delta)):
                status = "numerical_failure"
                message = "IK produced non-finite joint updates"
                break
            joint_delta = _limited_norm(joint_delta, options.max_joint_step_rad)
            old_joints = self._data.qpos[self._qpos_addresses].copy()
            unclipped = old_joints + joint_delta
            new_joints = np.clip(unclipped, self._lower, self._upper)
            hit_limits |= ~np.isclose(unclipped, new_joints, atol=1e-12, rtol=0.0)
            self._data.qpos[self._qpos_addresses] = new_joints
            self._mj.mj_forward(self._model, self._data)

            cost = position_error_norm + orientation_error_norm
            progress = previous_cost - cost
            if progress < options.minimum_progress or np.allclose(old_joints, new_joints, atol=1e-12):
                stagnation += 1
            else:
                stagnation = 0
            previous_cost = cost
            if stagnation >= options.stagnation_iterations:
                status = "unreachable"
                message = "IK stagnated; target is unreachable from the current pose and limits"
                break

        reached_position, reached_rotation = self._site_pose()
        if status == "max_iterations" and np.any(hit_limits):
            status = "unreachable"
            message = "IK reached joint limits before reaching the target"
        elif status == "max_iterations" and (
            position_error_norm > options.max_position_step_m
            or orientation_error_norm > options.max_orientation_step_rad
        ):
            status = "unreachable"
            message = "IK could not reach the requested Cartesian target"
        return self._result(
            False, status, message, min(iteration, options.max_iterations),
            position_error_norm, orientation_error_norm,
            target_position, reached_position, target_rotation, reached_rotation, hit_limits,
        )

    def _result(
        self,
        success: bool,
        status: IkStatus,
        message: str,
        iterations: int,
        position_error: float,
        orientation_error: float,
        target_position: np.ndarray,
        reached_position: np.ndarray,
        target_rotation: np.ndarray,
        reached_rotation: np.ndarray,
        hit_limits: np.ndarray,
    ) -> IkResult:
        return IkResult(
            success=success,
            status=status,
            message=message,
            joint_positions=tuple(float(v) for v in self._data.qpos[self._qpos_addresses]),  # type: ignore[arg-type]
            iterations=int(iterations),
            position_error_m=float(position_error),
            orientation_error_rad=float(orientation_error),
            target_position_m=tuple(float(v) for v in target_position),  # type: ignore[arg-type]
            reached_position_m=tuple(float(v) for v in reached_position),  # type: ignore[arg-type]
            hit_joint_limits=tuple(bool(v) for v in hit_limits),  # type: ignore[arg-type]
            target_rpy_rad=_rpy_from_rotation(target_rotation),
            reached_rpy_rad=_rpy_from_rotation(reached_rotation),
        )

    def _copy_live_kinematics(self, live_data) -> None:
        self._data.qpos[:] = live_data.qpos
        self._data.qvel[:] = 0.0
        if int(self._model.nmocap):
            self._data.mocap_pos[:] = live_data.mocap_pos
            self._data.mocap_quat[:] = live_data.mocap_quat
        self._mj.mj_forward(self._model, self._data)

    def _site_pose(self) -> tuple[np.ndarray, np.ndarray]:
        return (
            np.asarray(self._data.site_xpos[self._site_id], dtype=float).copy(),
            np.asarray(self._data.site_xmat[self._site_id], dtype=float).reshape(3, 3).copy(),
        )

    def _required_id(self, object_type, name: str) -> int:
        identifier = int(self._mj.mj_name2id(self._model, object_type, name))
        if identifier < 0:
            raise ValueError(f"MuJoCo model is missing required {name!r}")
        return identifier


def _limited_norm(vector: np.ndarray, maximum: float) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if norm <= maximum or norm == 0.0:
        return vector
    return vector * (maximum / norm)


def _rotation_from_rpy(rpy: Sequence[float]) -> np.ndarray:
    roll, pitch, yaw = (float(value) for value in rpy)
    sr, cr = math.sin(roll), math.cos(roll)
    sp, cp = math.sin(pitch), math.cos(pitch)
    sy, cy = math.sin(yaw), math.cos(yaw)
    rx = np.asarray(((1, 0, 0), (0, cr, -sr), (0, sr, cr)), dtype=float)
    ry = np.asarray(((cp, 0, sp), (0, 1, 0), (-sp, 0, cp)), dtype=float)
    rz = np.asarray(((cy, -sy, 0), (sy, cy, 0), (0, 0, 1)), dtype=float)
    return rz @ ry @ rx


def _rpy_from_rotation(rotation: np.ndarray) -> tuple[float, float, float]:
    pitch = math.asin(float(np.clip(-rotation[2, 0], -1.0, 1.0)))
    if abs(math.cos(pitch)) > 1e-7:
        roll = math.atan2(float(rotation[2, 1]), float(rotation[2, 2]))
        yaw = math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))
    else:
        roll = math.atan2(float(-rotation[1, 2]), float(rotation[1, 1]))
        yaw = 0.0
    return roll, pitch, yaw


def _rotation_from_quaternion_wxyz(quaternion: Sequence[float]) -> np.ndarray:
    w, x, y, z = (float(value) for value in quaternion)
    return np.asarray((
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
        (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
        (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
    ), dtype=float)


def _rotation_log(rotation: np.ndarray) -> np.ndarray:
    cosine = float(np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0))
    angle = math.acos(cosine)
    skew = np.asarray((
        rotation[2, 1] - rotation[1, 2],
        rotation[0, 2] - rotation[2, 0],
        rotation[1, 0] - rotation[0, 1],
    ))
    if angle < 1e-7:
        return 0.5 * skew
    sine = math.sin(angle)
    if abs(sine) < 1e-7:
        eigenvalues, eigenvectors = np.linalg.eigh((rotation + np.eye(3)) * 0.5)
        axis = eigenvectors[:, int(np.argmax(eigenvalues))]
        return angle * axis
    return (angle / (2.0 * sine)) * skew

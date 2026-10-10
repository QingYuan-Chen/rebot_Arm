"""Optional MuJoCo visual overlays, independent from the viewer main loop."""

from __future__ import annotations

import math
from types import MappingProxyType
from typing import Mapping, Sequence

from rebotarm_simulation.diagnostics.mujoco_telemetry import TelemetrySnapshot


ARM_JOINT_NAMES = tuple(f"joint{index}" for index in range(1, 7))
FIGURE_NAMES = ("tracking", "torque")


def _mujoco_module(explicit=None):
    if explicit is not None:
        return explicit
    try:
        import mujoco
    except ImportError:
        return None
    return mujoco


class GhostArmOverlay:
    """Append a transparent target-pose arm to ``viewer.user_scn``.

    Only geom group 2 is requested from MuJoCo, so collision proxies and scene
    fixtures cannot become part of the ghost.  Unsupported/fake viewers simply
    return ``False`` instead of failing the simulation loop.
    """

    def __init__(
        self,
        model,
        *,
        joint_names: Sequence[str] = ARM_JOINT_NAMES,
        rgba: Sequence[float] = (0.15, 0.75, 1.0, 0.28),
        mujoco_module=None,
    ) -> None:
        self._mj = _mujoco_module(mujoco_module)
        self._model = model
        color = tuple(float(value) for value in rgba)
        if len(color) != 4 or not all(math.isfinite(value) for value in color):
            raise ValueError("rgba must contain four finite values")
        if any(value < 0.0 or value > 1.0 for value in color):
            raise ValueError("rgba values must be from 0 to 1")
        self._rgba = color
        self._joint_names = tuple(str(name) for name in joint_names)
        if len(self._joint_names) != 6 or any(not name for name in self._joint_names):
            raise ValueError("joint_names must contain six non-empty names")
        self._data = None
        self._option = None
        self._perturb = None
        self._qpos_addresses: tuple[int, ...] = ()
        self._scene = None
        self._start = 0
        self._count = 0
        if self._mj is not None:
            self._initialize_mujoco_state()

    @property
    def available(self) -> bool:
        return self._data is not None and hasattr(self._mj, "mjv_addGeoms")

    @property
    def geom_count(self) -> int:
        return self._count

    def _initialize_mujoco_state(self) -> None:
        required = ("MjData", "MjvOption", "MjvPerturb", "mj_name2id")
        if any(not hasattr(self._mj, name) for name in required):
            return
        try:
            addresses = []
            for name in self._joint_names:
                joint_id = self._mj.mj_name2id(
                    self._model, self._mj.mjtObj.mjOBJ_JOINT, name
                )
                if joint_id < 0:
                    raise ValueError(f"MuJoCo model is missing arm joint {name}")
                addresses.append(int(self._model.jnt_qposadr[joint_id]))
            self._data = self._mj.MjData(self._model)
            self._option = self._mj.MjvOption()
            self._option.geomgroup[:] = 0
            self._option.geomgroup[2] = 1
            # Sites and joint decorators are also emitted by mjv_addGeoms;
            # disable them so the overlay contains arm visual geoms only.
            self._option.sitegroup[:] = 0
            self._option.jointgroup[:] = 0
            self._perturb = self._mj.MjvPerturb()
            self._qpos_addresses = tuple(addresses)
        except (AttributeError, TypeError):
            self._data = None
            self._option = None
            self._perturb = None
            self._qpos_addresses = ()

    def clear(self, viewer) -> bool:
        scene = getattr(viewer, "user_scn", None)
        if scene is None or scene is not self._scene or not hasattr(scene, "ngeom"):
            self._scene = None
            self._count = 0
            return False
        # Only truncate when our geoms are still the tail of the user scene.
        if int(scene.ngeom) == self._start + self._count:
            scene.ngeom = self._start
        self._scene = None
        self._count = 0
        return True

    def update(self, viewer, target_qpos: Sequence[float]) -> bool:
        if not self.available:
            return False
        scene = getattr(viewer, "user_scn", None)
        if scene is None or not hasattr(scene, "ngeom") or not hasattr(scene, "geoms"):
            return False
        try:
            values = tuple(float(value) for value in target_qpos)
        except (TypeError, ValueError):
            raise ValueError("target_qpos must contain six numeric values") from None
        if len(values) != 6 or not all(math.isfinite(value) for value in values):
            raise ValueError("target_qpos must contain six finite values")

        self.clear(viewer)
        for address, value in zip(self._qpos_addresses, values):
            self._data.qpos[address] = value
        self._mj.mj_forward(self._model, self._data)
        start = int(scene.ngeom)
        try:
            self._mj.mjv_addGeoms(
                self._model,
                self._data,
                self._option,
                self._perturb,
                self._mj.mjtCatBit.mjCAT_ALL,
                scene,
            )
        except (AttributeError, TypeError):
            return False
        end = int(scene.ngeom)
        for index in range(start, end):
            scene.geoms[index].rgba[:] = self._rgba
        self._scene = scene
        self._start = start
        self._count = end - start
        return self._count > 0


class TelemetryFigures:
    """Maintain switchable joint-tracking and torque ``MjvFigure`` objects."""

    def __init__(self, *, max_points: int = 500, mujoco_module=None) -> None:
        if isinstance(max_points, bool) or not isinstance(max_points, int) or max_points <= 1:
            raise ValueError("max_points must be an integer greater than one")
        self._mj = _mujoco_module(mujoco_module)
        self._max_points = min(max_points, 1001)
        self._selected = "tracking"
        self._figures: dict[str, object] = {}
        if self._mj is not None and hasattr(self._mj, "MjvFigure"):
            self._figures = {
                "tracking": self._new_figure(
                    "Joint tracking [rad, rad/s]",
                    ("actual [rad]", "target [rad]", "error [rad]", "velocity [rad/s]"),
                ),
                "torque": self._new_figure(
                    "Effort [N.m, N]",
                    ("requested [N.m]", "applied [N.m]", "max contact [N]"),
                ),
            }

    @property
    def available(self) -> bool:
        return len(self._figures) == 2

    @property
    def selected(self) -> str:
        return self._selected

    @property
    def active_figure(self):
        return self._figures.get(self._selected)

    @property
    def figures(self) -> Mapping[str, object]:
        return MappingProxyType(self._figures)

    def select(self, name: str) -> object | None:
        if name not in FIGURE_NAMES:
            raise ValueError(f"figure must be one of {FIGURE_NAMES}")
        self._selected = name
        return self.active_figure

    def toggle(self) -> object | None:
        index = (FIGURE_NAMES.index(self._selected) + 1) % len(FIGURE_NAMES)
        return self.select(FIGURE_NAMES[index])

    def _new_figure(self, title: str, lines: Sequence[str]):
        figure = self._mj.MjvFigure()
        figure.title = title
        figure.xlabel = "simulation time (s)"
        figure.flg_legend = 1
        figure.flg_extend = 0
        for index, name in enumerate(lines):
            figure.linename[index] = name
        colors = (
            (0.90, 0.90, 0.90),
            (0.10, 0.75, 1.00),
            (1.00, 0.35, 0.15),
            (0.95, 0.80, 0.15),
        )
        for index in range(len(lines)):
            figure.linergb[index] = colors[index]
        return figure

    @staticmethod
    def _range(values: Sequence[float]) -> tuple[float, float]:
        if not values:
            return -1.0, 1.0
        low, high = min(values), max(values)
        span = high - low
        padding = span * 0.05 if span > 0.0 else max(abs(low) * 0.05, 1e-6)
        return low - padding, high + padding

    @staticmethod
    def _write_lines(figure, times: tuple[float, ...], lines: Sequence[tuple[float, ...]]) -> None:
        for index in range(len(figure.linepnt)):
            figure.linepnt[index] = 0
        for line_index, values in enumerate(lines):
            count = min(len(times), len(values), figure.linedata.shape[1] // 2)
            figure.linepnt[line_index] = count
            if count:
                interleaved = figure.linedata[line_index]
                interleaved[: 2 * count : 2] = times[-count:]
                interleaved[1 : 2 * count : 2] = values[-count:]

    def update(self, snapshot: TelemetrySnapshot, *, joint_index: int = 0) -> bool:
        if not self.available:
            return False
        if isinstance(joint_index, bool) or not isinstance(joint_index, int) or not 0 <= joint_index < 6:
            raise ValueError("joint_index must be from 0 to 5")
        samples = snapshot.samples[-self._max_points :]
        times = tuple(sample.simulation_time for sample in samples)
        tracking = tuple(
            tuple(getattr(sample, field)[joint_index] for sample in samples)
            for field in (
                "joint_positions", "joint_targets", "joint_errors", "joint_velocities"
            )
        )
        torque = tuple(
            tuple(getattr(sample, field)[joint_index] for sample in samples)
            for field in ("requested_torques", "applied_torques")
        ) + (tuple(sample.max_contact_force_n for sample in samples),)
        self._write_lines(self._figures["tracking"], times, tracking)
        self._write_lines(self._figures["torque"], times, torque)
        x_low, x_high = self._range(times)
        for figure, lines in (
            (self._figures["tracking"], tracking),
            (self._figures["torque"], torque),
        ):
            y_low, y_high = self._range(tuple(value for line in lines for value in line))
            figure.range[0, :] = (x_low, x_high)
            figure.range[1, :] = (y_low, y_high)
        return True

    @staticmethod
    def _viewport_size(viewport) -> tuple[int, int]:
        if viewport is None:
            raise ValueError("viewer viewport is unavailable")
        if hasattr(viewport, "width") and hasattr(viewport, "height"):
            width, height = viewport.width, viewport.height
        else:
            try:
                width, height = viewport
            except (TypeError, ValueError):
                raise ValueError("viewport must expose width/height or be a pair") from None
        if isinstance(width, bool) or isinstance(height, bool):
            raise ValueError("viewport width and height must be positive integers")
        width, height = int(width), int(height)
        if width <= 0 or height <= 0:
            raise ValueError("viewport width and height must be positive integers")
        return width, height

    def attach_active(self, viewer, *, viewport=None) -> bool:
        """Attach the selected figure through MuJoCo's official Handle API."""
        figure = self.active_figure
        set_figures = getattr(viewer, "set_figures", None)
        if (
            figure is None
            or not callable(set_figures)
            or self._mj is None
            or not hasattr(self._mj, "MjrRect")
        ):
            return False
        try:
            width, height = self._viewport_size(
                viewport if viewport is not None else getattr(viewer, "viewport", None)
            )
        except ValueError:
            return False
        margin = min(10, width - 1, height - 1)
        figure_width = max(1, min(480, round(width * 0.40)))
        figure_height = max(1, min(320, round(height * 0.35)))
        rectangle = self._mj.MjrRect(
            max(0, width - figure_width - margin),
            margin,
            min(figure_width, width),
            min(figure_height, height),
        )
        set_figures([(rectangle, figure)])
        return True

    def clear(self, viewer) -> bool:
        clear_figures = getattr(viewer, "clear_figures", None)
        if not callable(clear_figures):
            return False
        clear_figures()
        return True

from __future__ import annotations

from pathlib import Path

import pytest

from rebotarm_simulation.diagnostics.mujoco_telemetry import MujocoTelemetryHistory
from rebotarm_simulation.core.mujoco_types import ControlStatus
from rebotarm_simulation.apps.mujoco_visualization import GhostArmOverlay, TelemetryFigures


ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "src/rebotarm_simulation/models/rebotarm/scene.xml"


def _status(offset: float) -> ControlStatus:
    return ControlStatus(
        mode="position",
        joint_targets=(offset + 0.2,) * 6,
        joint_positions=(offset,) * 6,
        joint_velocities=(0.1,) * 6,
        requested_torques=(2.0 + offset,) * 6,
        applied_torques=(1.5 + offset,) * 6,
        saturated=(False,) * 6,
        watchdog_remaining_s=None,
        gripper_target_width_m=0.04,
        gripper_width_m=0.039,
        gripper_control_force_n=(1.0, -1.0),
    )


def _snapshot(count: int = 4):
    history = MujocoTelemetryHistory()
    for index in range(count):
        history.append(index * 0.1, _status(float(index)))
    return history.snapshot()


class EmptyViewer:
    pass


class FigureViewer:
    def __init__(self):
        self.viewport = None
        self.figures = None
        self.cleared = False

    def set_figures(self, figures):
        self.figures = figures

    def clear_figures(self):
        self.cleared = True


def test_ghost_overlay_adds_only_transparent_visual_group_geoms() -> None:
    mujoco = pytest.importorskip("mujoco")
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    viewer = EmptyViewer()
    viewer.user_scn = mujoco.MjvScene(model, maxgeom=100)
    overlay = GhostArmOverlay(model)

    assert overlay.update(viewer, (0.0, -0.8, -1.0, 0.3, 0.0, 0.0))
    assert overlay.geom_count == 12  # ten arm visuals plus camera and bracket
    assert viewer.user_scn.ngeom == 12
    for index in range(viewer.user_scn.ngeom):
        geom = viewer.user_scn.geoms[index]
        assert int(model.geom_group[geom.objid]) == 2
        assert float(geom.rgba[3]) == pytest.approx(0.28)

    assert overlay.clear(viewer)
    assert viewer.user_scn.ngeom == 0


def test_ghost_overlay_replaces_its_previous_geoms_and_validates_target() -> None:
    mujoco = pytest.importorskip("mujoco")
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    viewer = EmptyViewer()
    viewer.user_scn = mujoco.MjvScene(model, maxgeom=100)
    overlay = GhostArmOverlay(model)
    overlay.update(viewer, (0.0,) * 6)
    first_count = viewer.user_scn.ngeom
    overlay.update(viewer, (0.1,) * 6)
    assert viewer.user_scn.ngeom == first_count

    with pytest.raises(ValueError, match="six finite"):
        overlay.update(viewer, (0.0,) * 5)


def test_ghost_overlay_degrades_for_viewer_without_user_scene() -> None:
    mujoco = pytest.importorskip("mujoco")
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    assert GhostArmOverlay(model).update(EmptyViewer(), (0.0,) * 6) is False


def test_figures_fill_bounded_interleaved_lines_and_ranges() -> None:
    pytest.importorskip("mujoco")
    figures = TelemetryFigures(max_points=3)
    assert figures.update(_snapshot(5), joint_index=2)

    tracking = figures.figures["tracking"]
    torque = figures.figures["torque"]
    assert tuple(tracking.linepnt[:4]) == (3, 3, 3, 3)
    assert tuple(torque.linepnt[:3]) == (3, 3, 3)
    assert tuple(tracking.linedata[0, :6:2]) == pytest.approx((0.2, 0.3, 0.4))
    assert tuple(tracking.linedata[0, 1:6:2]) == pytest.approx((2.0, 3.0, 4.0))
    assert float(tracking.range[0, 0]) < float(tracking.range[0, 1])
    assert float(tracking.range[1, 0]) < float(tracking.range[1, 1])
    assert "rad/s" in tracking.title
    assert "N.m" in torque.title
    assert bytes(tracking.linename[0]).decode() == "actual [rad]"
    assert bytes(torque.linename[2]).decode() == "max contact [N]"


def test_figures_switch_and_best_effort_attach() -> None:
    pytest.importorskip("mujoco")
    figures = TelemetryFigures()
    assert figures.selected == "tracking"
    tracking = figures.active_figure
    assert figures.toggle() is figures.figures["torque"]
    assert figures.selected == "torque"
    assert figures.select("tracking") is tracking

    viewer = FigureViewer()
    assert figures.attach_active(viewer, viewport=(800, 600))
    assert len(viewer.figures) == 1
    rectangle, attached = viewer.figures[0]
    assert attached is tracking
    assert (rectangle.left, rectangle.bottom, rectangle.width, rectangle.height) == (
        470, 10, 320, 210,
    )
    assert figures.clear(viewer)
    assert viewer.cleared is True
    assert figures.attach_active(EmptyViewer()) is False
    assert figures.clear(EmptyViewer()) is False
    with pytest.raises(ValueError, match="figure"):
        figures.select("invalid")


def test_figures_degrade_when_mujoco_figure_api_is_unavailable() -> None:
    figures = TelemetryFigures(mujoco_module=object())
    assert figures.available is False
    assert figures.update(_snapshot()) is False
    assert figures.attach_active(EmptyViewer()) is False


def test_figures_use_viewer_viewport_and_degrade_if_it_is_missing() -> None:
    pytest.importorskip("mujoco")
    figures = TelemetryFigures()
    viewer = FigureViewer()
    assert figures.attach_active(viewer) is False

    from types import SimpleNamespace

    viewer.viewport = SimpleNamespace(width=1000, height=700)
    assert figures.attach_active(viewer)
    rectangle, _figure = viewer.figures[0]
    assert (rectangle.left, rectangle.bottom, rectangle.width, rectangle.height) == (
        590, 10, 400, 245,
    )

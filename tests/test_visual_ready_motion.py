from pathlib import Path
from types import SimpleNamespace

import pytest

from rebotarm_motion.visual_ready_node import VisualReadyNode


def test_visual_ready_entrypoint_is_owned_by_motion_package():
    assert VisualReadyNode.__module__ == "rebotarm_motion.visual_ready_node"


def test_ready_motion_preserves_endpoints_duration_and_smoothstep():
    node = SimpleNamespace(get_parameter=lambda name: SimpleNamespace(value=4.0))
    trajectory = VisualReadyNode._build_trajectory(node, [0.] * 6, [.5] * 6)
    assert len(trajectory.points) == 81
    assert list(trajectory.points[0].positions) == [0.] * 6
    assert list(trajectory.points[-1].positions) == [.5] * 6
    assert trajectory.points[40].positions[0] == pytest.approx(.25)
    assert trajectory.points[-1].time_from_start.sec == 4
    positions = [p.positions[0] for p in trajectory.points]
    assert positions == sorted(positions)


def test_visual_ready_launches_use_motion_package():
    root = Path(__file__).resolve().parents[1]
    source = (root / "src/rebotarm_bringup/launch" / "includes" / "visual_lifecycle.launch.py").read_text()
    for prefix in source.split('executable="rebotarm_visual_ready"')[:-1]:
        assert 'package="rebotarm_motion"' in prefix[-90:]
    assert '[FindPackageShare("rebotarm_motion"), "config", "visual_ready.yaml"]' in source

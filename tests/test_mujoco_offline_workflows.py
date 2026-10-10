from pathlib import Path
from types import SimpleNamespace

import pytest

from rebotarm_simulation.execution.offline_trajectory import normalized_path


JOINTS = tuple(f"joint{i}" for i in range(1, 7))


def test_offline_path_reorders_names_and_rejects_bad_timing():
    reversed_names = JOINTS[::-1]
    path = normalized_path([(0.1, (6, 5, 4, 3, 2, 1))], reversed_names)
    assert path[0][1] == (1, 2, 3, 4, 5, 6)
    with pytest.raises(ValueError, match="increasing"):
        normalized_path([(0.1, (0,) * 6), (0.1, (0,) * 6)], JOINTS)
    with pytest.raises(ValueError, match="positive duration"):
        normalized_path([(0.0, (0,) * 6)], JOINTS)


def _sample(index, position):
    from rebotarm_teach.teach_recording import TeachSample
    return TeachSample(index * 0.05, JOINTS, position, (), (), {}, "IDLE")


def test_teach_preview_uses_prepared_path_without_touching_record(tmp_path: Path):
    pytest.importorskip("mujoco")
    from rebotarm_simulation.apps.teach_preview import preview_record
    from rebotarm_teach.teach_recording import encode_teach_sample

    path = tmp_path / "teach.jsonl"
    path.write_text("\n".join(encode_teach_sample(_sample(i, (0, -.1-i*.0005, -.2, .2, 0, 0))) for i in range(20)) + "\n")
    original = path.read_bytes()
    result = preview_record(path)
    assert result["raw_samples"] == 20
    assert result["prepared_points"] > 20
    assert result["simulation_only"] is True
    assert result["initial_bottle_xyz_m"] == pytest.approx((0.28, 0.0, 0.0))
    assert len(result["final_bottle_xyz_m"]) == 3
    assert result["trajectory"]["steps"] > 0
    assert result["trajectory"]["max_tracking_error_rad"] < 0.1
    assert path.read_bytes() == original


def test_teach_preview_does_not_reach_into_private_viewer_state():
    source = (
        Path(__file__).resolve().parents[1]
        / "src/rebotarm_simulation/rebotarm_simulation/apps/teach_preview.py"
    ).read_text(encoding="utf-8")
    assert "_RETAINED_UNSAFE_VIEWERS" not in source
    assert "close_passive_viewer_safely" in source


def test_public_viewer_close_waits_for_native_release():
    pytest.importorskip("mujoco")
    from rebotarm_simulation.apps.viewer_lifecycle import close_passive_viewer_safely

    events = []
    viewer = SimpleNamespace(m=object())
    viewer.close = lambda: (events.append("viewer.close"), setattr(viewer, "m", None))
    sim = SimpleNamespace(close=lambda: events.append("sim.close"))
    close_passive_viewer_safely(viewer, sim, object(), object())
    assert events == ["viewer.close", "sim.close"]


def test_public_viewer_close_timeout_preserves_simulation():
    pytest.importorskip("mujoco")
    from rebotarm_simulation.apps.viewer_lifecycle import close_passive_viewer_safely

    events = []
    viewer = SimpleNamespace(m=object(), close=lambda: events.append("viewer.close"))
    sim = SimpleNamespace(close=lambda: events.append("sim.close"))
    with pytest.raises(TimeoutError):
        close_passive_viewer_safely(
            viewer, sim, object(), object(), clock=lambda: 1.0, timeout=0.0
        )
    assert events == ["viewer.close"]


def test_teach_preview_rejects_structural_record_error(tmp_path: Path):
    from rebotarm_simulation.apps.teach_preview import preview_record
    from rebotarm_teach.teach_recording import encode_teach_sample

    path = tmp_path / "bad.jsonl"
    positions = (0, -.1, -.2, .2, 0, 0)
    path.write_text("\n".join(encode_teach_sample(_sample(i, positions)) for i in (0, 1, 1)) + "\n")
    with pytest.raises(ValueError, match="timestamps"):
        preview_record(path)

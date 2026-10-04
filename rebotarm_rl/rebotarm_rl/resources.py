"""Locate the shared canonical model without importing ROS or simulation code."""
import os
from pathlib import Path


def reach_scene_path() -> Path:
    configured = os.environ.get("REBOTARM_MJLAB_SCENE")
    if configured:
        path = Path(configured).expanduser().resolve()
    else:
        # Same-repository editable checkout only; wheels require an explicit asset path.
        repo = Path(__file__).resolve().parents[2]
        path = repo / "src/rebotarm_simulation/models/rebotarm/reach_scene.xml"
    if not path.is_file():
        raise FileNotFoundError(
            "Set REBOTARM_MJLAB_SCENE to the versioned reach_scene.xml; "
            "keep robot.xml and referenced meshes alongside it"
        )
    return path

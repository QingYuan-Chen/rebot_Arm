from __future__ import annotations

from pathlib import Path
import sys

ROS2_ROOT = Path(__file__).resolve().parents[1]
VISION_PATH = str(ROS2_ROOT / "src" / "rebotarm_vision")
if VISION_PATH not in sys.path:
    sys.path.insert(0, VISION_PATH)


def test_quaternion_transform_point_applies_rotation_and_translation():
    from rebotarm_vision.utils.transform_points import Transform3D, transform_point

    transform = Transform3D(
        translation=(1.0, 2.0, 3.0),
        rotation_xyzw=(0.0, 0.0, 0.7071067811865476, 0.7071067811865476),
    )

    point = transform_point(transform, (1.0, 0.0, 0.0))

    assert point == (1.0, 3.0, 3.0)


def test_transform_pose_components_rotates_orientation():
    from rebotarm_vision.utils.transform_points import Transform3D, transform_pose_components

    transform = Transform3D(
        translation=(1.0, 2.0, 3.0),
        rotation_xyzw=(0.0, 0.0, 0.7071067811865476, 0.7071067811865476),
    )

    position, orientation = transform_pose_components(
        transform,
        (1.0, 0.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )

    assert position == (1.0, 3.0, 3.0)
    assert orientation == (
        0.0,
        0.0,
        0.7071067811865476,
        0.7071067811865476,
    )

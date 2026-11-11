from __future__ import annotations

from pathlib import Path
import sys
from types import SimpleNamespace

import pytest


ROS2_ROOT = Path(__file__).resolve().parents[1]
VISION_SRC = ROS2_ROOT / "src" / "rebotarm_vision"
if str(VISION_SRC) not in sys.path:
    sys.path.insert(0, str(VISION_SRC))


def _pose(x: float, y: float, z: float):
    from geometry_msgs.msg import Pose

    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.w = 1.0
    return pose


def test_candidate_tf_adapter_returns_same_pose_when_frames_match():
    from rebotarm_vision.candidate_tf_adapter import transform_candidate_pose_to_target_frame

    called = False

    def lookup(_target_frame: str, _source_frame: str, _stamp):
        nonlocal called
        called = True
        raise AssertionError("lookup should not be called when frames already match")

    transformed = transform_candidate_pose_to_target_frame(
        _pose(0.1, 0.2, 0.3),
        source_frame="base_link",
        target_frame="base_link",
        stamp=object(),
        lookup_transform=lookup,
    )

    assert not called
    assert transformed.position.x == pytest.approx(0.1)
    assert transformed.position.y == pytest.approx(0.2)
    assert transformed.position.z == pytest.approx(0.3)


def test_candidate_tf_adapter_uses_lookup_when_frames_differ():
    from geometry_msgs.msg import TransformStamped

    from rebotarm_vision.candidate_tf_adapter import transform_candidate_pose_to_target_frame

    stamp = object()

    def lookup(target_frame: str, source_frame: str, lookup_stamp):
        assert target_frame == "base_link"
        assert source_frame == "camera_depth_frame"
        assert lookup_stamp is stamp
        transform = TransformStamped()
        transform.transform.translation.x = 0.3
        transform.transform.translation.y = -0.1
        transform.transform.translation.z = 0.2
        transform.transform.rotation.w = 1.0
        return transform

    transformed = transform_candidate_pose_to_target_frame(
        _pose(0.1, 0.2, 0.3),
        source_frame="camera_depth_frame",
        target_frame="base_link",
        stamp=stamp,
        lookup_transform=lookup,
    )

    assert transformed.position.x == pytest.approx(0.4)
    assert transformed.position.y == pytest.approx(0.1)
    assert transformed.position.z == pytest.approx(0.5)


def test_candidate_tf_query_uses_frame_stamp_and_reuses_same_frame_transform():
    from builtin_interfaces.msg import Time as RosTime
    from rebotarm_vision.candidate_ik_gateway import CandidateIkGateway

    calls = []
    stamp = RosTime(sec=12, nanosec=345)
    buffer = SimpleNamespace(
        lookup_transform=lambda target, source, query_time, timeout: calls.append(
            (target, source, query_time.nanoseconds)
        ) or object()
    )
    gateway = CandidateIkGateway(
        config=SimpleNamespace(service_timeout_sec=0.1), ik_client=None,
        validity_client=None, tf_buffer=buffer, joint_state=None,
        logger=SimpleNamespace(warn=lambda *_args: None),
        publish_ranked=lambda *_args: None, publish_empty=lambda *_args: None,
    )
    gateway.begin_frame()
    first = gateway.lookup_transform("base_link", "camera", stamp)
    second = gateway.lookup_transform("base_link", "camera", stamp)
    assert first is second
    assert calls == [("base_link", "camera", 12_000_000_345)]


@pytest.mark.parametrize("stamp", [None, SimpleNamespace(sec=0, nanosec=0)])
def test_candidate_tf_rejects_missing_sensor_stamp_before_lookup(stamp):
    from rebotarm_vision.candidate_ik_gateway import CandidateIkGateway
    from tf2_ros import TransformException

    calls = []
    gateway = CandidateIkGateway(
        config=SimpleNamespace(service_timeout_sec=0.1), ik_client=None,
        validity_client=None,
        tf_buffer=SimpleNamespace(lookup_transform=lambda *args, **kwargs: calls.append(args)),
        joint_state=None, logger=SimpleNamespace(warn=lambda *_args: None),
        publish_ranked=lambda *_args: None, publish_empty=lambda *_args: None,
    )
    with pytest.raises(TransformException, match="sensor timestamp"):
        gateway.lookup_transform("base_link", "camera", stamp)
    assert calls == []

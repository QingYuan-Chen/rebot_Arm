from types import SimpleNamespace

import pytest
from sensor_msgs.msg import Image

from rebotarm_vision.converters.image_msgs import camera_info_to_msg, depth_image_to_array


def test_camera_info_to_msg_reuses_intrinsic_matrix_fallbacks():
    message = camera_info_to_msg(
        {"k": [600.0, 0.0, 320.0, 0.0, 601.0, 240.0]},
        stamp=SimpleNamespace(sec=1, nanosec=2),
        frame_id="camera_color_frame",
    )

    assert list(message.k) == [600.0, 0.0, 320.0, 0.0, 601.0, 240.0, 0.0, 0.0, 1.0]
    assert list(message.p) == [600.0, 0.0, 320.0, 0.0, 0.0, 601.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]


def test_camera_info_to_msg_handles_missing_or_short_intrinsic_matrix():
    message = camera_info_to_msg({"k": [640.0]}, stamp=None, frame_id="camera")

    assert list(message.k) == [640.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]


@pytest.mark.parametrize("encoding", ["mono16", "16UC1"])
def test_depth_image_to_array_decodes_supported_encodings(encoding):
    message = Image(encoding=encoding, height=2, width=2, data=b"\x01\x00\x02\x00\x03\x00\x04\x00")
    assert depth_image_to_array(message).tolist() == [[1, 2], [3, 4]]


def test_depth_image_to_array_rejects_bad_encoding_or_payload():
    with pytest.raises(ValueError, match="unsupported depth encoding"):
        depth_image_to_array(Image(encoding="mono8", height=1, width=1, data=b"\x01"))
    with pytest.raises(ValueError, match="depth payload"):
        depth_image_to_array(Image(encoding="mono16", height=2, width=2, data=b"\x01\x00"))

from __future__ import annotations

import sys
import types

import numpy as np
import pytest


class _Profile:
    def __init__(self, *, width=640, height=480, fps=30, fmt="MJPG"):
        self._width = width
        self._height = height
        self._fps = fps
        self._fmt = fmt

    def get_width(self):
        return self._width

    def get_height(self):
        return self._height

    def get_fps(self):
        return self._fps

    def get_format(self):
        return self._fmt

    def get_intrinsic(self):
        return types.SimpleNamespace(
            fx=519.4,
            fy=519.1,
            cx=320.6,
            cy=241.3,
            width=self._width,
            height=self._height,
        )

    def get_distortion(self):
        return types.SimpleNamespace(
            k1=0.1,
            k2=0.2,
            k3=0.3,
            k4=0.4,
            k5=0.5,
            k6=0.6,
            p1=0.01,
            p2=0.02,
        )


class _DepthFrame:
    def __init__(self):
        self._data = np.array([[1000, 2000], [0, 65535]], dtype=np.uint16)

    def get_width(self):
        return 2

    def get_height(self):
        return 2

    def get_data(self):
        return self._data.tobytes()

    def get_depth_scale(self):
        return 0.5

    def get_format(self):
        return "Y16"

    def get_timestamp_us(self):
        return 1_234_000

    def get_system_timestamp_us(self):
        return 1_786_000_000_000_000

    def get_index(self):
        return 7


class _FrameSet:
    def get_color_frame(self):
        return None

    def get_depth_frame(self):
        return _DepthFrame()


class _Pipeline:
    def wait_for_frames(self, _timeout_ms):
        return _FrameSet()


def test_depth_scale_is_applied_and_frame_metadata_is_recorded(monkeypatch):
    from rebotarm_vision.camera.gemini2_driver import Gemini2Config, Gemini2Driver

    fake_sdk = types.SimpleNamespace(
        OBFormat=types.SimpleNamespace(MJPG="MJPG", RGB="RGB")
    )
    monkeypatch.setitem(sys.modules, "pyorbbecsdk", fake_sdk)

    driver = Gemini2Driver(
        Gemini2Config(
            color_width=640,
            color_height=480,
            color_fps=30,
            enable_depth=True,
            depth_width=640,
            depth_height=400,
            depth_fps=30,
            frame_timeout_ms=1000,
            enable_align=True,
        )
    )
    driver._pipeline = _Pipeline()

    color, depth_mm = driver.get_frame(allow_partial=True)

    assert color is None
    assert depth_mm.dtype == np.uint16
    assert depth_mm.tolist() == [[500, 1000], [0, 32768]]
    metadata = driver.get_frame_metadata()
    assert metadata["depth"]["depth_scale_mm"] == 0.5
    assert metadata["depth"]["timestamp_us"] == 1_234_000
    assert metadata["depth"]["system_timestamp_us"] == 1_786_000_000_000_000
    assert driver.get_frame_timestamp_ns("depth") == 1_786_000_000_000_000_000


def test_color_format_configuration_is_used_for_profile_selection():
    from rebotarm_vision.camera.gemini2_driver import Gemini2Config, Gemini2Driver

    driver = Gemini2Driver(
        Gemini2Config(
            color_width=640, color_height=480, color_fps=30,
            enable_depth=False, depth_width=0, depth_height=0, depth_fps=30,
            frame_timeout_ms=1000, enable_align=False, color_format="RGB",
        )
    )
    enum = types.SimpleNamespace(MJPG="MJPG", RGB="RGB")
    assert driver._color_formats(enum) == ("RGB",)
    driver._config.color_format = "unsupported"
    with pytest.raises(ValueError, match="camera.color_format"):
        driver._color_formats(enum)


def test_camera_info_uses_rational_polynomial_distortion_order():
    from rebotarm_vision.camera.gemini2_driver import Gemini2Driver

    payload = Gemini2Driver._camera_info_from_profile(_Profile())

    assert payload == {
        "width": 640,
        "height": 480,
        "fx": 519.4,
        "fy": 519.1,
        "cx": 320.6,
        "cy": 241.3,
        "distortion_model": "rational_polynomial",
        "d": [0.1, 0.2, 0.01, 0.02, 0.3, 0.4, 0.5, 0.6],
    }


def test_explicit_profile_selection_fails_closed_instead_of_silent_fallback():
    from rebotarm_vision.camera.gemini2_driver import Gemini2Driver

    class MissingProfiles:
        def get_video_stream_profile(self, *_args):
            raise RuntimeError("no match")

        def get_default_video_stream_profile(self):
            return _Profile(width=1280, height=800, fmt="Y16")

    with pytest.raises(RuntimeError, match="requested depth profile unavailable"):
        Gemini2Driver._select_video_profile(
            MissingProfiles(),
            stream_name="depth",
            width=1280,
            height=720,
            formats=("Y16",),
            fps=30,
        )


def test_default_profile_is_allowed_only_when_dimensions_are_unspecified():
    from rebotarm_vision.camera.gemini2_driver import Gemini2Driver

    default = _Profile(width=1280, height=800, fmt="Y16")

    class Profiles:
        def get_default_video_stream_profile(self):
            return default

    assert Gemini2Driver._select_video_profile(
        Profiles(),
        stream_name="depth",
        width=0,
        height=0,
        formats=("Y16",),
        fps=30,
    ) is default

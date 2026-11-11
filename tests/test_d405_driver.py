"""通过假 RealSense SDK 验证采集接线,单位,光学几何与会话收尾."""

from types import SimpleNamespace
import sys

import numpy as np
import pytest

from rebotarm_vision.camera.d405_driver import D405Config, D405Driver


class Profile:
    def __init__(self, stream):
        self.stream = stream
        self.intrinsics = SimpleNamespace(
            width=2, height=2, fx=500 if stream == "color" else 400, fy=501,
            ppx=1.0, ppy=0.5, model="brown", coeffs=[0.1, 0.2, 0.3, 0.4, 0.5],
        )

    def as_video_stream_profile(self):
        return self

    def get_intrinsics(self):
        return self.intrinsics

    def get_extrinsics_to(self, other):
        return SimpleNamespace(
            rotation=[0, 1, 0, -1, 0, 0, 0, 0, 1], translation=[0.01, -0.02, 0.03],
        )

    def width(self):
        return 2

    def height(self):
        return 2

    def fps(self):
        return 30

    def format(self):
        return "bgr8" if self.stream == "color" else "z16"


class Frame:
    def __init__(self, profile, data, domain="global", timestamp=1234567.25, units=0.0001):
        self.profile, self.data = profile, data
        self.domain, self.timestamp, self.units = domain, timestamp, units

    def get_profile(self):
        return self.profile

    def get_data(self):
        return self.data

    def get_units(self):
        return self.units

    def get_timestamp(self):
        return self.timestamp

    def get_frame_timestamp_domain(self):
        return self.domain

    def get_frame_number(self):
        return 7

    def get_width(self):
        return self.data.shape[1]

    def get_height(self):
        return self.data.shape[0]


class Frames:
    def __init__(self, color, depth):
        self.color, self.depth = color, depth

    def get_color_frame(self):
        return self.color

    def get_depth_frame(self):
        return self.depth


@pytest.fixture
def sdk(monkeypatch):
    state = SimpleNamespace(
        streams=[], disabled_all=False, sensor_options=[], starts=0, stops=0,
        aligned=0, queue=[], name="Intel RealSense D405", depth_scale=0.0001,
    )
    state.profiles = {stream: Profile(stream) for stream in ("color", "depth")}
    sensor = SimpleNamespace(
        supports=lambda option: True,
        set_option=lambda option, value: state.sensor_options.append((option, value)),
        get_depth_scale=lambda: state.depth_scale,
    )
    device = SimpleNamespace(
        get_info=lambda key: {
            "name": state.name, "serial": "123", "firmware": "5.17", "usb": "3.2",
        }[key],
        query_sensors=lambda: [sensor],
        first_depth_sensor=lambda: sensor,
    )
    active = SimpleNamespace(
        get_device=lambda: device, get_stream=lambda stream: state.profiles[stream],
    )

    class Config:
        def disable_all_streams(self):
            state.disabled_all = True

        def enable_stream(self, *args):
            state.streams.append(args)

        def resolve(self, pipeline):
            return active

    class Pipeline:
        def start(self, config):
            state.starts += 1
            return active

        def stop(self):
            state.stops += 1

        def try_wait_for_frames(self, timeout):
            assert timeout == 1000
            item = state.queue.pop(0)
            if isinstance(item, Exception):
                raise item
            return item is not None, item

    class Align:
        def __init__(self, stream):
            assert stream == "color"

        def process(self, frames):
            state.aligned += 1
            return frames

    state.pipeline = Pipeline()
    state.module = SimpleNamespace(
        pipeline=lambda: state.pipeline, config=Config, pipeline_wrapper=lambda pipe: pipe,
        align=Align, stream=SimpleNamespace(color="color", depth="depth"),
        format=SimpleNamespace(bgr8="bgr8", rgb8="rgb8", z16="z16"),
        camera_info=SimpleNamespace(name="name", serial_number="serial",
                                    firmware_version="firmware", usb_type_descriptor="usb"),
        option=SimpleNamespace(global_time_enabled="global_time"),
        distortion=SimpleNamespace(none="none", brown_conrady="brown",
                                   modified_brown_conrady="modified", inverse_brown_conrady="inverse",
                                   kannala_brandt4="kannala"),
        timestamp_domain=SimpleNamespace(system_time="system", global_time="global",
                                         hardware_clock="hardware"),
    )
    state.color = Frame(state.profiles["color"], np.full((2, 2, 3), [1, 2, 3], dtype=np.uint8))
    state.depth = Frame(state.profiles["depth"], np.array([[1000, 2000], [0, 65535]], dtype=np.uint16))
    state.frames = Frames(state.color, state.depth)
    monkeypatch.setitem(sys.modules, "pyrealsense2", state.module)
    return state


def test_open_uses_explicit_profiles_and_real_calibration(sdk):
    driver = D405Driver(D405Config())
    driver.open()
    assert sdk.disabled_all
    assert sdk.streams == [("color", 640, 480, "bgr8", 30), ("depth", 640, 480, "z16", 30)]
    assert sdk.sensor_options == [("global_time", 1.0)]
    assert driver.get_device_info()["name"] == "Intel RealSense D405"
    assert driver.get_camera_info("color")["d"] == [0.1, 0.2, 0.3, 0.4, 0.5]
    assert driver.get_camera_info("depth") == driver.get_camera_info("color")
    assert driver.get_camera_info("depth_native")["fx"] == 400
    calibration = driver.get_calibration_info()
    assert calibration["depth_scale_mm"] == pytest.approx(0.1)
    assert calibration["depth_to_color"]["rotation"] == [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
    assert calibration["depth_to_color"]["translation_mm"] == [10, -20, 30]
    assert calibration["aligned_depth_uses_color_coordinates"]
    driver.get_camera_info("color")["d"][0] = 99
    assert driver.get_camera_info("color")["d"][0] == 0.1
    driver.close()
    driver.close()
    assert sdk.stops == 1
    assert driver.get_camera_info("color") is None


def test_depth_uses_per_frame_units_and_preserves_invalid_zero(sdk):
    driver = D405Driver(D405Config())
    driver.open()
    sdk.depth.units = 0.0005
    sdk.queue.append(sdk.frames)
    color, depth = driver.get_frame()
    np.testing.assert_array_equal(depth, [[500, 1000], [0, 32768]])
    assert color.dtype == np.uint8 and depth.dtype == np.uint16
    assert sdk.aligned == 1
    assert driver.get_calibration_info()["depth_scale_mm"] == 0.5
    assert driver.get_frame_metadata()["depth"]["depth_scale_mm"] == 0.5
    sdk.color.data[:] = 0
    assert color[0, 0].tolist() == [1, 2, 3]


@pytest.mark.parametrize("domain,expected", [
    ("global", 1234567250000), ("system", 1234567250000), ("hardware", None),
])
def test_timestamp_only_exports_host_clock_domains(sdk, domain, expected):
    driver = D405Driver(D405Config())
    driver.open()
    sdk.color.domain = domain
    sdk.queue.append(sdk.frames)
    driver.get_frame()
    assert driver.get_frame_timestamp_ns("color") == expected


def test_missing_stream_and_timeout_never_reuse_old_frame_or_metadata(sdk):
    driver = D405Driver(D405Config())
    driver.open()
    sdk.queue.extend([sdk.frames, Frames(sdk.color, None), Frames(None, sdk.depth), None])
    assert all(frame is not None for frame in driver.get_frame())
    color, depth = driver.get_frame(allow_partial=True)
    assert color is not None and depth is None
    assert driver.get_frame_timestamp_ns("depth") is None
    assert driver.get_frame(allow_partial=True) == (None, None)
    assert driver.get_frame(allow_partial=True) == (None, None)
    assert driver.get_frame_metadata() == {}
    assert driver.last_debug_message == "frame_timeout"


def test_complete_mode_rejects_partial_and_warmup_waits_for_complete_pair(sdk):
    driver = D405Driver(D405Config())
    driver.open()
    sdk.queue.extend([Frames(sdk.color, None), Frames(None, sdk.depth)])
    assert not driver.warmup(2)
    sdk.queue.extend([sdk.frames, None, sdk.frames])
    assert driver.warmup(3)
    assert not sdk.queue


def test_rgb_conversion_and_unaligned_depth_intrinsics(sdk):
    driver = D405Driver(D405Config(color_format="RGB8", enable_align=False))
    driver.open()
    sdk.queue.append(sdk.frames)
    color, _ = driver.get_frame()
    assert color[0, 0].tolist() == [3, 2, 1]
    assert color.flags.c_contiguous
    assert driver.get_camera_info("depth") == driver.get_camera_info("depth_native")
    assert sdk.aligned == 0


def test_color_only_does_not_request_depth(sdk):
    driver = D405Driver(D405Config(enable_depth=False))
    driver.open()
    sdk.queue.append(Frames(sdk.color, None))
    color, depth = driver.get_frame()
    assert color is not None and depth is None
    assert len(sdk.streams) == 1
    assert driver.get_camera_info("depth") is None


@pytest.mark.parametrize("config", [
    D405Config(color_format="MJPG"), D405Config(color_width=0),
    D405Config(depth_height=0), D405Config(depth_fps=15),
    D405Config(frame_timeout_ms=0),
])
def test_invalid_configuration_fails_before_start(sdk, config):
    with pytest.raises(ValueError):
        D405Driver(config).open()
    assert sdk.starts == 0


def test_wrong_device_and_unsupported_profile_fail_visibly(sdk, monkeypatch):
    sdk.name = "Intel RealSense D435"
    with pytest.raises(RuntimeError, match="requires D405"):
        D405Driver(D405Config()).open()
    assert sdk.starts == 0
    sdk.name = "Intel RealSense D405"
    monkeypatch.setattr(sdk.pipeline, "start", lambda config: (_ for _ in ()).throw(
        RuntimeError("requested profile unavailable")))
    driver = D405Driver(D405Config())
    with pytest.raises(RuntimeError, match="profile unavailable"):
        driver.open()
    driver.close()
    assert sdk.stops == 0


@pytest.mark.parametrize("scale", [0, -1, float("nan")])
def test_invalid_calibration_releases_started_pipeline(sdk, scale):
    sdk.depth_scale = scale
    driver = D405Driver(D405Config())
    with pytest.raises(ValueError, match="depth scale"):
        driver.open()
    assert sdk.stops == 1
    driver.close()
    assert sdk.stops == 1


def test_unexpected_sdk_failure_propagates(sdk):
    driver = D405Driver(D405Config())
    driver.open()
    sdk.queue.append(RuntimeError("USB disconnected"))
    with pytest.raises(RuntimeError, match="USB disconnected"):
        driver.get_frame()
    driver.close()


@pytest.mark.parametrize("model,expected,length", [
    ("none", "plumb_bob", 5), ("brown", "plumb_bob", 5),
    ("kannala", "equidistant", 4),
])
def test_distortion_model_is_mapped_without_reordering(sdk, model, expected, length):
    profile = sdk.profiles["color"]
    profile.intrinsics.model = model
    info = D405Driver._camera_info_from_profile(profile)
    assert info["distortion_model"] == expected
    assert info["d"] == profile.intrinsics.coeffs[:length]


def test_d405_depth_probe_uses_current_candidate_topic_and_camera_range(monkeypatch):
    from rclpy.node import Node
    from rebotarm_vision.diagnostics.grasp_depth_probe_node import GraspDepthProbeNode

    parameters = {}
    topics = []
    monkeypatch.setattr(Node, "__init__", lambda self, name: None)
    monkeypatch.setattr(Node, "declare_parameter",
                        lambda self, name, default: parameters.setdefault(name, default))
    monkeypatch.setattr(Node, "get_parameter",
                        lambda self, name: SimpleNamespace(value=parameters[name]))
    monkeypatch.setattr(Node, "create_subscription",
                        lambda self, cls, topic, callback, qos: topics.append(topic))
    monkeypatch.setattr(Node, "get_logger", lambda self: SimpleNamespace(info=lambda msg: None))
    node = GraspDepthProbeNode()
    assert topics == ["/grasp/graspnet_candidates"]
    for value, expected in [
        (0, "invalid"), (float("nan"), "invalid"), (0.06, "too_close_unreliable"),
        (0.07, "reliable"), (0.1, "reliable"), (0.5, "reliable"), (0.51, "too_far"),
    ]:
        assert node._depth_status(value) == expected

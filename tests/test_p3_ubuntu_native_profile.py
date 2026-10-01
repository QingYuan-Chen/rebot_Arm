from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[1]


def _read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_visual_grasp_bringup_exposes_explicit_ubuntu_native_profile() -> None:
    launch = _read("src/rebotarm_bringup/launch/visual_grasp_system.launch.py")

    assert '"vision_profile"' in launch
    assert 'default_value="ubuntu_native"' in launch
    assert 'choices=["ubuntu_native"]' in launch
    assert '"config", "camera_ubuntu.yaml"' in launch
    assert '"yolo26s-seg.pt"' in launch
    assert 'vision_yolo_model_path = LaunchConfiguration("vision_yolo_model_path")' in launch
    assert "vision_profile" in launch and "ubuntu_native" in launch


def test_ubuntu_native_camera_profile_has_no_network_inputs() -> None:
    config = _read("src/rebotarm_vision/config/camera_ubuntu.yaml")

    assert "camera.type: gemini2" in config
    assert "ros.enable_network_detection" not in config
    assert "camera.network_" not in config
    assert "http://" not in config
    assert "https://" not in config


def test_retired_http_vision_paths_are_not_shipped() -> None:
    for path in (
        "src/rebotarm_vision/config/camera.yaml",
        "src/rebotarm_vision/rebotarm_vision/camera/network_mjpeg_driver.py",
        "src/rebotarm_vision/rebotarm_vision/network_graspnet_client.py",
        "src/rebotarm_vision/rebotarm_vision/local_graspnet_client.py",
        "scripts/ubuntu_graspnet_service.py",
    ):
        assert not (ROOT / path).exists(), path
    for path in (
        "src/rebotarm_vision/rebotarm_vision/vision_node.py",
        "src/rebotarm_vision/rebotarm_vision/graspnet_baseline_node.py",
        "src/rebotarm_bringup/launch/visual_grasp_system.launch.py",
    ):
        source = _read(path)
        assert "network_candidates_url" not in source
        assert "network_detection_client" not in source
        assert "network_mjpeg" not in source
        assert "8081" not in source
    assert not (
        ROOT / "src/rebotarm_bringup/launch/visual_grasp_perception_preview.launch.py"
    ).exists()
    node = _read("src/rebotarm_vision/rebotarm_vision/graspnet_baseline_node.py")
    assert "self.backend = self._create_inprocess_backend()" in node
    for callback in ("self._on_color", "self._on_depth", "self._on_camera_info", "self._on_detections"):
        assert callback in node


def test_vision_node_latches_camera_failure_and_publishes_empty_detections() -> None:
    node = _read("src/rebotarm_vision/rebotarm_vision/vision_node.py")

    assert "camera.max_empty_frames" in node
    assert "_publish_empty_detection" in node
    assert "_record_frame_failure" in node
    assert "if not frame_complete:" in node
    assert "empty detections will continue" in node


def test_plan_freshness_rejects_unset_old_and_accepts_recent_stamps() -> None:
    from rebotarm_vision.message_freshness import is_message_fresh, message_age_sec

    unset = SimpleNamespace(sec=0, nanosec=0)
    old = SimpleNamespace(sec=98, nanosec=0)
    recent = SimpleNamespace(sec=100, nanosec=0)

    assert message_age_sec(unset, now_ns=100_000_000_000) is None
    assert not is_message_fresh(unset, now_ns=100_000_000_000, max_age_sec=1.0)
    assert not is_message_fresh(old, now_ns=100_000_000_000, max_age_sec=1.0)
    assert is_message_fresh(recent, now_ns=100_000_000_000, max_age_sec=1.0)


def test_ubuntu_native_uses_reliable_qos_for_large_image_payloads() -> None:
    config = _read("src/rebotarm_vision/config/camera_ubuntu.yaml")
    source = _read("src/rebotarm_vision/rebotarm_vision/vision_node.py")

    assert "ros.image_reliability: reliable" in config
    assert 'self.declare_parameter("ros.image_reliability", "best_effort")' in source
    assert "ReliabilityPolicy.RELIABLE" in source
    assert "self._image_qos_profile()" in source

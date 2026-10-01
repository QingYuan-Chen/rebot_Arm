from pathlib import Path
import runpy
import shutil

import pytest


ROOT = Path(__file__).resolve().parents[1]


def _read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_ubuntu_camera_profile_uses_native_gemini_and_local_yolo() -> None:
    config = _read("src/rebotarm_vision/config/camera_ubuntu.yaml")

    assert "camera.type: gemini2" in config
    assert "camera.enable_depth: true" in config
    assert "yolo.device: \"0\"" in config
    assert "      - bottle" in config
    assert "      - cup" in config
    assert "      - person" not in config
    assert "ros.enable_detection: true" in config
    assert "http://" not in config
    assert "https://" not in config


def test_ubuntu_camera_config_uses_verified_hw_alignment_profiles() -> None:
    config = _read("src/rebotarm_vision/config/camera_ubuntu.yaml")

    assert "camera.color_width: 640" in config
    assert "camera.color_height: 480" in config
    assert "camera.depth_width: 640" in config
    assert "camera.depth_height: 400" in config
    assert "camera.enable_align: true" in config


def test_vision_node_publishes_camera_info_for_both_rgb_and_depth() -> None:
    node = _read("src/rebotarm_vision/rebotarm_vision/vision_node.py")

    assert '"/camera/color/camera_info"' in node
    assert '"/camera/depth/camera_info"' in node
    assert 'self._camera_info_payload("color"' in node
    assert 'self._camera_info_payload("depth"' in node
    assert 'CameraInfo,\n            "/camera/color/camera_info",\n            image_qos,' in node
    assert 'CameraInfo,\n            "/camera/depth/camera_info",\n            image_qos,' in node
    assert "qos_profile_sensor_data" not in node
    assert "except KeyboardInterrupt:" in node
    assert "if rclpy.ok():" in node


def test_ubuntu_launch_uses_installed_config_and_model() -> None:
    launch = _read("src/rebotarm_vision/launch/vision_ubuntu.launch.py")
    setup = _read("src/rebotarm_vision/setup.py")

    assert '"config" / "camera_ubuntu.yaml"' in launch
    assert '"yolo26s-seg.pt"' in launch
    assert 'DeclareLaunchArgument(' in launch
    assert '"yolo_model_path"' in launch
    assert '"launch/vision_ubuntu.launch.py"' in setup
    assert '"config/camera_ubuntu.yaml"' in setup
    assert 'Path("../../scripts/yolo26m-seg-fp16-b1-640-linux.engine")' in setup
    assert 'Path("../../scripts/yolo26s-seg.pt")' in setup


def test_vision_dependencies_preserve_ros_numpy_abi() -> None:
    requirements = _read("docs/setup/dependencies/vision.md")
    tensorrt_requirements = _read("docs/setup/dependencies/tensorrt.md")

    assert "numpy==1.26.4" in requirements
    assert "pyorbbecsdk2==2.0.18" in requirements
    assert "pyorbbecsdk2==2.1.1" not in requirements
    assert "tensorrt-cu12==10.13.3.9.post1" in tensorrt_requirements
    assert "tensorrt-cu12-bindings==10.13.3.9.post1" in tensorrt_requirements
    assert "tensorrt-cu12-libs==10.13.3.9.post1" in tensorrt_requirements


def test_ubuntu_vision_launcher_selects_only_the_vision_interpreter() -> None:
    launcher = _read("scripts/run_ubuntu_vision.sh")

    assert 'export REBOTARM_VISION_PYTHON=' in launcher
    assert 'export PYTHONPATH=' not in launcher
    assert '/bin/activate' not in launcher
    assert 'exec ros2 launch rebotarm_vision vision_ubuntu.launch.py "$@"' in launcher


def test_ubuntu_vision_setup_does_not_rebuild_workspace_with_venv() -> None:
    setup = _read("scripts/setup_ubuntu_vision.sh")

    assert '/bin/activate' not in setup
    assert '"${python_bin}" -m colcon' not in setup
    assert 'torch==2.11.0+cu128 torchvision==0.26.0+cu128' in setup
    assert 'install_python_dependencies.py" tensorrt --python "${python_bin}" --no-deps' in setup
    assert "import tensorrt" in setup


@pytest.mark.parametrize(
    "models",
    [(), ("yolo26s-seg.pt",), ("yolo26s-seg.pt", "yolo26m-seg-fp16-b1-640-linux.engine")],
)
def test_vision_packaging_allows_missing_runtime_models(tmp_path, monkeypatch, models):
    package = tmp_path / "src/rebotarm_vision"
    package.mkdir(parents=True)
    shutil.copyfile(ROOT / "src/rebotarm_vision/setup.py", package / "setup.py")
    tools = tmp_path / "scripts"
    tools.mkdir()
    for name in models:
        (tools / name).write_bytes(b"test model")
    captured = {}
    monkeypatch.setattr("setuptools.setup", lambda **kwargs: captured.update(kwargs))
    monkeypatch.chdir(package)
    runpy.run_path(str(package / "setup.py"), run_name="__main__")
    installed = dict(captured["data_files"]).get("share/rebotarm_vision/models", [])
    assert {Path(path).name for path in installed} == set(models)
    assert captured["name"] == "rebotarm_vision"


def test_install_docs_use_pinned_sdk_and_explicit_runtime_interpreters():
    readme = _read("docs/reference/ros_sdk.md")
    vision = _read("docs/setup/ubuntu_vision_setup_zh.md")
    simulation = _read("src/rebotarm_simulation/README_mujoco.md")
    assert "vcs import third_party < third_party/rebotarm_dependencies.repos" in readme
    assert "~/seeed/rebotarm_ros2" not in readme
    assert 'ros2 run --prefix "$GRASPNET_PYTHON"' in vision
    assert "尚未完成 `vision_profile:=ubuntu_native` 集成" not in vision
    assert "REBOTARM_MUJOCO_PYTHON" in simulation
    assert 'export REBOTARM_MUJOCO_PYTHON=' not in _read("scripts/source_local_environment.bash")
    assert "/usr/bin/python3 -m colcon build" in simulation
    assert "/bin/activate" not in simulation

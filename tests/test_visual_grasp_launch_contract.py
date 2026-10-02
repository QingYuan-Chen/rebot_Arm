"""当前视觉抓取 launch 组合契约。

这些测试只验证稳定边界：入口负责组合，阶段负责节点，profile 负责默认值，
legacy 入口负责旧参数兼容。不要再对已拆除的单体源码布局做字符串断言。
"""

from pathlib import Path
import re
import subprocess


ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / "src/rebotarm_bringup/launch"


def test_formal_entry_is_composition_only():
    text = (LAUNCH / "visual_grasp_system.launch.py").read_text(encoding="utf-8")
    assert len(text.splitlines()) < 300
    assert "DeclareLaunchArgument" in text
    assert '"visual_backend", "visual_lifecycle"' in text
    assert 'executable=' not in text


def test_stage_ownership_and_profiles_are_present():
    expected_stages = {
        "visual_backend",
        "visual_lifecycle",
        "visual_input",
        "grasp_candidate",
        "candidate_filter",
        "motion_execution",
        "grasp_executor",
        "grasp_preview",
    }
    for stage in expected_stages:
        path = LAUNCH / ("includes" if stage not in {"visual_backend", "visual_lifecycle"} else "includes") / f"{stage}.launch.py"
        assert path.exists(), path
    for path in (
        ROOT / "src/rebotarm_bringup/config/visual_backend_profile.yaml",
        ROOT / "src/rebotarm_vision/config/visual_strategy_profile.yaml",
        ROOT / "src/rebotarm_motion/config/visual_motion_profile.yaml",
        ROOT / "src/rebotarm_bringup/config/visual_grasp_interfaces.yaml",
    ):
        assert path.exists(), path


def test_legacy_entry_is_the_only_large_compatibility_surface():
    formal = (LAUNCH / "visual_grasp_system.launch.py").read_text(encoding="utf-8")
    legacy = (LAUNCH / "visual_grasp_legacy.launch.py").read_text(encoding="utf-8")
    assert len(re.findall(r"DeclareLaunchArgument", formal)) <= 30
    assert len(re.findall(r"DeclareLaunchArgument", legacy)) >= 100


def test_all_public_entries_expand_without_starting_processes():
    entries = (
        "visual_grasp_system.launch.py",
        "visual_grasp_compact.launch.py",
        "visual_readonly.launch.py",
        "visual_plan_only.launch.py",
        "visual_execute.launch.py",
    )
    for entry in entries:
        result = subprocess.run(
            ["bash", "-lc", f"source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch rebotarm_bringup {entry} --show-args"],
            cwd=ROOT,
            capture_output=True,
            text=True,
            timeout=30,
        )
        assert result.returncode == 0, result.stdout + result.stderr
        assert len(set(re.findall(r"^    '([^']+)':", result.stdout, re.M))) == 25


def test_retired_vision_paths_are_not_reintroduced():
    for path in (LAUNCH / "visual_grasp_system.launch.py", LAUNCH / "visual_grasp_legacy.launch.py"):
        text = path.read_text(encoding="utf-8")
        assert "network_mjpeg" not in text
        assert "network_candidates_url" not in text
        assert "remote_json" not in text

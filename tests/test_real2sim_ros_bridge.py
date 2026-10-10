from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from rebotarm_simulation.ros.real2sim_ros_node import (
    joint_state_message_to_sample,
    stamp_to_seconds,
    validate_topic_separation,
)
from rebotarm_simulation.apps.real2sim_viewer import build_parser as build_viewer_parser


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "src/rebotarm_simulation"


def _message(*, stamp=(1, 500_000_000), velocities=True):
    names = [f"joint{i}" for i in range(1, 7)]
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=stamp[0], nanosec=stamp[1])
        ),
        name=names,
        position=[0.0, -0.8, -1.0, 0.3, 0.0, 0.0],
        velocity=[0.0] * 6 if velocities else [],
    )


def test_joint_state_message_conversion_uses_stamp_names_and_gripper():
    sample = joint_state_message_to_sample(
        _message(), gripper_width=0.04, fallback_timestamp=9.0
    )

    assert sample.timestamp == pytest.approx(1.5)
    assert sample.joint_names == tuple(f"joint{i}" for i in range(1, 7))
    assert sample.positions[2] == -1.0
    assert sample.gripper_width == 0.04


def test_zero_message_stamp_uses_monotonic_fallback():
    sample = joint_state_message_to_sample(
        _message(stamp=(0, 0)), gripper_width=None, fallback_timestamp=9.5
    )

    assert sample.timestamp == 9.5


def test_message_and_topic_validation_rejects_unsafe_shapes_and_loop():
    message = _message()
    message.velocity = [0.0]
    with pytest.raises(ValueError, match="velocity length"):
        joint_state_message_to_sample(
            message, gripper_width=None, fallback_timestamp=1.0
        )
    with pytest.raises(ValueError, match="different"):
        validate_topic_separation("/rebotarm/joint_states", "/rebotarm/joint_states")
    with pytest.raises(ValueError, match="absolute"):
        validate_topic_separation("rebotarm/joint_states", "/real2sim/joint_states")
    with pytest.raises(ValueError):
        stamp_to_seconds(SimpleNamespace(sec=-1, nanosec=0))


def test_ros_bridge_is_read_only_and_uses_separate_output_namespace():
    source = (PACKAGE / "rebotarm_simulation/ros/real2sim_ros_node.py").read_text(
        encoding="utf-8"
    )
    config = yaml.safe_load(
        (PACKAGE / "config/real2sim_bridge.yaml").read_text(encoding="utf-8")
    )["rebotarm_real2sim_bridge"]["ros__parameters"]
    launch = (PACKAGE / "launch/real2sim_bridge.launch.py").read_text(encoding="utf-8")
    viewer = (PACKAGE / "rebotarm_simulation/apps/real2sim_viewer.py").read_text(
        encoding="utf-8"
    )

    assert "create_client" not in source
    assert "create_service" not in source
    assert "create_action" not in source
    assert config["source_joint_states_topic"] == "/rebotarm/joint_states"
    assert config["output_joint_states_topic"] == "/real2sim/joint_states"
    assert config["source_joint_states_topic"] != config["output_joint_states_topic"]
    assert "rebotarmcontroller" not in launch
    assert "use_hardware" not in launch
    assert "rebotarm_real2sim_bridge" in launch
    assert "create_client" not in viewer
    assert "create_service" not in viewer
    assert "viewer.sync()" in viewer


def test_real2sim_viewer_parser_preserves_ros_arguments():
    args, remaining = build_viewer_parser().parse_known_args(
        ["--duration", "2", "--ros-args", "-p", "mode:=physics"]
    )

    assert args.duration == 2.0
    assert remaining == ["--ros-args", "-p", "mode:=physics"]

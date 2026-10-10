from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import pytest

from rebotarm_simulation.core.real2sim import (
    JointMappingConfig,
    Real2SimMapper,
    Real2SimSynchronizer,
    RobotStateSample,
)


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "src/rebotarm_simulation/config/real2sim_mapping.yaml"


def _sample(
    timestamp=1.0,
    positions=(0.0, -0.8, -1.0, 0.3, 0.0, 0.0),
    velocities=(),
    gripper_width=0.06,
    names=("joint1", "joint2", "joint3", "joint4", "joint5", "joint6"),
):
    return RobotStateSample(
        timestamp=timestamp,
        joint_names=names,
        positions=positions,
        velocities=velocities,
        gripper_width=gripper_width,
    )


def test_mapping_config_loads_identity_rebotarm_profile():
    config = JointMappingConfig.from_yaml(CONFIG)

    assert config.source_joint_names == tuple(f"joint{i}" for i in range(1, 7))
    assert config.position_scale == (1.0,) * 6
    assert config.position_offset == (0.0,) * 6


def test_mapper_reorders_scales_offsets_filters_and_derives_velocity():
    config = JointMappingConfig(
        source_joint_names=("joint1", "joint2", "joint3", "joint4", "joint5", "joint6"),
        position_scale=(-1.0, 1.0, 1.0, 1.0, 1.0, 1.0),
        position_offset=(0.1, 0.0, 0.0, 0.0, 0.0, 0.0),
        filter_alpha=0.5,
        max_position_jump_rad=1.0,
    )
    mapper = Real2SimMapper(config)
    names = ("joint6", "joint5", "joint4", "joint3", "joint2", "joint1")
    first = mapper.map(_sample(names=names, positions=(0.0, 0.0, 0.3, -1.0, -0.8, 0.2)))
    second = mapper.map(
        _sample(
            timestamp=1.1,
            names=names,
            positions=(0.0, 0.0, 0.3, -1.0, -0.8, 0.0),
            gripper_width=0.04,
        )
    )

    assert first.positions[0] == pytest.approx(-0.1)
    assert second.positions[0] == pytest.approx(0.0)
    assert second.velocities[0] == pytest.approx(1.0)
    assert second.gripper_width == pytest.approx(0.05)


def test_mapper_rejects_missing_joint_non_monotonic_time_and_position_jump():
    mapper = Real2SimMapper(JointMappingConfig.from_yaml(CONFIG))
    with pytest.raises(ValueError, match="missing joints"):
        mapper.map(
            RobotStateSample(
                timestamp=1.0,
                joint_names=("joint1",),
                positions=(0.0,),
            )
        )
    mapper.map(_sample())
    with pytest.raises(ValueError, match="strictly increasing"):
        mapper.map(_sample(timestamp=1.0))
    with pytest.raises(ValueError, match="jump"):
        mapper.map(_sample(timestamp=1.1, positions=(1.0, -0.8, -1.0, 0.3, 0.0, 0.0)))


class FakeSimulation:
    def __init__(self):
        self.positions = [0.0] * 6
        self.targets = [0.0] * 6
        self.width = 0.06
        self.time = 0.0
        self.mirror_calls = 0

    def mirror_joint_state(self, positions, velocities, *, gripper_width=None):
        self.positions[:] = positions
        if gripper_width is not None:
            self.width = gripper_width
        self.mirror_calls += 1
        return self._state()

    def set_joint_position_targets(self, positions):
        self.targets[:] = positions

    def set_gripper_width(self, width):
        self.width = width

    def step(self, count):
        self.time += count * 0.002
        self.positions[:] = [
            current + 0.5 * (target - current)
            for current, target in zip(self.positions, self.targets)
        ]
        return self._state()

    def _state(self):
        return SimpleNamespace(
            joint_positions=tuple(self.positions) + (self.width / 2.0, -self.width / 2.0),
            gripper_width=self.width,
            simulation_time=self.time,
        )


def test_mirror_mode_finishes_at_exact_source_state():
    simulation = FakeSimulation()
    sync = Real2SimSynchronizer(
        simulation,
        Real2SimMapper(JointMappingConfig.from_yaml(CONFIG)),
        mode="mirror",
    )

    result = sync.apply(_sample())

    assert result.max_tracking_error_rad == pytest.approx(0.0)
    assert result.simulated_positions == pytest.approx(_sample().positions)
    assert simulation.mirror_calls == 2


def test_physics_mode_uses_targets_and_reports_tracking_error():
    simulation = FakeSimulation()
    sync = Real2SimSynchronizer(
        simulation,
        Real2SimMapper(JointMappingConfig.from_yaml(CONFIG)),
        mode="physics",
    )

    result = sync.apply(_sample())

    assert result.simulated_positions == pytest.approx(
        tuple(value * 0.5 for value in _sample().positions)
    )
    assert result.max_tracking_error_rad == pytest.approx(0.5)
    assert simulation.mirror_calls == 0

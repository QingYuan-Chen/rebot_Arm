"""Regression checks for control/snapshot composition and native access."""
from dataclasses import replace

import pytest


@pytest.fixture
def sim():
    pytest.importorskip("mujoco")
    from rebotarm_simulation.core.mujoco_sim import RebotArmMujoco
    with RebotArmMujoco() as instance:
        yield instance


@pytest.mark.parametrize("mode", ["pos_vel", "hold", "gravity_comp"])
def test_snapshot_replays_physics_and_controller_at_nonzero_phase(sim, mode):
    sim.reset_home()
    sim.set_joint_position_targets((.1, -.1, -.2, .2, 0, .1))
    sim.set_gripper_width(.03)
    sim.set_control_mode(mode)
    sim.step(7)
    checkpoint = sim.save_state()
    first = sim.step(23)
    continued = sim.save_state()
    sim.restore_state(checkpoint)
    assert sim.step(23) == first
    assert sim.save_state() == continued


@pytest.mark.parametrize("change", [
    {"model_identity": -1}, {"model_fingerprint": "wrong"},
    {"model_dimensions": (0,)}, {"state_spec": -1}, {"state": ()},
    {"control_targets": ()}, {"position_integral": ()},
    {"velocity_integral": ()}, {"applied_torque": ()},
    {"control_mode": "invalid"},
])
def test_invalid_snapshot_is_rejected_without_mutation(sim, change):
    sim.step(7)
    before = sim.save_state()
    invalid = replace(before, **change)
    with pytest.raises(ValueError):
        sim.restore_state(invalid)
    assert sim.save_state() == before


def test_forged_nonfinite_snapshot_is_rejected_before_physics_write(sim):
    sim.step(7)
    before = sim.save_state()
    invalid = replace(before)
    object.__setattr__(invalid, "position_integral", (float("nan"),) * 6)
    with pytest.raises(ValueError):
        sim.restore_state(invalid)
    assert sim.save_state() == before


def test_reset_positions_aligns_all_six_targets_and_preserves_gripper(sim):
    sim.set_joint_position_targets((.2, -.2, -.3, .3, .1, .2))
    sim.set_gripper_width(.04)
    targets = (.1, -.1, -.2, .2, 0, .1)
    state = sim.reset_joint_positions(targets)
    assert state.joint_positions[:6] == targets
    assert sim.control_targets[:6] == targets
    assert sim.control_targets[-2:] == (.02, -.02)


def test_native_borrow_requires_open_simulation(sim):
    model, data = sim.borrow_viewer_handles()
    assert model.nq == len(data.qpos)
    sim.close()
    with pytest.raises(RuntimeError, match="closed"):
        sim.borrow_viewer_handles()

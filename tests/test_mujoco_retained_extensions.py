from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]


def test_bottle_randomization_is_repeatable_and_stays_in_scene(bottle_scene):
    pytest.importorskip("mujoco")
    from rebotarm_simulation.core.mujoco_sim import RebotArmMujoco

    with RebotArmMujoco(ROOT / "src/rebotarm_simulation/models/rebotarm/scene_bottle.xml") as sim:
        first = sim.randomize_bottle_pose(seed=17)
        second = sim.randomize_bottle_pose(seed=17)
        assert first == second
        assert 0.22 <= first[0] <= 0.38
        assert -0.14 <= first[1] <= 0.14
        assert first[2] == 0.0
        assert sim.get_state().object_poses["bottle"] == first


def test_invalid_joint_reset_does_not_partially_change_state():
    pytest.importorskip("mujoco")
    from rebotarm_simulation.core.mujoco_sim import RebotArmMujoco

    with RebotArmMujoco() as sim:
        before = sim.save_state()
        with pytest.raises(ValueError, match="joint6"):
            sim.reset_joint_positions([0.1, -0.1, -0.2, 0.2, 0.0, 100.0])
        assert sim.save_state() == before


def test_contact_details_are_bounded_and_finite(bottle_scene):
    pytest.importorskip("mujoco")
    from rebotarm_simulation.core.mujoco_sim import RebotArmMujoco

    with RebotArmMujoco(bottle_scene) as sim:
        sim.reset_home()
        sim.step(300)
        contacts = sim.get_contacts()
        assert contacts
        assert all(contact.penetration_depth >= 0 for contact in contacts)
        assert all(len(contact.normal) == 3 for contact in contacts)


def test_local_training_and_mirroring_entries_are_retained_without_virtual_vision():
    setup = (ROOT / "src/rebotarm_simulation/setup.py").read_text(encoding="utf-8")
    for name in ("rebotarm_sim2real", "rebotarm_real2sim_bridge", "rebotarm_mujoco_pick_batch"):
        assert name in setup
    assert not (ROOT / "src/rebotarm_simulation/models/rebotarm/scene_cube.xml").exists()
    assert (ROOT / "src/rebotarm_simulation/launch/real2sim_bridge.launch.py").exists()
    assert not (ROOT / "src/rebotarm_simulation/rebotarm_simulation/virtual_camera.py").exists()

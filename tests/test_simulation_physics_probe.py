from types import SimpleNamespace
from pathlib import Path
import pytest
from rebotarm_simulation.diagnostics.physics_probe import PhysicsProbe, data_is_finite, positive_steps


@pytest.mark.parametrize('steps', [True, 0, -1, 1.5, '2', float('nan')])
def test_steps_require_positive_integer(steps):
    with pytest.raises(ValueError): positive_steps(steps)


@pytest.mark.parametrize('field', ['qpos', 'qvel', 'actuator_force', 'time'])
def test_probe_rejects_each_nonfinite_field(field):
    data = SimpleNamespace(qpos=[0.], qvel=[0.], actuator_force=[0.], time=0.)
    setattr(data, field, float('nan') if field == 'time' else [float('inf')])
    assert not data_is_finite(data)


def test_intermediate_nonfinite_sample_cannot_be_hidden_by_later_recovery():
    data = SimpleNamespace(qpos=[0.], qvel=[0.], actuator_force=[0.], time=0.)
    def step(model, state):
        state.time += 1
        state.actuator_force = [float('nan') if state.time == 1 else 0.]
    probe = PhysicsProbe(SimpleNamespace(mj_step=step), object(), data, Path('fake.xml'))
    assert probe.advance(2) is False
    assert data_is_finite(data)


def test_health_and_smoke_use_same_physical_probe():
    pytest.importorskip('mujoco')
    from rebotarm_simulation.core.resource_paths import model_resource
    from rebotarm_simulation.diagnostics.mujoco_health import check_model_health
    from rebotarm_simulation.diagnostics.mujoco_runner import run_smoke
    path = model_resource('models/rebotarm/robot.xml')
    health = check_model_health(path, steps=10)
    smoke = run_smoke(path, seconds=.02)
    assert health.physics_step_finite == smoke.finite
    assert health.simulation_time == pytest.approx(smoke.sim_time)
    assert health.actuator_count == smoke.nu


def test_observations_are_detached_and_closed_runtime_rejects_reads():
    pytest.importorskip('mujoco')
    from rebotarm_simulation.core.mujoco_sim import RebotArmMujoco
    sim = RebotArmMujoco()
    try:
        assert sim.has_body('table') and not sim.has_body('bottle')
        saved = sim.save_state()
        observation = sim.get_state()
        contacts = sim.get_contacts()
        assert sim.save_state() == saved
        sim.step(2)
        sim.restore_state(saved)
        assert sim.get_state() == observation
        assert sim.get_contacts() == contacts
    finally:
        sim.close()
    for read in (sim.get_state, sim.get_contacts, lambda: sim.has_body('bottle')):
        with pytest.raises(RuntimeError): read()

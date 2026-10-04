import numpy as np
import pytest
import torch
import mjlab

from rebotarm_drl.reach_model import (
    DT, HOLD_STEPS, HOME, LOW, HIGH, MARGIN, load_catalogue, model_spec, valid_path,
)
from rebotarm_drl.reach import advance_hold, integrate_command


def test_hold_requires_consecutive_qualified_samples_and_resets_on_violation():
    count = torch.zeros(2, dtype=torch.long)
    for _ in range(HOLD_STEPS - 1):
        count = advance_hold(count, torch.tensor([True, True]))
    assert not (count >= HOLD_STEPS).any()
    count = advance_hold(count, torch.tensor([True, False]))
    assert count.tolist() == [HOLD_STEPS, 0]
    assert advance_hold(count, torch.tensor([False, True])).tolist() == [0, 1]


def test_increment_is_once_per_policy_step_clipped_and_limit_bounded():
    q = torch.tensor(HOME).repeat(2, 1)
    low, high = torch.tensor(LOW+MARGIN), torch.tensor(HIGH-MARGIN)
    out = integrate_command(q, torch.full_like(q, 2.), low, high)
    assert torch.allclose(out-q, torch.full_like(q, .01))
    assert torch.allclose(integrate_command(high.repeat(2, 1), torch.ones_like(q), low, high), high.repeat(2, 1))
    with pytest.raises(ValueError, match="finite"):
        integrate_command(q, torch.full_like(q, float("nan")), low, high)


def test_catalogue_is_reachable_collision_free_and_held_out():
    import mujoco
    model = model_spec().compile()
    assert model.nq == model.nv == model.nu == 6
    data = mujoco.MjData(model)
    bank = load_catalogue()
    assert len(bank["train"]) == 128 and len(bank["eval"]) == 16
    for row in bank["train"] + bank["eval"]:
        assert valid_path(model, data, np.array(row["start_rad"]), np.array(row["fk_witness_rad"]))
        assert np.allclose(data.site_xpos[model.site("ee_site").id], row["target_m"], atol=1e-7)
    for row in bank["eval"]:
        assert min(np.linalg.norm(np.array(row["target_m"])-r["target_m"]) for r in bank["train"]) >= .01


def test_training_guard_rejects_this_host_before_training_import(evaluation_host):
    from rebotarm_drl.reach_workflow import train
    with pytest.raises(RuntimeError, match="training is forbidden"):
        train(confirmed=True)


def test_ik_baseline_solves_held_out_targets_without_witness():
    import mujoco
    from rebotarm_drl.reach_eval import solve_ik, smooth_command
    model = model_spec().compile()
    data = mujoco.MjData(model)
    for row in load_catalogue()["eval"]:
        start = np.array(row["start_rad"])
        q = solve_ik(model, data, start, row["target_m"])
        assert valid_path(model, data, start, q)
        assert np.linalg.norm(data.site_xpos[model.site("ee_site").id]-row["target_m"]) < .001
        assert np.allclose(smooth_command(start, q, 0., 3.), start)
        assert np.allclose(smooth_command(start, q, 3., 3.), q)


def test_mjlab_plugin_registers_only_expected_project_task(evaluation_host):
    from mjlab.tasks.registry import list_tasks, load_env_cfg, load_runner_cls
    from rebotarm_drl.reach_model import TASK_ID
    from rebotarm_drl.reach_runner import GuardedRunner
    assert [name for name in list_tasks() if name.startswith("RebotArm-")] == [TASK_ID]
    assert load_env_cfg(TASK_ID).actions["reach"].split == "train"
    assert load_env_cfg(TASK_ID, play=True).actions["reach"].split == "eval"
    assert load_runner_cls(TASK_ID) is GuardedRunner
    from types import SimpleNamespace
    with pytest.raises(RuntimeError, match="training is forbidden"):
        GuardedRunner.learn(SimpleNamespace(remote_training_confirmed=True))

import json

import pytest
import torch

from rebotarm_drl.export import export_torchscript, verify_export


def test_onnx_equivalence_and_checksum_are_verified(tmp_path):
    # Untrained arithmetic fixture; no optimizer or learning runs on this host.
    module = torch.nn.Linear(3, 2).eval()
    source = tmp_path / "fixture.pt"
    torch.jit.trace(module, torch.zeros(1, 3)).save(str(source))
    contract = {"schema_version": 1, "input_dim": 3, "output_dim": 2,
                "semantics": {"purpose": "test_fixture_only"}}
    output = tmp_path / "export"
    report = export_torchscript(source, contract, output)
    assert report["max_abs_error"] < 1e-5
    verify_export(output, contract)
    with pytest.raises(ValueError, match="contract"):
        verify_export(output, {**contract, "input_dim": 4})
    with (output / "policy.onnx").open("ab") as f:
        f.write(b"changed")
    with pytest.raises(ValueError, match="checksum"):
        verify_export(output, contract)


def test_rsl_actor_export_preserves_normalizer_and_group_order(tmp_path):
    import yaml
    import onnxruntime as ort
    import numpy as np
    from rsl_rl.models import MLPModel
    from tensordict import TensorDict
    from rebotarm_drl.export import export_rsl_checkpoint

    config = {"actor": {"class_name": "MLPModel", "hidden_dims": [4], "activation": "tanh",
                        "obs_normalization": True, "distribution_cfg": None},
              "obs_groups": {"actor": ["a", "b"]}}
    obs = TensorDict({"a": torch.tensor([[1., 2.]]), "b": torch.tensor([[3.]])}, batch_size=[1])
    actor = MLPModel(obs, config["obs_groups"], "actor", 2, hidden_dims=[4], activation="tanh", obs_normalization=True)
    # Set a nonidentity fixture; no learning or optimizer steps.
    actor.obs_normalizer._mean.fill_(0.5)
    actor.obs_normalizer._std.fill_(2.0)
    actor.eval()
    source, cfg = tmp_path / "fixture.pt", tmp_path / "agent.yaml"
    torch.save({"actor_state_dict": actor.state_dict()}, source)
    cfg.write_text(yaml.safe_dump(config))
    contract = {"schema_version": 1, "input_dim": 3, "output_dim": 2,
                "semantics": {"purpose": "fixture_only"},
                "input_groups": [{"name": "a", "size": 2}, {"name": "b", "size": 1}]}
    output = tmp_path / "export"
    export_rsl_checkpoint(source, cfg, contract, output)
    result = ort.InferenceSession(str(output / "policy.onnx")).run(None, {"observations": np.array([[1., 2., 3.]], dtype=np.float32)})[0]
    np.testing.assert_allclose(result, actor(obs).detach().numpy(), atol=1e-5)
    contract["input_groups"].reverse()
    with pytest.raises(ValueError, match="order"):
        export_rsl_checkpoint(source, cfg, contract, tmp_path / "invalid")
    contract["input_groups"].reverse()
    del config["actor"]["activation"]
    cfg.write_text(yaml.safe_dump(config))
    with pytest.raises(ValueError, match="explicitly"):
        export_rsl_checkpoint(source, cfg, contract, tmp_path / "missing_config")


def test_actual_reach_training_config_roundtrips_to_export(tmp_path):
    import mjlab
    import yaml
    from dataclasses import asdict
    from tensordict import TensorDict
    from rsl_rl.models import MLPModel
    from rebotarm_drl.tasks import runner_cfg
    from rebotarm_drl.reach_workflow import save_agent_config
    from rebotarm_drl.reach_model import deployment_contract
    from rebotarm_drl.export import export_rsl_checkpoint

    cfg = tmp_path / "agent.yaml"
    agent = runner_cfg()
    save_agent_config(cfg, agent)
    restored = yaml.safe_load(cfg.read_text())
    assert restored["obs_groups"]["actor"] == ["actor"]
    obs = TensorDict({"actor": torch.zeros(1, 34)}, batch_size=[1])
    acfg = asdict(agent.actor)
    keys = ("hidden_dims", "activation", "obs_normalization", "distribution_cfg")
    actor = MLPModel(obs, restored["obs_groups"], "actor", 6, **{key: acfg[key] for key in keys})
    checkpoint = tmp_path/"untrained-fixture.pt"
    torch.save({"actor_state_dict": actor.state_dict()}, checkpoint)
    result = export_rsl_checkpoint(checkpoint, cfg, deployment_contract(), tmp_path/"export")
    assert result["inference_samples"] == 40
    assert result["max_abs_error"] < 1e-5

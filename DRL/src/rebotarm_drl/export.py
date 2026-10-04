"""Export supplied trained policies without choosing a learning task."""
import json
from pathlib import Path

from .contracts import contract_hash, sha256, validate_contract, verify_contract


def _export_module(module, contract, output, source_files):
    import numpy as np
    import onnx
    import onnxruntime as ort
    import torch

    validate_contract(contract)
    output = Path(output)
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(f"export directory is not empty: {output}")
    output.mkdir(parents=True, exist_ok=True)
    module = module.cpu().eval()
    dummy = torch.zeros(1, contract["input_dim"])
    with torch.inference_mode():
        result = module(dummy)
    if tuple(result.shape) != (1, contract["output_dim"]):
        raise ValueError("policy output shape differs from deployment contract")
    model_path = output / "policy.onnx"
    torch.onnx.export(module, dummy, str(model_path), opset_version=18,
                      input_names=["observations"], output_names=["actions"],
                      dynamic_axes={"observations": {0: "batch"}, "actions": {0: "batch"}},
                      dynamo=False)
    onnx.checker.check_model(onnx.load(model_path))
    session = ort.InferenceSession(str(model_path), providers=["CPUExecutionProvider"])
    rng = np.random.default_rng(20261003)
    maximum = 0.0
    # Single and batched inference must both preserve the policy arithmetic.
    for batch in [1, 7, 32]:
        data = rng.normal(size=(batch, contract["input_dim"])).astype(np.float32)
        with torch.inference_mode():
            reference = module(torch.from_numpy(data)).numpy()
        actual = session.run(None, {"observations": data})[0]
        if actual.shape != reference.shape or not np.isfinite(reference).all() or not np.isfinite(actual).all():
            raise ValueError("non-finite or mismatched policy inference")
        maximum = max(maximum, float(np.max(np.abs(reference - actual))))
        np.testing.assert_allclose(actual, reference, rtol=1e-5, atol=1e-5)
    report = {"schema_version": 1, "contract": contract, "contract_sha256": contract_hash(contract),
              "model_sha256": sha256(model_path), "max_abs_error": maximum,
              "inference_samples": 40, "sources": {str(Path(p).name): sha256(p) for p in source_files},
              "validation_scope": "ONNX arithmetic equivalence; task performance evaluated separately"}
    (output / "manifest.json").write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    return report


def export_torchscript(source, contract, output):
    import torch
    return _export_module(torch.jit.load(str(source), map_location="cpu"), contract, output, [source])


def export_rsl_checkpoint(checkpoint, agent_config, contract, output):
    """RSL-RL 5.4.2 MLP actor export, including its fitted observation normalizer."""
    import copy
    import torch
    import yaml
    from tensordict import TensorDict
    from rsl_rl.models import MLPModel

    validate_contract(contract)
    config = yaml.safe_load(Path(agent_config).read_text())
    actor_config = copy.deepcopy(config["actor"])
    if actor_config.pop("class_name", None) != "MLPModel" or actor_config.get("rnn_type") or actor_config.get("cnn_cfg"):
        raise ValueError("RSL export supports explicit MLPModel configs; export other architectures to TorchScript first")
    groups = contract.get("input_groups")
    if not isinstance(groups, list) or not groups:
        raise ValueError("RSL contract requires ordered input_groups with name and size")
    names = [g["name"] for g in groups]
    sizes = [g["size"] for g in groups]
    if len(set(names)) != len(names) or any(isinstance(x, bool) or not isinstance(x, int) or x <= 0 for x in sizes):
        raise ValueError("invalid RSL input_groups")
    if sum(sizes) != contract["input_dim"] or names != config["obs_groups"]["actor"]:
        raise ValueError("RSL observation group order/dimensions differ from contract")
    allowed = {"hidden_dims", "activation", "obs_normalization", "distribution_cfg"}
    if not allowed.issubset(actor_config):
        raise ValueError("agent config must explicitly supply MLP architecture, normalization and distribution")
    if any(actor_config[key] is None for key in ("hidden_dims", "activation", "obs_normalization")):
        raise ValueError("agent config has missing MLP architecture values")
    ignored_mlp_fields = {"cnn_cfg", "rnn_type", "rnn_hidden_dim", "rnn_num_layers"}
    if set(actor_config) - allowed - ignored_mlp_fields:
        raise ValueError("unsupported actor config fields")
    kwargs = {key: value for key, value in actor_config.items() if key in allowed and value is not None}
    obs = TensorDict({g["name"]: torch.zeros(1, g["size"]) for g in groups}, batch_size=[1])
    actor = MLPModel(obs, config["obs_groups"], "actor", contract["output_dim"], **kwargs)
    saved = torch.load(checkpoint, map_location="cpu", weights_only=True)
    actor.load_state_dict(saved["actor_state_dict"], strict=True)
    return _export_module(actor.as_onnx(verbose=False), contract, output, [checkpoint, agent_config])


def verify_export(output, expected_contract):
    output = Path(output)
    report = json.loads((output / "manifest.json").read_text())
    verify_contract(report["contract"], expected_contract)
    if report["contract_sha256"] != contract_hash(report["contract"]):
        raise ValueError("deployment contract checksum mismatch")
    if sha256(output / "policy.onnx") != report["model_sha256"]:
        raise ValueError("ONNX model checksum mismatch")
    return report

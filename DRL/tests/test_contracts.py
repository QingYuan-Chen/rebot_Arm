import json
from pathlib import Path

import pytest

from rebotarm_drl.contracts import contract_hash, verify_contract
from rebotarm_drl.runtime import check_training_host


def test_export_contract_detects_caller_provided_semantic_changes():
    expected = {"schema_version": 1, "input_names": ["state"], "output_names": ["command"], "units": "caller-defined"}
    saved = json.loads(json.dumps(expected))
    verify_contract(saved, expected)
    saved["units"] = "different"
    with pytest.raises(ValueError, match="contract"):
        verify_contract(saved, expected)
    assert contract_hash(saved) != contract_hash(expected)


def test_training_rejected_on_export_workstation(tmp_path):
    policy = tmp_path / "host.json"
    policy.write_text(json.dumps({"evaluation_host_ids": ["local-id"]}))
    with pytest.raises(RuntimeError, match="export/evaluation"):
        check_training_host(policy, host_id="local-id", confirmed=True)
    with pytest.raises(RuntimeError, match="confirm"):
        check_training_host(policy, host_id="remote-id", confirmed=False)
    check_training_host(policy, host_id="remote-id", confirmed=True)


def test_unconfigured_training_host_fails_closed(tmp_path):
    with pytest.raises((FileNotFoundError, ValueError)):
        check_training_host(tmp_path / "missing.json", host_id="remote-id", confirmed=True)

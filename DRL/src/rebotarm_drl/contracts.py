"""Explicit deployment contracts; task semantics are always caller-provided."""
import hashlib
import json


def validate_contract(contract):
    if contract.get("schema_version") != 1:
        raise ValueError("deployment contract requires schema_version=1")
    for key in ("input_dim", "output_dim"):
        value = contract.get(key)
        if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
            raise ValueError(f"deployment contract requires positive {key}")
    if not isinstance(contract.get("semantics"), dict) or not contract["semantics"]:
        raise ValueError("deployment contract requires caller-provided semantics")
    contract_hash(contract)  # Also rejects NaN and non-JSON values.

def contract_hash(contract):
    return hashlib.sha256(json.dumps(contract, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def verify_contract(saved, expected):
    if contract_hash(saved) != contract_hash(expected):
        raise ValueError("observation/action contract differs from this installation")


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()

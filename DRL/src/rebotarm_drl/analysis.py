"""Paired comparison of caller-defined evaluation protocols and metrics."""
import math

from .contracts import contract_hash


def _trials(report):
    rows = report.get("trials", [])
    if not rows or len({r["id"] for r in rows}) != len(rows):
        raise ValueError("evaluation requires nonempty, unique trial ids")
    result = {}
    for row in rows:
        metrics = row["metrics"]
        if not metrics or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in metrics.values()):
            raise ValueError("trial metrics must contain finite numeric values")
        result[row["id"]] = metrics
    return result


def compare_reports(candidate, baseline):
    import numpy as np
    if not candidate.get("protocol") or contract_hash(candidate["protocol"]) != contract_hash(baseline.get("protocol")):
        raise ValueError("evaluation protocol mismatch; do not compare different conditions")
    a, b = _trials(candidate), _trials(baseline)
    if set(a) != set(b):
        raise ValueError("paired trial ids differ")
    ids = sorted(a)
    metrics = set(a[ids[0]])
    if any(set(a[key]) != metrics or set(b[key]) != metrics for key in ids):
        raise ValueError("trial metric names differ")
    result = {}
    for name in sorted(metrics):
        av = np.asarray([a[key][name] for key in ids], dtype=float)
        bv = np.asarray([b[key][name] for key in ids], dtype=float)
        delta = av - bv
        result[name] = {"candidate_mean": float(av.mean()), "baseline_mean": float(bv.mean()),
                        "candidate_p95": float(np.percentile(av, 95)), "baseline_p95": float(np.percentile(bv, 95)),
                        "mean_paired_delta": float(delta.mean()), "paired_delta_std": float(delta.std())}
    return {"protocol": candidate["protocol"], "paired_trials": len(ids), "metrics": result,
            "interpretation": "delta=candidate-baseline; metric direction and acceptance criteria are caller-defined"}

import pytest

from rebotarm_drl.analysis import compare_reports
from rebotarm_drl.replay import validate_recording


def test_comparison_requires_same_protocol_and_paired_trials():
    baseline = {"protocol": {"suite": "fixture", "seed": 11}, "trials": [
        {"id": "a", "metrics": {"error": 0.2}}, {"id": "b", "metrics": {"error": 0.4}}]}
    candidate = {"protocol": baseline["protocol"], "trials": [
        {"id": "b", "metrics": {"error": 0.3}}, {"id": "a", "metrics": {"error": 0.1}}]}
    report = compare_reports(candidate, baseline)
    assert report["metrics"]["error"]["mean_paired_delta"] == pytest.approx(-0.1)
    candidate["protocol"] = {"suite": "different", "seed": 11}
    with pytest.raises(ValueError, match="protocol"):
        compare_reports(candidate, baseline)
    candidate["protocol"] = baseline["protocol"]
    candidate["trials"].pop()
    with pytest.raises(ValueError, match="trial"):
        compare_reports(candidate, baseline)


def test_recording_rejects_nonmonotonic_time_and_wrong_dimension():
    import numpy as np
    qpos = np.zeros((3, 4))
    validate_recording(qpos, np.array([0.0, 0.1, 0.2]), nq=4)
    with pytest.raises(ValueError, match="time"):
        validate_recording(qpos, np.array([0.0, 0.1, 0.1]), nq=4)
    with pytest.raises(ValueError, match="shape"):
        validate_recording(qpos, np.array([0.0, 0.1, 0.2]), nq=5)

import json

import numpy as np
import pytest

from rebotarm_drl.cli import main
from rebotarm_drl.replay import replay, validate_recording


def test_replay_validates_time_and_model_dimensions(tmp_path):
    xml = tmp_path / "fixture.xml"
    xml.write_text('<mujoco><worldbody><body><joint type="hinge"/><geom size="0.1"/></body></worldbody></mujoco>')
    recording = tmp_path / "fixture.npz"
    np.savez(recording, qpos=np.zeros((2, 1)), time_s=np.array([0., .1]))
    assert replay(xml, recording, headless=True)["frames"] == 2
    with pytest.raises(ValueError, match="increasing"):
        validate_recording(np.zeros((2, 1)), np.array([0., 0.]), nq=1)
    with pytest.raises(ValueError, match="shape"):
        validate_recording(np.zeros((2, 2)), np.array([0., .1]), nq=1)


def test_cli_compares_explicit_reports(tmp_path, capsys):
    report = tmp_path / "report.json"
    report.write_text(json.dumps({"protocol": {"fixture": True},
                                 "trials": [{"id": "a", "metrics": {"x": 2.0}}]}))
    assert main(["compare", "--candidate", str(report), "--baseline", str(report)]) == 0
    assert json.loads(capsys.readouterr().out)["metrics"]["x"]["mean_paired_delta"] == 0


def test_cli_refuses_training_on_this_evaluation_host(evaluation_host):
    with pytest.raises(RuntimeError, match="training is forbidden"):
        main(["train", "--confirm-remote-training"])

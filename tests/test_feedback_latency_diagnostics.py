"""延迟证据汇总:接收空档不能被恢复后的新帧或缺失 DDS 时间戳掩盖."""
import importlib.util
from pathlib import Path

import pytest


SCRIPT = Path(__file__).resolve().parents[1]/'scripts/diagnose_feedback_latency.py'
spec = importlib.util.spec_from_file_location('feedback_latency_diagnostics', SCRIPT)
diagnostics = importlib.util.module_from_spec(spec)
spec.loader.exec_module(diagnostics)


def test_stalled_callbacks_remain_stale_even_if_recovered_samples_are_fresh(tmp_path):
    topic = '/probe/joints/joint1/state'
    rows = []
    for mono in (1., 3.1, 3.4):
        wall = round((1000.+mono)*1e9)
        rows.append(dict(event='callback', mono=mono, wall=wall, topic=topic,
                         stamp=wall-20_000_000, dds_source=wall-2_000_000,
                         dds_received=wall-1_000_000))
    rows += [dict(event='health', ages={topic: .25}, complete=True, status_age=.7),
             dict(event='health', ages={topic: .02}, complete=True, status_age=.1),
             dict(event='summary', cpu=.1, elapsed=4.)]
    path = tmp_path/'sub.jsonl'
    diagnostics.write_rows(path, rows)
    result = diagnostics.summarize(path)
    assert result['maximum_callback_source_age_ms'] == pytest.approx(20.)
    assert result['maximum_joint1_gap_ms'] == pytest.approx(300.)
    assert result['maximum_monitor_source_age_ms'] == pytest.approx(250.)
    assert result['stale_observations'] == result['heartbeat_timeouts'] == 1
    assert result['maximum_dds_transport_ms'] == pytest.approx(1.)
    assert result['maximum_dds_to_callback_ms'] == pytest.approx(1.)
    assert result['wall_monotonic_offset_span_ms'] == pytest.approx(0.)


def test_missing_dds_timestamps_cannot_be_reported_as_zero_transport_delay(tmp_path):
    path = tmp_path/'sub.jsonl'
    rows = [dict(event='callback', mono=t, wall=round(t*1e9),
                 topic='/probe/joints/joint1/state', stamp=round(t*1e9)-10_000_000,
                 dds_source=0, dds_received=0) for t in (1., 3.1)]
    rows += [dict(event='health', ages={}, complete=False, status_age=None),
             dict(event='summary', cpu=.1, elapsed=4.)]
    diagnostics.write_rows(path, rows)
    result = diagnostics.summarize(path)
    assert result['dds_timestamp_samples'] == 0
    assert result['maximum_dds_transport_ms'] is None
    assert result['maximum_dds_to_callback_ms'] is None
    assert result['missing_observations'] == result['heartbeat_timeouts'] == 1


def test_offline_tool_rejects_hardware_backend_import():
    with pytest.raises(RuntimeError, match='禁止'):
        diagnostics.BlockDevices().find_spec('motorbridge')

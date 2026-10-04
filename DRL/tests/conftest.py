"""Host-policy tests must never depend on, or train on, the executing machine."""
import json

import pytest


@pytest.fixture
def evaluation_host(monkeypatch):
    from rebotarm_drl import runtime
    host = json.loads(runtime.POLICY.read_text())["evaluation_host_ids"][0]
    monkeypatch.setattr(runtime, "machine_id", lambda: host)

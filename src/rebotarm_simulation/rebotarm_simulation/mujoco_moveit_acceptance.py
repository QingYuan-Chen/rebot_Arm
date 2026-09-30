"""Compatibility CLI forwarding to the motion-owned MoveIt acceptance probe.

The optional probe runs in a separate process so the physics package does not
import MoveIt adapters or duplicate their implementation. The motion probe
verifies the unique headless MuJoCo action owner before sending any goal.
"""
from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
from typing import Sequence


def run_acceptance(*, timeout: float = 30.0) -> dict:
    timeout = float(timeout)
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError("timeout must be positive and finite")
    result = subprocess.run(
        [sys.executable, "-m", "rebotarm_motion.mujoco_moveit_acceptance",
         "--timeout", str(timeout)],
        capture_output=True, text=True, timeout=8 * timeout + 10,
    )
    if result.returncode not in (0, 2):
        raise RuntimeError(result.stderr.strip() or "motion acceptance probe failed")
    payload = json.loads(result.stdout)
    if not isinstance(payload, dict) or not isinstance(payload.get("ok"), bool):
        raise RuntimeError("invalid motion acceptance result")
    if result.returncode != (0 if payload["ok"] else 2):
        raise RuntimeError("motion acceptance exit code disagrees with result")
    return payload


def main(argv: Sequence[str] | None = None, *, stdout=None, stderr=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=30.0)
    args = parser.parse_args(argv)
    stdout = stdout or sys.stdout
    stderr = stderr or sys.stderr
    try:
        payload = run_acceptance(timeout=args.timeout)
    except Exception as exc:
        print(f"error: {type(exc).__name__}: {exc}", file=stderr)
        return 1
    print(json.dumps(payload, ensure_ascii=False, sort_keys=True), file=stdout)
    return 0 if payload["ok"] else 2


if __name__ == "__main__":
    raise SystemExit(main())

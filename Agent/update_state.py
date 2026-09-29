#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import re
import subprocess
from datetime import datetime
from pathlib import Path
from typing import Any


AGENT_DIR = Path(__file__).resolve().parent
ROOT = AGENT_DIR.parent
PROJECT_STATUS = AGENT_DIR / "PROJECT_STATUS.md"
CURRENT_STATUS = AGENT_DIR / "CURRENT_STATUS.md"
EXECUTION_FLOW = AGENT_DIR / "EXECUTION_FLOW.md"
MEMORY = AGENT_DIR / "MEMORY.md"
ACTIVITY_LOG = AGENT_DIR / "ACTIVITY_LOG.md"
STATE = AGENT_DIR / "STATE.json"

PHASE_RE = re.compile(r"^## (P\d+): (.+?) \[weight=(\d+)\]\s*$")
CHECK_RE = re.compile(r"^- \[([ xX])\] (.+)$")


def run_git(*args: str) -> str | None:
    result = subprocess.run(
        ["git", *args],
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    if result.returncode != 0:
        return None
    return result.stdout.strip()


def parse_phases() -> list[dict[str, Any]]:
    phases: list[dict[str, Any]] = []
    current: dict[str, Any] | None = None
    for line in PROJECT_STATUS.read_text(encoding="utf-8").splitlines():
        phase_match = PHASE_RE.match(line)
        if phase_match:
            current = {
                "id": phase_match.group(1),
                "name": phase_match.group(2),
                "weight": int(phase_match.group(3)),
                "completed_items": 0,
                "total_items": 0,
            }
            phases.append(current)
            continue
        if line.startswith("## "):
            current = None
            continue
        if current is None:
            continue
        check_match = CHECK_RE.match(line)
        if not check_match:
            continue
        current["total_items"] += 1
        if check_match.group(1).lower() == "x":
            current["completed_items"] += 1

    for phase in phases:
        total = phase["total_items"]
        completed = phase["completed_items"]
        phase["completion_percent"] = round(100.0 * completed / total, 1) if total else 0.0
    return phases


def parse_section_bullets(path: Path, heading: str) -> list[str]:
    items: list[str] = []
    active = False
    for line in path.read_text(encoding="utf-8").splitlines():
        if line == heading:
            active = True
            continue
        if active and line.startswith("## "):
            break
        if active:
            match = re.match(r"^- (.+)$", line)
            if match:
                items.append(match.group(1))
    return items


def parse_numbered_queue() -> list[str]:
    items: list[str] = []
    active = False
    for line in EXECUTION_FLOW.read_text(encoding="utf-8").splitlines():
        if line == "## 当前执行队列":
            active = True
            continue
        if active and line.startswith("## "):
            break
        if active:
            match = re.match(r"^\d+\. (.+)$", line)
            if match:
                items.append(match.group(1))
    return items


def parse_active_phase() -> str | None:
    for line in CURRENT_STATUS.read_text(encoding="utf-8").splitlines():
        if line.startswith("- 当前阶段："):
            return line.split("：", 1)[1].rstrip("。").strip()
    return None


def parse_current_checklist() -> dict[str, Any]:
    done = total = 0
    active = False
    for line in CURRENT_STATUS.read_text(encoding="utf-8").splitlines():
        if line == "## 当前验收清单":
            active = True
            continue
        if active and line.startswith("## "):
            break
        if not active:
            continue
        match = CHECK_RE.match(line)
        if not match:
            continue
        total += 1
        if match.group(1).lower() == "x":
            done += 1
    return {
        "completed_items": done,
        "total_items": total,
        "completion_percent": round(100.0 * done / total, 1) if total else 0.0,
    }


def clean_field(value: str) -> str:
    return " ".join(value.replace("|", "/").split())


def append_event(actor: str, event: str, note: str, verification: str) -> None:
    timestamp = datetime.now().astimezone().isoformat(timespec="seconds")
    line = (
        f"- {timestamp} | actor={clean_field(actor)} | event={clean_field(event)} "
        f"| note={clean_field(note)} | verification={clean_field(verification) or '-'}\n"
    )
    with ACTIVITY_LOG.open("a", encoding="utf-8") as handle:
        handle.write(line)


def last_event() -> str | None:
    lines = [
        line
        for line in ACTIVITY_LOG.read_text(encoding="utf-8").splitlines()
        if line.startswith("- ")
    ]
    return lines[-1] if lines else None


def build_state() -> dict[str, Any]:
    phases = parse_phases()
    current_checklist = parse_current_checklist()
    total_weight = sum(phase["weight"] for phase in phases)
    weighted = sum(
        phase["weight"] * phase["completion_percent"] / 100.0
        for phase in phases
    )
    status = run_git("status", "--porcelain")
    return {
        "schema_version": 1,
        "generated_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "plan_source": str(PROJECT_STATUS.relative_to(ROOT)),
        "current_status_source": str(CURRENT_STATUS.relative_to(ROOT)),
        "completion_basis": "weighted verified checklist; not code volume or hardware readiness",
        "overall_completion_percent": round(100.0 * weighted / total_weight, 1) if total_weight else 0.0,
        "active_phase": parse_active_phase(),
        "current_checklist": current_checklist,
        "phases": phases,
        "blockers": parse_section_bullets(MEMORY, "## 当前阻塞"),
        "next_actions": parse_numbered_queue(),
        "git": {
            "branch": run_git("branch", "--show-current"),
            "head": run_git("rev-parse", "--short", "HEAD"),
            "working_tree_dirty": bool(status),
        },
        "last_event": last_event(),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Refresh reBotArm Agent state")
    parser.add_argument(
        "--event",
        choices=("start", "checkpoint", "blocked", "verified", "complete", "status-refresh"),
    )
    parser.add_argument("--actor", default="Agent")
    parser.add_argument("--note", default="")
    parser.add_argument("--verification", default="")
    args = parser.parse_args()

    if args.event:
        if not args.note:
            parser.error("--note is required when --event is provided")
        append_event(args.actor, args.event, args.note, args.verification)

    state = build_state()
    STATE.write_text(
        json.dumps(state, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(
        f"state updated: overall={state['overall_completion_percent']:.1f}% "
        f"active={state['active_phase']} branch={state['git']['branch']}"
    )


if __name__ == "__main__":
    main()

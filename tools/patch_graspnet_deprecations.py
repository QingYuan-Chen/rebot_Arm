#!/usr/bin/env python3
"""Apply small compatibility fixes to the ignored local GraspNet checkout.

The upstream baseline is intentionally not vendored in Git.  This idempotent
patch keeps its current PyTorch API usage quiet without modifying the model
or inference behavior.
"""

from pathlib import Path
import argparse


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    args = parser.parse_args()
    path = args.root / "utils" / "loss_utils.py"
    if not path.is_file():
        raise SystemExit(f"GraspNet checkout not found: {path}")
    text = path.read_text(encoding="utf-8")
    updated = text.replace("torch.cross(axis_x, axis_y)\n", "torch.cross(axis_x, axis_y, dim=-1)\n")
    if updated != text:
        path.write_text(updated, encoding="utf-8")
        print(f"patched {path}")
    else:
        print(f"already patched {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

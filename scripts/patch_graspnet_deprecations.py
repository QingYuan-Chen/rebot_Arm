#!/usr/bin/env python3
"""对指定的本地 GraspNet 检出幂等应用 PyTorch 兼容修改。

为 torch.cross 显式声明向量末维，保持现有推理计算；仅在精确旧片段存在时写文件。
第三方检出由本地部署准备，并保持在主仓库跟踪范围之外。
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

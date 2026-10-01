#!/usr/bin/env python3
"""Install a dependency profile from its single Markdown requirements block."""
from __future__ import annotations

import argparse
from pathlib import Path
import re
import shlex
import subprocess
import sys


DEPENDENCIES = Path(__file__).resolve().parents[1] / "docs/setup/dependencies"
PROFILES = ("runtime", "vision", "graspnet", "mujoco", "rl", "tensorrt")


def read_requirements(document: Path, parents: tuple[Path, ...] = ()) -> list[str]:
    document = document.resolve()
    if document in parents:
        raise ValueError(f"依赖文档循环包含：{document}")
    blocks = re.findall(
        r"^```requirements[ \t]*\n(.*?)^```[ \t]*$",
        document.read_text(encoding="utf-8"), re.MULTILINE | re.DOTALL,
    )
    if len(blocks) != 1:
        raise ValueError(f"文档必须有且仅有一个完整的 requirements 代码块：{document}")
    arguments: list[str] = []
    for raw in blocks[0].splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith(("-r ", "--requirement ")):
            tokens = shlex.split(line)
            if len(tokens) != 2:
                raise ValueError(f"无效的 requirements 包含：{line}")
            arguments.extend(read_requirements(
                document.parent / tokens[1], parents + (document,),
            ))
        elif line.startswith("--"):
            arguments.extend(shlex.split(line))
        else:
            arguments.append(line)
    return arguments


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("profile", choices=PROFILES)
    parser.add_argument("--python", default=sys.executable, help="Target Python interpreter")
    parser.add_argument("--no-deps", action="store_true")
    parser.add_argument("--user", action="store_true")
    parser.add_argument("--break-system-packages", action="store_true")
    parser.add_argument("--extra-index-url", action="append", default=[])
    parser.add_argument("--show", action="store_true", help="Print command without installing")
    args = parser.parse_args(argv)
    try:
        requirements = read_requirements(DEPENDENCIES / f"{args.profile}.md")
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    command = [args.python, "-m", "pip", "install"]
    for flag in ("no_deps", "user", "break_system_packages"):
        if getattr(args, flag):
            command.append("--" + flag.replace("_", "-"))
    for url in args.extra_index_url:
        command.extend(["--extra-index-url", url])
    command.extend(requirements)
    if args.show:
        print(shlex.join(command))
        return 0
    return subprocess.run(command, check=False).returncode


if __name__ == "__main__":
    raise SystemExit(main())

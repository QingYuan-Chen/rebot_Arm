"""Rebuild the locked environment in its destination; never copy a venv."""
import argparse
import os
from pathlib import Path
import platform
import subprocess
import sys
import venv

ROOT = Path(__file__).resolve().parents[1]


def main(argv=None):
    parser = argparse.ArgumentParser(description="在本目录创建锁版本环境；默认离线安装")
    parser.add_argument("--online", action="store_true")
    args = parser.parse_args(argv)
    if sys.version_info[:2] != (3, 12) or platform.system() != "Linux" or platform.machine() != "x86_64":
        parser.error("需要 Linux x86_64 / Python 3.12")
    target = ROOT / ".venv"
    if target.exists():
        parser.error(".venv 已存在；为保护现有环境，请在新解压目录重建")
    wheels = ROOT / "wheelhouse"
    if not args.online and not wheels.is_dir():
        parser.error("离线安装缺少 wheelhouse；使用完整部署包或显式 --online")
    env = dict(os.environ, PIP_CACHE_DIR=str(ROOT / ".cache/pip"),
               PIP_DISABLE_PIP_VERSION_CHECK="1", PYTHONNOUSERSITE="1")
    env.pop("PYTHONPATH", None)
    env.pop("PYTHONHOME", None)
    venv.EnvBuilder(with_pip=True).create(target)
    python = str(target / "bin/python")
    cmd = [python, "-m", "pip", "--isolated", "install", "--require-hashes", "--only-binary=:all:",
           "--no-cache-dir", "-r", str(ROOT / "requirements.lock")]
    if not args.online:
        cmd += ["--no-index", "--find-links", str(wheels)]
    subprocess.run(cmd, cwd=ROOT, env=env, check=True)
    subprocess.run([python, "-m", "pip", "--isolated", "install", "--no-index", "--no-cache-dir",
                    "--no-build-isolation", "--no-deps", "-e", str(ROOT)], cwd=ROOT, env=env, check=True)
    subprocess.run([python, "-m", "pip", "check"], cwd=ROOT, env=env, check=True)
    print(f"环境已就绪：{target}；包含 Reach-and-Hold 任务，未启动训练。")


if __name__ == "__main__":
    main()

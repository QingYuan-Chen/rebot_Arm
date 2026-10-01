#!/usr/bin/env python3
"""Build and validate the pinned MotorBridge feedback and zeroing safety patch.

This tool never constructs a MotorBridge Controller or Motor. Installation is
available only through the explicit ``--install-user`` mode.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
from types import ModuleType
import venv


ROOT = Path(__file__).resolve().parents[1]
UPSTREAM_URL = "https://github.com/motorbridge/motorbridge.git"
UPSTREAM_COMMIT = "2b7b350914ace47ba06e85fcad333143de2b057b"
PATCHED_VERSION = "0.4.7+rebotarm.1"
PATCH_SHA256 = "3d40ad4f8779c4bacad68a2e1941fde10d36f2af9c41baea10f7ae37e7aaba6a"
UPSTREAM_DM_SERIAL_TIMEOUT_MS = 10
PATCH_PATH = ROOT / "patches/motorbridge/0001-add-feedback-sequence-api.patch"
BUILD_ROOT = ROOT / "build_motorbridge_fresh_feedback"
SOURCE_DIR = BUILD_ROOT / "source"
TARGET_DIR = BUILD_ROOT / "target"
WHEEL_DIR = BUILD_ROOT / "wheel"


def validate_runtime_contract(module: ModuleType | object) -> None:
    """Reject a MotorBridge module that cannot prove feedback receive identity."""

    motor_type = getattr(module, "Motor", None)
    sequence_method = getattr(motor_type, "get_state_with_sequence", None)
    if motor_type is None or not callable(sequence_method):
        raise RuntimeError(
            "MotorBridge runtime is missing Motor.get_state_with_sequence; "
            "the unpatched 0.4.6 package cannot prove fresh feedback"
        )

    capabilities_fn = getattr(module, "abi_capabilities", None)
    if not callable(capabilities_fn):
        raise RuntimeError(
            "MotorBridge runtime is missing the abi_capabilities() contract"
        )
    try:
        capabilities = capabilities_fn()
        feature_enabled = capabilities["features"]["feedback_sequence"] is True
    except (KeyError, TypeError) as exc:
        raise RuntimeError(
            "MotorBridge ABI is missing features.feedback_sequence=true"
        ) from exc
    if not feature_enabled:
        raise RuntimeError(
            "MotorBridge ABI is missing features.feedback_sequence=true"
        )

    version = getattr(module, "__version__", None)
    if version != PATCHED_VERSION:
        raise RuntimeError(
            f"MotorBridge runtime must be {PATCHED_VERSION}, found {version!r}"
        )


def _run(
    command: list[str],
    *,
    cwd: Path | None = None,
    env: dict[str, str] | None = None,
    capture: bool = False,
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        command,
        cwd=cwd,
        env=env,
        check=True,
        text=True,
        stdout=subprocess.PIPE if capture else None,
        stderr=subprocess.PIPE if capture else None,
    )


def _git_output(*args: str, env: dict[str, str] | None = None) -> str:
    result = _run(
        ["git", "-C", str(SOURCE_DIR), *args],
        env=env,
        capture=True,
    )
    return result.stdout.strip()


def _verify_patch_file() -> None:
    if not PATCH_PATH.is_file():
        raise RuntimeError(f"MotorBridge patch is missing: {PATCH_PATH}")
    digest = hashlib.sha256(PATCH_PATH.read_bytes()).hexdigest()
    if digest != PATCH_SHA256:
        raise RuntimeError(
            "MotorBridge patch digest does not match the reviewed source patch: "
            f"expected {PATCH_SHA256}, found {digest}"
        )


def _verify_dm_serial_timeout_budget() -> None:
    source = SOURCE_DIR / "motor_core" / "src" / "dm_serial.rs"
    expected = (
        ".timeout(Duration::from_millis("
        f"{UPSTREAM_DM_SERIAL_TIMEOUT_MS}))"
    )
    if not source.is_file() or expected not in source.read_text(encoding="utf-8"):
        raise RuntimeError(
            "MotorBridge dm-serial timeout budget does not match the reviewed "
            f"upstream value of {UPSTREAM_DM_SERIAL_TIMEOUT_MS} ms"
        )


def _checkout_matches_patched_snapshot() -> bool:
    BUILD_ROOT.mkdir(parents=True, exist_ok=True)
    descriptor, index_name = tempfile.mkstemp(
        prefix=".motorbridge-expected-index-",
        dir=BUILD_ROOT,
    )
    os.close(descriptor)
    index_path = Path(index_name)
    index_path.unlink()
    env = os.environ.copy()
    env["GIT_INDEX_FILE"] = str(index_path)
    try:
        _git_output("read-tree", "HEAD", env=env)
        _run(
            [
                "git",
                "-C",
                str(SOURCE_DIR),
                "apply",
                "--cached",
                str(PATCH_PATH),
            ],
            env=env,
            capture=True,
        )
        worktree_diff = _run(
            ["git", "-C", str(SOURCE_DIR), "diff", "--quiet"],
            env=env,
        )
        untracked = _git_output(
            "ls-files",
            "--others",
            env=env,
        )
        return worktree_diff.returncode == 0 and untracked == ""
    except subprocess.CalledProcessError:
        return False
    finally:
        index_path.unlink(missing_ok=True)


def _prepare_source_checkout() -> None:
    BUILD_ROOT.mkdir(parents=True, exist_ok=True)
    if not SOURCE_DIR.exists():
        _run(
            [
                "git",
                "clone",
                "--no-checkout",
                "--filter=blob:none",
                UPSTREAM_URL,
                str(SOURCE_DIR),
            ]
        )
        _run(
            [
                "git",
                "-C",
                str(SOURCE_DIR),
                "checkout",
                "--detach",
                UPSTREAM_COMMIT,
            ]
        )
    elif not (SOURCE_DIR / ".git").exists():
        raise RuntimeError(
            f"Refusing unexpected MotorBridge checkout without .git: {SOURCE_DIR}"
        )

    head = _git_output("rev-parse", "HEAD")
    if head != UPSTREAM_COMMIT:
        raise RuntimeError(
            f"Refusing MotorBridge checkout at unexpected HEAD {head}; "
            f"expected {UPSTREAM_COMMIT}"
        )
    origin = _git_output("remote", "get-url", "origin")
    if origin != UPSTREAM_URL:
        raise RuntimeError(
            f"Refusing MotorBridge checkout with unexpected origin {origin!r}; "
            f"expected {UPSTREAM_URL!r}"
        )

    status = _git_output("status", "--porcelain", "--untracked-files=all")
    if status:
        if not _checkout_matches_patched_snapshot():
            raise RuntimeError(
                "Refusing dirty MotorBridge checkout: its content is not exactly "
                "the reviewed feedback-sequence patch"
            )
        return

    _run(
        [
            "git",
            "-C",
            str(SOURCE_DIR),
            "apply",
            "--check",
            str(PATCH_PATH),
        ]
    )
    _run(
        ["git", "-C", str(SOURCE_DIR), "apply", str(PATCH_PATH)]
    )
    if not _checkout_matches_patched_snapshot():
        raise RuntimeError("Applied MotorBridge patch does not match reviewed content")


def _cargo_environment() -> tuple[str, dict[str, str]]:
    env = os.environ.copy()
    explicit = env.get("MOTORBRIDGE_CARGO")
    cargo_path = Path(explicit).expanduser() if explicit else None
    if cargo_path is None:
        discovered = shutil.which("cargo")
        if discovered:
            cargo_path = Path(discovered)
    if cargo_path is None:
        candidates = sorted(
            (BUILD_ROOT / "toolchain/rustup/toolchains").glob("*/bin/cargo")
        )
        if candidates:
            cargo_path = candidates[-1]
    if cargo_path is None or not cargo_path.is_file():
        raise RuntimeError(
            "Rust cargo was not found. Install a Rust toolchain or set "
            "MOTORBRIDGE_CARGO=/absolute/path/to/cargo."
        )

    toolchain_bin = cargo_path.parent
    rustc_path = toolchain_bin / "rustc"
    if rustc_path.is_file():
        env["RUSTC"] = str(rustc_path)
        env["PATH"] = f"{toolchain_bin}{os.pathsep}{env.get('PATH', '')}"
    env.setdefault("CARGO_HOME", str(BUILD_ROOT / "toolchain/cargo"))
    env["CARGO_TARGET_DIR"] = str(TARGET_DIR)
    return str(cargo_path), env


def _build_rust_artifacts() -> tuple[Path, Path]:
    cargo, env = _cargo_environment()
    _run(
        [
            cargo,
            "build",
            "--manifest-path",
            str(SOURCE_DIR / "Cargo.toml"),
            "-p",
            "motor_abi",
            "-p",
            "ws_gateway",
            "--release",
        ],
        cwd=SOURCE_DIR,
        env=env,
    )

    if sys.platform.startswith("linux"):
        abi_path = TARGET_DIR / "release/libmotor_abi.so"
        gateway_path = TARGET_DIR / "release/ws_gateway"
    elif sys.platform == "darwin":
        abi_path = TARGET_DIR / "release/libmotor_abi.dylib"
        gateway_path = TARGET_DIR / "release/ws_gateway"
    elif sys.platform.startswith("win"):
        abi_path = TARGET_DIR / "release/motor_abi.dll"
        gateway_path = TARGET_DIR / "release/ws_gateway.exe"
    else:
        raise RuntimeError(f"Unsupported build platform: {sys.platform}")
    for artifact in (abi_path, gateway_path):
        if not artifact.is_file():
            raise RuntimeError(f"Expected MotorBridge build artifact is missing: {artifact}")
    return abi_path, gateway_path


def _build_wheel(abi_path: Path, gateway_path: Path) -> Path:
    WHEEL_DIR.mkdir(parents=True, exist_ok=True)

    with tempfile.TemporaryDirectory(
        prefix=".motorbridge-wheel-source-",
        dir=BUILD_ROOT,
    ) as temp_name:
        package_source = Path(temp_name) / "python"
        shutil.copytree(SOURCE_DIR / "bindings/python", package_source)
        env = os.environ.copy()
        env["MOTORBRIDGE_LIB"] = str(abi_path)
        env["MOTORBRIDGE_WS_GATEWAY_BIN"] = str(gateway_path)
        _run(
            [
                sys.executable,
                "-m",
                "pip",
                "wheel",
                "--no-deps",
                "--no-build-isolation",
                "--wheel-dir",
                str(WHEEL_DIR),
                str(package_source),
            ],
            env=env,
        )

    wheels = sorted(WHEEL_DIR.glob(f"motorbridge-{PATCHED_VERSION}-*.whl"))
    if len(wheels) != 1:
        raise RuntimeError(
            f"Expected exactly one MotorBridge wheel in {WHEEL_DIR}, found {len(wheels)}"
        )
    return wheels[0]


def _smoke_test_wheel(wheel_path: Path) -> None:
    with tempfile.TemporaryDirectory(
        prefix=".motorbridge-smoke-",
        dir=BUILD_ROOT,
    ) as temp_name:
        venv_dir = Path(temp_name) / "venv"
        venv.EnvBuilder(with_pip=True).create(venv_dir)
        python = venv_dir / ("Scripts/python.exe" if os.name == "nt" else "bin/python")
        _run(
            [
                str(python),
                "-m",
                "pip",
                "install",
                "--no-deps",
                str(wheel_path),
            ]
        )
        smoke = (
            "import importlib.util, pathlib; "
            f"p=pathlib.Path({str(Path(__file__).resolve())!r}); "
            "s=importlib.util.spec_from_file_location('mb_setup', p); "
            "m=importlib.util.module_from_spec(s); s.loader.exec_module(m); "
            "import motorbridge; m.validate_runtime_contract(motorbridge); "
            "d=pathlib.Path(motorbridge.__file__).resolve().parent; "
            "assert any((d/'lib').glob('libmotor_abi.*')); "
            "assert (d/'bin'/'ws_gateway').is_file() or "
            "(d/'bin'/'ws_gateway.exe').is_file(); "
            "print(motorbridge.__version__)"
        )
        _run([str(python), "-c", smoke])


def build_patched_wheel() -> Path:
    _verify_patch_file()
    _prepare_source_checkout()
    _verify_dm_serial_timeout_budget()
    abi_path, gateway_path = _build_rust_artifacts()
    wheel_path = _build_wheel(abi_path, gateway_path)
    _smoke_test_wheel(wheel_path)
    return wheel_path


def _check_installed() -> None:
    module = importlib.import_module("motorbridge")
    validate_runtime_contract(module)
    print(
        "MotorBridge runtime contract OK: "
        f"version={module.__version__} feedback_sequence=true"
    )


def _install_user(wheel_path: Path) -> None:
    _run(
        [
            sys.executable,
            "-m",
            "pip",
            "install",
            "--user",
            "--break-system-packages",
            "--force-reinstall",
            "--no-deps",
            str(wheel_path),
        ]
    )
    _check_installed()


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build or validate the pinned MotorBridge freshness patch"
    )
    modes = parser.add_mutually_exclusive_group(required=True)
    modes.add_argument(
        "--build-only",
        action="store_true",
        help="build and smoke-test the wheel without changing user Python",
    )
    modes.add_argument(
        "--install-user",
        action="store_true",
        help="build, smoke-test, then explicitly replace the user package",
    )
    modes.add_argument(
        "--check-installed",
        action="store_true",
        help="validate the installed module without network or building",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)
    try:
        if args.check_installed:
            _check_installed()
            return 0
        wheel_path = build_patched_wheel()
        print(f"Verified patched wheel: {wheel_path}")
        if args.install_user:
            _install_user(wheel_path)
        else:
            print("User Python was not modified (--build-only).")
        return 0
    except (ImportError, RuntimeError, subprocess.CalledProcessError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

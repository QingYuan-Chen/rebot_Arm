from __future__ import annotations

import importlib.util
import hashlib
from pathlib import Path
import subprocess
from types import SimpleNamespace

import pytest


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "scripts/setup_motorbridge_fresh_feedback.py"
SPEC = importlib.util.spec_from_file_location(
    "setup_motorbridge_fresh_feedback",
    MODULE_PATH,
)
assert SPEC is not None and SPEC.loader is not None
SETUP = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SETUP)


def _git(repo: Path, *args: str) -> str:
    result = subprocess.run(
        ["git", "-C", str(repo), *args],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    return result.stdout.strip()


def _configure_checkout(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> tuple[Path, Path]:
    source = tmp_path / "source"
    source.mkdir()
    _git(source, "init", "-q")
    _git(source, "config", "user.name", "MotorBridge Setup Test")
    _git(source, "config", "user.email", "motorbridge-setup@example.invalid")
    (source / ".gitignore").write_text("*.ignored\n", encoding="utf-8")
    tracked = source / "state.txt"
    tracked.write_text("base\n", encoding="utf-8")
    _git(source, "add", ".gitignore", "state.txt")
    _git(source, "commit", "-qm", "base")
    commit = _git(source, "rev-parse", "HEAD")
    origin = "https://example.invalid/motorbridge.git"
    _git(source, "remote", "add", "origin", origin)

    tracked.write_text("patched\n", encoding="utf-8")
    patch_path = tmp_path / "feedback.patch"
    patch_path.write_text(_git(source, "diff", "--binary") + "\n", encoding="utf-8")
    _git(source, "restore", "state.txt")

    monkeypatch.setattr(SETUP, "BUILD_ROOT", tmp_path / "build")
    monkeypatch.setattr(SETUP, "SOURCE_DIR", source)
    monkeypatch.setattr(SETUP, "PATCH_PATH", patch_path)
    monkeypatch.setattr(SETUP, "UPSTREAM_COMMIT", commit)
    monkeypatch.setattr(SETUP, "UPSTREAM_URL", origin)
    monkeypatch.setattr(
        SETUP,
        "PATCH_SHA256",
        hashlib.sha256(patch_path.read_bytes()).hexdigest(),
    )
    return source, tracked


def _module_with_contract(
    *,
    method_present: bool,
    feedback_sequence: bool | None,
    version: str = "0.4.7+rebotarm.1",
) -> SimpleNamespace:
    motor = type("Motor", (), {})
    if method_present:
        motor.get_state_with_sequence = lambda self: (None, 0)

    features = {}
    if feedback_sequence is not None:
        features["feedback_sequence"] = feedback_sequence
    return SimpleNamespace(
        __version__=version,
        Motor=motor,
        abi_capabilities=lambda: {"features": features},
    )


def test_runtime_contract_rejects_missing_sequence_method() -> None:
    module = _module_with_contract(
        method_present=False,
        feedback_sequence=True,
    )

    with pytest.raises(RuntimeError, match="Motor.get_state_with_sequence"):
        SETUP.validate_runtime_contract(module)


def test_runtime_contract_rejects_missing_sequence_capability() -> None:
    module = _module_with_contract(
        method_present=True,
        feedback_sequence=None,
    )

    with pytest.raises(RuntimeError, match="features.feedback_sequence"):
        SETUP.validate_runtime_contract(module)


def test_runtime_contract_accepts_patched_api_and_capability() -> None:
    module = _module_with_contract(
        method_present=True,
        feedback_sequence=True,
    )

    assert SETUP.validate_runtime_contract(module) is None


def test_runtime_contract_rejects_unexpected_package_version() -> None:
    module = _module_with_contract(
        method_present=True,
        feedback_sequence=True,
        version="0.4.6",
    )

    with pytest.raises(RuntimeError, match="0.4.7\\+rebotarm.1"):
        SETUP.validate_runtime_contract(module)


def test_upstream_dm_serial_timeout_budget_is_verified(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    source = tmp_path / "source"
    dm_serial = source / "motor_core/src/dm_serial.rs"
    dm_serial.parent.mkdir(parents=True)
    dm_serial.write_text(
        ".timeout(Duration::from_millis(10))\n",
        encoding="utf-8",
    )
    monkeypatch.setattr(SETUP, "SOURCE_DIR", source)

    assert SETUP._verify_dm_serial_timeout_budget() is None

    dm_serial.write_text(
        ".timeout(Duration::from_millis(1))\n",
        encoding="utf-8",
    )
    with pytest.raises(RuntimeError, match="10 ms"):
        SETUP._verify_dm_serial_timeout_budget()



def test_patch_digest_mismatch_is_rejected(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    patch_path = tmp_path / "feedback.patch"
    patch_path.write_text("unexpected patch\n", encoding="utf-8")
    monkeypatch.setattr(SETUP, "PATCH_PATH", patch_path)
    monkeypatch.setattr(SETUP, "PATCH_SHA256", "0" * 64)

    with pytest.raises(RuntimeError, match="digest"):
        SETUP._verify_patch_file()


def test_checked_in_patch_digest_matches_setup_contract() -> None:
    assert SETUP._verify_patch_file() is None


def test_patch_artifact_passes_git_diff_check(tmp_path: Path) -> None:
    repo = tmp_path / "repo"
    repo.mkdir()
    _git(repo, "init", "-q")
    patch_copy = repo / "feedback.patch"
    patch_copy.write_bytes(SETUP.PATCH_PATH.read_bytes())
    _git(repo, "add", "feedback.patch")

    result = subprocess.run(
        ["git", "-C", str(repo), "diff", "--cached", "--check"],
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    assert result.returncode == 0, result.stdout + result.stderr


def test_checkout_rejects_wrong_head(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    _configure_checkout(monkeypatch, tmp_path)
    monkeypatch.setattr(SETUP, "UPSTREAM_COMMIT", "0" * 40)

    with pytest.raises(RuntimeError, match="unexpected HEAD"):
        SETUP._prepare_source_checkout()


def test_checkout_rejects_wrong_origin(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    _configure_checkout(monkeypatch, tmp_path)
    monkeypatch.setattr(SETUP, "UPSTREAM_URL", "https://wrong.invalid/repo.git")

    with pytest.raises(RuntimeError, match="unexpected origin"):
        SETUP._prepare_source_checkout()


def test_already_patched_checkout_is_accepted_idempotently(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    _, tracked = _configure_checkout(monkeypatch, tmp_path)

    SETUP._prepare_source_checkout()
    SETUP._prepare_source_checkout()

    assert tracked.read_text(encoding="utf-8") == "patched\n"


def test_checkout_rejects_additional_tracked_change(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    _, tracked = _configure_checkout(monkeypatch, tmp_path)
    SETUP._prepare_source_checkout()
    tracked.write_text("patched plus unrelated change\n", encoding="utf-8")

    with pytest.raises(RuntimeError, match="Refusing dirty"):
        SETUP._prepare_source_checkout()


@pytest.mark.parametrize("unexpected_name", ["extra.txt", "extra.ignored"])
def test_checkout_rejects_all_additional_untracked_files(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    unexpected_name: str,
) -> None:
    source, _ = _configure_checkout(monkeypatch, tmp_path)
    SETUP._prepare_source_checkout()
    (source / unexpected_name).write_text("unexpected\n", encoding="utf-8")

    with pytest.raises(RuntimeError, match="Refusing dirty"):
        SETUP._prepare_source_checkout()


def test_check_installed_mode_never_builds_or_installs(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    checked = []
    monkeypatch.setattr(SETUP, "_check_installed", lambda: checked.append(True))
    monkeypatch.setattr(
        SETUP,
        "build_patched_wheel",
        lambda: pytest.fail("check mode must not build"),
    )
    monkeypatch.setattr(
        SETUP,
        "_install_user",
        lambda wheel: pytest.fail("check mode must not install"),
    )

    assert SETUP.main(["--check-installed"]) == 0
    assert checked == [True]


@pytest.mark.parametrize(
    "arguments",
    [[], ["--build-only", "--check-installed"]],
)
def test_cli_rejects_missing_or_multiple_modes(arguments: list[str]) -> None:
    with pytest.raises(SystemExit) as exc_info:
        SETUP._parse_args(arguments)

    assert exc_info.value.code == 2


def test_install_mode_passes_successfully_built_wheel_to_installer(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    wheel = tmp_path / "verified.whl"
    installed = []
    monkeypatch.setattr(SETUP, "build_patched_wheel", lambda: wheel)
    monkeypatch.setattr(SETUP, "_install_user", lambda path: installed.append(path))

    assert SETUP.main(["--install-user"]) == 0
    assert installed == [wheel]


def test_install_mode_does_not_install_after_failed_build(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    installed = []

    def fail_build() -> Path:
        raise RuntimeError("build failed")

    monkeypatch.setattr(SETUP, "build_patched_wheel", fail_build)
    monkeypatch.setattr(SETUP, "_install_user", lambda path: installed.append(path))

    assert SETUP.main(["--install-user"]) == 1
    assert installed == []


def test_build_patched_wheel_runs_all_gates_and_smoke_in_order(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    abi = tmp_path / "libmotor_abi.so"
    gateway = tmp_path / "ws_gateway"
    wheel = tmp_path / "motorbridge.whl"
    events = []
    monkeypatch.setattr(SETUP, "_verify_dm_serial_timeout_budget", lambda: events.append("timeout"))
    monkeypatch.setattr(SETUP, "_verify_patch_file", lambda: events.append("verify"))
    monkeypatch.setattr(
        SETUP,
        "_prepare_source_checkout",
        lambda: events.append("checkout"),
    )

    def build_rust() -> tuple[Path, Path]:
        events.append("rust")
        return abi, gateway

    def build_wheel(received_abi: Path, received_gateway: Path) -> Path:
        assert (received_abi, received_gateway) == (abi, gateway)
        events.append("wheel")
        return wheel

    def smoke(received_wheel: Path) -> None:
        assert received_wheel == wheel
        events.append("smoke")

    monkeypatch.setattr(SETUP, "_build_rust_artifacts", build_rust)
    monkeypatch.setattr(SETUP, "_build_wheel", build_wheel)
    monkeypatch.setattr(SETUP, "_smoke_test_wheel", smoke)

    assert SETUP.build_patched_wheel() == wheel
    assert events == ["verify", "checkout", "timeout", "rust", "wheel", "smoke"]


def test_build_patched_wheel_propagates_smoke_failure(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    wheel = tmp_path / "motorbridge.whl"
    monkeypatch.setattr(SETUP, "_verify_dm_serial_timeout_budget", lambda: None)
    monkeypatch.setattr(SETUP, "_verify_patch_file", lambda: None)
    monkeypatch.setattr(SETUP, "_prepare_source_checkout", lambda: None)
    monkeypatch.setattr(
        SETUP,
        "_build_rust_artifacts",
        lambda: (tmp_path / "libmotor_abi.so", tmp_path / "ws_gateway"),
    )
    monkeypatch.setattr(SETUP, "_build_wheel", lambda abi, gateway: wheel)

    def fail_smoke(received_wheel: Path) -> None:
        assert received_wheel == wheel
        raise RuntimeError("smoke failed")

    monkeypatch.setattr(SETUP, "_smoke_test_wheel", fail_smoke)

    with pytest.raises(RuntimeError, match="smoke failed"):
        SETUP.build_patched_wheel()


def test_install_mode_does_not_install_when_real_build_smoke_fails(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    wheel = tmp_path / "motorbridge.whl"
    monkeypatch.setattr(SETUP, "_verify_dm_serial_timeout_budget", lambda: None)
    installed = []
    monkeypatch.setattr(SETUP, "_verify_patch_file", lambda: None)
    monkeypatch.setattr(SETUP, "_prepare_source_checkout", lambda: None)
    monkeypatch.setattr(
        SETUP,
        "_build_rust_artifacts",
        lambda: (tmp_path / "libmotor_abi.so", tmp_path / "ws_gateway"),
    )
    monkeypatch.setattr(SETUP, "_build_wheel", lambda abi, gateway: wheel)

    def fail_smoke(received_wheel: Path) -> None:
        assert received_wheel == wheel
        raise RuntimeError("smoke failed")

    monkeypatch.setattr(SETUP, "_smoke_test_wheel", fail_smoke)
    monkeypatch.setattr(SETUP, "_install_user", lambda path: installed.append(path))

    assert SETUP.main(["--install-user"]) == 1
    assert installed == []


def test_build_only_mode_never_installs(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    wheel = tmp_path / "verified.whl"
    monkeypatch.setattr(SETUP, "build_patched_wheel", lambda: wheel)
    monkeypatch.setattr(
        SETUP,
        "_install_user",
        lambda path: pytest.fail("build-only mode must not install"),
    )

    assert SETUP.main(["--build-only"]) == 0

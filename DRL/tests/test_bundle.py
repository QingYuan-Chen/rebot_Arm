import tarfile
from pathlib import Path

from rebotarm_drl.bundle import build_bundle, verify_bundle


def test_bundle_preserves_sources_but_excludes_credentials_env_and_runs(tmp_path):
    root = tmp_path / "DRL"
    for name in ["README.md", "pyproject.toml", "uv.lock", "src/rebotarm_drl/task.py",
                 "scripts/setup.sh", "tests/test_task.py", "remote.local.json",
                 ".venv/token", ".cache/key", "runs/model.pt", ".env"]:
        p = root / name
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(name)
    dest = tmp_path / "bundle.tar.gz"
    build_bundle(root, dest)
    with tarfile.open(dest) as archive:
        names = archive.getnames()
    assert "DRL/src/rebotarm_drl/task.py" in names
    assert not any(x.endswith("remote.local.json") or "/.venv/" in x or "/runs/" in x or x.endswith(".env") for x in names)
    verify_bundle(dest)


def test_bundle_does_not_follow_symlink_outside_root(tmp_path):
    import pytest
    root = tmp_path / "DRL"
    (root / "src").mkdir(parents=True)
    secret = tmp_path / "secret"
    secret.write_text("private")
    (root / "src/escape").symlink_to(secret)
    with pytest.raises(ValueError, match="symlink"):
        build_bundle(root, tmp_path / "bundle.tar.gz")

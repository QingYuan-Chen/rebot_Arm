"""Portable source/wheel bundles. Never archive a non-relocatable .venv."""
import io
import json
import tarfile
from pathlib import Path, PurePosixPath

from .contracts import sha256

ROOT_FILES = {"README.md", "pyproject.toml", "uv.lock", "requirements.lock", "COLCON_IGNORE", "LICENSE", "THIRD_PARTY_NOTICES.md"}
ROOT_DIRS = {"src", "scripts", "tests"}


def build_bundle(root, destination, *, with_wheels=False):
    root, destination = Path(root).resolve(), Path(destination).resolve()
    if destination.exists():
        raise FileExistsError(destination)
    dirs = ROOT_DIRS | ({"wheelhouse"} if with_wheels else set())
    files = []
    if with_wheels and not any((root / "wheelhouse").glob("*.whl")):
        raise ValueError("offline bundle requires wheelhouse")
    candidates = [root / name for name in ROOT_FILES if (root / name).exists()]
    for name in sorted(dirs):
        directory = root / name
        if directory.is_symlink():
            raise ValueError(f"bundle refuses symlink: {name}")
        if directory.exists():
            candidates.extend(directory.rglob("*"))
    for path in sorted(candidates):
        relative = path.relative_to(root)
        if relative.parts[0] not in dirs and relative.as_posix() not in ROOT_FILES:
            continue
        if any(p in {"__pycache__", ".pytest_cache"} or p.endswith(".egg-info") for p in relative.parts):
            continue
        if path.suffix in {".pyc", ".pyo"}:
            continue
        if path.is_symlink():
            raise ValueError(f"bundle refuses symlink: {relative}")
        if path.is_file():
            files.append((path, relative.as_posix()))
    manifest = {"schema_version": 1, "with_wheels": with_wheels,
                "files": {rel: sha256(path) for path, rel in files}}
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tarfile.open(destination, "w:gz", compresslevel=1) as archive:
        for path, rel in files:
            archive.add(path, arcname=f"DRL/{rel}", recursive=False)
        content = json.dumps(manifest, indent=2).encode()
        entry = tarfile.TarInfo("DRL/BUNDLE_MANIFEST.json")
        entry.size = len(content)
        archive.addfile(entry, io.BytesIO(content))
    checksum = sha256(destination)
    destination.with_name(destination.name + ".sha256").write_text(f"{checksum}  {destination.name}\n")
    verify_bundle(destination)
    return manifest


def verify_bundle(path):
    import hashlib
    with tarfile.open(path) as archive:
        members = archive.getmembers()
        names = [x.name for x in members]
        if len(set(names)) != len(names):
            raise ValueError("duplicate archive members")
        for item in members:
            p = PurePosixPath(item.name)
            if not item.isfile() or p.is_absolute() or ".." in p.parts or p.parts[0] != "DRL":
                raise ValueError("unsafe archive member")
        manifest = json.load(archive.extractfile("DRL/BUNDLE_MANIFEST.json"))
        expected = {"DRL/" + p for p in manifest["files"]} | {"DRL/BUNDLE_MANIFEST.json"}
        if set(names) != expected:
            raise ValueError("archive inventory mismatch")
        for name, digest in manifest["files"].items():
            h = hashlib.sha256()
            with archive.extractfile("DRL/" + name) as source:
                for data in iter(lambda: source.read(1024 * 1024), b""):
                    h.update(data)
            if h.hexdigest() != digest:
                raise ValueError(f"archive checksum mismatch: {name}")
    return manifest

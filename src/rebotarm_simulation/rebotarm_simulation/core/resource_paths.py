"""One package-resource resolver for source checkouts and installed deployments.

Priority: checkout sibling packages, Python prefix share, AMENT_PREFIX_PATH,
then the optional ament index. Explicit model filenames bypass discovery.
"""
from pathlib import Path
import os
import sys


def _source_package_root() -> Path | None:
    for parent in Path(__file__).resolve().parents:
        if (parent / "package.xml").is_file() and (parent / "rebotarm_simulation/__init__.py").is_file():
            return parent
    return None


def source_repository_root() -> Path:
    root = _source_package_root()
    if root is not None and (root.parent.parent / "src/rebotarm_moveit_config").is_dir():
        return root.parent.parent
    raise FileNotFoundError("model generation needs a checkout; pass --repo-root explicitly")


def package_resource(package: str, relative: str) -> Path:
    source = _source_package_root()
    candidates = []
    if source is not None:
        candidates.append(source.parent / package / relative)
    prefixes = [sys.prefix, *filter(None, os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep))]
    candidates.extend(Path(prefix) / "share" / package / relative for prefix in prefixes)
    for candidate in candidates:
        if candidate.exists():
            return candidate
    try:
        from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
    except ImportError:
        pass
    else:
        try:
            candidate = Path(get_package_share_directory(package)) / relative
            candidates.append(candidate)
            if candidate.exists():
                return candidate
        except PackageNotFoundError:
            pass
    raise FileNotFoundError(f"Cannot locate {package}/{relative}; searched: {candidates}")


def model_resource(relative: str = "models/rebotarm/scene.xml", *, explicit=None) -> Path:
    path = Path(explicit).expanduser() if explicit is not None else package_resource("rebotarm_simulation", relative)
    if not path.is_file():
        raise FileNotFoundError(path)
    return path.resolve()

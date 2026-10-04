"""Explicitly snapshot canonical model assets for self-contained remote execution."""
import hashlib
import json
import shutil
import subprocess
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parent
SOURCE = REPO / "src/rebotarm_simulation/models/rebotarm"
DEST = ROOT / "src/rebotarm_drl/assets/rebotarm"


def main():
    names = ["robot.xml"]
    names += [mesh.attrib["file"] for mesh in ET.parse(SOURCE / "robot.xml").findall("./asset/mesh")]
    names += ["assets/gemini2/ORBBEC_LICENSE.txt", "assets/gemini2/SEEED_LICENSE.txt", "assets/gemini2/sources.json"]
    manifest = {"source_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=REPO, text=True).strip(),
                "source_path": str(SOURCE.relative_to(REPO)), "files": {}}
    for name in names:
        source, target = SOURCE / name, DEST / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, target)
        manifest["files"][name] = hashlib.sha256(source.read_bytes()).hexdigest()
    (DEST / "provenance.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"snapshotted {len(names)} files into {DEST}")


if __name__ == "__main__":
    main()

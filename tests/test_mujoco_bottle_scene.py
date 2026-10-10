from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "src/rebotarm_simulation/models/rebotarm/scene.xml"


def test_default_tabletop_has_no_bottle_and_retains_table():
    root = ET.parse(SCENE).getroot()
    assert root.find(".//body[@name='table']") is not None
    assert root.find(".//body[@name='bottle']") is None
    assert root.find(".//body[@name='test_cube']/freejoint") is not None
    for key in root.findall('keyframe/key'):
        assert len(key.get('qpos').split()) == 15

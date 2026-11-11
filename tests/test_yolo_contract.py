import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest


def test_requested_open_vocabulary_initialization_failure_is_visible(monkeypatch):
    class Model:
        def set_classes(self, _classes):
            raise RuntimeError('prompt setup failed')
    monkeypatch.setitem(sys.modules, 'ultralytics', SimpleNamespace(YOLO=lambda _path: Model()))
    path = Path(__file__).resolve().parents[1] / 'src/rebotarm_vision/rebotarm_vision/detector/yolo_detector.py'
    spec = importlib.util.spec_from_file_location('audit_yolo_detector', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    with pytest.raises(RuntimeError, match='prompt setup failed'):
        module.YoloDetector('model.pt', 'cpu', .4, .5, True, ['cup'])
    with pytest.raises(ValueError, match='requires custom classes'):
        module.YoloDetector('model.pt', 'cpu', .4, .5, True, [])

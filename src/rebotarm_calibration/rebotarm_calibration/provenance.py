"""对实际加载目录的源码计算指纹，覆盖未提交修改及安装包，记录标定运行来源。"""
import hashlib
from pathlib import Path
import platform
import cv2
import numpy as np


def runtime_provenance(settings):
    root = Path(__file__).parent
    hashes = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
              for p in sorted(root.glob('*.py'))}
    return {'source_sha256': hashes, 'python_version': platform.python_version(),
            'opencv_version': cv2.__version__, 'numpy_version': np.__version__,
            'capture_settings': {k: v for k, v in settings.items() if k != 'session_directory'}}


def dataset_fingerprint(dataset):
    """摘要绑定采集元数据及两组原样本；审核状态、报告和排除列表由工作流另行记录。"""
    import json
    evidence = {key: dataset[key] for key in
                ('schema_version', 'metadata', 'training_samples', 'validation_samples')}
    encoded = json.dumps(evidence, sort_keys=True, separators=(',', ':'),
                         ensure_ascii=False, allow_nan=False).encode('utf-8')
    return hashlib.sha256(encoded).hexdigest()

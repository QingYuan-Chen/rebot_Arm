from pathlib import Path
from types import SimpleNamespace
import ast
import pytest
from rebotarm_simulation.ros.message_codec import trajectory_to_sampler
from rebotarm_simulation.apps import viewer_lifecycle

ROOT=Path(__file__).resolve().parents[1]/'src/rebotarm_simulation/rebotarm_simulation'


def test_execution_has_no_ros_message_conversion_or_old_runtime():
    assert not (ROOT/'execution/runtime.py').exists()
    for path in (ROOT/'execution').glob('*.py'):
        source=path.read_text()
        for node in ast.walk(ast.parse(source)):
            if isinstance(node,ast.ImportFrom):
                assert not (node.module or '').startswith(('rclpy','rebotarm_simulation.ros'))
        assert 'nanosec' not in source or path.name == 'offline_trajectory.py'


def test_ros_codec_preserves_partial_joint_initial_values_and_rejects_limits():
    p=SimpleNamespace(positions=[.4],time_from_start=SimpleNamespace(sec=1,nanosec=0))
    message=SimpleNamespace(joint_names=['joint2'],points=[p])
    sampler=trajectory_to_sampler(message,initial_positions=[.1]*6)
    assert sampler.sample(.5)==pytest.approx((.1,.25,.1,.1,.1,.1))
    with pytest.raises(ValueError,match='duration'):
        trajectory_to_sampler(message,initial_positions=[.1]*6,max_duration_sec=.5)
    with pytest.raises(ValueError,match='too many'):
        trajectory_to_sampler(SimpleNamespace(joint_names=['joint2'],points=[p,p]),initial_positions=[.1]*6,max_points=1)


def test_viewer_close_failure_retains_ownership_until_native_release():
    events=[]
    def fail():raise RuntimeError('native close failure')
    viewer=SimpleNamespace(m=object(),close=fail)
    sim=SimpleNamespace(close=lambda:events.append('sim close'))
    retained=[]
    original=viewer_lifecycle._RETAINED_UNSAFE_VIEWERS
    viewer_lifecycle._RETAINED_UNSAFE_VIEWERS=retained
    try:
        with pytest.raises(RuntimeError):
            viewer_lifecycle.close_passive_viewer_safely(viewer,sim,object(),object())
        assert len(retained)==1 and events==[]
        viewer.m=None
        with pytest.raises(RuntimeError):
            viewer_lifecycle.close_passive_viewer_safely(viewer,sim,object(),object())
        assert events==['sim close']
    finally:
        viewer_lifecycle._RETAINED_UNSAFE_VIEWERS=original

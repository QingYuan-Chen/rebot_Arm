from pathlib import Path
from types import SimpleNamespace
import ast
import pytest
from rebotarm_simulation.ros.message_codec import trajectory_points
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


def test_ros_codec_requires_complete_six_axes_and_rejects_limits():
    p=SimpleNamespace(positions=[.4]*6, velocities=[], accelerations=[],
        time_from_start=SimpleNamespace(sec=1,nanosec=0))
    message=SimpleNamespace(joint_names=[f'joint{i}' for i in range(1,7)],points=[p])
    names, points = trajectory_points(message)
    assert names == tuple(message.joint_names) and points[0].positions == (.4,)*6
    with pytest.raises(ValueError,match='exactly'):
        trajectory_points(SimpleNamespace(joint_names=['joint2'],points=[p]))
    with pytest.raises(ValueError,match='duration'):
        trajectory_points(message,max_duration_sec=.5)
    with pytest.raises(ValueError,match='too many'):
        trajectory_points(SimpleNamespace(joint_names=message.joint_names,points=[p,p]),max_points=1)


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

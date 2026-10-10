"""仿真初始姿态使用 safe_home，且不再保留 ready 配置。"""
from pathlib import Path
from types import SimpleNamespace as NS
import yaml

ROOT = Path(__file__).resolve().parents[1]

def test_simulation_initial_pose_matches_safe_home():
    profile = yaml.safe_load((ROOT/'src/rebotarm_motion/config/visual_motion_profile.yaml').read_text())['launch']
    from rebotarm_preview.preview_config import SAFE_HOME_JOINT_POSITIONS
    assert profile['sim_initial_joint_positions'] == list(SAFE_HOME_JOINT_POSITIONS)
    assert not any('ready' in key for key in profile)
    launch = (ROOT/'src/rebotarm_bringup/launch/includes/visual_backend.launch.py').read_text()
    assert '"initial_joint_positions": sim_initial_joint_positions' in launch

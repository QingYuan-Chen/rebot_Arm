from pathlib import Path


ROOT = Path(__file__).parents[1]
README = ROOT / "src" / "rebotarm_simulation" / "README_mujoco.md"
COMMANDS = ROOT / "docs" / "reference/commands/rebotarm_feature_commands.md"
SIM2REAL = ROOT / "docs" / "sim2real_workflow_zh.md"
PICK = ROOT / "docs" / "mujoco_pick_zh.md"
REAL2SIM = ROOT / "docs" / "real2sim_bridge_zh.md"


def _text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_mujoco_readme_documents_reproducible_install_and_runtime_commands():
    text = _text(README)
    required = (
        "Ubuntu 24.04",
        "ROS 2 Jazzy",
        "Python 3.12",
        "python3 -m venv --system-site-packages third_party/rebotarm_mujoco_venv",
        "source /opt/ros/jazzy/setup.bash",
        "python3 scripts/install_python_dependencies.py mujoco --python third_party/rebotarm_mujoco_venv/bin/python",
        "/usr/bin/python3 -m colcon build --symlink-install --packages-select rebotarm_simulation",
        "MUJOCO_GL=egl",
        "rebotarm_mujoco_health --renderer-timeout",
        "rebotarm_mujoco_acceptance --skip-renderer",
        "rebotarm_mujoco_cli run --duration",
        "rebotarm_mujoco_contact_check",
        '"requested_duration"',
        '"achieved_duration"',
        "DISPLAY",
        "XAUTHORITY",
        "rebotarm_mujoco_viewer --duration",
        "joints 0.0 -0.8 -1.0 0.3 0.0 0.0",
        "--no-command-input",
        "PYTHONPATH=src/rebotarm_simulation third_party/rebotarm_mujoco_venv/bin/python -m rebotarm_simulation.mujoco_health",
        "PYTHONPATH=src/rebotarm_simulation third_party/rebotarm_mujoco_venv/bin/python -m rebotarm_simulation.mujoco_cli",
        "PYTHONPATH=src/rebotarm_simulation third_party/rebotarm_mujoco_venv/bin/python -m rebotarm_simulation.mujoco_viewer",
    )
    for value in required:
        assert value in text
    assert ".venv-mujoco-ros" not in text


def test_mujoco_readme_documents_ros_interfaces_examples_and_moveit_safety():
    text = _text(README)
    required = (
        "ros2 launch rebotarm_simulation mujoco_sim.launch.py",
        "/rebotarm/follow_joint_trajectory",
        "/rebotarm/joint_states",
        "/rebotarm/gripper/set",
        "/rebotarm/gripper/state",
        "/rebotarm/trajectory_stop",
        "/rebotarm/sim/set_mode",
        "/diagnostics",
        "/clock",
        "ros2 action send_goal",
        "ros2 service call /rebotarm/gripper/set",
        "ros2 service call /rebotarm/trajectory_stop",
        "rebotarm_mujoco_ros_acceptance --timeout 15",
        "rebotarm_mujoco_moveit_acceptance --timeout 30",
        "use_hardware:=false",
        "MoveIt",
        "ros2 launch rebotarm_simulation mujoco_moveit_sim.launch.py use_sim_time:=true",
        "use_rviz:=false",
        "fake joint state publisher",
    )
    for value in required:
        assert value in text


def test_mujoco_readme_documents_api_architecture_sync_and_troubleshooting():
    text = _text(README)
    required = (
        "RebotArmMujoco",
        "command_joint_positions",
        "command_gripper_width",
        "get_control_status",
        "get_state()",
        "XYZW",
        "save_state()",
        "restore_state()",
        "set_object_pose",
        "randomize_scene()",
        "RebotArmReachEnv",
        "reset(seed=7)",
        "step([0.0] * 7)",
        "step_done([0.0] * 7)",
        "RandomizationConfig",
        "TrajectoryRecorder",
        "training_profile",
        "sample_from_last_step",
        "rebotarm_mujoco_batch --episodes",
        "云服务器",
        "本地 Ubuntu VM",
        "Gymnasium",
        "训练",
        "重复节点",
        "SSH",
        "EGL",
        "goal tolerance",
        "self-collision",
        "use_hardware",
        "rebotarmcontroller",
        "CAN",
        "串口",
        "真实夹爪",
        "shebang",
        "仅结束 launch 父进程",
        "ros2 node list",
        "pgrep -af rebotarm_mujoco_node",
        "ps -fp <confirmed-pid>",
        "kill -TERM <confirmed-pid>",
    )
    for value in required:
        assert value in text
    assert "pkill" not in text
    assert "Gymnasium CPU Reach" in text
    assert "不代表 GPU PPO 训练质量或实机验收" in text


def test_feature_commands_links_to_mujoco_readme_and_safe_entry_points():
    text = _text(COMMANDS)
    assert "../../../src/rebotarm_simulation/README_mujoco.md" in text
    assert "ros2 launch rebotarm_simulation mujoco_sim.launch.py" in text
    assert "rebotarm_mujoco_cli run --duration" in text
    assert "mujoco_rviz_viewer.launch.py" in text


def test_mujoco_readme_documents_local_urdf_to_mjcf_workflow() -> None:
    text = _text(README)
    required = (
        "src/rebotarm_moveit_config/config/rebotarm.urdf",
        "src/rebotarm_simulation/models/rebotarm/robot.xml",
        "rebotarm_urdf_to_mjcf --repo-root",
        "rebotarm_urdf_to_mjcf --repo-root . --check",
        "mujoco>=3.3,<4",
        "原始 STL",
        "sha256sum",
        "不连接实机",
    )
    for value in required:
        assert value in text


def test_sim2real_documentation_has_complete_no_hardware_workflow() -> None:
    readme = _text(README)
    text = _text(SIM2REAL)
    assert "../../docs/sim2real_workflow_zh.md" in readme
    for value in (
        "不连接机械臂实机",
        "rebotarm_simulation.sim2real_cli rollout",
        "--randomization-profile training_profile",
        "--record logs/sim2real/seed-7-reference.jsonl",
        "rebotarm_simulation.sim2real_cli replay",
        "rebotarm_simulation.sim2real_cli compare",
        "rebotarm_simulation.sim2real_cli batch-check",
        "seed_reproducible",
        "joint_limit",
        "actuator_torque",
        "contact_penetration",
        "source` 设置为 `real",
    ):
        assert value in text


def test_pick_documentation_defines_task_contract_and_acceptance() -> None:
    readme = _text(README)
    text = _text(PICK)
    assert "../../docs/mujoco_pick_zh.md" in readme
    for value in (
        "不进行强化学习训练，也不连接实机",
        "RebotArmPickEnv",
        "rebotarm_simulation.mujoco_pick_batch",
        "force_closure_candidate",
        "dropped_after_grasp",
        "success_rate=0",
        "脚本专家或 IK 抓取基线",
    ):
        assert value in text


def test_real2sim_documentation_defines_read_only_bridge_and_viewer() -> None:
    readme = _text(README)
    text = _text(REAL2SIM)
    assert "../../docs/real2sim_bridge_zh.md" in readme
    for value in (
        "默认只读 ROS 2 状态",
        "/real2sim/joint_states",
        "real2sim_mapping.yaml",
        "real2sim_acceptance",
        "real2sim_bridge.launch.py",
        "real2sim_viewer",
        "mode:=mirror",
        "不把 MuJoCo 接触力反馈给实机电机",
    ):
        assert value in text

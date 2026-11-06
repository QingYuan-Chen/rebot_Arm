"""MoveIt 规划请求显式起点验证。"""
from types import SimpleNamespace
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from rebotarm_motion.moveit_planner import MoveItMotionPlanner
NAMES = [f"joint{i}" for i in range(1, 7)]

def test_moveit_request_uses_explicit_full_preview_start():
    planner = object.__new__(MoveItMotionPlanner)
    planner._start_state_provider = None
    planner._client = SimpleNamespace(wait_for_service=lambda **kwargs: True)
    planner._group_name = "arm"
    planner._planning_pipeline = "ompl"
    planner._planner_id = ""
    planner._num_attempts = 1
    planner._planning_time = 2.0
    planner._build_pose_constraints = lambda pose: SimpleNamespace()
    planner._call_plan_service = lambda request: request
    start = JointState()
    start.name = NAMES
    start.position = [0.2] * 6

    request = planner.plan_pose(PoseStamped(), start_joint_state=start)
    # 显式六轴起点仍以 diff 合入场景，保留真实手指和附着物。
    assert request.motion_plan_request.start_state.is_diff
    assert list(request.motion_plan_request.start_state.joint_state.position) == [0.2] * 6


"""按同一规划场景快照检查轨迹，返回精确系数，不拥有执行资源。"""

import hashlib
import copy
import json
import math
from pathlib import Path
import threading
import time

from .polynomial_trajectory import JOINT_NAMES, TrajectoryPoint
from .trajectory_preparation import PreparationContext, analyze_trajectory, load_reference_limits


def scene_revision(message):
    """仅绑定碰撞环境；反馈关节和消息时间不构成场景版本。"""
    from rosidl_runtime_py.convert import message_to_ordereddict

    data = message_to_ordereddict(message)
    selected = {name: data[name] for name in (
        'world', 'allowed_collision_matrix', 'fixed_frame_transforms', 'link_padding', 'link_scale')}
    selected['attached_collision_objects'] = data['robot_state']['attached_collision_objects']

    def without_stamps(value):
        if isinstance(value, dict):
            return {key: without_stamps(item) for key, item in value.items() if key != 'stamp'}
        if isinstance(value, list):
            return [without_stamps(item) for item in value]
        return value

    return hashlib.sha256(json.dumps(without_stamps(selected), sort_keys=True,
        separators=(',', ':'), allow_nan=False).encode()).hexdigest()


class SceneCollisionChecker:
    """一个 read_only 临界区覆盖全部采样；RobotState 是局部副本。"""

    def __init__(self, monitor):
        self.monitor = monitor

    def prepare(self, request, lower, upper, limits):
        now = time.monotonic()
        if not math.isfinite(request.deadline_monotonic) or now >= request.deadline_monotonic:
            raise ValueError('trajectory preparation deadline expired')
        context = PreparationContext(request.session, request.generation,
            tuple(request.hold_positions), tuple(request.feedback_positions),
            request.feedback_observed_at, request.stationary, request.enabled)
        fingers = tuple(float(value) for value in request.gripper_positions)
        if len(fingers) != 2 or not all(math.isfinite(value) for value in fingers):
            raise ValueError('complete finite fixed gripper state required')
        trajectory = request.trajectory
        if not 0 < len(trajectory.points) <= 10_000:
            raise ValueError('trajectory point budget exceeded')
        points = tuple(TrajectoryPoint(p.time_from_start.sec + p.time_from_start.nanosec*1e-9,
            p.positions, p.velocities, p.accelerations) for p in trajectory.points)
        with self.monitor.read_only() as scene:
            revision = scene_revision(scene.planning_scene_message)
            if request.scene_revision != revision:
                raise ValueError('planning scene changed before preparation')
            # 必须复制附着物，空 RobotState 会漏掉被夹持物体的碰撞几何。
            state = copy.copy(scene.current_state)
            def collision(names, positions):
                for q in positions:
                    if time.monotonic() >= request.deadline_monotonic:
                        raise TimeoutError('trajectory preparation deadline expired')
                    state.joint_positions = dict(zip((*names, 'left_finger_joint',
                        'right_finger_joint'), (*q, *fingers)))
                    state.update()
                    # 空组检查全部连杆，包含固定夹爪与环境/自身碰撞。
                    if scene.is_state_colliding(state, '', False):
                        return dict(state='collision')
                return dict(state='pass', checked_samples=len(positions),
                            requested_samples=len(positions))
            prepared, report = analyze_trajectory(trajectory.joint_names, points,
                lower=lower, upper=upper, limits=limits, context=context, now=now,
                collision_check=collision)
            if prepared is None:
                raise ValueError(json.dumps(report, ensure_ascii=False, allow_nan=False))
            if time.monotonic() >= request.deadline_monotonic:
                raise TimeoutError('trajectory preparation deadline expired')
            payload = prepared.to_payload()
            payload.update(scene_revision=revision, gripper_positions=fingers,
                           gripper_generation=request.gripper_generation)
            return payload, revision


def main(args=None):
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from moveit.planning import MoveItPy
    from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from rebotarm_msgs.srv import PrepareArmTrajectory
    from std_msgs.msg import String

    rclpy.init(args=args)
    node = Node('trajectory_preparation', automatically_declare_parameters_from_overrides=True)
    if not node.has_parameter('arm_namespace'):
        node.declare_parameter('arm_namespace', 'rebotarm')
    namespace = node.get_parameter('arm_namespace').value.strip('/')
    configuration = {}
    for name, parameter in node.get_parameters_by_prefix('').items():
        if name.startswith('qos_overrides.'):
            continue
        parent = configuration
        parts = name.split('.')
        for part in parts[:-1]:
            parent = parent.setdefault(part, {})
        parent[parts[-1]] = parameter.value
    options = configuration.setdefault('planning_scene_monitor_options', {})
    # MoveItCpp 使用 pipeline_names，move_group 的同名参数是字符串数组。
    pipelines = configuration.get('planning_pipelines', ['ompl'])
    if isinstance(pipelines, list):
        configuration['planning_pipelines'] = {'pipeline_names': pipelines}
    # Jazzy TEM 会在初始化后拒绝未知参数；提前声明 /clock QoS 避免延迟启用仿真时间时崩溃。
    configuration['qos_overrides'] = {'/clock': {'subscription': dict(
        durability='volatile', reliability='best_effort', history='keep_last', depth=1)}}
    # Jazzy 的这些 topic 是 C++ 常量，配置键不会生效，必须显式重映射。
    robot = MoveItPy(node_name='trajectory_preparation_scene', config_dict=configuration,
        provide_planning_service=False, remappings={
            '/joint_states': f'/{namespace}/visual_joint_states',
            '/planning_scene': '/monitored_planning_scene',
            '/monitored_planning_scene': f'/{namespace}/preparation_scene'})
    monitor = robot.get_planning_scene_monitor()
    checker = SceneCollisionChecker(monitor)
    limits = load_reference_limits(Path(get_package_share_directory('rebotarm_moveit_config'))/'config')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publisher = node.create_publisher(String, f'/{namespace}/trajectory_scene_revision', qos)
    lock = threading.RLock()
    initialized = False

    def publish_revision():
        nonlocal initialized
        started = time.monotonic()
        with lock:
            if not initialized:
                initialized = monitor.request_planning_scene_state('/get_planning_scene')
            if initialized:
                with monitor.read_only() as scene:
                    publisher.publish(String(data=scene_revision(scene.planning_scene_message)))
        elapsed = time.monotonic()-started
        if elapsed > .03:
            node.get_logger().warning(f'scene revision elapsed={elapsed:.4f}s')

    def prepare(request, response):
        received = time.monotonic()
        try:
            with lock:
                if not initialized:
                    raise RuntimeError('planning scene snapshot is not initialized')
                payload, revision = checker.prepare(request, *limits)
                response.prepared_json = json.dumps(payload, allow_nan=False)
                response.scene_revision, response.success = revision, True
                response.message = 'prepared_v1'
        except Exception as exc:
            response.success, response.message = False, str(exc)
        node.get_logger().info(f'prepare success={response.success} '
            f'input_age={received-request.feedback_observed_at:.4f}s '
            f'elapsed={time.monotonic()-received:.4f}s: {response.message[:160]}')
        return response

    group = ReentrantCallbackGroup()
    node.create_service(PrepareArmTrajectory, f'/{namespace}/prepare_arm_trajectory',
                        prepare, callback_group=group)
    # 场景摘要不能重入排队占满所有服务线程；碰撞服务保留独立回调组。
    node.create_timer(.05, publish_revision, callback_group=MutuallyExclusiveCallbackGroup(),
                      clock=rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.STEADY_TIME))
    # 本节点只有准备和摘要，二者原本就由场景锁串行；单执行器避免 Python 回调池竞争。
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        robot.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

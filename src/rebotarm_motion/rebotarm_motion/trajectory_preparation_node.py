"""PrepareArmTrajectory 内部服务及碰撞场景版本发布节点。

文件职责
    把后端冻结的执行快照交给运动层准备，并在 MoveItPy 同一只读场景锁内检查
    全部采样点；响应返回检查过的原始系数，供控制器或 MuJoCo 原子接纳。
对外接口
    scene_revision 计算碰撞相关场景摘要；SceneCollisionChecker.prepare 返回
    (载荷, 场景版本)；main 注册 /<namespace>/prepare_arm_trajectory 服务和
    trajectory_scene_revision 持久化话题。服务成功仅表示准备通过，不启动运动。
安全约束
    仅准备六轴；普通请求固定真实夹爪，网页请求仅在局部副本中使用闭合模型。
    不修改场景、不拥有执行资源、不访问设备；场景不匹配、超时、碰撞或未知均失败。
    期限使用主机单调时钟，与仿真 /clock 分开；耗时检查不得放入后端设备周期。
    场景版本是轮询已观察到的状态，执行端负责变化后的停止，不提供动态避障保证。
文件结构
    场景摘要 → 锁内批量碰撞准备器 → ROS/MoveItPy 装配、版本发布与服务回调。
"""

import hashlib
import copy
import json
import math
from pathlib import Path
import threading
import time

from .polynomial_trajectory import JOINT_NAMES, TrajectoryPoint
from .trajectory_preparation import PreparationContext, analyze_trajectory, load_reference_limits
from rebotarm_trajectory.joint_contract import canonical_input
from rebotarm_trajectory.capability import decode_capability


def scene_revision(message):
    """仅绑定碰撞环境；反馈关节和消息时间不构成场景版本。"""
    from rosidl_runtime_py.convert import message_to_ordereddict

    data = message_to_ordereddict(message)
    # 关节反馈由接纳方单独核验；把它或发布时刻纳入摘要会使正常反馈不断作废轨迹。
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

    def __init__(self, monitor, config_directory=None):
        self.monitor = monitor
        self.config_directory = config_directory

    def prepare(self, request, lower, upper, limits):
        """在请求期限内返回完整检查结果；任何异常交由服务回调转成明确失败响应。"""
        now = time.monotonic()
        if not math.isfinite(request.deadline_monotonic) or now >= request.deadline_monotonic:
            raise ValueError('trajectory preparation deadline expired')
        context = PreparationContext(request.session, request.generation,
            tuple(request.hold_positions), tuple(request.feedback_positions),
            request.feedback_observed_at, request.stationary, request.enabled)
        fingers = tuple(float(value) for value in request.gripper_positions)
        if len(fingers) != 2 or not all(math.isfinite(value) for value in fingers):
            raise ValueError('complete finite fixed gripper state required')
        policy = getattr(request, 'gripper_collision_policy', 0)
        if policy not in (0, 1):
            raise ValueError('unknown gripper collision policy')
        collision_fingers = (0., 0.) if policy == 1 else fingers
        trajectory = request.trajectory
        if not 0 < len(trajectory.points) <= 10_000:
            raise ValueError('trajectory point budget exceeded')
        points = tuple(TrajectoryPoint(p.time_from_start.sec + p.time_from_start.nanosec*1e-9,
            p.positions, p.velocities, p.accelerations) for p in trajectory.points)
        if any(p.effort for p in trajectory.points):
            raise ValueError('FJT effort is not a supported per-request torque limit')
        names, points = canonical_input(trajectory.joint_names, points, context.hold)
        capability = None
        capability_json = getattr(request, 'capability_json', '')
        if capability_json:
            capability = decode_capability(capability_json)
        from rebotarm_trajectory.planning import decode_plan, require_plan_context
        planning = decode_plan(trajectory.header.frame_id, names, points,
            capability_revision=capability['revision'] if capability else None, require_execution=True)
        require_plan_context(planning, capability_revision=capability['revision'] if capability else '',
            session=request.session, generation=request.generation, hold=context.hold)
        if self.config_directory is not None and capability is not None:
            from .coordinated_trajectory import effective_limits
            lower, upper, limits = effective_limits(self.config_directory, names, capability)
        elif set(limits) != set(names):
            raise ValueError('matching joint limits required for preparation')
        # 整批共用一个场景快照，不能逐点释放锁后把不同环境的检查结果拼成一次通过。
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
                    values = dict(zip(names, q))
                    values.update(zip(('left_finger_joint', 'right_finger_joint'), collision_fingers))
                    state.joint_positions = values
                    state.update()
                    # 空组检查全部连杆，包含当前样本双指与环境/自身碰撞。
                    if scene.is_state_colliding(state, '', False):
                        return dict(state='collision')
                return dict(state='pass', checked_samples=len(positions),
                            requested_samples=len(positions))
            prepared, report = analyze_trajectory(names, points,
                lower=lower, upper=upper, limits=limits, context=context, now=now,
                collision_check=collision, include_reference_comparison=False,
                preserve_points=planning is not None)
            if prepared is None:
                raise ValueError(json.dumps(report, ensure_ascii=False, allow_nan=False))
            if time.monotonic() >= request.deadline_monotonic:
                raise TimeoutError('trajectory preparation deadline expired')
            payload = prepared.to_payload()
            # 六轴准备上下文之外的绑定也必须原样带回，后端不能只采信 success 字段。
            payload.update(scene_revision=revision, gripper_positions=fingers,
                           gripper_generation=request.gripper_generation,
                           gripper_collision_policy=policy)
            if capability is not None:
                payload['capability_revision'] = capability['revision']
            return payload, revision


def main(args=None):
    """创建只负责准备的节点；延迟导入 ROS/MoveItPy，数值测试可直接导入检查器。"""
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
    config_directory = Path(get_package_share_directory('rebotarm_moveit_config'))/'config'
    checker = SceneCollisionChecker(monitor, config_directory)
    limits = load_reference_limits(config_directory)
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publisher = node.create_publisher(String, f'/{namespace}/trajectory_scene_revision', qos)
    lock = threading.RLock()
    initialized = False

    last_revision = None

    def publish_revision():
        """先取得完整场景，再发布摘要；尚未初始化时不能为准备请求提供空场景许可。"""
        nonlocal initialized, last_revision
        started = time.monotonic()
        with lock:
            if not initialized:
                initialized = monitor.request_planning_scene_state('/get_planning_scene')
            if initialized:
                with monitor.read_only() as scene:
                    revision = scene_revision(scene.planning_scene_message)
                # 持久化话题已保留最新值。重复摘要不能挤占设备线程的停止/使能请求队列。
                if revision != last_revision:
                    publisher.publish(String(data=revision))
                    last_revision = revision
        elapsed = time.monotonic()-started
        if elapsed > .03:
            node.get_logger().warning(f'scene revision elapsed={elapsed:.4f}s')

    def prepare(request, response):
        """串行使用已初始化场景，捕获检查异常并保留原因；不向执行后端发起命令。"""
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
    # 摘要定时器不重入，服务使用独立回调组；当前执行器仍串行处理二者。
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

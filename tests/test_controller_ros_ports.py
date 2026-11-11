"""ROS 职责端口与真实请求运行时的集成；注册和消息出口用内存替身。"""
import threading
import time
from types import SimpleNamespace
import pytest

from builtin_interfaces.msg import Time
from control_msgs.action import FollowJointTrajectory, GripperCommand
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

from rebotarmcontroller.runtime.lifecycle import shutdown_policy
from rebotarmcontroller.ros.motor_passthrough import MotorPassthrough
from rebotarmcontroller.ros.ros_actions import ArmActions, GoalResponse
import rebotarmcontroller.ros.ros_actions as action_module
from rebotarmcontroller.ros.ros_publishers import JointStatePublisher
from rebotarmcontroller.ros.ros_services import ArmServices
from test_controller_local_runtime import make_local_manager, wait_until
from test_device_request_arbitration import motor_command


class RegistrationNode:
    """记录正式构造器注册的接口；不建立 DDS、执行器或串口。"""
    slow_group = reentrant_group = sensor_qos = None

    def __init__(self):
        self.services, self.actions, self.subscriptions, self.publishers = {}, {}, {}, {}
        self.messages = []
        self.timers = []

    def create_service(self, kind, name, callback, **kwargs):
        self.services[name] = callback
        return callback

    def create_client(self, kind, name, **kwargs):
        return SimpleNamespace(service_is_ready=lambda: False)

    def create_subscription(self, kind, name, callback, *args, **kwargs):
        self.subscriptions[name] = callback
        return callback

    def create_publisher(self, kind, name, *args, **kwargs):
        messages = self.publishers[name] = []
        return SimpleNamespace(publish=messages.append)

    def create_timer(self, period, callback, **kwargs):
        self.timers.append(callback)
        return callback

    def tick(self):
        for callback in self.timers:
            callback()

    def get_clock(self):
        nanoseconds = time.time_ns()
        return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time(
            sec=nanoseconds//10**9, nanosec=nanoseconds % 10**9)))

    def get_logger(self):
        return SimpleNamespace(error=self.messages.append, warn=self.messages.append, warning=self.messages.append,
                               info=self.messages.append)


class CancelAfterSubmission:
    """第一次检查未取消，提交后发出取消意图，覆盖带句柄/代次的撤销路径。"""
    def __init__(self, request):
        self.request, self.state, self.checks = request, 'active', 0

    @property
    def is_cancel_requested(self):
        self.checks += 1
        return self.checks > 1

    def publish_feedback(self, feedback): pass
    def succeed(self): self.state = 'succeeded'
    def abort(self): self.state = 'aborted'
    def canceled(self): self.state = 'canceled'


@pytest.mark.parametrize('moveit,factor', [(False,1.), (True,2.)])
def test_gripper_protocol_preserves_total_opening_and_converts_moveit_left_finger(moveit, factor):
    """同一独立执行器只接受总开口；MoveIt 的反馈和结果仍使用左指坐标。"""
    actions = ArmActions.__new__(ArmActions)
    submitted, feedback = [], []
    actions._gripper = SimpleNamespace(
        set_gripper_target=lambda width, effort: submitted.append((width,effort)) or 7,
        gripper_execution_status=lambda generation: ('succeeded',''),
        gripper_position_m=lambda: .06, get_gripper_state=lambda: (0.,0.,.5))
    goal = SimpleNamespace(request=GripperCommand.Goal(),is_cancel_requested=False,
        publish_feedback=feedback.append,succeed=lambda:None,abort=lambda:pytest.fail('unexpected abort'))
    goal.request.command.position = .03
    goal.request.command.max_effort = 1.
    result = actions.execute_gripper_command(goal,moveit=moveit)
    assert submitted == [(.03*factor,1.)]
    assert result.reached_goal and result.position == .06/factor
    assert feedback[-1].position == .06/factor and feedback[-1].reached_goal


def test_registered_ros_callbacks_keep_queue_policy_and_live_owner_state(monkeypatch):
    assembly, opened = make_local_manager(monkeypatch)
    node = RegistrationNode()

    def action_server(node, kind, name, **callbacks):
        node.actions[name] = SimpleNamespace(**callbacks)
        return node.actions[name]

    monkeypatch.setattr(action_module, 'ActionServer', action_server)
    publisher = JointStatePublisher(node, 'ports', 100., runtime=assembly.runtime,
        lifecycle=assembly.lifecycle, feedback=assembly.feedback, gripper=assembly.gripper)
    node.publish_arm_status = publisher.publish_status
    ArmServices(node, 'ports', lifecycle=assembly.lifecycle, arm=assembly.arm,
                gripper=assembly.gripper, gravity=assembly.gravity)
    actions = ArmActions(node, 'ports', runtime=assembly.runtime, lifecycle=assembly.lifecycle,
               feedback=assembly.feedback, arm=assembly.arm, gripper=assembly.gripper)
    MotorPassthrough(node, 'ports', 'reject', runtime=assembly.runtime,
                     arm=assembly.arm, gripper=assembly.gripper)
    assert not opened and not assembly.runtime.writer_active
    gripper = node.actions['/ports/gripper/command']
    trajectory = node.actions['/ports/follow_joint_trajectory']
    arm_request = FollowJointTrajectory.Goal()
    arm_request.trajectory.joint_names = list(assembly.runtime.joint_names)
    assert gripper.goal_callback(None) == trajectory.goal_callback(arm_request) == GoalResponse.REJECT
    observed, device_threads = [], []
    assembly.lifecycle.connect()
    submit = assembly.runtime.requests.submit

    def record(operation, **options):
        assert not assembly.lifecycle.lock._is_owned()
        observed.append(options)
        return submit(operation, **options)

    monkeypatch.setattr(assembly.runtime.requests, 'submit', record)
    for motor in opened[0].motors:
        original = motor._record

        def device_call(*args, original=original):
            device_threads.append(threading.current_thread())
            return original(*args)

        monkeypatch.setattr(motor, '_record', device_call)
    monkeypatch.setattr(assembly.feedback, 'get_joint_state', lambda: (_ for _ in ()).throw(
        AssertionError('publisher requested synchronous device feedback')))

    def trigger(name):
        response = node.services['/ports/'+name](Trigger.Request(), Trigger.Response())
        assert response.success, response.message

    try:
        node.subscriptions['/ports/joints/joint1/cmd'](motor_command())
        assert observed == [dict(resource='arm', priority=10, ttl=.25, invalidate=())]
        assert any('disabled' in message for message in node.messages)
        observed.clear()
        trigger('enable')
        assert observed == [dict(resource='device', priority=5, ttl=5., invalidate=('device',))]
        assert node.publishers['/ports/arm_status'][-1].enabled
        assert gripper.goal_callback(None) == GoalResponse.ACCEPT
        assert trajectory.goal_callback(arm_request) == GoalResponse.REJECT
        assert any('preparation service unavailable' in message for message in node.messages)
        observed.clear()
        trigger('trajectory_stop')
        assert observed == [dict(resource='device', priority=0, ttl=5., invalidate=())]
        node.tick()
        assert node.publishers['/ports/joint_states'][-1].name == assembly.runtime.joint_names

        observed.clear()
        request = GripperCommand.Goal()
        request.command.position = .03
        goal = CancelAfterSubmission(request)
        result = gripper.execute_callback(goal)
        assert goal.state == 'canceled' and not result.reached_goal
        assert observed == [dict(resource='gripper', priority=10, ttl=2., invalidate=()),
                            dict(resource='gripper', priority=0, ttl=2., invalidate=())]
        assert not assembly.gripper.gripper_active and assembly.lifecycle.enabled

        request = FollowJointTrajectory.Goal()
        request.trajectory.joint_names = assembly.runtime.joint_names
        start = assembly.feedback.get_cached_joint_state()[0].tolist()
        end = list(start)
        end[0] += .01
        request.trajectory.points = [JointTrajectoryPoint(positions=start), JointTrajectoryPoint(positions=end)]
        request.trajectory.points[-1].time_from_start.sec = 3
        assembly.arm.update_trajectory_scene('scene-a')
        wait_until(lambda: assembly.arm.admission_snapshot()['ready'])
        actions._prepare_client = SimpleNamespace(service_is_ready=lambda: True)
        assert trajectory.goal_callback(request) == GoalResponse.ACCEPT
        goal = CancelAfterSubmission(request)
        # 模拟取消在服务派发前到达；仍由正式取消路径释放本请求预约并确认保持。
        goal.checks = 1
        trajectory.execute_callback(goal)
        assert goal.state == 'canceled' and assembly.lifecycle.enabled
        assert assembly.runtime.control_loop_active
        assert not shutdown_policy(assembly.lifecycle, arm=assembly.arm, gravity=assembly.gravity,
                                   safe_home=None, logger=node.get_logger())
        observed.clear()
        trigger('disable')
        assert observed == [dict(resource='device', priority=1, ttl=10., invalidate=('device',))]
        assert not node.publishers['/ports/arm_status'][-1].enabled
        assert not node.publishers['/ports/arm_status'][-1].control_loop_active
        assert device_threads and set(device_threads) == {assembly.runtime.control_thread}
        assert all(motor.state.status_code == 0 for motor in opened[0].motors)
        assert shutdown_policy(assembly.lifecycle, arm=assembly.arm, gravity=assembly.gravity,
                               safe_home=None, logger=node.get_logger())
        assert not assembly.runtime.writer_active
    finally:
        # 这里只操作本进程内存假句柄，不接触真实设备。
        if assembly.lifecycle.enabled:
            assembly.lifecycle.disable()
        assembly.lifecycle.shutdown()


@pytest.mark.parametrize('rate', [0., -1., float('nan'), float('inf'), -float('inf'), 1e-320, 1e20])
def test_invalid_joint_state_rate_fails_before_hardware_construction(monkeypatch, rate):
    import rebotarmcontroller.rebotarm_controller as module

    values = {}
    monkeypatch.setattr(module.Node, '__init__', lambda self, name: None)
    monkeypatch.setattr(module.Node, 'declare_parameter',
                        lambda self, name, default, **kwargs: values.setdefault(name, default))
    monkeypatch.setattr(module.Node, 'get_parameter',
                        lambda self, name: SimpleNamespace(value=rate if name == 'joint_state_rate' else values[name]))
    monkeypatch.setattr(module, 'HardwareManager', lambda **kwargs: pytest.fail('hardware constructed'))
    with pytest.raises(ValueError, match='joint_state_rate'):
        module.reBotArmController()


def test_fractional_publish_rate_is_preserved():
    node = RegistrationNode()
    periods = []
    node.create_timer = lambda period, callback, **kwargs: periods.append(period)
    runtime = SimpleNamespace(joint_names=[], mode='pos_vel', control_loop_active=False)
    lifecycle = SimpleNamespace(enabled=False, execution='IDLE')
    feedback = SimpleNamespace(get_feedback_status=lambda: ([], []))
    JointStatePublisher(node, 'ports', .5, runtime=runtime, lifecycle=lifecycle,
                        feedback=feedback, gripper=SimpleNamespace(has_gripper=False))
    assert periods == [2.0]

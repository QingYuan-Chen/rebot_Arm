"""空闲 localhost 域中的合成反馈验证；禁止设备导入和运动请求。"""
import importlib.abc
import json
import sys
import threading
import time
from types import SimpleNamespace as NS


class BlockDevices(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, *args):
        if fullname.split('.')[0] in ('motorbridge', 'serial', 'reBotArm_control_py'):
            raise RuntimeError('feedback probe forbids devices: ' + fullname)


sys.meta_path.insert(0, BlockDevices())

import rclpy
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState, Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from rebotarm_msgs.msg import ArmStatus, JointMotorState, GraspPlan, GraspCandidateArray
from std_srvs.srv import Trigger
from rebotarm_msgs.srv import MoveToPoseIK, ExecutePose
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_ros import TransformBroadcaster
from rebotarm_motion.feedback_receiver import MotionFeedbackReceiver, stamp_ns
from rebotarm_motion.pose_compat_node import PoseCompatNode, PoseRun
from rebotarm_motion.pose_execution_node import PoseExecutionNode
from rebotarm_vision.nodes.visual_grasp_executor_node import VisualGraspExecutorNode
from rebotarm_calibration.handeye_capture_node import HandeyeCaptureNode


def until(predicate, timeout=4.):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.005)
    raise AssertionError('condition timed out')


rclpy.init(args=['--ros-args', '-p', 'arm_namespace:=feedback_probe',
    '-p', 'session_directory:=' + sys.argv[1],
    '-r', '/feedback_probe/joint_states:=/probe_remapped/joints',
    '-r', '/tf:=/probe_remapped/tf'])
source = Node('feedback_probe_source')
# 检查隔离域，不在已有会话中启动验证。
until(lambda: len(source.get_node_names_and_namespaces()) >= 1)
time.sleep(.3)
assert source.get_node_names_and_namespaces() == [(source.get_name(), source.get_namespace())]
nodes = []
stop = threading.Event()
publishing = threading.Event()
publishing.set()
release = threading.Event()
publisher_thread = None
worker = None
executor = MultiThreadedExecutor(num_threads=6)
try:
    compat, motion, visual, calibration = PoseCompatNode(), PoseExecutionNode(), VisualGraspExecutorNode(), HandeyeCaptureNode()
    nodes = [compat, motion, visual, calibration]
    for node in [source] + nodes:
        executor.add_node(node)
    worker = threading.Thread(target=executor.spin, daemon=True)
    worker.start()
    specs = (
        ('joints', JointState, '/probe_remapped/joints', qos_profile_sensor_data),
        ('status', ArmStatus, '/feedback_probe/arm_status', 10),
        ('gripper', JointMotorState, '/feedback_probe/gripper/state', qos_profile_sensor_data),
        ('plan', GraspPlan, '/grasp/filtered_plan', 10),
        ('candidates', GraspCandidateArray, '/grasp/filtered_candidates', 10),
        ('image', Image, '/camera/color/image_raw', qos_profile_sensor_data),
        ('info', CameraInfo, '/camera/color/camera_info', qos_profile_sensor_data),
        ('alternate', Image, '/alternate/image', qos_profile_sensor_data),
        ('alternate_info', CameraInfo, '/alternate/info', qos_profile_sensor_data),
    )
    publishers = {key: source.create_publisher(kind, topic, qos) for key, kind, topic, qos in specs}
    tf = TransformBroadcaster(source)
    sequence = [0]
    def publish():
        while not stop.wait(.01):
            if not publishing.is_set():
                continue
            sequence[0] += 1
            stamp = source.get_clock().now().to_msg()
            msg = JointState()
            msg.header.stamp = stamp
            msg.name = [f'joint{i}' for i in range(1, 7)]
            msg.position = [0., -1., -1., 0., .3, 0.]
            msg.velocity, msg.effort = [0.]*6, [0.]*6
            publishers['joints'].publish(msg)
            status = ArmStatus()
            status.header.stamp, status.mode = stamp, 'pos_vel'
            status.enabled, status.control_loop_active = True, True
            status.per_joint_status_code, status.state_machine = [1]*6, 'IDLE'
            publishers['status'].publish(status)
            gripper = JointMotorState()
            gripper.header.stamp, gripper.position, gripper.status_code = stamp, .046, 1
            publishers['gripper'].publish(gripper)
            plan = GraspPlan()
            plan.header.stamp, plan.valid = stamp, True
            publishers['plan'].publish(plan)
            candidates = GraspCandidateArray()
            candidates.header.stamp = stamp
            publishers['candidates'].publish(candidates)
            for key, info_key, frame in (('image', 'info', 'optical'),
                                         ('alternate', 'alternate_info', 'alternate')):
                image = Image()
                image.header.stamp, image.header.frame_id = stamp, frame
                image.width, image.height, image.encoding, image.step = 1, 1, 'bgr8', 3
                image.data = bytes([sequence[0] % 255, 0, 0])
                info = CameraInfo()
                info.header = image.header
                publishers[key].publish(image)
                publishers[info_key].publish(info)
            transform = TransformStamped()
            transform.header.stamp, transform.header.frame_id = stamp, 'base'
            transform.child_frame_id, transform.transform.rotation.w = 'end', 1.
            tf.sendTransform(transform)
    publisher_thread = threading.Thread(target=publish, daemon=True)
    publisher_thread.start()
    until(lambda: compat._feedback_receiver.snapshot().gripper is not None
          and motion._feedback_receiver.snapshot().status is not None
          and calibration._inputs.snapshot().image is not None
          and visual._tf_buffer.can_transform('base', 'end', rclpy.time.Time()))
    # 真正的 ROS 同步服务回调持业务锁等待，独立接收不得随它停顿。
    entered = threading.Event()
    wait_returned = threading.Event()
    hold_sec = float(sys.argv[2]) if len(sys.argv) > 2 else .65
    def wait_service(request, response):
        with compat.lock, motion._motion_lock, visual._state.lock:
            entered.set()
            release.wait(timeout=hold_sec + 5.)
        wait_returned.set()
        response.success = True
        return response
    compat.create_service(Trigger, '/feedback_probe/wait', wait_service, callback_group=compat.group)
    client = source.create_client(Trigger, '/feedback_probe/wait')
    assert client.wait_for_service(timeout_sec=3.)
    future = client.call_async(Trigger.Request())
    assert entered.wait(3.)
    first_sequence = visual._input_receiver.mailbox.sequence
    observed = []
    end = time.monotonic() + hold_sec
    while time.monotonic() < end:
        now_ns = source.get_clock().now().nanoseconds
        a, b = compat._feedback_receiver.snapshot(), motion._feedback_receiver.snapshot()
        camera = calibration._inputs.snapshot().image
        transform = visual._tf_buffer.lookup_transform('base', 'end', rclpy.time.Time())
        stamps = (stamp_ns(a.joints), stamp_ns(b.joints), stamp_ns(camera), stamp_ns(transform))
        observed.append(max((now_ns-stamp)/1e9 for stamp in stamps))
        time.sleep(.005)
    assert max(observed) <= .15, max(observed)
    assert visual._input_receiver.mailbox.sequence - first_sequence >= 20
    assert not wait_returned.is_set(), 'business locks released before observation ended'
    release.set()
    until(future.done)
    assert future.result().success
    print(json.dumps(dict(sync_wait_maximum_age_ms=max(observed)*1000,
                          sync_wait_observation_sec=hold_sec,
                          independent_observations=len(observed))), flush=True)
    # 正式同步位姿服务等待内存 FJT 的接纳和终态；不构造硬件控制器。
    fake_goals = []
    def execute_fake(goal):
        fake_goals.append(goal.request)
        time.sleep(.2)
        goal.succeed()
        return FollowJointTrajectory.Result(error_code=0)
    fake_action = ActionServer(source, FollowJointTrajectory, '/feedback_probe/follow_joint_trajectory',
                              execute_callback=execute_fake, callback_group=ReentrantCallbackGroup())
    compat.checker = NS(check_positions=lambda **_: dict(state='pass', message='in-memory probe'))
    ik_client = source.create_client(MoveToPoseIK, '/feedback_probe/move_to_pose_ik')
    pose_client = source.create_client(ExecutePose, '/feedback_probe/motion_execution/execute_pose')
    assert ik_client.wait_for_service(timeout_sec=3.) and pose_client.wait_for_service(timeout_sec=3.)
    with compat.lock:
        compat._ready()
        target = compat._pose_message()
    pending = ik_client.call_async(MoveToPoseIK.Request(target_pose=target))
    until(pending.done)
    assert pending.result().success, pending.result()
    until(lambda: compat.active is None)
    trajectory = JointTrajectory(joint_names=[f'joint{i}' for i in range(1, 7)])
    point = JointTrajectoryPoint(positions=[0., -1., -1., 0., .3, 0.])
    point.time_from_start.nanosec = 200_000_000
    trajectory.points = [point]
    motion._planner = NS(plan_pose=lambda *a, **kw: NS(success=True, trajectory=trajectory, message='fake plan'))
    pending = pose_client.call_async(ExecutePose.Request(execute=True, timeout_sec=3.))
    until(pending.done)
    assert pending.result().success, pending.result()
    assert len(fake_goals) == 2
    fake_action.destroy()
    # 暂停源，独立轮询统计空档；恢复后的新帧不能抹去计数或旧任务失效。
    with compat.lock:
        compat._ready()
        old_epoch = compat.epoch
    publishing.clear()
    end = time.monotonic() + .25
    stale_samples = 0
    while time.monotonic() < end:
        snapshot = compat._feedback_receiver.snapshot()
        stale_samples += source.get_clock().now().nanoseconds-stamp_ns(snapshot.joints) > 150_000_000
        time.sleep(.005)
    assert stale_samples > 0
    publishing.set()
    until(lambda: source.get_clock().now().nanoseconds-stamp_ns(compat._feedback_receiver.snapshot().joints) < 100_000_000)
    recovered = compat._feedback_receiver.snapshot()
    assert recovered.epoch > old_epoch and recovered.joint_gaps >= 1
    with compat.lock:
        run = PoseRun(old_epoch, time.monotonic()+2.)
        compat.active = run
        try:
            compat._check_run(run)
            raise AssertionError('old pose continued after feedback outage')
        except RuntimeError:
            assert run.cancel
        finally:
            compat.active = None
    # 话题切换之后，手动送达的旧代次回调也不能重新填入旧图像。
    old_generation = calibration._inputs.generation
    old_callback = calibration._inputs._callback(old_generation, 'image')
    calibration.configure_topics(dict(image_topic='/alternate/image', camera_info_topic='/alternate/info'))
    old_image = Image()
    old_image.header.frame_id = 'obsolete'
    old_callback(old_image)
    until(lambda: calibration._inputs.snapshot().image is not None)
    assert calibration._inputs.snapshot().image.header.frame_id == 'alternate'
    assert calibration._inputs.generation == old_generation+1
    assert calibration._inputs.snapshot().infos
    # 新节点继承话题重映射，但不继承父节点名称；显式关闭后线程必须已退出。
    remapped_owner = Node('owner', cli_args=['--ros-args', '-r', '__node:=renamed_owner'])
    receiver = MotionFeedbackReceiver(remapped_owner, 'feedback_probe')
    until(lambda: receiver.snapshot().joints is not None)
    assert receiver.get_name() == 'renamed_owner_feedback'
    receiver.close()
    receiver.close()
    assert not receiver.thread.is_alive()
    # 父节点不进入执行器时，接收节点仍独立处理重映射后的仿真时钟和源年龄。
    sim_owner = Node('sim_owner', cli_args=['--ros-args', '-r', '/clock:=/probe_remapped/clock'],
                     parameter_overrides=[Parameter('use_sim_time', value=True)])
    sim_receiver = MotionFeedbackReceiver(sim_owner, 'sim_probe')
    try:
        clock_publisher = source.create_publisher(Clock, '/probe_remapped/clock', 10)
        sim_joints = source.create_publisher(JointState, '/sim_probe/joint_states', qos_profile_sensor_data)
        clock_message = Clock()
        clock_message.clock.sec = 10
        def clock_received():
            clock_publisher.publish(clock_message)
            return sim_receiver.get_clock().now().nanoseconds == (
                clock_message.clock.sec * 1_000_000_000 + clock_message.clock.nanosec)
        until(clock_received)
        sim_message = JointState()
        sim_message.header.stamp.sec = 10
        def joints_received():
            sim_joints.publish(sim_message)
            return sim_receiver.snapshot().joints is not None
        until(joints_received)
        assert sim_owner.get_clock().now().nanoseconds == 0
        assert sim_receiver.snapshot().epoch == 0
        clock_message.clock.nanosec = 200_000_000
        until(clock_received)
        sim_snapshot = sim_receiver.snapshot()
        assert sim_snapshot.joint_gaps == 1 and sim_snapshot.epoch == 1
        assert abs(sim_snapshot.maximum_joint_age - .2) < 1e-9
        print(json.dumps(dict(independent_sim_clock_ns=sim_receiver.get_clock().now().nanoseconds,
                              sim_clock_gap_count=sim_snapshot.joint_gaps)), flush=True)
    finally:
        sim_receiver.close()
        sim_owner.destroy_node()
    assert not sim_receiver.thread.is_alive()
    # 独立上下文关闭也须正常唤醒并结束接收线程。
    from rclpy.context import Context
    context = Context()
    context.init(args=[])
    isolated_owner = Node('context_owner', context=context, use_global_arguments=False)
    isolated_receiver = MotionFeedbackReceiver(isolated_owner, 'unused')
    context.shutdown()
    isolated_receiver.close()
    assert not isolated_receiver.thread.is_alive()
    isolated_owner.destroy_node()
    remapped_owner.destroy_node()
    print(json.dumps(dict(stale_observations=stale_samples, recovered_gap_count=recovered.joint_gaps,
                          topic_generation=calibration._inputs.generation)), flush=True)
finally:
    release.set()
    stop.set()
    if publisher_thread is not None:
        publisher_thread.join(timeout=2.)
    executor.shutdown(timeout_sec=5.)
    if worker is not None:
        worker.join(timeout=3.)
    receiver_threads = []
    for node in nodes:
        receiver = (node._inputs if isinstance(node, HandeyeCaptureNode) else
                    node._input_receiver if isinstance(node, VisualGraspExecutorNode) else node._feedback_receiver)
        receiver_threads.append(receiver.thread)
        node.destroy_node()
    source.destroy_node()
    assert not any(thread.is_alive() for thread in receiver_threads)
    rclpy.try_shutdown()
    print('receiver threads joined; no device imports or commands', flush=True)

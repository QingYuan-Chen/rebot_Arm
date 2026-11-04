#!/usr/bin/env python3
"""无硬件反馈延迟对照工具。

职责：用生产 JointStatePublisher 发布内存缓存，在独立进程比较 ROS 单线程与
多线程接收，记录源时间、发布时刻、DDS 时间、回调入口和独立监测时的数据年龄。
接口：指定空闲 --domain-id、--output-dir；--with-pose 加入空闲位姿兼容节点，
--without-qos-events 仅用于隔离默认 QoS 事件等待的影响。
约束：不用 HardwareManager、不导入设备 SDK、不打开串口、不创建运动客户端。
数据是假反馈，不能当作真机验收。150 ms 门限与生产时间戳语义保持不变。
"""
from __future__ import annotations

import argparse
import importlib.abc
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[1]


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain-id', type=int, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=20.)
    parser.add_argument('--with-pose', action='store_true')
    parser.add_argument('--without-qos-events', action='store_true')
    # 子进程也用相同脚本，避免通过 shell 拼接命令或改变全局 ROS 环境。
    parser.add_argument('--role', choices=('run', 'pub', 'sub', 'pose'), default='run', help=argparse.SUPPRESS)
    parser.add_argument('--executor', choices=('single', 'multi'), default='multi', help=argparse.SUPPRESS)
    parser.add_argument('--namespace', default='feedback_latency_probe', help=argparse.SUPPRESS)
    parser.add_argument('--case', default='multi', help=argparse.SUPPRESS)
    args = parser.parse_args()
    if not 1 <= args.domain_id <= 232 or args.duration < 5.:
        parser.error('需要非零隔离域（1～232），观测时间至少 5 秒')
    return args


class BlockDevices(importlib.abc.MetaPathFinder):
    """本工具只允许消息与纯计算依赖；误引入设备访问时立即失败。"""

    def find_spec(self, fullname, *args):
        if fullname.split('.')[0] in ('reBotArm_control_py', 'motorbridge', 'damiao_motor', 'serial'):
            raise RuntimeError('离线延迟工具禁止导入设备 SDK 或串口')


def write_rows(path, rows):
    with path.open('x') as stream:
        stream.writelines(json.dumps(row, ensure_ascii=False)+'\n' for row in rows)


def probe(args):
    """单个角色的有界采样；先在内存记录，结束后写盘，避免观测日志阻塞回调。"""
    import rclpy
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.event_handler import SubscriptionEventCallbacks
    from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from rebotarm_msgs.msg import ArmStatus, JointMotorState

    rclpy.init()
    rows, workers = [], []
    stop = threading.Event()

    def record(event, **values):
        rows.append(dict(event=event, mono=time.monotonic(), wall=time.time_ns(), **values))

    if args.role == 'pose':
        from rebotarm_motion.pose_compat_node import PoseCompatNode
        # 只创建协议接口；没有控制器、动作请求或设备访问入口。
        node = PoseCompatNode()
    else:
        node = Node('feedback_latency_'+args.role)

    if args.role == 'pub':
        from rebotarmcontroller.ros.ros_publishers import JointStatePublisher

        class Cache:
            joint_names = [f'joint{i}' for i in range(1, 7)]
            has_gripper = True
            mode, enabled, control_loop_active, execution = 'pos_vel', True, True, 'IDLE'

            def __init__(self):
                self.lock = threading.Lock()
                self.sample = (1, time.monotonic())

            def update(self):
                while not stop.wait(.02):
                    with self.lock:
                        self.sample = (self.sample[0]+1, time.monotonic())
                        record('cache', sequence=self.sample[0], observed=self.sample[1])

            def get_cached_joint_sample(self):
                with self.lock:
                    sequence, observed = self.sample
                return [0., 0., -.01745, 0., 0., 0.], [0.]*6, [0.]*6, [1]*6, (sequence,)*6, observed

            def get_cached_gripper_sample(self):
                with self.lock:
                    sequence, observed = self.sample
                return .046, 0., 0., 1, sequence, observed

            def get_feedback_status(self):
                return [1]*6, []

        cache = Cache()
        workers.append(threading.Thread(target=cache.update))
        original_create = node.create_publisher

        def create(kind, topic, *values, **kwargs):
            publisher = original_create(kind, topic, *values, **kwargs)

            def publish(msg):
                record('publish', topic=topic, stamp=stamp_ns(msg))
                publisher.publish(msg)

            return SimpleNamespace(publish=publish)

        node.create_publisher = create
        node.sensor_qos = qos_profile_sensor_data
        node.reentrant_group = ReentrantCallbackGroup()
        output = JointStatePublisher(
            node, args.namespace, 100., runtime=cache, lifecycle=cache,
            feedback=cache, gripper=cache,
        )

    if args.role == 'sub':
        latest = {}
        snapshot_lock = threading.Lock()
        prefix = '/'+args.namespace
        status_topic = prefix+'/arm_status'

        def callback(topic):
            def receive(msg, info):
                stamp, received = stamp_ns(msg), time.monotonic()
                with snapshot_lock:
                    latest[topic] = (stamp, received)
                # Jazzy 的 MessageInfo 是字典；DDS 时间为墙钟，不与单调时钟直接相减。
                record('callback', topic=topic, stamp=stamp,
                       dds_source=info['source_timestamp'], dds_received=info['received_timestamp'])
            return receive

        kwargs = {}
        if args.without_qos_events:
            kwargs['event_callbacks'] = SubscriptionEventCallbacks(use_default_callbacks=False)
        topics = [prefix+f'/joints/joint{i}/state' for i in range(1, 7)]+[prefix+'/gripper/state']
        subscriptions = [node.create_subscription(JointMotorState, topic, callback(topic),
                         qos_profile_sensor_data, **kwargs) for topic in topics]
        subscriptions.append(node.create_subscription(ArmStatus, status_topic, callback(status_topic),
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL), **kwargs))
        started = time.monotonic()

        def monitor():
            while not stop.wait(.005):
                if time.monotonic()-started < 2.:
                    continue
                # 回调与监测使用同一份完整快照，避免把并发读写误判成消息不同步。
                with snapshot_lock:
                    snapshot = dict(latest)
                wall, mono = time.time_ns(), time.monotonic()
                ages = {topic: (wall-snapshot[topic][0])/1e9 for topic in topics if topic in snapshot}
                record('health', ages=ages, complete=len(ages) == 7,
                       status_age=None if status_topic not in snapshot else mono-snapshot[status_topic][1])

        workers.append(threading.Thread(target=monitor))

    executor = (SingleThreadedExecutor() if args.executor == 'single'
                else MultiThreadedExecutor(num_threads=4))
    executor.add_node(node)
    for worker in workers:
        worker.start()
    cpu_start, started = time.process_time(), time.monotonic()
    try:
        while time.monotonic()-started < args.duration:
            executor.spin_once(timeout_sec=.05)
    finally:
        stop.set()
        for worker in workers:
            worker.join(timeout=2.)
        # 先停止并等待执行器线程，再销毁节点；不在运行中销毁回调使用的句柄。
        executor.shutdown(timeout_sec=5.)
        record('summary', elapsed=time.monotonic()-started, cpu=time.process_time()-cpu_start)
        node.destroy_node()
        rclpy.try_shutdown()
        write_rows(args.output_dir/(args.case+'_'+args.role+'.jsonl'), rows)


def stamp_ns(msg):
    return msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec


def summarize(path):
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    callbacks = [row for row in rows if row['event'] == 'callback']
    # 去掉发现与启动窗口；发布进程比接收端多活 3 秒，不把收尾断流计入过期。
    first = min((row['mono'] for row in callbacks), default=float('inf'))+2.
    callbacks = [row for row in callbacks if row['mono'] >= first]
    joints = [row for row in callbacks if row['topic'].endswith('/joints/joint1/state')]
    health = [row for row in rows if row['event'] == 'health']
    dds = [row for row in callbacks if row['dds_source'] > 0 and row['dds_received'] > 0]
    offsets = [row['wall']-round(row['mono']*1e9) for row in callbacks]
    peak = lambda values: max(values, default=0.)
    return dict(
        callbacks=len(callbacks),
        maximum_joint1_gap_ms=1000*peak(b['mono']-a['mono'] for a, b in zip(joints, joints[1:])),
        maximum_callback_source_age_ms=peak((r['wall']-r['stamp'])/1e6 for r in callbacks
                                           if not r['topic'].endswith('/arm_status')),
        dds_timestamp_samples=len(dds),
        maximum_dds_transport_ms=max(((r['dds_received']-r['dds_source'])/1e6 for r in dds), default=None),
        maximum_dds_to_callback_ms=max(((r['wall']-r['dds_received'])/1e6 for r in dds), default=None),
        wall_monotonic_offset_span_ms=(max(offsets)-min(offsets))/1e6 if offsets else None,
        maximum_monitor_source_age_ms=1000*peak(age for r in health for age in r['ages'].values()),
        stale_observations=sum(any(age > .15 for age in r['ages'].values()) for r in health),
        missing_observations=sum(not r['complete'] for r in health),
        heartbeat_timeouts=sum(r['status_age'] is None or r['status_age'] > .6 for r in health),
        cpu_seconds=rows[-1]['cpu'], elapsed_seconds=rows[-1]['elapsed'])


def publisher_summary(path):
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    joints = [r for r in rows if r['event'] == 'publish' and r['topic'].endswith('/joint_states')]
    return dict(
        maximum_publish_gap_ms=1000*max((b['mono']-a['mono'] for a, b in zip(joints, joints[1:])), default=0.),
        maximum_publish_source_age_ms=max(((r['wall']-r['stamp'])/1e6 for r in joints), default=0.))


def run(args):
    """顺序运行两组对照；只终止本工具创建的无硬件子进程。"""
    import rclpy
    from rclpy.node import Node
    from rclpy.utilities import get_rmw_implementation_identifier

    args.output_dir.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    check = Node('feedback_latency_domain_check')
    try:
        deadline = time.monotonic()+1.5
        while time.monotonic() < deadline:
            rclpy.spin_once(check, timeout_sec=.1)
        others = [(name, namespace) for name, namespace in check.get_node_names_and_namespaces()
                  if (name, namespace) != (check.get_name(), check.get_namespace())]
        if others:
            raise RuntimeError(f'隔离域已有节点，停止实验：{others}')
    finally:
        check.destroy_node()
        rclpy.try_shutdown()
    namespace = 'feedback_latency_probe_'+str(os.getpid())
    report = dict(domain=args.domain_id, namespace=namespace, hardware=False,
                  rmw=get_rmw_implementation_identifier(),
                  qos_events=not args.without_qos_events, with_pose=args.with_pose, cases={})
    for mode in ('multi', 'single'):
        children = []
        try:
            roles = [('pub', args.duration+3., 'multi'), ('sub', args.duration, mode)]
            if args.with_pose:
                roles.append(('pose', args.duration, 'multi'))
            for role, duration, executor in roles:
                command = [sys.executable, str(Path(__file__).resolve()), '--role', role,
                    '--domain-id', str(args.domain_id), '--output-dir', str(args.output_dir.resolve()),
                    '--duration', str(duration), '--executor', executor, '--namespace', namespace,
                    '--case', mode]
                if args.without_qos_events:
                    command.append('--without-qos-events')
                if role == 'pose':
                    command += ['--ros-args', '-p', 'arm_namespace:='+namespace]
                stream = (args.output_dir/(mode+'_'+role+'.log')).open('x')
                child = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT)
                children.append((child, stream))
                if role == 'pub':
                    time.sleep(.5)
            for child, _ in children:
                if child.wait(timeout=args.duration+20.) != 0:
                    raise RuntimeError('离线子进程失败；请检查输出目录中的日志')
            report['cases'][mode] = summarize(args.output_dir/(mode+'_sub.jsonl'))
            report['cases'][mode]['publisher'] = publisher_summary(args.output_dir/(mode+'_pub.jsonl'))
            print(json.dumps({mode: report['cases'][mode]}, ensure_ascii=False), flush=True)
        finally:
            for child, stream in children:
                if child.poll() is None:
                    child.terminate()
                    try:
                        child.wait(timeout=5.)
                    except subprocess.TimeoutExpired:
                        child.kill()
                        child.wait()
                stream.close()
    with (args.output_dir/'comparison.json').open('x') as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)


def main():
    # ROS 参数仅交给 ROS 初始化，避免和本工具参数互相解析。
    ros_args = sys.argv.index('--ros-args') if '--ros-args' in sys.argv else len(sys.argv)
    extra_ros = sys.argv[ros_args:]
    sys.argv = sys.argv[:ros_args]
    args = arguments()
    sys.argv = [sys.argv[0]]+extra_ros
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    sys.meta_path.insert(0, BlockDevices())
    for package in (ROOT/'src').iterdir():
        if package.is_dir():
            sys.path.insert(0, str(package))
    if args.role == 'run':
        run(args)
    else:
        probe(args)


if __name__ == '__main__':
    main()

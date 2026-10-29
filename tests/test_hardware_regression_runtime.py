"""隔离 ROS 域内验证回归客户端；只启动合成节点，不访问任何设备。"""
import os
from pathlib import Path
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]


def test_feedback_survives_blocked_command_callback_and_fault_stays_latched(tmp_path):
    rclpy = pytest.importorskip('rclpy')
    if not getattr(rclpy, '__file__', None):
        pytest.skip('ROS runtime is not installed')
    result = subprocess.run([sys.executable, str(Path(__file__).resolve()), str(tmp_path)],
        env=dict(os.environ, ROS_DOMAIN_ID='226', ROS_LOCALHOST_ONLY='1'),
        capture_output=True, text=True, timeout=25)
    assert result.returncode == 0, result.stdout + result.stderr


def run_probe(root):
    import json
    import threading
    import time
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rebotarm_msgs.msg import JointMotorState, ArmStatus
    from std_srvs.srv import Trigger

    names = [f'joint{i}' for i in range(1, 7)] + ['gripper']
    # 即使测试域恰好被真实系统使用，也绝不发布或调用实际 /rebotarm 接口。
    prefix = f'/regression_fake_{os.getpid()}'
    fake_name = f'regression_fake_{os.getpid()}'
    (root / 'baseline.json').write_text(json.dumps({n: .04 if n == 'gripper' else 0. for n in names}))
    audit_path = root / 'controller_audit.jsonl'
    audit_path.touch()
    paused = threading.Event()
    stop_calls = []

    class FakeController(Node):
        def __init__(self):
            super().__init__(fake_name)
            self.sequence = 0
            self.pubs = {n: self.create_publisher(JointMotorState,
                f'{prefix}/gripper/state' if n == 'gripper' else f'{prefix}/joints/{n}/state',
                qos_profile_sensor_data) for n in names}
            self.status = self.create_publisher(ArmStatus, f'{prefix}/arm_status',
                QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.create_service(Trigger, f'{prefix}/enable', self.reply)
            self.create_service(Trigger, f'{prefix}/disable', self.reply)
            self.create_service(Trigger, f'{prefix}/trajectory_stop', self.stop)
            self.create_timer(.01, self.publish_feedback)

        def reply(self, request, response):
            response.success = True
            return response

        def stop(self, request, response):
            stop_calls.append(time.monotonic())
            response.success = True
            response.message = 'simulated stop'
            return response

        def publish_feedback(self):
            if paused.is_set():
                return
            self.sequence += 1
            observed = time.monotonic()
            stamp = self.get_clock().now().to_msg()
            with audit_path.open('a') as stream:
                for name in names:
                    msg = JointMotorState()
                    msg.header.stamp = stamp
                    msg.joint_name = name
                    msg.position = .04 if name == 'gripper' else 0.
                    msg.status_code = 0
                    self.pubs[name].publish(msg)
                    data = dict(name=name, sequence=self.sequence, observed_at=observed,
                        position=-.04*5/.09 if name == 'gripper' else 0., velocity=0., torque=0., status=0)
                    stream.write(json.dumps(dict(event='verified_feedback', data=data)) + '\n')
            status = ArmStatus()
            status.header.stamp = stamp
            status.per_joint_status_code = [0] * 6
            self.status.publish(status)

    # 复制正式脚本，仅注入一个需要 300 ms 的服务完成回调；真实接收路径不改写。
    script = (ROOT / 'scripts/hardware_regression.py').read_text()
    assert '/rebotarm/' in script
    script = script.replace('/rebotarm/', prefix + '/')
    script = script.replace("expected='reBotArmController'", f"expected={fake_name!r}")
    script = script.replace('from hardware_regression_io import Audit',
        f'sys.path.insert(0, {str(ROOT / "scripts")!r})\nfrom hardware_regression_io import Audit')
    script = script.replace('import argparse, fcntl, json, math, os, queue, threading, time',
                            'import argparse, fcntl, json, math, os, queue, sys, threading, time')
    marker = "emit('ready',baseline=base,status=snap()[1])"
    assert script.count(marker) == 1
    script = script.replace(marker, marker + "\n"
        "original_event_record=events.record\n"
        "def delayed_event_record(event,**values):\n"
        "    if event == 'monitor_failure':\n"
        f"        Path({str(root / 'audit_blocked')!r}).touch()\n"
        f"        while not Path({str(root / 'release_audit')!r}).exists(): time.sleep(.005)\n"
        "    return original_event_record(event,**values)\n"
        "events.record=delayed_event_record\n"
        "def injected_blocked_callback(_):\n"
        "    events.record('probe_enter',phase=phase)\n"
        "    time.sleep(.30)\n"
        "    events.record('probe_exit',phase=phase)\n"
        "probe_future=services['trajectory_stop'].call_async(Trigger.Request())\n"
        "probe_future.add_done_callback(injected_blocked_callback)")
    script_path = root / 'instrumented_client.py'
    script_path.write_text(script)

    def read_rows(path):
        if not path.exists():
            return []
        # 忽略正在写入的最后半行，不掩盖已经结束的审计失败。
        return [json.loads(line) for line in path.read_text().split('\n')[:-1] if line]

    def wait_event(name, timeout=8.):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if proc.poll() is not None:
                raise RuntimeError(f'回归客户端提前退出 {proc.returncode}')
            rows = read_rows(root / 'test_events.jsonl')
            if any(row.get('event') == name for row in rows):
                return rows
            time.sleep(.01)
        raise RuntimeError(f'未收到事件 {name}')

    rclpy.init()
    fake = FakeController()
    executor = SingleThreadedExecutor()
    executor.add_node(fake)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    with (root / 'client_output.txt').open('w') as output:
        proc = subprocess.Popen([sys.executable, str(script_path), str(root)], cwd=ROOT,
            stdin=subprocess.PIPE, stdout=output, stderr=subprocess.STDOUT, text=True)
        try:
            rows = wait_event('probe_exit')
            enter = next(x['mono'] for x in rows if x.get('event') == 'probe_enter')
            leave = next(x['mono'] for x in rows if x.get('event') == 'probe_exit')
            assert leave - enter >= .29
            assert not [x for x in rows if x.get('event') == 'monitor_failure']
            feedback = read_rows(root / 'received.jsonl')
            sample = [x['received'] for x in feedback if x.get('event') == 'motor'
                      and x.get('name') == 'joint2' and enter <= x['received'] <= leave]
            assert len(sample) >= 5, f'服务回调阻塞期间反馈未继续：{sample}'
            health = [x for x in feedback if x.get('event') == 'health' and enter <= x['mono'] <= leave]
            assert health and not any(x['issues'] for x in health), health

            # 在隔离节点人为中断全部反馈：不得把恢复后的新帧覆盖故障。
            paused.set()
            deadline = time.monotonic() + 3.
            while time.monotonic() < deadline and not (root / 'audit_blocked').exists():
                time.sleep(.01)
            assert (root / 'audit_blocked').exists(), '故障审计没有进入阻塞探针'
            deadline = time.monotonic() + 1.
            while time.monotonic() < deadline and len(stop_calls) < 2:
                time.sleep(.01)
            assert len(stop_calls) == 2, '故障审计被阻塞时，停止请求仍须到达模拟服务'
            (root / 'release_audit').touch()
            rows = wait_event('monitor_stop_response')
            rows = wait_event('monitor_failure')
            rows = wait_event('monitor_stop_call_start')
            fault = next(x for x in rows if x.get('event') == 'monitor_failure')
            assert any(issue in fault['issues'] for issue in ('source_stale', 'raw_feedback_incomplete_or_stale'))
            start = next(x for x in rows if x.get('event') == 'monitor_stop_call_start')
            returned = next(x for x in rows if x.get('event') == 'monitor_stop_call_return')
            response = next(x for x in rows if x.get('event') == 'monitor_stop_response')
            assert fault['fault_mono'] <= start['at_mono'] <= returned['mono'] <= response['mono']
            assert response['success'] and len(stop_calls) == 2  # 注入探针及故障停止各一次
            paused.clear()
            time.sleep(.3)
            proc.stdin.write('snapshot\nquit\n')
            proc.stdin.flush()
            proc.wait(timeout=5)
            assert proc.returncode == 0
            rows = read_rows(root / 'test_events.jsonl')
            snapshot = next(x for x in rows if x.get('event') == 'snapshot')
            assert snapshot['fault'] == fault['issues']
        finally:
            if proc.poll() is None:
                proc.kill()
                proc.wait()
            executor.shutdown(timeout_sec=3.)
            thread.join(3.)
            fake.destroy_node()
            rclpy.try_shutdown()


if __name__ == '__main__':
    run_probe(Path(sys.argv[1]))

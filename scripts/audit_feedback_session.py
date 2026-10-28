#!/usr/bin/env python3
"""同一会话的有界异步审计入口；默认不打开串口、不启动硬件。

用 --module 包.模块运行既有 main(args)，只配置 rebotarm.audit 日志。
可选 --serial 由本进程独占真实串口并桥接 PTY，传入参数中的 {audit_serial}
替换为 PTY。不会构造第二份 SDK、不主动发命令；原始字节和 SDK 序号分别
留存，不能把缓存观测时间当作线上的到达时间。审计丢弃/写入错误须使验收无效。
"""
import argparse
import importlib
import hashlib
import importlib.metadata
import json
import logging
import os
from pathlib import Path
import queue
import select
import sys
import threading
import time
import tty


class AuditWriter(logging.Handler):
    """生产者只做有界入队；JSON 编码和磁盘操作只在审计线程中执行。"""

    def __init__(self, path, session, *, capacity=8192):
        super().__init__()
        self.stream = Path(path).open('x', encoding='utf-8')
        self.session = session
        self.queue = queue.Queue(maxsize=capacity)
        self.dropped = self.errors = self.written = self.accepted = 0
        self.failure = ''
        self.stopping = threading.Event()
        self.worker = threading.Thread(target=self._write, name='feedback-audit', daemon=True)
        self.worker.start()

    def emit(self, record):
        # 不格式化消息、不等待磁盘；payload 必须是调用点复制的普通数值。
        item = dict(session=self.session, pid=os.getpid(), mono_ns=time.monotonic_ns(),
                    wall_ns=time.time_ns(), event=record.msg, data=getattr(record, 'audit', {}))
        try:
            self.queue.put_nowait(item)
            self.accepted += 1
        except queue.Full:
            self.dropped += 1

    def _line(self, item):
        try:
            self.stream.write(json.dumps(item, ensure_ascii=False, allow_nan=False) + '\n')
            self.stream.flush()
            self.written += 1
        except Exception as exc:
            self.errors += 1
            self.failure = str(exc)

    def _write(self):
        while not self.stopping.is_set() or not self.queue.empty():
            try:
                self._line(self.queue.get(timeout=.05))
            except queue.Empty:
                continue

    def finish(self):
        self.stopping.set()
        self.worker.join(5.)
        if self.worker.is_alive():
            raise RuntimeError('audit writer did not drain; evidence incomplete')
        summary = dict(session=self.session, event='audit_summary', accepted=self.accepted,
                       written=self.written, dropped=self.dropped, errors=self.errors,
                       failure=self.failure)
        self._line(summary)
        self.stream.close()
        return summary


class SerialRelay:
    """仅转发字节的单串口所有者；转发失败原样断流，不伪造反馈或命令。"""

    def __init__(self, channel, baud):
        import serial
        self.serial = serial.Serial(channel, baudrate=baud, timeout=0, exclusive=True,
                                    write_timeout=.05)
        self.master, self.slave = os.openpty()
        tty.setraw(self.slave)
        os.set_blocking(self.master, False)
        self.channel = os.ttyname(self.slave)
        self.stop = threading.Event()
        self.error = ''
        self.worker = threading.Thread(target=self._run, name='audit-serial-relay', daemon=True)
        self.worker.start()

    def _run(self):
        logger = logging.getLogger('rebotarm.audit')
        try:
            while not self.stop.is_set():
                ready, _, _ = select.select([self.master, self.serial.fileno()], [], [], .02)
                for fd in ready:
                    data = os.read(fd, 4096)
                    if not data:
                        raise RuntimeError('serial relay EOF')
                    direction = 'tx' if fd == self.master else 'rx'
                    logger.info('raw_serial', extra={'audit': dict(direction=direction, hex=data.hex())})
                    if direction == 'tx':
                        if self.serial.write(data) != len(data):
                            raise RuntimeError('partial serial write')
                    else:
                        # 保留分段/粘包的原始字节；不能用解码后的值代替原始帧。
                        view = memoryview(data)
                        while view:
                            if not select.select([], [self.master], [], .05)[1]:
                                raise RuntimeError('PTY relay backpressure')
                            view = view[os.write(self.master, view):]
        except Exception as exc:
            self.error = str(exc)
            logger.info('relay_error', extra={'audit': dict(error=self.error)})

    def close(self):
        self.stop.set()
        self.worker.join(1.)
        if self.worker.is_alive():
            raise RuntimeError('serial relay did not stop')
        os.close(self.master)
        os.close(self.slave)
        self.serial.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--session', required=True, help='控制器和运动节点使用相同会话名')
    parser.add_argument('--module', required=True)
    parser.add_argument('--serial', help='仅在现场授权后使用；唯一真实串口所有者')
    parser.add_argument('--baud', type=int, default=921600)
    parser.add_argument('args', nargs=argparse.REMAINDER)
    options = parser.parse_args(argv)
    if options.serial and not any('{audit_serial}' in value for value in options.args):
        parser.error('--serial requires {audit_serial} in node arguments')
    writer = AuditWriter(options.output, options.session)
    logger = logging.getLogger('rebotarm.audit')
    logger.addHandler(writer)
    logger.setLevel(logging.INFO)
    logger.propagate = False
    relay = None
    try:
        relay = SerialRelay(options.serial, options.baud) if options.serial else None
        args = options.args[1:] if options.args[:1] == ['--'] else options.args
        args = [arg.replace('{audit_serial}', relay.channel) if relay else arg for arg in args]
        module = importlib.import_module(options.module)
        package_root = Path(module.__file__).parent
        source_hashes = {str(path.relative_to(package_root)): hashlib.sha256(path.read_bytes()).hexdigest()
                         for path in sorted(package_root.rglob('*.py'))}
        try:
            motorbridge_version = importlib.metadata.version('motorbridge')
        except importlib.metadata.PackageNotFoundError:
            motorbridge_version = 'unavailable'
        logger.info('session_start', extra={'audit': dict(module=options.module,
                    raw_serial=bool(relay), args=args, source_sha256=source_hashes,
                    motorbridge_version=motorbridge_version)})
        module.main(args=args)
    finally:
        if relay:
            relay.close()
        logger.removeHandler(writer)
        summary = writer.finish()
        print(json.dumps(summary, ensure_ascii=False), file=sys.stderr)
        if summary['dropped'] or writer.errors or (relay and relay.error):
            raise RuntimeError('audit incomplete; inspect dropped/errors/relay_error')


if __name__ == '__main__':
    main()

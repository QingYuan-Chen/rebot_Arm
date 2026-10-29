"""现场回归客户端的有界异步审计；写入失败或丢弃使会话失效。"""
import json
import queue
import threading
import time


class Audit:
    def __init__(self, path, capacity=8192):
        self._file = path.open('x', encoding='utf-8')
        self._queue = queue.Queue(maxsize=capacity)
        self._failure = threading.Event()
        self._stop = threading.Event()
        self._record_lock = threading.Lock()
        self.accepted = 0
        self.written = 0
        self.dropped = 0
        self.errors = 0
        self._worker = threading.Thread(target=self._write, name='regression-audit', daemon=True)
        self._worker.start()

    @property
    def failed(self):
        return self._failure.is_set() or not self._worker.is_alive()

    def record(self, event, **values):
        item = dict(event=event, mono=time.monotonic(), wall=time.time_ns(), **values)
        with self._record_lock:
            try:
                self._queue.put_nowait(item)
                self.accepted += 1
            except queue.Full:
                self.dropped += 1
                self._failure.set()

    def _write(self):
        while not self._stop.is_set() or not self._queue.empty():
            try:
                item = self._queue.get(timeout=.05)
            except queue.Empty:
                continue
            try:
                self._file.write(json.dumps(item, ensure_ascii=False, allow_nan=False) + '\n')
                self._file.flush()
                self.written += 1
            except Exception:
                self.errors += 1
                self._failure.set()

    def close(self):
        self._stop.set()
        self._worker.join(5.)
        if self._worker.is_alive():
            self._failure.set()
            raise RuntimeError('验收审计线程未退出')
        summary = dict(event='audit_summary', accepted=self.accepted, written=self.written,
                       dropped=self.dropped, errors=self.errors)
        try:
            self._file.write(json.dumps(summary) + '\n')
            self._file.close()
        except Exception:
            self._failure.set()
            raise
        if self._failure.is_set() or self.accepted != self.written:
            raise RuntimeError(f'验收审计不完整: {summary}')

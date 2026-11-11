"""面板的 HTTP 服务：本机页面、状态查询、SSE 状态流、模型资源与 POST 命令入口。

用标准库 ThreadingHTTPServer 实现，每个连接一个线程；面板节点在启动时创建它，并放进
独立守护线程里 serve_forever。服务只绑定参数给定的 host/port，用于本机 UI/API，不做远程暴露。

GET 路由
    /                       面板页面（内嵌 HTML 文本）
    /api/status             当前状态快照（JSON，非有限数值已转为 null）
    /api/config             面板参数汇总，供前端构建各控件的取值范围
    /api/teach_record_info  单条示教记录的信息（查询参数 path 或 record_path）
    /api/teach_records      示教记录列表
    /api/teach_trajectory   轨迹预览点（查询参数 path、max_points，默认 500）
    /robot/urdf             URDF 原文，网格 URI 已改写为 /robot/meshes/
    /robot/meshes/<文件名>  网格文件（model/stl），文件名经防穿越校验
    /events                 SSE 状态流，按 sse_interval_sec 周期持续推送

POST 路由统一交给同包 status_panel_api 的路由表分发，未登记的路径直接 404。

安全与容错约定
    - POST 只接受路由表内的路径；请求体必须是 JSON 对象，非法请求回 400，且不会触发任何硬件动作。
    - 业务结果用 HTTP 状态码表达是否受理：accepted 为真回 200，否则回 400，前端据此提示原因。
    - 客户端断开（BrokenPipe/ConnectionAborted/ConnectionReset）不视为错误，直接结束该请求线程。
    - 标准库默认会把每个请求写一行 stderr 访问日志，这里用空实现关闭，避免高频轮询刷屏。
"""

from __future__ import annotations

import json
import re
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, unquote, urlsplit

from .status_panel_api import dispatch_post_request, is_allowed_post_path
from .status_panel_state import TeleopStatusStore, encode_sse_event
from .web_robot_assets import rewrite_package_mesh_uris, safe_mesh_path


def _write_response(handler: BaseHTTPRequestHandler, status: int, body: bytes, content_type: str) -> None:
    """写一个带 Content-Type 与 Content-Length 的完整响应。"""
    handler.send_response(status)
    handler.send_header("Content-Type", content_type)
    handler.send_header("Content-Length", str(len(body)))
    handler.end_headers()
    handler.wfile.write(body)


def _json_bytes(payload: dict) -> bytes:
    """把字典编码成紧凑的 UTF-8 JSON 字节串（去掉分隔符后的空格以减小响应体积）。"""
    return json.dumps(payload, separators=(",", ":")).encode("utf-8")


def create_status_panel_server(
    *,
    host: str,
    port: int,
    node: object,
    store: TeleopStatusStore,
    html_page: str,
    urdf_path: Path,
    mesh_dir: Path,
    sse_interval_sec: float,
) -> ThreadingHTTPServer:
    """创建并返回面板 HTTP 服务器实例（已绑定端口，但尚未开始 serve_forever）。

    参数
        host / port：监听地址与端口，来自面板参数；仅用于本机 UI/API。
        node：面板节点对象，通过其 _panel_config、_teach_* 等方法取配置与示教信息；
            这里用鸭子类型持有，避免 HTTP 层反向依赖节点实现。
        store：共享状态存储，GET 查询与 SSE 推送都从它取快照。
        html_page：首页 HTML 文本。
        urdf_path / mesh_dir：URDF 文件与网格目录，分别服务于 /robot/urdf 与 /robot/meshes/。
        sse_interval_sec：SSE 两次推送之间的间隔秒数，由面板参数 sse_rate_hz 取倒数得到。

    绑定失败（端口被占用等）会直接抛出 OSError，属启动期必须暴露的错误。
    """

    class StatusPanelRequestHandler(BaseHTTPRequestHandler):
        def handle(self):  # noqa: D401
            """吞掉客户端主动断开引发的异常，避免请求线程里抛出无意义的堆栈。

            面板页面会定期轮询并在离开时关闭连接，这类断开是正常现象，不应记为错误。
            """
            try:
                super().handle()
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError):
                return

        def do_GET(self):  # noqa: N802
            """处理全部只读 GET 路由；其中 SSE 路由会持续推送直到连接断开。"""
            url = urlsplit(self.path)
            route = url.path
            if route == "/":
                _write_response(self, 200, html_page.encode("utf-8"), "text/html; charset=utf-8")
                return
            if route == "/calibration":
                from importlib.resources import files
                page = files('rebotarm_dashboard.status_panel_assets').joinpath('calibration.html').read_bytes()
                _write_response(self, 200, page, "text/html; charset=utf-8")
                return
            if route == "/api/calibration/export":
                sid = parse_qs(url.query).get('session_id', [''])[0]
                if not re.fullmatch(r'[0-9a-f]{32}', sid):
                    _write_response(self, 400, b'{"message":"invalid session_id"}', 'application/json')
                    return
                try:
                    result = node._handle_calibration_command({'command': 'status', 'session_id': sid})
                    if not result.get('success'):
                        _write_response(self, 503, _json_bytes(result), 'application/json')
                        return
                    body = json.dumps(result['session'], ensure_ascii=False, indent=2, allow_nan=False).encode('utf-8')
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json; charset=utf-8')
                    self.send_header('Content-Disposition', f'attachment; filename="{sid}.json"')
                    self.send_header('Cache-Control', 'no-store')
                    self.send_header('Content-Length', str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                except Exception as exc:
                    _write_response(self, 503, _json_bytes({'message': str(exc)}), 'application/json')
                return
            if route == "/api/status":
                _write_response(self, 200, _json_bytes(store.snapshot_dict()), "application/json")
                return
            if route == "/api/config":
                _write_response(self, 200, _json_bytes(node._panel_config()), "application/json")  # noqa: SLF001
                return
            if route == "/api/teach_record_info":
                query = parse_qs(url.query)
                # 兼容 path 与 record_path 两种查询参数名，取不到时用空串表示“用默认记录”。
                record_path = query.get("path", query.get("record_path", [""]))[0]
                _write_response(
                    self,
                    200,
                    _json_bytes(node._teach_record_info(record_path or None)),  # noqa: SLF001
                    "application/json",
                )
                return
            if route == "/api/teach_records":
                _write_response(self, 200, _json_bytes(node._teach_records()), "application/json")  # noqa: SLF001
                return
            if route == "/api/teach_trajectory":
                query = parse_qs(url.query)
                record_path = query.get("path", [""])[0]
                # max_points 限制单次返回的预览点数，缺省 500，防止长记录一次撑爆前端。
                max_points = int(query.get("max_points", ["500"])[0])
                _write_response(
                    self,
                    200,
                    _json_bytes(node._teach_trajectory(record_path or None, max_points=max_points)),  # noqa: SLF001
                    "application/json",
                )
                return
            if route == "/robot/urdf":
                # 浏览器无法解析网格包协议，返回前先把网格 URI 改写成面板自己的路由。
                body = rewrite_package_mesh_uris(urdf_path.read_text(encoding="utf-8")).encode("utf-8")
                _write_response(self, 200, body, "application/xml; charset=utf-8")
                return
            if route.startswith("/robot/meshes/"):
                name = unquote(route.removeprefix("/robot/meshes/"))
                path = safe_mesh_path(mesh_dir, name)
                if path is None:
                    # 文件名非法或文件不存在，统一回 404，不泄露目录内容。
                    self.send_response(404)
                    self.end_headers()
                    return
                _write_response(self, 200, path.read_bytes(), "model/stl")
                return
            if route == "/events":
                # SSE：先发流式头部，之后按固定周期持续推送状态帧，直到该连接结束。
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                try:
                    while True:
                        self.wfile.write(encode_sse_event(store.snapshot_dict()).encode("utf-8"))
                        self.wfile.flush()
                        time.sleep(sse_interval_sec)
                except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError):
                    # 浏览器断连时结束该请求线程。
                    return
            # 以上分支都没命中时的兜底响应（/events 分支内部是死循环，正常不会走到这里）。
            self.send_response(404)
            self.end_headers()

        def do_POST(self):  # noqa: N802
            """处理命令类 POST：先查路由白名单，再按需读取 JSON 请求体并分发。"""
            if not is_allowed_post_path(self.path):
                self.send_response(404)
                self.end_headers()
                return
            try:
                def read_payload() -> dict:
                    """按需读取请求体并解析为 JSON 对象。

                    缺少或长度为 0 的请求体按空对象处理；解析结果不是对象（数组、字符串等）
                    时抛 ValueError，由外层统一转成 400 响应。
                    """
                    length = int(self.headers.get("Content-Length", "0"))
                    raw = self.rfile.read(length) if length > 0 else b"{}"
                    payload = json.loads(raw.decode("utf-8"))
                    if not isinstance(payload, dict):
                        raise ValueError("payload must be an object")
                    return payload

                result = dispatch_post_request(node, self.path, read_payload)
                # 业务层用 accepted 表达是否受理：受理回 200，否则回 400 供前端提示原因。
                success = result.get("success") if self.path == "/api/calibration/command" else result.get("accepted")
                status = 200 if success else 400
            except Exception as exc:
                # 请求体非法、处理方法抛异常等都归为“无效请求”，不向客户端泄露内部堆栈。
                result = {"accepted": False, "message": f"invalid web execute request: {exc}"}
                status = 400
            _write_response(self, status, _json_bytes(result), "application/json")

        def log_message(self, *_args):
            """关闭标准库默认的访问/错误日志输出（默认写 stderr），避免高频请求刷屏。"""
            return

    return ThreadingHTTPServer((host, port), StatusPanelRequestHandler)

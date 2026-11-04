from __future__ import annotations

# 示教录制节点：把操作者拖动机械臂期间的关节反馈逐行写入示教记录文件。
#
# 职责
# ----
# 本节点是示教数据链路的源头，只负责“采集并落盘原始示教数据”，不做任何轨迹
# 预处理、回放校验或运动执行。
#
# 对外接口（``arm_namespace`` 默认取 "rebotarm"，下列名称中的该段可被参数替换）
# ----------------------------------------------------------------------------
# - 订阅 ``/{arm_namespace}/joint_states``：关节位置/速度/力矩，BEST_EFFORT + KEEP_LAST(depth=10)。
# - 订阅 ``/{arm_namespace}/arm_status``：状态机名称，RELIABLE + TRANSIENT_LOCAL，
# 以保证本节点晚于控制器启动时仍能收到最后一次状态（判断是否处于重力补偿）。
# - 订阅 ``/{arm_namespace}/joints/{joint_name}/state``：每个关节的电机状态码，用于
# 确认反馈健康（状态码必须为 0 或 1）。
# - 发布 ``/{arm_namespace}/teleop/recording_status``：JSON 文本状态，含写入状态、
# 已写样本数、实际采样率、文件大小等，供上层界面展示。
# - 服务 ``/{arm_namespace}/teleop/teach_record/start``：显式开始录制（截断旧文件）。
# - 服务 ``/{arm_namespace}/teleop/teach_record/set_path``：录制前设置记录文件路径。
# - 服务 ``/{arm_namespace}/teleop/teach_record/stop``：停止录制并关闭文件。
# - 客户端调用 ``/{arm_namespace}/gravity_compensation/start``：可选的自动进入重力补偿。
#
# 关键流程
# --------
# 定时器以 ``sample_rate_hz`` 周期触发 ``TeachRecorderNode._write_sample``，
# 把缓存的关节状态转换为一条示教样本并追加写入记录文件（JSONL，每行一条 JSON）。
# 写入门控依次为：录制已开启 -> 已收到 joint_states -> （可选）状态机为 GRAVITY_COMP
# -> 反馈时间戳新鲜且来自同一批次 -> （可选）电机状态健康。任一门控不通过都只更新
# 状态并跳过本次写入，不会中断录制。
#
# 安全约束
# --------
# 1. 示教依赖重力补偿：``require_gravity_comp`` 为真时，只有状态机报告 GRAVITY_COMP
# 才写样本，避免把抱闸/失能状态的“假拖动”数据记进示教文件。
# 2. 反馈新鲜度：拒绝过期或来自未来的时间戳，防止复写旧数据或错位对齐。
# 3. 反馈同批次：要求关节状态与电机状态来自同一时间戳，避免位置与状态码不同步。

import json
import select
import sys
import termios
import tty
from contextlib import suppress
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rebotarm_msgs.msg import ArmStatus, JointMotorState
from rebotarm_msgs.srv import SetTeachRecordPath
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .parameter_helpers import sensor_qos_kwargs
from .teach_recording import encode_teach_sample, is_quit_key
from .recording_feedback import feedback_teach_sample, stamp_nanoseconds

class TeachRecorderNode(Node):
    """示教录制节点。

    生命期：节点启动后声明参数、按需建立订阅/定时器；当 ``start_on_launch``
    为真时立即开始录制（追加模式），否则等待 ``start`` 服务。节点销毁时关闭记录
    文件并恢复终端属性。

    回调模型：所有服务回调与定时器回调都跑在同一个 executor 线程上，因此对
    ``self._handle``、计数器和缓存状态的访问无需额外加锁；这也意味着任何回调都
    不能长时间阻塞。
    """

    def __init__(self) -> None:
        # 参数含义（声明顺序与下方 declare_parameter 一致）：
        # arm_namespace：话题与服务名称中的机械臂命名空间前缀，会去掉首尾斜杠，
        # 留空表示直接使用顶层名称。
        #
        # record_path：示教记录文件路径（JSONL）。父目录不存在时会自动创建。
        # sample_rate_hz：写样本的定时器频率（Hz），默认 50.0；写入门控不通过时该
        #   周期只产生一次状态更新而不落盘。
        # require_gravity_comp：为真时要求状态机处于 GRAVITY_COMP 才允许写入样本。
        # require_motor_status：为真时要求同一反馈批次内每个关节的电机状态码为 0/1。
        # feedback_timeout_sec：允许的反馈时延上限（秒），超过即视为过期数据。
        # auto_start_gravity_comp：为真时本节点周期性尝试调用重力补偿启动服务。
        # auto_start_gravity_comp_retry_sec：上述重试周期（秒），内部下限取 0.2 秒。
        # auto_start_gravity_comp_max_attempts：重试次数上限，<=0 表示不限次数。
        # keyboard_quit_enabled：为真时监听终端按键以退出（需要 TTY）。
        # quit_key：触发退出的按键字符，与 ``keyboard_quit_enabled`` 配合使用。
        # start_on_launch：为真时节点启动即开始录制，否则等待 start 服务。
        # joint_names：期望记录的关节顺序，也是写入样本中各向量的顺序。
        super().__init__("teach_recorder_node")

        self.declare_parameter("arm_namespace", "rebotarm")
        self.declare_parameter("record_path", "teleop_records/teach_record.jsonl")
        self.declare_parameter("sample_rate_hz", 50.0)
        self.declare_parameter("require_gravity_comp", True)
        self.declare_parameter("require_motor_status", True)
        self.declare_parameter("feedback_timeout_sec", 0.15)
        self.declare_parameter("auto_start_gravity_comp", False)
        self.declare_parameter("auto_start_gravity_comp_retry_sec", 1.0)
        self.declare_parameter("auto_start_gravity_comp_max_attempts", 30)
        self.declare_parameter("keyboard_quit_enabled", True)
        self.declare_parameter("quit_key", "q")
        self.declare_parameter("start_on_launch", True)
        self.declare_parameter(
            "joint_names",
            ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
        )
        self._arm_namespace = str(self.get_parameter("arm_namespace").value).strip("/")
        self._joint_names = tuple(str(v) for v in self.get_parameter("joint_names").value)
        self._latest_gripper_state = None
        self._record_path = Path(str(self.get_parameter("record_path").value))
        self._record_path.parent.mkdir(parents=True, exist_ok=True)
        self._handle = None
        self._recording_active = False
        self._require_gravity_comp = bool(self.get_parameter("require_gravity_comp").value)
        # 反馈缓存：由订阅回调不断刷新，写入定时器只读取最新一帧。
        self._latest_joint_state: JointState | None = None
        self._arm_state = ""
        self._motor_status: dict[str, int] = {}
        self._motor_stamps: dict[str, int] = {}
        # 已落盘样本的关节时间戳（纳秒），用于保证记录内时间戳严格递增、不重写同一帧。
        self._last_written_stamp_ns: int | None = None
        self._recording_started_ns = 0
        self._samples_written = 0
        # 样本 stamp 是相对录制开始时刻的秒数，因此这里的起止时间也用于统计实际采样率。
        self._first_sample_stamp: float | None = None
        self._last_sample_stamp: float | None = None
        self._writing_state = "open"
        self._last_write_error = ""
        self._auto_start_attempts = 0
        self._auto_start_in_flight = False
        self._terminal_settings = None
        self._keyboard_stream = None
        self._owns_keyboard_stream = False
        self._shutdown_requested = False
        # 状态话题以 JSON 文本发布，深度 10 只保留最近若干条状态，丢失旧状态无影响。
        self._status_pub = self.create_publisher(
            String,
            f"/{self._arm_namespace}/teleop/recording_status",
            10,
        )
        self.create_service(
            Trigger,
            f"/{self._arm_namespace}/teleop/teach_record/start",
            self._handle_start_recording,
        )
        self.create_service(
            SetTeachRecordPath,
            f"/{self._arm_namespace}/teleop/teach_record/set_path",
            self._handle_set_record_path,
        )
        self.create_service(
            Trigger,
            f"/{self._arm_namespace}/teleop/teach_record/stop",
            self._handle_stop_recording,
        )
        sensor_qos_spec = sensor_qos_kwargs()
        # 关节/电机反馈按传感器流处理：BEST_EFFORT + KEEP_LAST，允许丢帧，
        # 只要新数据持续到达即可，不重传历史帧。
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=int(sensor_qos_spec["depth"]),
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        # 状态机信息必须可靠且保留最后一条：本节点可能晚于控制器启动，
        # 若没有 TRANSIENT_LOCAL 就拿不到当前状态，重力补偿门控会一直不通过。
        arm_status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            JointState,
            f"/{self._arm_namespace}/joint_states",
            self._on_joint_state,
            sensor_qos,
        )
        self.create_subscription(
            ArmStatus,
            f"/{self._arm_namespace}/arm_status",
            self._on_arm_status,
            arm_status_qos,
        )
        for joint_name in self._joint_names:
            self.create_subscription(
                JointMotorState,
                f"/{self._arm_namespace}/joints/{joint_name}/state",
                self._on_motor_state,
                sensor_qos,
            )
        self.create_subscription(JointMotorState,f'/{self._arm_namespace}/gripper/state',
            lambda message:setattr(self,'_latest_gripper_state',message),sensor_qos)
        self._gravity_start_client = self.create_client(
            Trigger,
            f"/{self._arm_namespace}/gravity_compensation/start",
        )
        if bool(self.get_parameter("auto_start_gravity_comp").value):
            # 下限 0.2 秒，避免把重试周期压到过密而拖垮执行器线程。
            retry_sec = max(float(self.get_parameter("auto_start_gravity_comp_retry_sec").value), 0.2)
            self.create_timer(retry_sec, self._try_auto_start_gravity_comp)
        # 采样周期下限由 1 Hz 保护，防止 sample_rate_hz 被设为 0 或负数时除零。
        period = 1.0 / max(float(self.get_parameter("sample_rate_hz").value), 1.0)
        self.create_timer(period, self._write_sample)
        if bool(self.get_parameter("keyboard_quit_enabled").value):
            self._setup_keyboard_quit()
            # 按键轮询固定 20 Hz：足够灵敏，又不会明显占用执行器线程。
            self.create_timer(0.05, self._poll_quit_key)
        if bool(self.get_parameter("start_on_launch").value):
            # 启动即录制采用追加模式，避免同一次会话中的服务重启覆盖已有示教数据。
            self._start_recording(truncate=False)
            message = f"recording to {self._record_path}; press q to stop"
        else:
            message = f"teach recorder idle; service start writes to {self._record_path}"
        self._publish_status("ready" if self._recording_active else "idle", message)

    def _start_recording(self, *, truncate: bool) -> None:
        """打开记录文件并进入录制状态；文件已打开时直接返回（幂等）。

        ``truncate`` 为真表示覆盖写入（start 服务显式要求重录），为假表示追加写入
        （节点启动自动录制）。同时把样本计数与时间基准清零，使新一段录制的 stamp
        从 0 秒重新开始。
        """
        if self._handle is not None:
            return
        self._record_path.parent.mkdir(parents=True, exist_ok=True)
        mode = "w" if truncate else "a"
        self._handle = self._record_path.open(mode, encoding="utf-8")
        self._recording_active = True
        self._recording_started_ns = self.get_clock().now().nanoseconds
        self._last_written_stamp_ns = None
        self._samples_written = 0
        self._first_sample_stamp = None
        self._last_sample_stamp = None
        self._writing_state = "open"
        self._last_write_error = ""

    def _normalize_record_path(self, value: str) -> Path:
        """把服务请求里的路径规范化到 ``teleop_records/`` 目录下的 .jsonl 文件。

        只保留文件名部分（反斜杠统一按分隔符处理），因此调用方无法借该服务写到
        目录之外，也避免覆盖工程里其他文件。名称为空时回退成 "teach_record"，
        缺少扩展名时补 ".jsonl"；补全后若名称为空、恰为扩展名或含 ".." 片段，
        则抛 ValueError 拒绝。
        """
        raw = str(value).strip()
        if not raw:
            raw = "teach_record"
        raw = raw.replace("\\", "/")
        name = Path(raw).name
        if not name.endswith(".jsonl"):
            name = f"{name}.jsonl"
        if name in (".jsonl", "/", "") or ".." in Path(name).parts:
            raise ValueError("invalid teach record file name")
        return Path("teleop_records") / name

    def _stop_recording(self) -> None:
        """关闭记录文件并标记停止；未在录制时也可安全调用。"""
        if self._handle is not None:
            self._handle.close()
            self._handle = None
        self._recording_active = False
        self._writing_state = "stopped"

    def _handle_start_recording(self, _request, response):
        """start 服务回调：开始录制（覆盖同名旧文件）。已在录制时视为成功并返回当前路径。"""
        if self._recording_active:
            response.success = True
            response.message = f"teach recording already active: {self._record_path}"
            self._publish_status("recording", response.message)
            return response
        try:
            self._start_recording(truncate=True)
            response.success = True
            response.message = f"teach recording started: {self._record_path}"
            self._publish_status("starting", response.message)
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            self._publish_status("error", f"failed to start teach recording: {exc}")
        return response

    def _handle_set_record_path(self, request, response):
        """set_path 服务回调：设置下一次录制的记录文件路径。

        录制过程中拒绝修改（否则同一段录制的数据会跨文件），失败时回填当前生效路径
        供调用方参考。
        """
        if self._recording_active:
            response.success = False
            response.message = "cannot change record path while recording"
            response.normalized_path = str(self._record_path)
            return response
        try:
            self._record_path = self._normalize_record_path(request.record_path)
            self._record_path.parent.mkdir(parents=True, exist_ok=True)
            response.success = True
            response.message = f"teach record path set: {self._record_path}"
            response.normalized_path = str(self._record_path)
            self._publish_status("idle", response.message)
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            response.normalized_path = str(self._record_path)
        return response

    def _handle_stop_recording(self, _request, response):
        """stop 服务回调：停止录制并把本次落盘的样本数回给调用方；重复调用视为成功。"""
        if not self._recording_active:
            response.success = True
            response.message = "teach recording already stopped"
            self._publish_status("stopped", response.message)
            return response
        self._stop_recording()
        response.success = True
        response.message = f"teach recording stopped: samples={self._samples_written}"
        self._publish_status("stopped", response.message)
        return response

    def _setup_keyboard_quit(self) -> None:
        """准备单键读取的终端环境。

        启动方式不同会导致 stdin 不是终端（例如被 launch 重定向），此时改用
        ``/dev/tty`` 直连控制终端，并记录“该流由本节点打开”以便销毁时关闭。
        使用 cbreak 模式（而非 raw）保留信号处理，同时关闭行缓冲，使按键无需回车
        即可被读到；原始终端属性保存在 ``_terminal_settings``，销毁时恢复。
        任何失败都只降级为警告：没有键盘退出不影响录制本身。
        """
        try:
            self._keyboard_stream = sys.stdin
            if not self._keyboard_stream.isatty():
                tty_path = Path("/dev/tty")
                if tty_path.exists():
                    self._keyboard_stream = tty_path.open("r", encoding="utf-8", buffering=1)
                    self._owns_keyboard_stream = True
            if self._keyboard_stream.isatty():
                self._terminal_settings = termios.tcgetattr(self._keyboard_stream)
                tty.setcbreak(self._keyboard_stream.fileno())
                self.get_logger().info("press q to stop teach recording")
            else:
                self.get_logger().warn("keyboard quit unavailable: stdin is not a terminal")
        except Exception as exc:
            self.get_logger().warn(f"keyboard quit unavailable: {exc}")
            self._keyboard_stream = None

    def _poll_quit_key(self) -> None:
        """轮询退出按键；命中后先停录再请求关闭 rclpy。

        超时为 0 的 select 保证回调不被阻塞；读取到空字符串时直接返回，不会误判为
        按键。命中退出键后先停录（关闭文件）、再关闭 rclpy，保证已写入的样本先落盘。
        """
        if self._shutdown_requested:
            return
        if self._keyboard_stream is None:
            return
        try:
            readable, _, _ = select.select([self._keyboard_stream], [], [], 0.0)
            if not readable:
                return
            key = self._keyboard_stream.read(1)
        except Exception:
            return
        if key == "":
            return
        if is_quit_key(key, quit_key=str(self.get_parameter("quit_key").value)):
            self._shutdown_requested = True
            self._publish_status("stopped", "quit key pressed; stopping recorder")
            self.get_logger().info("quit key pressed; stopping teach recorder")
            self._stop_recording()
            if rclpy.ok():
                rclpy.shutdown()

    def _try_auto_start_gravity_comp(self) -> None:
        """尝试自动请求进入重力补偿；状态已是 GRAVITY_COMP 或已有请求在途时跳过。

        超时设为 0 表示非阻塞探测服务是否就绪，未就绪只累加尝试次数并发布等待状态，
        不阻塞执行器线程。达到 ``auto_start_gravity_comp_max_attempts`` 上限后不再重试，
        改为提示人工启动，避免无限重试掩盖真实故障。同一时刻只允许一个请求在途
        （``_auto_start_in_flight``），避免重复触发启动流程；是否真的重复启动由被调用
        服务一侧负责判定。
        """
        if self._shutdown_requested or self._arm_state == "GRAVITY_COMP":
            return
        if self._auto_start_in_flight:
            return
        max_attempts = int(self.get_parameter("auto_start_gravity_comp_max_attempts").value)
        if max_attempts > 0 and self._auto_start_attempts >= max_attempts:
            self._publish_status("waiting", "gravity compensation start service unavailable; start manually")
            return
        if not self._gravity_start_client.wait_for_service(timeout_sec=0.0):
            self._auto_start_attempts += 1
            self._publish_status(
                "waiting",
                f"waiting for gravity compensation start service; attempt={self._auto_start_attempts}",
            )
            return
        self._auto_start_attempts += 1
        self._auto_start_in_flight = True
        future = self._gravity_start_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_auto_start_gravity_comp_done)

    def _on_auto_start_gravity_comp_done(self, future) -> None:
        """重力补偿启动请求的完成回调：只发布状态，不做重试（重试由定时器负责）。"""
        self._auto_start_in_flight = False
        try:
            response = future.result()
        except Exception as exc:
            self._publish_status("waiting", f"gravity compensation start failed: {exc}")
            return
        if bool(getattr(response, "success", False)):
            self._publish_status("starting", "gravity compensation start requested")
        else:
            message = str(getattr(response, "message", "gravity compensation start rejected"))
            self._publish_status("waiting", message)

    def _on_joint_state(self, msg: JointState) -> None:
        self._latest_joint_state = msg

    def _on_arm_status(self, msg: ArmStatus) -> None:
        self._arm_state = str(msg.state_machine)

    def _on_motor_state(self, msg: JointMotorState) -> None:
        # 与电机状态一起记录其时间戳，写入时要求与关节状态时间戳完全一致，
        # 保证“位置”和“健康状态”属于同一帧反馈。
        self._motor_status[str(msg.joint_name)] = int(msg.status_code)
        self._motor_stamps[str(msg.joint_name)] = stamp_nanoseconds(msg.header.stamp)

    def _write_sample(self) -> None:
        """写样本定时器回调：门控通过时把最新反馈追加为一行 JSONL。

        门控顺序（任一不通过都只更新状态并返回，不写入、不报错）：

        1. 录制已开启，否则状态置 idle；
        2. 已收到关节状态，否则等待 joint_states；
        3. ``require_gravity_comp`` 为真时必须处于 GRAVITY_COMP，否则等待状态机；
        4. 交给反馈构造函数做时间戳与一致性校验，失败时抛出 ValueError 并降级为等待；
        5. 构造返回 None 表示该帧已被写过或是录制开始前的旧帧，静默跳过。

        每行写完立即 flush，使异常退出时已落盘的样本仍可用（示教过程可随时中断）。
        """
        if not self._recording_active:
            self._writing_state = "idle"
            self._publish_status("idle", "teach recorder idle")
            return
        joint_state = self._latest_joint_state
        if joint_state is None:
            self._writing_state = "waiting_joint_states"
            self._publish_status("waiting", "waiting for joint_states")
            return
        if self._require_gravity_comp and self._arm_state != "GRAVITY_COMP":
            self._writing_state = "waiting_gravity_comp"
            self._publish_status("waiting", "waiting for GRAVITY_COMP state")
            return
        try:
            sample = feedback_teach_sample(
                joint_state, joint_names=self._joint_names,
                motor_status=self._motor_status, motor_stamps=self._motor_stamps,
                arm_state=self._arm_state, now_ns=self.get_clock().now().nanoseconds,
                started_ns=self._recording_started_ns,
                last_stamp_ns=self._last_written_stamp_ns,
                timeout_sec=float(self.get_parameter("feedback_timeout_sec").value),
                require_motor_status=bool(self.get_parameter("require_motor_status").value),
            )
        except ValueError as exc:
            self._writing_state = "waiting_feedback"
            self._publish_status("waiting", str(exc))
            return
        if sample is None:
            return
        from dataclasses import replace
        import math
        gripper = self._latest_gripper_state
        source_ns = stamp_nanoseconds(gripper.header.stamp) if gripper is not None else None
        arm_ns = stamp_nanoseconds(joint_state.header.stamp)
        timeout_ns = int(float(self.get_parameter('feedback_timeout_sec').value)*1e9)
        valid = (gripper is not None and source_ns is not None
            and 0 <= self.get_clock().now().nanoseconds-source_ns <= timeout_ns
            and abs(arm_ns-source_ns) <= timeout_ns
            and int(gripper.status_code) in (0,1) and math.isfinite(gripper.position)
            and 0 <= gripper.position <= .09)
        sample = replace(sample,format_version=2,gripper_width_m=float(gripper.position) if valid else None,
            gripper_source_stamp=(source_ns-self._recording_started_ns)/1e9 if source_ns is not None else None,
            gripper_valid=valid)
        sample_stamp = sample.stamp
        if self._first_sample_stamp is None:
            self._first_sample_stamp = sample_stamp
        self._last_sample_stamp = sample_stamp
        try:
            if self._handle is None:
                raise OSError("record file is not open")
            self._handle.write(encode_teach_sample(sample) + "\n")
            self._handle.flush()
        except OSError as exc:
            self._writing_state = "error"
            self._last_write_error = str(exc)
            self._publish_status("error", f"failed to write teach sample: {exc}")
            return
        self._writing_state = "writing"
        self._last_write_error = ""
        self._last_written_stamp_ns = stamp_nanoseconds(joint_state.header.stamp)
        self._samples_written += 1
        self._publish_status("recording", f"samples={self._samples_written}")

    def _publish_status(self, state: str, message: str) -> None:
        """把当前录制状态以紧凑 JSON 发布到状态话题。

        ``state`` 是上层界面直接展示的状态串（如 recording/waiting/error），
        ``writing_state`` 则记录更细的写入子状态，两者配合便于定位卡在哪个门控。
        实际采样率按 (样本数 - 1) / 首末样本时间差估算，因此样本不足 2 条或时间跨度
        为 0 时保持 0.0；文件大小读取失败（文件尚未创建）时按 0 处理。
        """
        msg = String()
        elapsed_sec = 0.0
        if self._first_sample_stamp is not None and self._last_sample_stamp is not None:
            elapsed_sec = max(0.0, self._last_sample_stamp - self._first_sample_stamp)
        actual_sample_rate_hz = 0.0
        if elapsed_sec > 0.0 and self._samples_written > 1:
            actual_sample_rate_hz = (self._samples_written - 1) / elapsed_sec
        file_size_bytes = 0
        with suppress(OSError):
            file_size_bytes = int(self._record_path.stat().st_size)
        if state == "stopped":
            self._writing_state = "stopped"
        msg.data = json.dumps(
            {
                "state": state,
                "message": message,
                "record_path": str(self._record_path),
                "samples": self._samples_written,
                "elapsed_sec": elapsed_sec,
                "sample_rate_hz": float(self.get_parameter("sample_rate_hz").value),
                "actual_sample_rate_hz": actual_sample_rate_hz,
                "last_sample_time": self._last_sample_stamp,
                "file_size_bytes": file_size_bytes,
                "writing_state": self._writing_state,
                "last_write_error": self._last_write_error,
                "arm_state": self._arm_state,
                "require_gravity_comp": self._require_gravity_comp,
                "gravity_comp_active": self._arm_state == "GRAVITY_COMP",
            },
            separators=(",", ":"),
        )
        self._status_pub.publish(msg)

    def destroy_node(self) -> bool:
        """销毁节点前先收尾：停止录制、恢复终端属性、关闭自打开的终端流。

        ``TCSADRAIN`` 保证已提交的输出写完后再恢复，避免终端残留 cbreak 状态导致
        后续命令无法正常回显。无论收尾过程如何，都要把异常交给父类销毁流程处理。
        """
        try:
            self._stop_recording()
            if self._terminal_settings is not None:
                stream = self._keyboard_stream if self._keyboard_stream is not None else sys.stdin
                termios.tcsetattr(stream, termios.TCSADRAIN, self._terminal_settings)
            if self._owns_keyboard_stream and self._keyboard_stream is not None:
                self._keyboard_stream.close()
        finally:
            return super().destroy_node()


def main(args=None) -> None:
    """进程入口：初始化 rclpy、创建节点并自旋，退出时保证节点销毁与 rclpy 关闭。

    Ctrl-C 在自旋与收尾两个阶段都可能到达，因此分处捕获并抑制，确保记录文件仍被
    正常关闭而不留下半行数据。
    """
    rclpy.init(args=args)
    node = TeachRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        with suppress(KeyboardInterrupt):
            node.destroy_node()
        if rclpy.ok():
            with suppress(KeyboardInterrupt):
                rclpy.shutdown()


if __name__ == "__main__":
    main()

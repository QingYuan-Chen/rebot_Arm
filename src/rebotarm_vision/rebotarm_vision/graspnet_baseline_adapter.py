"""GraspNet 基线抓取检测的适配层（本视觉包内的纯算法/接口模块，不直接执行运动）。

职责与在系统中的位置
--------------------
本模块把"ROS 侧的彩色图 + 深度图 + CameraInfo + 2D 检测框"翻译成 GraspNet 基线推理引擎
的输入，再把推理输出翻译成下游可消费的抓取候选数组消息。它自己不订阅话题、不发布消息，
真正的 ROS 节点（抓取候选节点）负责取帧、缓存与话题收发。

对外接口（供上层节点与测试调用）
--------------------------------
- :func:`closest_timestamped_frame`：在时间戳缓存里找与目标时间戳最接近的一帧，用于彩色图/
  深度图/检测结果的时间对齐；找不到（目标时间戳非正或缓存为空）时返回 None。
- :func:`build_point_cloud_for_detection`：按检测框裁剪深度图并反投影出相机坐标系下的点云
  （单位 m），同时取回对应像素颜色；返回 :class:`PointCloudCrop`。
- :func:`predictions_to_candidate_array` / :func:`payload_to_candidate_array`：把推理结果
  （对象、字典或嵌套 payload）统一转成候选数组消息，按输入顺序即得分降序排列，最优候选
  ``best_index`` 置 0（列表为空时保持 -1）。
- :class:`GraspNetBackendProtocol` / :class:`InProcessGraspNetBackend`：正式节点使用的推理接口。

正式后端
--------
``InProcessGraspNetBackend`` 是当前 Ubuntu 生产路线，在本 ROS 候选节点进程内直接加载完整的
   RGB-D 推理引擎，并保留原有的全场景碰撞点云、目标掩膜/深度分离、确定性采样、投影过滤与
   夹爪开口过滤，同时去掉了历史上的 localhost JSON/HTTP 传输层。

坐标与单位约定
--------------
- 点云与候选位姿同处相机光学坐标系（z 轴朝前、x 朝右、y 朝下），与深度图 ``frame_id``
  （通常为 ``camera_depth_frame``）一致；下游由 TF 适配把位姿换到机械臂基座系。
- 长度统一用米：深度图整型像素值按 0.001 换算或由上游直接给浮点米制数组；四元数无量纲。
- 颜色统一为 RGB 顺序、取值归一化到 [0, 1]；若输入是 OpenCV 的 BGR 则需要翻转通道。

安全语义
--------
后端不可用（模型路径或权重缺失、模块导入失败等）时，``available`` 为 False 且 ``infer``
抛出 RuntimeError：宁可显式失败也不返回空候选去"假装正常"。本模块只产出候选位姿建议，
任何抓取执行都必须经过下游的 IK 可行性、碰撞检查与执行门控，不得在此处绕过。
"""

from __future__ import annotations

from dataclasses import dataclass
import importlib
import importlib.util
import math
import sys
from pathlib import Path
from typing import Any, Iterable, Protocol

import numpy as np

from rebotarm_msgs.msg import Detection2D, GraspCandidate, GraspCandidateArray


@dataclass(frozen=True)
class CameraIntrinsics:
    """针孔相机内参（全部为像素单位，与深度图同一分辨率）。

    fx / fy：x、y 方向焦距，单位 pixel；反投影时 ``x = (u - cx) * z / fx``。
    cx / cy：主点（光心）像素坐标，单位 pixel，通常接近图像中心。
    反投影只在像素坐标与米之间换算，不改变坐标系朝向。
    """

    fx: float
    fy: float
    cx: float
    cy: float


@dataclass(frozen=True)
class PointCloudCrop:
    """由检测框裁剪出的局部点云及其颜色。

    points：形状 (N, 3) 的 float32 数组，相机坐标系下的 (x, y, z)，单位 m。
    colors：形状 (N, 3) 的 float32 数组，与 points 逐行对应，RGB 顺序、取值范围 [0, 1]。
    无有效深度时两数组都是形状 (0, 3) 的空数组（下游需容忍空输入）。
    """

    points: np.ndarray
    colors: np.ndarray


def closest_timestamped_frame(frames, target_timestamp_ns: int):
    """在帧缓存中返回与目标时间戳最接近的一帧，找不到时返回 None。

    frames 是形如 ``(timestamp_ns, 数据[, 其他字段])`` 的可迭代序列（节点侧用 maxlen=8 的
    双端队列维护最近若干帧）。target_timestamp_ns 为 0 或负值表示"无有效目标时间戳"，
    此时不猜帧、直接返回 None；缓存为空同理。
    """
    if target_timestamp_ns <= 0 or not frames:
        return None
    return min(frames, key=lambda item: abs(int(item[0]) - target_timestamp_ns))


@dataclass(frozen=True)
class GraspNetPrediction:
    """一次 GraspNet 推理给出的单个抓取建议（与消息字段一一对应）。

    score：抓取得分，无量纲，越大越好；已由后端按分数降序排列。
    translation_xyz：末端夹爪中心在相机坐标系下的位置，单位 m。
    rotation_matrix：3x3 旋转矩阵（行优先），描述末端抓取姿态；正交化由后端保证，
      转四元数时才会再次归一化。
    width_m：抓取该目标所需的夹爪开口宽度，单位 m；超出夹爪行程的候选必须被下游丢弃。
    object_length_m：目标在夹爪闭合方向之外的估计长度，单位 m；用于判断夹爪是否与目标或
      桌面干涉。取不到该信息时保持 0.0（表示未知，而不是"长度为零"）。
    """

    score: float
    translation_xyz: tuple[float, float, float]
    rotation_matrix: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]
    width_m: float
    object_length_m: float = 0.0


class GraspNetBackendProtocol(Protocol):
    """推理后端需要满足的最小接口（结构化子类型，不要求真实继承）。

    ``available`` 为 False 表示后端未配置成功，调用方应走"失败关闭"分支而不是继续推理；
    ``infer`` 接收点云与点数上限，返回按得分降序的预测列表。
    """

    @property
    def available(self) -> bool:
        ...

    def infer(self, *, points: np.ndarray, colors: np.ndarray, max_grasps: int) -> list[GraspNetPrediction]:
        ...


def build_point_cloud_for_detection(
    depth_mm: np.ndarray,
    color_bgr: np.ndarray | None,
    detection: Detection2D,
    intrinsics: CameraIntrinsics,
    *,
    min_depth_m: float,
    max_depth_m: float,
) -> PointCloudCrop:
    """把检测框内的深度像素反投影为相机坐标系点云，并取回对应颜色。

    depth_mm：二维深度图，整型时单位为毫米（内部乘 0.001 转为米），非二维会抛 ValueError。
    color_bgr：与深度同分辨率的 BGR 三通道图；传 None 时用 0.5 的灰色常量填充颜色，
      以便后端在无彩色输入时仍能推理。
    detection：二维检测结果，使用 x_min/y_min/x_max/y_max 作为感兴趣区域（ROI），单位 pixel。
    intrinsics：针孔内参，见 :class:`CameraIntrinsics`。
    min_depth_m / max_depth_m：有效深度闭区间，单位 m；区间外的像素（含 0 值与 NaN/Inf）
      一律视为无效并丢弃，用于滤掉深度空洞与量程外噪声。
    返回 :class:`PointCloudCrop`；ROI 内无任何有效像素时返回空点云（而不是抛异常）。
    """
    depth = np.asarray(depth_mm)
    if depth.ndim != 2:
        raise ValueError("depth image must be a 2D array")
    height, width = depth.shape
    # ROI 边界裁剪到图像范围内；x_max/y_max 用 min(width, ...) 而非 width-1，配合下面的
    # 半开区间切片即可覆盖最后一列/行。再用 max(x_min + 1, ...) 保证切片至少一行/一列，
    # 避免倒置或零尺寸 ROI 导致空切片与 np.nonzero 的歧义。
    x_min = max(0, min(width - 1, int(detection.x_min)))
    y_min = max(0, min(height - 1, int(detection.y_min)))
    x_max = max(x_min + 1, min(width, int(detection.x_max)))
    y_max = max(y_min + 1, min(height, int(detection.y_max)))

    # 深度整型单位为毫米，* 0.001 转成米；用 float32 与后端输入精度保持一致。
    roi_depth_m = depth[y_min:y_max, x_min:x_max].astype(np.float32) * 0.001
    # isfinite 排除 NaN/Inf，深度区间过滤排除 0（无效）与量程外像素。
    valid = np.isfinite(roi_depth_m) & (roi_depth_m >= float(min_depth_m)) & (roi_depth_m <= float(max_depth_m))
    v_local, u_local = np.nonzero(valid)
    if len(u_local) == 0:
        return PointCloudCrop(points=np.empty((0, 3), dtype=np.float32), colors=np.empty((0, 3), dtype=np.float32))

    z = roi_depth_m[v_local, u_local]
    # 把 ROI 内的局部像素下标还原成整幅图像的像素坐标（u 为列，v 为行）。
    u = u_local.astype(np.float32) + float(x_min)
    v = v_local.astype(np.float32) + float(y_min)
    # 针孔反投影：u/v 相对主点的偏移按焦距缩放后乘深度，得到相机坐标系 x/y（单位 m）。
    x = (u - float(intrinsics.cx)) * z / float(intrinsics.fx)
    y = (v - float(intrinsics.cy)) * z / float(intrinsics.fy)
    points = np.column_stack((x, y, z)).astype(np.float32)

    if color_bgr is None:
        colors = np.ones_like(points, dtype=np.float32) * 0.5
    else:
        color = np.asarray(color_bgr)
        if color.shape[:2] != depth.shape:
            # 颜色与深度分辨率不一致会导致像素索引错位，必须显式报错而不是静默取色。
            raise ValueError("color and depth image sizes must match")
        bgr = color[v.astype(np.int32), u.astype(np.int32), :3].astype(np.float32) / 255.0
        # 输入是 OpenCV 的 BGR，推理引擎要求 RGB，故翻转最后一维通道顺序。
        colors = bgr[:, ::-1].astype(np.float32)
    return PointCloudCrop(points=points, colors=colors)


def predictions_to_candidate_array(
    predictions: Iterable[GraspNetPrediction],
    *,
    frame_id: str,
    class_name: str,
    max_candidates: int,
    source: str = "graspnet_baseline",
) -> GraspCandidateArray:
    """把预测列表转成抓取候选数组消息。

    predictions：已按得分降序排列的预测（本函数不再排序，只按顺序截断）。
    frame_id：候选位姿所在坐标系，会同时写入数组头与每个候选头。
    class_name：目标类别名，用于按标签匹配；调用方可对单个候选覆盖。
    max_candidates：最多保留的候选数；非正数会被截断为 0，得到空数组。
    source：候选来源标识，写入 ``GraspCandidate.source`` 便于排障溯源。
    返回的数组 ``best_index`` 为 0（因为有候选时首个即最优），无候选时保持 -1。
    """
    array = GraspCandidateArray()
    array.header.frame_id = frame_id
    array.best_index = -1
    for prediction in list(predictions)[: max(0, int(max_candidates))]:
        candidate = GraspCandidate()
        candidate.header.frame_id = frame_id
        candidate.class_name = class_name
        candidate.confidence = float(prediction.score)
        candidate.pose.position.x = float(prediction.translation_xyz[0])
        candidate.pose.position.y = float(prediction.translation_xyz[1])
        candidate.pose.position.z = float(prediction.translation_xyz[2])
        # 消息姿态用四元数表示，故这里做一次旋转矩阵到四元数的转换并归一化。
        qx, qy, qz, qw = _rotation_matrix_to_quaternion(prediction.rotation_matrix)
        candidate.pose.orientation.x = qx
        candidate.pose.orientation.y = qy
        candidate.pose.orientation.z = qz
        candidate.pose.orientation.w = qw
        candidate.jaw_width = float(prediction.width_m)
        candidate.object_length = float(prediction.object_length_m)
        candidate.valid = True
        candidate.source = source
        array.candidates.append(candidate)
    if array.candidates:
        array.best_index = 0
    return array


def payload_to_candidate_array(
    payload: dict,
    *,
    fallback_frame_id: str,
    max_candidates: int,
) -> GraspCandidateArray:
    """把推理 payload 字典转成抓取候选数组消息（字段缺失时都有兜底）。

    payload 约定字段：``candidates``（候选列表，元素可为 dict 或对象）、``source``、
    ``class_name``、``frame_id``、``timestamp_ns``。frame_id 缺失或为空串时退回
    fallback_frame_id；class_name 逐候选保留（缺省用 payload 顶层值）。
    max_candidates：候选列表的截断上限，同样先于解析生效。
    时间戳（纳秒）大于 0 时才写入数组头与各候选头，避免把"无时间戳"写成 1970 年。
    """
    predictions: list[GraspNetPrediction] = []
    class_names: list[str] = []
    for item in list(payload.get("candidates", []))[: max(0, int(max_candidates))]:
        # 单条候选解析失败只跳过该条（容错），不影响其余候选，也不抛出到调用方。
        if not isinstance(item, dict):
            continue
        try:
            predictions.append(_prediction_from_raw(item))
            class_names.append(str(item.get("class_name", payload.get("class_name", ""))))
        except Exception:
            continue
    source = str(payload.get("source", "graspnet_baseline"))
    # 注意 payload 里的空 frame_id 也要退回 fallback，否则消息头坐标系为空会让下游 TF 查询失败。
    frame_id = str(payload.get("frame_id", fallback_frame_id) or fallback_frame_id)
    class_name = class_names[0] if class_names else str(payload.get("class_name", ""))
    candidates = predictions_to_candidate_array(
        predictions,
        frame_id=frame_id,
        class_name=class_name,
        max_candidates=max_candidates,
        source=source,
    )
    # zip 按位置配对：predictions 与 class_names 一一对应，这里逐条恢复各自的类别名。
    for candidate, name in zip(candidates.candidates, class_names):
        candidate.class_name = name
    timestamp_ns = int(payload.get("timestamp_ns", 0) or 0)
    if timestamp_ns > 0:
        # 纳秒拆成"秒 + 纳秒余数"两段，符合消息头 stamp 的字段定义。
        candidates.header.stamp.sec = timestamp_ns // 1_000_000_000
        candidates.header.stamp.nanosec = timestamp_ns % 1_000_000_000
        for candidate in candidates.candidates:
            candidate.header.stamp.sec = candidates.header.stamp.sec
            candidate.header.stamp.nanosec = candidates.header.stamp.nanosec
    return candidates


class InProcessGraspNetBackend:
    """在本 ROS 候选节点进程内直接加载完整的 RGB-D GraspNet 推理引擎。

    这是当前 Ubuntu 生产路线。它刻意保留本地推理引擎既有行为：全场景碰撞点云、目标掩膜/
    深度分离、确定性采样、投影过滤与夹爪开口过滤，同时去掉了 localhost JSON/HTTP 传输层。

    构造期不抛异常：任何配置或导入错误都被捕获并记录到 ``backend_error``，``available``
    随之变为 False，由调用方决定如何失败关闭并给出可读的排障信息。
    """

    def __init__(
        self,
        *,
        model_root: str,
        checkpoint_path: str,
        device: str = "cuda:0",
        module_name: str = "graspnet_baseline_inference",
        module_path: str = "",
        num_point: int = 20_000,
    ) -> None:
        self.model_root = str(model_root).strip()
        self.checkpoint_path = str(checkpoint_path).strip()
        self.device = str(device).strip()
        self.module_name = str(module_name).strip()
        self.module_path = str(module_path).strip()
        self.num_point = int(num_point)
        self._runner = None
        self.backend_error = "model_root_or_checkpoint_unset"
        # 模型目录或权重二选一缺失时直接返回，保持 available=False 并给出具体原因码。
        if not self.model_root or not self.checkpoint_path:
            return
        try:
            model_path = Path(self.model_root).expanduser()
            checkpoint = Path(self.checkpoint_path).expanduser()
            if not model_path.is_dir():
                raise FileNotFoundError(f"model root not found: {model_path}")
            if not checkpoint.is_file():
                raise FileNotFoundError(f"checkpoint not found: {checkpoint}")
            module = _load_inprocess_backend_module(
                self.module_name,
                module_path=self.module_path,
            )
            runner_cls = getattr(module, "GraspNetBaselineInference")
            self._runner = runner_cls(
                model_root=str(model_path),
                checkpoint_path=str(checkpoint),
                device=self.device,
                # num_point 是送进网络的采样点数：GPU 显存与推理时延的主要决定项。
                num_point=self.num_point,
            )
            self.backend_error = ""
        except Exception as exc:
            # 保留 "异常类型: 消息" 形式的诊断串，节点会原样打日志。
            self.backend_error = f"{type(exc).__name__}: {exc}"

    @property
    def available(self) -> bool:
        return self._runner is not None

    @property
    def last_stage_counts(self) -> dict[str, Any]:
        """推理引擎各过滤阶段的计数快照（如 scene_points/raw/published/empty_reason）。

        引擎未提供该属性或类型不符时返回空字典；每次都返回浅拷贝，避免调用方改动引擎内部状态。
        """
        value = getattr(self._runner, "last_stage_counts", None)
        return dict(value) if isinstance(value, dict) else {}

    def infer(
        self,
        *,
        timestamp_ns: int,
        frame_id: str,
        color_bgr: np.ndarray,
        depth_m: np.ndarray,
        camera_info: dict[str, float],
        detection: dict[str, Any],
        max_grasps: int,
        max_jaw_width_m: float | None,
        min_depth_m: float = 0.05,
        max_depth_m: float = 1.5,
    ) -> dict[str, Any]:
        """在进程内跑一次完整 RGB-D 推理，返回与 payload 字典同构的结果。

        入参均为米制/ROS 侧原始数据：color_bgr 为 HxWx3 的 uint8 BGR 图；depth_m 为 HxW
        的 float32 米制深度（必须有限且非负）；camera_info 至少含 fx/fy/cx/cy；detection
        为单个检测字典（含 class_name、像素框、可选掩膜等）；max_jaw_width_m 为夹爪开口
        上限，None 表示不过滤。timestamp_ns 必须为正、frame_id 必须非空，否则抛 ValueError
        ——上游时间戳策略已保证这两个前提，这里再兜一层，避免产出无法溯源或无法做 TF 的候选。
        返回 dict：source 固定为进程内路线标识，backend_configured=True、stale=False，
        并带上时间戳、坐标系、类别名与候选列表。
        """
        if self._runner is None:
            raise RuntimeError(
                f"GraspNet in-process backend is not configured: {self.backend_error}"
            )
        # 转成连续内存的 uint8/float32，避免非连续视图或 dtype 不符在推理引擎里出错。
        color = np.ascontiguousarray(color_bgr, dtype=np.uint8)
        depth = np.ascontiguousarray(depth_m, dtype=np.float32)
        if color.ndim != 3 or color.shape[2] != 3:
            raise ValueError("color_bgr must have shape HxWx3")
        if depth.ndim != 2 or color.shape[:2] != depth.shape:
            raise ValueError("depth_m must match color image dimensions")
        # 深度含 NaN/Inf 或负值会让反投影出无效点，属于调用方数据错误，直接拒绝。
        if not np.isfinite(depth).all() or np.any(depth < 0.0):
            raise ValueError("depth_m must contain finite non-negative values")
        if int(timestamp_ns) <= 0:
            raise ValueError("timestamp_ns must be positive")
        if not str(frame_id).strip():
            raise ValueError("frame_id must be non-empty")
        intrinsics = {
            key: float(camera_info[key]) for key in ("fx", "fy", "cx", "cy")
        }
        # 引擎对深度缩放的默认值是 0.001（适配整型毫米图）；这里传的是浮点米制数组，
        # 因此显式置 1.0，表示"数值已是米，无需再缩放"。
        intrinsics["depth_scale_m"] = 1.0
        intrinsics["workspace_min_depth_m"] = float(min_depth_m)
        intrinsics["workspace_max_depth_m"] = float(max_depth_m)
        candidates = self._runner.infer(
            color_bgr=color,
            # 本地推理引擎沿用历史参数名 depth_mm，但 depth_scale_m=1.0 使这个 float32 数组
            # 明确是米制数据；改名会破坏引擎接口，故保持原样。
            depth_mm=depth,
            # 引擎内部按置信度取最高的一条检测，这里把单条检测包成列表以复用其接口。
            detections=[dict(detection)],
            camera_info=intrinsics,
            max_grasps=int(max_grasps),
            max_jaw_width_m=max_jaw_width_m,
        )
        return {
            "source": "ubuntu_inprocess_graspnet",
            "backend_configured": True,
            "stale": False,
            "timestamp_ns": int(timestamp_ns),
            "frame_id": str(frame_id),
            "class_name": str(detection.get("class_name", "")),
            "candidates": list(candidates),
        }


def _load_inprocess_backend_module(module_name: str, *, module_path: str = ""):
    """定位并加载进程内推理引擎模块，按三条优先级依次尝试。

    1. 显式配置的 module_path：直接按文件路径加载；
    2. 常规 import：模块已装进当前 Python 环境时最为直接；
    3. 兜底在共享资源目录的 graspnet_backend/ 与源码树各级 tools/ 下按文件名查找，
       以兼容未安装、直接从源码树运行的场景。
    只有确认是"该模块本身找不到"（ModuleNotFoundError.name 相同）才继续兜底；若只是该模块
    内部缺依赖，则原样抛出，避免把真实环境问题掩盖成路径问题。
    """
    configured_path = Path(module_path).expanduser() if module_path else None
    if configured_path is not None:
        return _load_python_module_from_path(module_name, configured_path)
    try:
        return importlib.import_module(module_name)
    except ModuleNotFoundError as exc:
        # exc.name 与请求的模块名不同 => 缺的是引擎内部依赖，交由调用方报错。
        if exc.name != module_name:
            raise
        candidates: list[Path] = []
        try:
            from ament_index_python.packages import get_package_share_directory

            candidates.append(
                Path(get_package_share_directory("rebotarm_vision"))
                / "graspnet_backend"
                / f"{module_name}.py"
            )
        except Exception:
            # 资源索引不可用（例如未 source 环境）时跳过该候选，继续走源码树兜底。
            pass
        source = Path(__file__).resolve()
        for parent in source.parents:
            candidates.append(parent / "tools" / f"{module_name}.py")
        for candidate in candidates:
            if candidate.is_file():
                return _load_python_module_from_path(module_name, candidate)
        raise


def _load_python_module_from_path(module_name: str, path: Path):
    """按文件路径把模块加载进 sys.modules 并执行，返回模块对象。

    先注册再 exec_module，保证模块内部的自引用/循环导入能看到自己。path 不存在时抛
    FileNotFoundError，spec 构造失败时抛 ImportError。
    """
    if not path.is_file():
        raise FileNotFoundError(f"backend module not found: {path}")
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load backend module from {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


def _prediction_from_raw(item) -> GraspNetPrediction:
    """把推理引擎的原始输出统一成 :class:`GraspNetPrediction`。

    兼容三种形态：已是预测对象则原样返回；dict 用 ``rotation_matrix``/``rotation``、
    ``translation_xyz``/``translation``、``score``/``confidence``、``width_m``/``width``、
    ``object_length_m``/``object_length`` 两套键名取值；其余对象按其属性取同名值。
    缺字段一律退化为 0.0，缺平移/旋转则会在 ``_tuple3``/``_matrix3`` 中抛 ValueError。
    """
    if isinstance(item, GraspNetPrediction):
        return item
    if isinstance(item, dict):
        rotation = item.get("rotation_matrix", item.get("rotation"))
        translation = item.get("translation_xyz", item.get("translation"))
        return GraspNetPrediction(
            score=float(item.get("score", item.get("confidence", 0.0))),
            translation_xyz=_tuple3(translation),
            rotation_matrix=_matrix3(rotation),
            width_m=float(item.get("width_m", item.get("width", 0.0))),
            object_length_m=float(item.get("object_length_m", item.get("object_length", 0.0))),
        )
    return GraspNetPrediction(
        score=float(getattr(item, "score", getattr(item, "confidence", 0.0))),
        translation_xyz=_tuple3(getattr(item, "translation_xyz", getattr(item, "translation", None))),
        rotation_matrix=_matrix3(getattr(item, "rotation_matrix", getattr(item, "rotation", None))),
        width_m=float(getattr(item, "width_m", getattr(item, "width", 0.0))),
        object_length_m=float(getattr(item, "object_length_m", getattr(item, "object_length", 0.0))),
    )


def _tuple3(values) -> tuple[float, float, float]:
    """展平后取前三个元素为浮点三元组（平移向量，单位 m）；不足 3 个则抛 ValueError。"""
    arr = np.asarray(values, dtype=np.float64).reshape(-1)
    if arr.size < 3:
        raise ValueError("expected at least 3 values")
    return (float(arr[0]), float(arr[1]), float(arr[2]))


def _matrix3(values) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    """按行优先把输入整形成 3x3 嵌套元组；元素个数不是 9 的倍数时 reshape 会抛 ValueError。"""
    arr = np.asarray(values, dtype=np.float64).reshape(3, 3)
    return tuple(tuple(float(v) for v in row) for row in arr)  # type: ignore[return-value]


def _rotation_matrix_to_quaternion(matrix) -> tuple[float, float, float, float]:
    """把 3x3 旋转矩阵转成 (x, y, z, w) 顺序的单位四元数。

    采用按对角线元素大小分支的标准算法（Shepperd/Bar-Itzhack 思路）：先看迹 trace；trace 过小
    甚至为负时，选绝对值最大的对角元所在分支求解。这样可避免开方内出现接近 0 或负数的量，
    保证数值稳定。最后统一归一化以满足消息里"单位四元数"的约定；退化到无法归一化
    （模长 <= 1e-12，例如零矩阵）时返回单位四元数 (0, 0, 0, 1)，即"不旋转"。
    姿态与位置同处相机坐标系，二者必须配套解读。
    """
    m = np.asarray(matrix, dtype=np.float64).reshape(3, 3)
    trace = float(np.trace(m))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m[2, 1] - m[1, 2]) / s
        qy = (m[0, 2] - m[2, 0]) / s
        qz = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        qw = (m[2, 1] - m[1, 2]) / s
        qx = 0.25 * s
        qy = (m[0, 1] + m[1, 0]) / s
        qz = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        qw = (m[0, 2] - m[2, 0]) / s
        qx = (m[0, 1] + m[1, 0]) / s
        qy = 0.25 * s
        qz = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        qw = (m[1, 0] - m[0, 1]) / s
        qx = (m[0, 2] + m[2, 0]) / s
        qy = (m[1, 2] + m[2, 1]) / s
        qz = 0.25 * s
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (float(qx / norm), float(qy / norm), float(qz / norm), float(qw / norm))

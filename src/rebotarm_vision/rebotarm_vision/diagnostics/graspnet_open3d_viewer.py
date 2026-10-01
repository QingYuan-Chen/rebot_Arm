"""GraspNet 原始候选的 Open3D 可视化查看器（仅调试用）。

职责与位置
----------
本节点把"彩色图 + 深度图 + 相机内参 + 检测框/掩膜 + GraspNet 候选"在同一时刻拼成一幅
三维场景：RGB-D 反投影得到的彩色点云，加上前若干名候选的简化夹爪位姿，显示在独立的
Open3D 窗口中。它是**纯只读调试工具**：只订阅话题，不发布任何消息，不提供 RViz、运动规划
或硬件接口，因此可以随时打开/关闭而不影响抓取执行链路。

订阅话题（全部可通过参数修改）
------------------------------
- `input_color_topic`（默认 `/camera/color/image_raw`）
- `input_depth_topic`（默认 `/camera/depth/image_raw`）
- `input_camera_info_topic`（默认 `/camera/depth/camera_info`）
- `input_candidates_topic`（默认 `/grasp/graspnet_candidates`）
- 检测话题固定为 `/grasp/detections`，用于按掩膜/检测框裁剪出目标点云；未收到检测时显示整幅场景。

线程模型
--------
`main` 用**后台线程**跑单线程执行器接收并缓存消息，主线程只做"取出待渲染候选 → 更新窗口 →
轮询窗口事件"；共享状态 `pending` 用 `self.lock` 保护，主线程渲染时先取走并清空，避免重复渲染。
窗口关闭（`poll` 返回 False）即退出主循环。

硬编码的显示常量
----------------
- 深度有效范围 0.15 ~ 1.5 m（见 `_scene_cloud`），刻意比候选生成节点的 1.2 m 更宽，便于看到背景；
- 只绘制前 5 名候选（见 `Open3DGraspWindow.update`），最优候选用绿色并加绘坐标轴；
- 夹爪用三块薄板近似（见 `_gripper_geometries`），只表达开口方向与位置，不代表真实碰撞体。
"""

from __future__ import annotations

from collections import deque
import threading

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from rebotarm_msgs.msg import Detection2DArray, GraspCandidateArray


def _stamp_ns(stamp) -> int:
    # 统一换算成整数纳秒，便于与缓存帧做最近邻比较。
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _color_array(msg: Image) -> np.ndarray:
    """把彩色图像消息转成 RGB 排列的 uint8 数组（Open3D 按 RGB 解读颜色）。

    仅接受 `bgr8` 与 `rgb8`；因此这里的通道翻转方向与推理节点相反（那边统一成 BGR）。
    其他编码抛 `ValueError`，由回调记警告并丢弃该帧。
    """
    if msg.encoding not in ("bgr8", "rgb8"):
        raise ValueError(f"unsupported color encoding: {msg.encoding}")
    image = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
    if msg.encoding == "bgr8":
        # ::-1 反转为 RGB，供 Open3D 点云着色。
        image = image[:, :, ::-1]
    return np.ascontiguousarray(image)


def _depth_array(msg: Image) -> np.ndarray:
    """把深度图像消息转成 uint16 的"毫米"数组（形状 HxW），不做任何缩放。

    只接受 16 位单通道深度（`16UC1` / `mono16`）。米制换算留给 `_scene_cloud`，
    以便与推理侧的 depth_scale 参数保持一致的处理顺序。
    """
    if msg.encoding not in ("16UC1", "mono16"):
        raise ValueError(f"unsupported depth encoding: {msg.encoding}")
    return np.frombuffer(msg.data, dtype=np.uint16).reshape((msg.height, msg.width))


def _pose_matrix(candidate) -> np.ndarray:
    """把候选的四元数位姿转成 4x4 齐次变换矩阵（前三维旋转 + 平移）。

    四元数先按模长归一化（ROS 的四元数约定为单位四元数，但消息里可能带数值误差）；
    模长接近 0 的退化输入、或矩阵中出现 NaN/Inf 时抛 `ValueError`，由调用方跳过该候选。
    """
    pose = candidate.pose
    qx, qy, qz, qw = (
        float(pose.orientation.x),
        float(pose.orientation.y),
        float(pose.orientation.z),
        float(pose.orientation.w),
    )
    norm = float(np.linalg.norm([qx, qy, qz, qw]))
    if norm < 1e-9:
        # 阈值 1e-9：小于它就视为零四元数，无法定义姿态。
        raise ValueError("candidate quaternion is zero")
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    matrix = np.eye(4, dtype=np.float64)
    # 标准四元数 → 旋转矩阵展开式（已代入单位四元数下的化简结果）。
    matrix[:3, :3] = [
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ]
    matrix[:3, 3] = [float(pose.position.x), float(pose.position.y), float(pose.position.z)]
    if not np.isfinite(matrix).all():
        raise ValueError("candidate pose is not finite")
    return matrix


def _scene_cloud(color, depth, info, max_points: int, detection=None):
    """由彩色图、深度图与内参反投影出彩色的目标点云。

    参数与语义：
    - `depth`：uint16 毫米深度图，这里乘以 0.001 转成米；
    - 有效深度区间固定为 [0.15, 1.5] m，比抓取侧的 1.2 m 更宽，便于观察背景；
    - `detection` 不为空时只保留目标区域：优先用掩膜多边形（`has_mask` 且至少 6 个数），
      否则退化为轴对齐矩形框，并把框裁剪到图像范围内（保证 x_max > x_min、y_max > y_min，
      避免出现空切片）；
    - 有效像素超过 `max_points` 时按等间隔（`linspace`）抽样，保持空间分布均匀而不改变顺序；
    - 反投影用针孔模型：x = (u - cx)·z/fx，y = (v - cy)·z/fy，坐标系与深度光学系一致；
    - 颜色归一化到 [0, 1] 供 Open3D 使用。

    返回 `(points, colors)`；无有效像素时返回两个空数组，调用方可直接清空窗口几何体。
    """
    depth_m = depth.astype(np.float32) * 0.001
    valid = np.isfinite(depth_m) & (depth_m >= 0.15) & (depth_m <= 1.5)
    if detection is not None:
        target_mask = np.zeros(depth.shape, dtype=np.uint8)
        polygon = list(getattr(detection, "mask_polygon_xy", []))
        if bool(getattr(detection, "has_mask", False)) and len(polygon) >= 6:
            # 至少 3 个顶点才构成多边形；fillPoly 内部按整数像素填充。
            vertices = np.asarray(polygon, dtype=np.float32).reshape(-1, 2)
            cv2.fillPoly(target_mask, [np.round(vertices).astype(np.int32)], 1)
        else:
            # 矩形框裁剪：四个边界都夹到图像范围内，并保证至少 1 像素宽/高。
            x_min = max(0, min(depth.shape[1] - 1, int(detection.x_min)))
            y_min = max(0, min(depth.shape[0] - 1, int(detection.y_min)))
            x_max = max(x_min + 1, min(depth.shape[1], int(detection.x_max)))
            y_max = max(y_min + 1, min(depth.shape[0], int(detection.y_max)))
            target_mask[y_min:y_max, x_min:x_max] = 1
        valid &= target_mask.astype(bool)
    # v 为行号（对应 y），u 为列号（对应 x）。
    v, u = np.nonzero(valid)
    if len(u) == 0:
        return np.empty((0, 3)), np.empty((0, 3))
    if len(u) > max_points:
        indices = np.linspace(0, len(u) - 1, max_points, dtype=np.int64)
        u, v = u[indices], v[indices]
    z = depth_m[v, u]
    # 针孔反投影：k[0]=fx、k[4]=fy、k[2]=cx、k[5]=cy。
    x = (u.astype(np.float32) - float(info.k[2])) * z / float(info.k[0])
    y = (v.astype(np.float32) - float(info.k[5])) * z / float(info.k[4])
    points = np.column_stack((x, y, z)).astype(np.float64)
    colors = color[v, u].astype(np.float32) / 255.0
    return points, colors.astype(np.float64)


def _gripper_geometries(o3d, matrix, jaw_width: float, color):
    """用三块薄板拼出夹爪的示意几何体，并按候选位姿摆到场景里。

    布局在候选自身坐标系下定义：x 负方向为夹爪指向（指尖朝向），开口沿 y 轴对称，
    两块手指板加一块根部连接板。尺寸单位为米，`jaw_width` 被夹到 [0.006, 0.2] m，
    使极端候选不会画出过大/退化的图形；这里只是可视化近似，不代表真实碰撞体。
    """
    width = float(np.clip(jaw_width, 0.006, 0.2))
    # 手指长度 0.06 m、板厚 0.004 m，为固定的示意尺寸。
    finger_length = 0.06
    thickness = 0.004
    geometries = []
    # 每条为 (长, 宽, 厚, 平移偏移)：两条手指 + 一条根部横梁。
    bars = [
        (finger_length, thickness, thickness, (-finger_length, -width / 2 - thickness, -thickness / 2)),
        (finger_length, thickness, thickness, (-finger_length, width / 2, -thickness / 2)),
        (thickness, width + 2 * thickness, thickness, (-finger_length - thickness, -width / 2 - thickness, -thickness / 2)),
    ]
    for sx, sy, sz, offset in bars:
        mesh = o3d.geometry.TriangleMesh.create_box(width=sx, height=sy, depth=sz)
        mesh.translate(offset)
        # 先在本体坐标系平移定位，再整体变换到候选位姿。
        mesh.transform(matrix)
        mesh.paint_uniform_color(color)
        mesh.compute_vertex_normals()
        geometries.append(mesh)
    return geometries


class Open3DGraspWindow:
    """Open3D 窗口封装：负责清屏、重建几何体与视角、事件轮询。

    仅应在本进程持有；`update` 每帧调用一次，`poll` 返回窗口是否仍然存活。
    `fitted` 保证视角只在首次渲染时自动调整，之后由用户鼠标操作接管，不被每帧重置。
    """

    def __init__(self, *, point_size: float = 2.0):
        # 延迟导入：没有图形/Open3D 环境的机器上，只要不构造本类就不会 import 失败。
        import open3d as o3d

        self.o3d = o3d
        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        if not self.vis.create_window("GraspNet | RGB-D cloud + raw grasp candidates", 1280, 800):
            # 无 DISPLAY / 窗口系统不可用时明确报错，提示需要在桌面会话中运行。
            raise RuntimeError("Open3D window creation failed; run from a desktop session")
        options = self.vis.get_render_option()
        options.point_size = float(point_size)
        # 深色背景，便于观察点云颜色。
        options.background_color = np.array([0.035, 0.045, 0.055])
        self.fitted = False

    def update(self, points, colors, candidates):
        """用最新一帧点云与候选重建场景。

        只绘制前 5 名候选：第 0 名（最优）用绿色并额外绘制 0.035 m 的坐标轴，
        其余用橙红色；位姿非法（`_pose_matrix` 抛错）的候选直接跳过，不影响其它候选显示。
        """
        self.vis.clear_geometries()
        cloud = self.o3d.geometry.PointCloud()
        cloud.points = self.o3d.utility.Vector3dVector(points)
        cloud.colors = self.o3d.utility.Vector3dVector(colors)
        # reset_bounding_box=False：保持用户当前视角，不因新增几何体而重新自动取景。
        self.vis.add_geometry(cloud, reset_bounding_box=False)
        for index, candidate in enumerate(candidates[:5]):
            try:
                matrix = _pose_matrix(candidate)
            except ValueError:
                continue
            color = [0.15, 1.0, 0.3] if index == 0 else [1.0, 0.3, 0.12]
            for geometry in _gripper_geometries(
                self.o3d, matrix, float(candidate.jaw_width), color
            ):
                self.vis.add_geometry(geometry, reset_bounding_box=False)
            if index == 0:
                axes = self.o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.035)
                axes.transform(matrix)
                self.vis.add_geometry(axes, reset_bounding_box=False)
        if not self.fitted:
            # 首帧自动定位：看向点云中位数（近似目标中心），相机沿 -z 看向场景，屏幕上方向为 -y，
            # 取景系数 0.7 略微拉近；之后的帧不再重置，交给用户交互。
            self.vis.reset_view_point(True)
            view = self.vis.get_view_control()
            focus = np.median(points, axis=0) if len(points) else [0.0, 0.0, 0.5]
            view.set_lookat(focus)
            view.set_front([0.0, 0.0, -1.0])
            view.set_up([0.0, -1.0, 0.0])
            view.set_zoom(0.7)
            self.fitted = True
        self.vis.update_renderer()

    def poll(self):
        """处理窗口事件并重绘；返回 False 表示用户关闭了窗口。"""
        alive = self.vis.poll_events()
        self.vis.update_renderer()
        return alive

    def close(self):
        self.vis.destroy_window()


class GraspNetOpen3DViewer(Node):
    """订阅视觉话题并在 Open3D 窗口中显示原始 GraspNet 候选的调试节点。

    构造时可注入自定义窗口对象（`window`），便于无图形环境下用假窗口做测试。
    节点本身不发布任何消息；渲染由 `render_pending` 在主线程驱动，ROS 回调在另一线程执行。
    """

    def __init__(self, window=None):
        super().__init__("rebotarm_graspnet_open3d_viewer")
        self.declare_parameter("input_color_topic", "/camera/color/image_raw")
        self.declare_parameter("input_depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("input_camera_info_topic", "/camera/depth/camera_info")
        self.declare_parameter("input_candidates_topic", "/grasp/graspnet_candidates")
        # 点云抽样上限：越大越细腻，但每帧重建耗时与显存占用上升。
        self.declare_parameter("max_points", 30000)
        # 图像帧与候选之间允许的最大时间差，单位秒；超出则本帧不渲染，避免位姿与点云错位。
        self.declare_parameter("max_frame_age_sec", 1.5)
        # 点云显示点大小（像素），仅影响观感。
        self.declare_parameter("point_size", 2.0)
        # 参数读取的简写：value("name") 等价于 self.get_parameter("name").value。
        value = lambda name: self.get_parameter(name).value
        self.window = window or Open3DGraspWindow(point_size=float(value("point_size")))
        # 彩色/深度各缓存最近 8 帧，用于与候选时间戳做最近邻匹配。
        self.colors = deque(maxlen=8)
        self.depths = deque(maxlen=8)
        self.info = None
        self.detection = None
        self.pending = None
        # 保护 pending 的锁：ROS 回调线程写入，主线程渲染时取走。
        self.lock = threading.Lock()
        self.create_subscription(Image, str(value("input_color_topic")), self._on_color, qos_profile_sensor_data)
        self.create_subscription(Image, str(value("input_depth_topic")), self._on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, str(value("input_camera_info_topic")), self._on_info, qos_profile_sensor_data)
        # 检测话题为固定值：用于按目标区域裁剪点云。
        self.create_subscription(Detection2DArray, "/grasp/detections", self._on_detections, 10)
        self.create_subscription(GraspCandidateArray, str(value("input_candidates_topic")), self._on_candidates, 10)
        self.get_logger().info("Open3D raw GraspNet viewer ready; no RViz, motion, or hardware interfaces")

    def _on_color(self, message):
        try:
            self.colors.append((_stamp_ns(message.header.stamp), _color_array(message)))
        except ValueError as exc:
            self.get_logger().warn(str(exc))

    def _on_depth(self, message):
        try:
            self.depths.append((_stamp_ns(message.header.stamp), _depth_array(message)))
        except ValueError as exc:
            self.get_logger().warn(str(exc))

    def _on_info(self, message):
        # 内参不做缓存管理：最近一条持续有效，点云反投影始终使用它。
        self.info = message

    def _on_detections(self, message):
        # 只保留置信度最高的检测，用于裁剪目标点云（与候选生成节点取同一目标的策略一致）。
        detections = list(message.detections)
        self.detection = max(detections, key=lambda item: float(item.confidence)) if detections else None

    def _on_candidates(self, message):
        # 空候选视为"清屏"信号：把 pending 置空，主线程不会再去渲染上一帧的夹爪。
        if not message.candidates:
            with self.lock:
                self.pending = None
            return
        stamp = _stamp_ns(message.header.stamp)
        with self.lock:
            self.pending = (stamp, message)

    def render_pending(self):
        """渲染一次待处理候选；返回窗口是否仍然存活（窗口关闭即返回 False）。

        每一帧做三件事：取出并清空 pending；按时间戳就近找彩色/深度帧，并检查与候选的时间差
        不超过 `max_frame_age_sec`；重建点云与夹爪几何体。数据不齐或过期时只刷新窗口事件，
        不更新场景。点云构建失败（`ValueError`）或候选列表为空（`IndexError`）只记警告。
        """
        with self.lock:
            pending, self.pending = self.pending, None
        if pending is not None and self.info is not None:
            stamp, candidates = pending
            color = min(self.colors, key=lambda item: abs(item[0] - stamp), default=None)
            depth = min(self.depths, key=lambda item: abs(item[0] - stamp), default=None)
            max_age_ns = int(float(self.get_parameter("max_frame_age_sec").value) * 1e9)
            if color and depth and abs(color[0] - stamp) <= max_age_ns and abs(depth[0] - stamp) <= max_age_ns:
                try:
                    points, colors = _scene_cloud(
                        color[1],
                        depth[1],
                        self.info,
                        int(self.get_parameter("max_points").value),
                        self.detection,
                    )
                    self.window.update(points, colors, list(candidates.candidates))
                    # 候选已按得分降序排列，第 0 个即最优；日志同步打印其位姿便于比对与记录。
                    best = candidates.candidates[0]
                    self.get_logger().info(
                        "best grasp: "
                        f"frame={candidates.header.frame_id} "
                        f"score={float(best.confidence):.4f} "
                        f"jaw={float(best.jaw_width):.4f}m "
                        f"position=({float(best.pose.position.x):.4f}, "
                        f"{float(best.pose.position.y):.4f}, "
                        f"{float(best.pose.position.z):.4f}) "
                        f"quaternion=({float(best.pose.orientation.x):.4f}, "
                        f"{float(best.pose.orientation.y):.4f}, "
                        f"{float(best.pose.orientation.z):.4f}, "
                        f"{float(best.pose.orientation.w):.4f})"
                    )
                except (ValueError, IndexError) as exc:
                    self.get_logger().warn(f"Open3D grasp frame rejected: {exc}")
        return self.window.poll()

    def destroy_node(self):
        # 先关掉图形窗口再销毁节点，避免退出时残留窗口进程。
        self.window.close()
        return super().destroy_node()


def main(args=None):
    """入口：无 DISPLAY 时直接失败；否则后台线程收消息、主线程跑窗口事件循环。"""
    if not __import__("os").environ.get("DISPLAY"):
        raise RuntimeError("Open3D viewer requires a desktop DISPLAY")
    rclpy.init(args=args)
    node = GraspNetOpen3DViewer()
    # 单线程执行器放到后台线程：ROS 回调与窗口渲染互不阻塞（共享状态由 node.lock 保护）。
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        while rclpy.ok() and node.render_pending():
            pass
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        # 限时 join，避免执行器线程卡住导致进程无法退出。
        thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

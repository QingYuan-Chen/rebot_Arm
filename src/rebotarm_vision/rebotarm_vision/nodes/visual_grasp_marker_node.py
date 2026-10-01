"""把视觉抓取计划渲染成 RViz 标记的显示模块。

职责与位置：本模块位于视觉链路的显示末端，只负责把上层选出的抓取计划（默认来自
``/grasp/plan`` 一类话题，实际由启动组合指向过滤后的计划话题）画成标记，供操作者确认
"末端去哪里抓、以什么姿态抓、夹爪张开多宽"。它是纯显示通道：不发布任何运动或夹爪
指令，也不做限位、碰撞与可达性判断，因此关闭标记不会影响抓取流程本身。

对外接口：
- 订阅 ``input_topic``（默认 ``/grasp/plan``）：抓取计划消息；
- 发布 ``output_topic``（默认 ``/grasp/visual_markers``）：标记数组，供 RViz 以
  ``MarkerArray`` 显示。

主要流程：
1. ``_convert_plan`` 先做坐标系对齐：计划的 ``header.frame_id`` 与 ``target_frame``
   （默认 ``base_link``）不一致时，用 TF 把候选位姿、接近点位姿与抓取点位姿一起变换到
   目标坐标系；TF 暂时查询失败时保留原坐标系继续发布，只在日志中告警——可视化不应
   因为缺一帧变换而拖住或中断抓取流程；
2. ``VisualGraspMarkerBuilder`` 生成标记：可选示意圆柱、候选位置点、接近点、抓取点、叠加
   TCP 偏移后的实际夹持点、接近方向箭头、夹爪张开方向线段，以及类别与置信度文字；
3. 计划 ``valid`` 为假时默认只发一个删除标记，清掉画面上的旧计划，避免把上一次的位姿
   误读成当前计划。

注意：标记反映的是计划值与几何估计（圆柱尺寸由夹爪开口与目标长度钳制得到），不是实测
反馈，不能用它判断机械臂是否真的到位。
"""

from __future__ import annotations

from copy import deepcopy
import math

import rclpy
from geometry_msgs.msg import Pose
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from rebotarm_msgs.msg import GraspPlan

from ..utils.tf_message_adapter import transform_from_msg, transform_pose_message


def copy_pose(pose: Pose) -> Pose:
    """逐字段复制位姿消息，返回互不共享存储的新对象。

    显式读取每个分量并转成 ``float``：一是避免后续对副本的改写（例如重置姿态）影响调用
    方持有的原消息，二是能兼容测试里的轻量替身对象。位置单位 m，姿态为四元数（无单位）。
    """
    copied = Pose()
    copied.position.x = float(pose.position.x)
    copied.position.y = float(pose.position.y)
    copied.position.z = float(pose.position.z)
    copied.orientation.x = float(pose.orientation.x)
    copied.orientation.y = float(pose.orientation.y)
    copied.orientation.z = float(pose.orientation.z)
    copied.orientation.w = float(pose.orientation.w)
    return copied


def copy_point(point):
    """按原对象类型复制一个三维点（通常取位姿消息里的 position）。

    用 ``point.__class__()`` 构造副本，因此对真实消息类型与测试替身同样适用；不做类型
    检查，单位与输入一致（m）。
    """
    copied = point.__class__()
    copied.x = float(point.x)
    copied.y = float(point.y)
    copied.z = float(point.z)
    return copied


def point_from_xyz(reference_pose: Pose, xyz: tuple[float, float, float]):
    """以 ``reference_pose`` 中位置字段的同类对象，构造坐标为 ``xyz`` 的点。

    只借用类型、不读取 ``reference_pose`` 的数值，目的是让新建的点能直接放进同一条消息
    （例如标记的 ``points`` 列表）而不引起类型不匹配。单位 m。
    """
    point = reference_pose.position.__class__()
    point.x = float(xyz[0])
    point.y = float(xyz[1])
    point.z = float(xyz[2])
    return point


def identity_orientation(pose: Pose) -> Pose:
    """返回位置不变、姿态被重置为单位四元数的位姿副本。

    单位四元数 (x, y, z, w) = (0, 0, 0, 1) 表示姿态与参考坐标系的轴对齐。目标圆柱与文字
    标签只关心位置和尺寸，用单位姿态可避免被倾斜的抓取姿态带偏，从而始终竖直显示。
    """
    copied = copy_pose(pose)
    copied.orientation.x = 0.0
    copied.orientation.y = 0.0
    copied.orientation.z = 0.0
    copied.orientation.w = 1.0
    return copied


def object_diameter(jaw_width: float, minimum: float) -> float:
    """由夹爪开口宽度估算目标圆柱直径并做钳制，单位 m。

    被夹持目标的直径通常与夹爪开口同量级，这里按 1.8 倍开口作为"能把目标包住"的视觉
    估计；上限 0.16 m，防止异常大的开口把标记画得过分夸张；下限取 ``minimum``（默认
    0.06 m），保证很小的目标仍看得见；负的开口按 0 处理。
    """
    return max(float(minimum), min(max(float(jaw_width), 0.0) * 1.8, 0.16))


def object_height(object_length: float, minimum: float) -> float:
    """把候选估计的目标长度钳制成圆柱高度，单位 m。

    上限 0.35 m，避免过长的估计把圆柱拉高到遮挡视野；负值按 0 处理；最后与 ``minimum``
    （默认 0.12 m）取大，因此当 ``minimum`` 大于 0.35 时以 ``minimum`` 为准。
    """
    return max(float(minimum), min(max(float(object_length), 0.0), 0.35))


def _rotate_vector_by_quaternion(
    vector: tuple[float, float, float],
    quaternion: tuple[float, float, float, float],
) -> tuple[float, float, float]:
    """把三维向量按四元数旋转，等价于 ``v' = q ⊗ v ⊗ q*``。

    实现用中间量 ``t = 2 * (q_vec × v)``，再取 ``v' = v + w * t + q_vec × t``，比构造
    旋转矩阵更省算力。四元数按 (x, y, z, w) 传入并先归一化；模长 ≤ 1e-9（姿态未初始化
    或非法）时直接返回原向量，避免除零。参数与返回值同处一个坐标系，单位不变。
    """
    x, y, z = (float(vector[0]), float(vector[1]), float(vector[2]))
    qx, qy, qz, qw = (float(quaternion[0]), float(quaternion[1]), float(quaternion[2]), float(quaternion[3]))
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-9:
        return x, y, z
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

    # 四元数向量旋转的展开式：v' = v + 2*w*(q_vec × v) + 2*(q_vec × (q_vec × v))，
    # 下面的 t 即 2*(q_vec × v)，最后三行按分量组合出结果。
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return (
        x + qw * tx + (qy * tz - qz * ty),
        y + qw * ty + (qz * tx - qx * tz),
        z + qw * tz + (qx * ty - qy * tx),
    )


def _pose_quaternion(pose: Pose) -> tuple[float, float, float, float]:
    """取出位姿的姿态分量，返回 (x, y, z, w) 顺序的四元数元组（无单位）。"""
    return (
        float(pose.orientation.x),
        float(pose.orientation.y),
        float(pose.orientation.z),
        float(pose.orientation.w),
    )


def _offset_pose_position(pose: Pose, offset_xyz: tuple[float, float, float]) -> Pose:
    """在 ``pose`` 的自身坐标系内平移，返回姿态不变的新位姿。

    ``offset_xyz`` 是局部坐标偏移（单位 m），先按 ``pose`` 的姿态旋转到父坐标系，再叠加到
    位置上，因此偏移方向会随末端姿态一起转动——这正是表达"TCP 相对末端坐标系的固定安装
    偏移"所需的语义。
    """
    copied = copy_pose(pose)
    dx, dy, dz = _rotate_vector_by_quaternion(offset_xyz, _pose_quaternion(pose))
    copied.position.x = float(pose.position.x) + dx
    copied.position.y = float(pose.position.y) + dy
    copied.position.z = float(pose.position.z) + dz
    return copied


class VisualGraspMarkerBuilder:
    """把一份抓取计划渲染成标记集合的纯构造函数。

    本类不持有订阅/发布对象，也不查询 TF：坐标系换算由调用方完成后把 ``frame_id`` 传
    进来，因此可以在测试中直接实例化并断言标记内容。标记的名称空间(ns)与编号(id)是
    RViz 端的稳定标识，二者成对决定"新标记覆盖哪一个旧标记"，不要随意改动。
    """

    def __init__(
        self,
        *,
        object_min_diameter_m: float = 0.06,
        object_min_height_m: float = 0.12,
        upright_object_marker: bool = True,
        show_object_marker: bool = True,
        show_object_center_marker: bool = True,
        show_object_label: bool = True,
        label_height_m: float = 0.055,
        tcp_offset_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0),
        gripper_open_axis_local_xyz: tuple[float, float, float] = (0.0, 1.0, 0.0),
        show_tcp_markers: bool = True,
        show_approach_arrow: bool = True,
        show_gripper_open_axis: bool = True,
    ) -> None:
        """记录渲染选项；这些参数只影响显示效果，不改变抓取语义。

        - ``object_min_diameter_m``：目标圆柱的最小直径，单位 m（默认 0.06）；
        - ``object_min_height_m``：目标圆柱的最小高度，单位 m（默认 0.12）；
        - ``upright_object_marker``：为真时目标圆柱与文字标签强制竖直，忽略抓取姿态；
        - ``show_object_marker``：控制示意圆柱显示；不改变候选或计划内容；
        - ``show_object_center_marker``：控制候选位置绿点显示；
        - ``show_object_label``：控制类别与置信度文字显示；
        - ``label_height_m``：类别文字的显示高度，单位 m；
        - ``tcp_offset_xyz``：TCP 相对计划位姿所描述的末端坐标系的局部安装偏移，单位 m，
          默认全零（即计划位姿本身就是夹持点）；用于把计划位姿换算成实际夹持点位置；
        - ``gripper_open_axis_local_xyz``：夹爪张开方向（两指连线方向）在 TCP 局部坐标系
          下的向量（默认 (0, 1, 0) 即局部 y 轴），只取方向、内部会归一化；
        - ``show_tcp_markers`` / ``show_approach_arrow`` / ``show_gripper_open_axis``：
          三组附加标记的开关，关闭可减少画面元素。
        """
        self._object_min_diameter_m = float(object_min_diameter_m)
        self._object_min_height_m = float(object_min_height_m)
        self._upright_object_marker = bool(upright_object_marker)
        self._show_object_marker = bool(show_object_marker)
        self._show_object_center_marker = bool(show_object_center_marker)
        self._show_object_label = bool(show_object_label)
        self._label_height_m = float(label_height_m)
        if self._label_height_m <= 0:
            raise ValueError("label_height_m must be positive")
        self._tcp_offset_xyz = tuple(float(v) for v in tcp_offset_xyz)
        self._gripper_open_axis_local_xyz = tuple(float(v) for v in gripper_open_axis_local_xyz)
        self._show_tcp_markers = bool(show_tcp_markers)
        self._show_approach_arrow = bool(show_approach_arrow)
        self._show_gripper_open_axis = bool(show_gripper_open_axis)

    def build(self, plan: GraspPlan, *, frame_id: str, stamp) -> MarkerArray:
        """按计划生成整套标记，返回可直接发布的标记数组。

        - ``plan``：抓取计划消息；``valid`` 为假时只返回一个删除标记（见 ``_delete_all``），
          让 RViz 清空旧画面；
        - ``frame_id``：所有标记的参考坐标系，应为已与计划对齐的目标坐标系；
        - ``stamp``：写入标记头的时间戳；节点侧传零时刻，表示"始终按最新可用变换显示"。

        标记清单：可选 ``visual_object`` 示意圆柱、``visual_object_center`` 候选位置、
        ``visual_pregrasp``/``visual_grasp`` 计划给出的接近点与抓取点、
        ``visual_pregrasp_tcp``/``visual_grasp_tcp`` 叠加 TCP 偏移后的实际夹持点、
        ``visual_approach_arrow`` 接近方向箭头、``visual_gripper_open_axis`` 夹爪张开方向
        线段、``visual_object_label`` 类别与置信度文字。
        """
        markers = MarkerArray()
        if not plan.valid:
            markers.markers.append(self._delete_all(frame_id, stamp))
            return markers

        candidate_pose = copy_pose(plan.candidate.pose)
        # 圆柱只示意目标位置与大小，默认摆正为竖直，避免随抓取姿态一起倾斜。
        if self._upright_object_marker:
            candidate_pose = identity_orientation(candidate_pose)

        # 计划未填 jaw_width（此时为 0，假值）时回退到候选自身的夹爪宽度，保证尺寸可用。
        if self._show_object_marker:
            markers.markers.append(
                self._object_marker(
                    frame_id,
                    stamp,
                    candidate_pose,
                    jaw_width=float(plan.jaw_width or plan.candidate.jaw_width),
                    object_length=float(plan.candidate.object_length),
                )
            )
        # 球标记的尺寸参数是直径（单位 m），自小而大依次为：目标中心 0.025、接近点 0.035、
        # 抓取点 0.04；接近点用蓝色、抓取点用红色，便于一眼区分两个关键位姿。
        if self._show_object_center_marker:
            markers.markers.append(
                self._sphere_marker(
                    frame_id,
                    stamp,
                    4,
                    identity_orientation(plan.candidate.pose),
                    "visual_object_center",
                    0.025,
                    (0.0, 1.0, 0.18, 1.0),
                )
            )
        markers.markers.append(
            self._sphere_marker(
                frame_id,
                stamp,
                1,
                plan.pregrasp_pose,
                "visual_pregrasp",
                0.035,
                (0.1, 0.65, 1.0, 1.0),
            )
        )
        markers.markers.append(
            self._sphere_marker(
                frame_id,
                stamp,
                2,
                plan.grasp_pose,
                "visual_grasp",
                0.04,
                (1.0, 0.18, 0.12, 1.0),
            )
        )
        # 计划位姿描述末端执行器目标，叠加 TCP 安装偏移后才是夹爪真正夹持的位置；接近
        # 箭头与张开方向线段都以这两个 TCP 点为基准绘制。
        pregrasp_tcp = _offset_pose_position(plan.pregrasp_pose, self._tcp_offset_xyz)
        grasp_tcp = _offset_pose_position(plan.grasp_pose, self._tcp_offset_xyz)
        # TCP 点比对应的计划点略小，避免两个球完全互相遮挡。
        if self._show_tcp_markers:
            markers.markers.append(
                self._sphere_marker(
                    frame_id,
                    stamp,
                    5,
                    pregrasp_tcp,
                    "visual_pregrasp_tcp",
                    0.022,
                    (0.0, 0.42, 1.0, 1.0),
                )
            )
            markers.markers.append(
                self._sphere_marker(
                    frame_id,
                    stamp,
                    6,
                    grasp_tcp,
                    "visual_grasp_tcp",
                    0.026,
                    (1.0, 0.0, 0.0, 1.0),
                )
            )
        if self._show_approach_arrow:
            markers.markers.append(self._approach_arrow_marker(frame_id, stamp, pregrasp_tcp, grasp_tcp))
        if self._show_gripper_open_axis:
            markers.markers.append(
                self._gripper_open_axis_marker(
                    frame_id,
                    stamp,
                    grasp_tcp,
                    jaw_width=float(plan.jaw_width or plan.candidate.jaw_width),
                )
            )
        if self._show_object_label:
            markers.markers.append(self._text_marker(frame_id, stamp, plan))
        return markers

    def _base_marker(self, frame_id: str, stamp, marker_id: int, marker_type: int, ns: str) -> Marker:
        """构造通用标记骨架：写头信息、名称空间、编号、类型与动作。

        统一设置 ``action = ADD``（新增/覆盖同 ns+id 的旧标记）、``lifetime.sec = 0``
        （永不过期，靠后续消息刷新或删除）与 ``frame_locked = True``（位姿跟随该坐标系在
        TF 中的更新而移动，而不是固定在发布瞬间的位姿）。
        """
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = ns
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.lifetime.sec = 0
        marker.frame_locked = True
        return marker

    def _delete_all(self, frame_id: str, stamp) -> Marker:
        """返回一个 DELETEALL 标记，请求显示端清除此前发布的整批标记。

        名称空间与类型只是载体：DELETEALL 的语义是"这个计划无效，画面上不应再保留上一次
        的抓取位姿"，避免操作者把过期标记误当成当前计划。
        """
        marker = self._base_marker(frame_id, stamp, 0, Marker.CUBE, "visual_grasp")
        marker.action = Marker.DELETEALL
        return marker

    def _object_marker(self, frame_id: str, stamp, pose: Pose, *, jaw_width: float, object_length: float) -> Marker:
        """生成目标圆柱标记（名称空间 ``visual_object``、编号 0）。

        圆柱在标记协议里沿局部 z 轴延伸，因此 ``scale.x``/``scale.y`` 是直径、``scale.z``
        是高度，两者分别由 ``object_diameter``/``object_height`` 钳制得到（单位 m）；半透明
        绿色（alpha 0.82）填充，便于看到落在圆柱内部的接近点与抓取点。
        """
        marker = self._base_marker(frame_id, stamp, 0, Marker.CYLINDER, "visual_object")
        marker.pose = copy_pose(pose)
        diameter = object_diameter(jaw_width, self._object_min_diameter_m)
        height = object_height(object_length, self._object_min_height_m)
        marker.scale.x = diameter
        marker.scale.y = diameter
        marker.scale.z = height
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.18
        marker.color.a = 0.82
        return marker

    def _sphere_marker(
        self,
        frame_id: str,
        stamp,
        marker_id: int,
        pose: Pose,
        ns: str,
        size: float,
        color: tuple[float, float, float, float],
    ) -> Marker:
        """生成均匀缩放的球标记：``size`` 是直径（单位 m），``color`` 为 (r, g, b, a) 颜色。"""
        marker = self._base_marker(frame_id, stamp, marker_id, Marker.SPHERE, ns)
        marker.pose = copy_pose(pose)
        marker.scale.x = float(size)
        marker.scale.y = float(size)
        marker.scale.z = float(size)
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        return marker

    def _approach_arrow_marker(self, frame_id: str, stamp, pregrasp_tcp: Pose, grasp_tcp: Pose) -> Marker:
        """生成接近方向箭头：由接近点 TCP 指向抓取点 TCP（名称空间 ``visual_approach_axis``）。

        箭头用 ``points`` 的两个端点定方向，方向即实际进给方向；按标记协议 ``scale.x`` 是
        箭杆直径、``scale.y`` 是箭头直径、``scale.z`` 是箭头长度，单位 m，取橙色不透明。
        """
        marker = self._base_marker(frame_id, stamp, 7, Marker.ARROW, "visual_approach_axis")
        marker.points = [copy_point(pregrasp_tcp.position), copy_point(grasp_tcp.position)]
        marker.scale.x = 0.012
        marker.scale.y = 0.026
        marker.scale.z = 0.026
        marker.color.r = 1.0
        marker.color.g = 0.48
        marker.color.b = 0.0
        marker.color.a = 1.0
        return marker

    def _gripper_open_axis_marker(self, frame_id: str, stamp, grasp_tcp: Pose, *, jaw_width: float) -> Marker:
        """生成夹爪张开方向线段：以抓取 TCP 为中心，沿张开轴左右各半个开口。

        张开轴由 TCP 局部方向 ``gripper_open_axis_local_xyz`` 经抓取姿态旋转到参考坐标系；
        方向向量退化为零时回退为参考坐标系 +y 并按单位长度处理，避免除零且线段仍可见。
        ``half_width`` 下限取 0.01 m：夹爪接近闭合（开口很小）时仍画出一段看得见的线；线段
        总长即开口宽度，颜色为青色。``LINE_LIST`` 只用 ``scale.x`` 作为线宽。
        """
        marker = self._base_marker(frame_id, stamp, 8, Marker.LINE_LIST, "visual_gripper_open_axis")
        axis = _rotate_vector_by_quaternion(self._gripper_open_axis_local_xyz, _pose_quaternion(grasp_tcp))
        axis_len = math.sqrt(sum(component * component for component in axis))
        if axis_len <= 1e-9:
            axis = (0.0, 1.0, 0.0)
            axis_len = 1.0
        axis = tuple(component / axis_len for component in axis)
        half_width = max(0.01, float(jaw_width) * 0.5)
        center = grasp_tcp.position
        p0 = (
            float(center.x) - axis[0] * half_width,
            float(center.y) - axis[1] * half_width,
            float(center.z) - axis[2] * half_width,
        )
        p1 = (
            float(center.x) + axis[0] * half_width,
            float(center.y) + axis[1] * half_width,
            float(center.z) + axis[2] * half_width,
        )
        marker.points = [point_from_xyz(grasp_tcp, p0), point_from_xyz(grasp_tcp, p1)]
        marker.scale.x = 0.01
        marker.color.r = 0.0
        marker.color.g = 0.95
        marker.color.b = 1.0
        marker.color.a = 1.0
        return marker

    def _text_marker(self, frame_id: str, stamp, plan: GraspPlan) -> Marker:
        """生成类别与置信度文字（名称空间 ``visual_object_label``、编号 3）。

        文字贴在候选点上方；显示圆柱时另外抬升半个示意高度；字体高度由
        ``label_height_m`` 控制。置信度固定保留两位小数，类别为空时由 ``strip()`` 去掉多余
        空格。``TEXT_VIEW_FACING`` 让文字始终正对相机，便于任意视角阅读。
        """
        marker = self._base_marker(frame_id, stamp, 3, Marker.TEXT_VIEW_FACING, "visual_object_label")
        marker.pose = identity_orientation(plan.candidate.pose)
        height = object_height(float(plan.candidate.object_length), self._object_min_height_m)
        marker.pose.position.z += (height * 0.5 if self._show_object_marker else 0.0) + self._label_height_m
        marker.scale.z = self._label_height_m
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = f"{plan.candidate.class_name} {float(plan.candidate.confidence):.2f}".strip()
        return marker


class VisualGraspMarkerNode(Node):
    """订阅抓取计划并发布标记数组的节点（可执行入口 ``rebotarm_visual_grasp_markers``）。

    生命周期：构造时声明参数、建立 TF 监听器、发布者与订阅，之后由 ``main`` 自旋直至进程
    退出。回调 ``_on_plan`` 在订阅回调线程内同步完成"坐标系换算 → 生成标记 → 发布"，不做
    耗时计算，因此不需要额外线程或工作队列。
    """

    def __init__(self) -> None:
        super().__init__("rebotarm_visual_grasp_markers")
        # 话题与坐标系：input_topic 默认与常规计划话题一致，实际由启动组合指向过滤后的计划；
        # target_frame 最终会被 strip()，留空表示不做 TF 换算、直接沿用计划自带坐标系。
        self.declare_parameter("input_topic", "/grasp/plan")
        self.declare_parameter("output_topic", "/grasp/visual_markers")
        self.declare_parameter("target_frame", "base_link")
        # 以下几项与 Builder 的构造参数一一对应，含义见 VisualGraspMarkerBuilder.__init__。
        self.declare_parameter("object_min_diameter_m", 0.06)
        self.declare_parameter("object_min_height_m", 0.12)
        self.declare_parameter("upright_object_marker", True)
        self.declare_parameter("show_object_marker", True)
        self.declare_parameter("show_object_center_marker", True)
        self.declare_parameter("show_object_label", True)
        self.declare_parameter("label_height_m", 0.055)
        self.declare_parameter("tcp_offset_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("gripper_open_axis_local_xyz", [0.0, 1.0, 0.0])
        self.declare_parameter("show_tcp_markers", True)
        self.declare_parameter("show_approach_arrow", True)
        self.declare_parameter("show_gripper_open_axis", True)
        # 计划无效时是否也发一个删除标记：默认 True，让画面立刻清空；置 False 则保留上一次
        # 的有效标记，便于对照排查"这次为什么没有计划"。
        self.declare_parameter("publish_invalid_delete", True)

        self._input_topic = str(self.get_parameter("input_topic").value)
        self._output_topic = str(self.get_parameter("output_topic").value)
        self._target_frame = str(self.get_parameter("target_frame").value).strip()
        self._publish_invalid_delete = bool(self.get_parameter("publish_invalid_delete").value)
        self._builder = VisualGraspMarkerBuilder(
            object_min_diameter_m=float(self.get_parameter("object_min_diameter_m").value),
            object_min_height_m=float(self.get_parameter("object_min_height_m").value),
            upright_object_marker=bool(self.get_parameter("upright_object_marker").value),
            show_object_marker=bool(self.get_parameter("show_object_marker").value),
            show_object_center_marker=bool(self.get_parameter("show_object_center_marker").value),
            show_object_label=bool(self.get_parameter("show_object_label").value),
            label_height_m=float(self.get_parameter("label_height_m").value),
            tcp_offset_xyz=self._tuple3("tcp_offset_xyz"),
            gripper_open_axis_local_xyz=self._tuple3("gripper_open_axis_local_xyz"),
            show_tcp_markers=bool(self.get_parameter("show_tcp_markers").value),
            show_approach_arrow=bool(self.get_parameter("show_approach_arrow").value),
            show_gripper_open_axis=bool(self.get_parameter("show_gripper_open_axis").value),
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._publisher = self.create_publisher(MarkerArray, self._output_topic, 10)
        self.create_subscription(GraspPlan, self._input_topic, self._on_plan, 10)
        self.get_logger().info(
            f"visual grasp markers ready: input={self._input_topic}, output={self._output_topic}, "
            f"target_frame={self._target_frame}"
        )

    def _tuple3(self, name: str) -> tuple[float, float, float]:
        """读取一个必须恰好含 3 个元素的向量参数，返回浮点元组。

        元素个数不符时抛 ``ValueError``：向量参数配错属于启动期问题，直接失败比静默截断或
        补零更安全（否则偏移方向会被悄悄改掉）。
        """
        values = self.get_parameter(name).value
        if len(values) != 3:
            raise ValueError(f"{name} must contain exactly 3 values")
        return tuple(float(value) for value in values)

    def _on_plan(self, plan: GraspPlan) -> None:
        """计划回调：换算坐标系后生成并发布标记数组。

        标记时间戳固定取零时刻（``Time()`` 的默认值）：RViz 对零时间戳按"使用最新可用变换"
        处理，可避免计划时间戳与 TF 缓存时间不一致时出现外推报错或标记闪烁。
        计划无效且 ``publish_invalid_delete`` 为假时直接返回，保留画面上的既有标记；否则由
        Builder 产生删除标记并发布，实现"无效即清屏"。
        """
        marker_stamp = rclpy.time.Time().to_msg()
        if not plan.valid and not self._publish_invalid_delete:
            return
        converted = self._convert_plan(plan)
        markers = self._builder.build(converted, frame_id=str(converted.header.frame_id), stamp=marker_stamp)
        self._publisher.publish(markers)

    def _convert_plan(self, plan: GraspPlan) -> GraspPlan:
        """把计划中的各位姿换算到 ``target_frame``，返回供渲染使用的副本。

        不原地修改输入（先 ``deepcopy``），以免影响同话题上的其他订阅者。以下情况跳过换算：
        未配置目标坐标系、源坐标系为空、两者已经相同（此时只补齐 ``header.frame_id``）。
        TF 查询超时 0.2 s 或失败时打印告警并返回未换算的副本：标记仍按原坐标系发布，宁可画
        在源坐标系里，也不要因缺一帧变换让可视化整体消失（发布话题上仍有有效标记）。
        换算只改三个位姿及其坐标系标记；``jaw_width``、``object_length``、类别、置信度等
        标量字段与坐标系无关，原样保留。
        """
        source_frame = str(plan.header.frame_id)
        if not self._target_frame or not source_frame or source_frame == self._target_frame:
            converted = deepcopy(plan)
            converted.header.frame_id = source_frame or self._target_frame
            return converted
        converted = deepcopy(plan)
        try:
            tf_msg = self._tf_buffer.lookup_transform(
                self._target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"skip visual markers: cannot transform {source_frame} -> {self._target_frame}: {exc}"
            )
            return converted
        transform = transform_from_msg(tf_msg)
        converted.header.frame_id = self._target_frame
        converted.candidate.header.frame_id = self._target_frame
        converted.candidate.pose = transform_pose_message(converted.candidate.pose, transform)
        converted.pregrasp_pose = transform_pose_message(converted.pregrasp_pose, transform)
        converted.grasp_pose = transform_pose_message(converted.grasp_pose, transform)
        return converted


def main(args=None) -> None:
    """节点入口：初始化、自旋，退出时释放节点资源。

    用户 Ctrl-C（``KeyboardInterrupt``）与外部关闭请求（``ExternalShutdownException``）都按
    正常退出处理，不打印堆栈；只有 ``rclpy`` 仍可用时才调用 ``shutdown()``，避免重复关闭。
    """
    rclpy.init(args=args)
    node = VisualGraspMarkerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

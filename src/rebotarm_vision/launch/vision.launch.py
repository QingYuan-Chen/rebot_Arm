"""视觉包的主启动文件:真实相机路径下启动感知链路所需的节点组合.

启动内容:

1. 可选手眼静态 TF:显式传入 D405 标定文件后,把相机光学坐标系挂到末端;
2. 视觉主节点 ``rebotarm_vision_node``:彩色/深度采集,检测与深度融合,参数来自
   ``camera_config`` 中同名段;
3. 夹爪 TCP 静态帧节点 ``rebotarm_grasp_tcp_frame``:发布末端到抓取 TCP 的固定偏移,
   供抓取位姿换算使用;对齐 RGB-D 的两个光学帧发布同位姿关系.

参数来源:节点参数统一从 ``camera_config`` 指向的 YAML 读取,再按启动参数覆盖.两个
节点都用 ``vision_python_executable`` 作为解释器前缀(默认取环境变量
``REBOTARM_VISION_PYTHON``,未设置时用 ``python3``),便于把带推理依赖的独立虚拟环境
解释器固定下来.

安全边界:本文件只负责感知,不启动运动执行,不使能硬件.任何基于视觉结果的运动都必须
由上层在通过规划,碰撞检查与执行门控后下发.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node

from rebotarm_vision.handeye_config import load_handeye_config


def _node_parameters(config_path: str, node_name: str) -> dict:
    """从 YAML 中取出指定节点的 ``ros__parameters`` 段并转成普通 dict.

    只做读取与类型检查,不填充默认值:文件为空或缺该节点段时返回空字典,交由节点的
    ``declare_parameter`` 兜底;``ros__parameters`` 不是映射时抛 ``RuntimeError``-
    宁可启动失败,也不要带着读错的参数跑起来.
    """

    payload = yaml.safe_load(Path(config_path).read_text(encoding="utf-8")) or {}
    node_payload = payload.get(node_name, {})
    parameters = node_payload.get("ros__parameters", {}) if isinstance(node_payload, dict) else {}
    if not isinstance(parameters, dict):
        raise RuntimeError(f"invalid ros__parameters for {node_name}: {config_path}")
    return dict(parameters)


def _launch_setup(context):
    """延迟执行的回调:等所有启动参数(含环境变量默认值)确定后再组装节点.

    流程:读取启动参数 → 载入手眼标定配置 → 组装公共环境变量 → 用 ``yolo_model_path``
    / ``yolo_device`` 覆盖视觉节点的 YOLO 参数 → 组装节点参数 → 返回节点列表.
    """

    camera_config = LaunchConfiguration("camera_config").perform(context)
    handeye_config = LaunchConfiguration("handeye_config").perform(context)
    yolo_model_path = LaunchConfiguration("yolo_model_path").perform(context)
    yolo_device = LaunchConfiguration("yolo_device").perform(context)
    # 更换相机后旧外参失效;默认只采图,完成 D405 标定后显式加载结果.
    handeye = load_handeye_config(Path(handeye_config).expanduser()) if handeye_config else None

    # OpenCV highgui 的 Qt 后端需要指定平台插件与字体目录,否则在无 Wayland/缺字体的
    # 主机上创建预览窗口会失败;这两个键有测试锁定,不要改动字面量.
    common_environment = {
        "QT_QPA_PLATFORM": "xcb",
        "QT_QPA_FONTDIR": "/usr/share/fonts/truetype/dejavu",
    }
    # 只有显式传入非空启动参数才覆盖 YAML,空串表示沿用配置文件里的取值.
    vision_overrides = {}
    if yolo_model_path:
        vision_overrides["yolo.model_path"] = yolo_model_path
    if yolo_device:
        vision_overrides["yolo.device"] = yolo_device
    vision_parameters = _node_parameters(camera_config, "rebotarm_vision_node")
    vision_parameters.update(vision_overrides)
    tcp_parameters = _node_parameters(camera_config, "rebotarm_grasp_tcp_frame")

    transforms = []
    if handeye is not None:
        transforms.append(Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="rebotarm_handeye_static_tf",
            output="screen",
            arguments=handeye.as_static_transform_arguments(),
        ))
    else:
        transforms.append(LogInfo(msg=(
            "D405 handeye_config is empty: camera-to-arm TF is unpublished; "
            "pass a calibrated D405 handeye_config to enable robot-frame candidate filtering."
        )))
    # 自定义 YAML 可只覆盖部分参数;缺省值与视觉节点声明保持一致.
    if vision_parameters.get("camera.enable_depth", True) and vision_parameters.get("camera.enable_align", True):
        color_frame = vision_parameters.get("ros.frame_id_color", "camera_color_frame")
        depth_frame = vision_parameters.get("ros.frame_id_depth", "camera_depth_frame")
        optical_parent = handeye.child_frame if handeye else color_frame
        if optical_parent not in (color_frame, depth_frame):
            raise ValueError("D405 handeye child_frame must match a configured RGB-D optical frame")
        if color_frame != depth_frame:
            optical_child = depth_frame if optical_parent == color_frame else color_frame
            # 深度已重投影到彩色光学系;此处只连接两个消息名,不再叠加出厂外参.
            transforms.append(Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="rebotarm_d405_aligned_optical_tf",
                output="screen",
                arguments=["--frame-id", optical_parent, "--child-frame-id", optical_child,
                           "--x", "0", "--y", "0", "--z", "0",
                           "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1"],
            ))

    return transforms + [
        # 视觉主节点:负责取图,检测与深度融合,不下发任何运动指令.
        Node(
            package="rebotarm_vision",
            executable="rebotarm_vision_node",
            prefix=LaunchConfiguration("vision_python_executable"),
            name="rebotarm_vision_node",
            output="screen",
            parameters=[vision_parameters],
            additional_env=common_environment,
        ),
        # 夹爪 TCP 静态帧:发布末端到抓取 TCP 的固定偏移(父子坐标系与偏移量取自 YAML).
        Node(
            package="rebotarm_vision",
            executable="rebotarm_grasp_tcp_frame",
            prefix=LaunchConfiguration("vision_python_executable"),
            name="rebotarm_grasp_tcp_frame",
            output="screen",
            parameters=[tcp_parameters],
            additional_env=common_environment,
        ),
    ]


def generate_launch_description():
    """声明全部启动参数,并注册延迟执行的 ``_launch_setup``.

    默认值的含义与安全考虑:

    - ``camera_config`` 默认取包内 D405 配置;``handeye_config`` 默认为空,
      完成新相机标定后显式传入文件;
    - ``vision_python_executable``:解释器前缀,优先取环境变量
      ``REBOTARM_VISION_PYTHON``,未设置时用 ``python3``;
    - ``yolo_model_path`` / ``yolo_device`` 默认空串:表示不覆盖 YAML 里的模型路径与
      推理设备.
    """

    vision_share = Path(get_package_share_directory("rebotarm_vision"))

    return LaunchDescription(
        [
            # 相机与节点参数配置文件(内部按节点名分段).
            DeclareLaunchArgument(
                "camera_config",
                default_value=str(vision_share / "config" / "camera_ubuntu.yaml"),
            ),
            # 手眼标定结果(父/子坐标系与平移,四元数),决定相机在机械臂下的位姿.
            DeclareLaunchArgument(
                "handeye_config",
                default_value="",
                description="Calibrated D405 handeye YAML; empty starts camera without arm TF",
            ),
            # 视觉节点解释器:环境变量优先,缺省 python3.
            DeclareLaunchArgument(
                "vision_python_executable",
                default_value=EnvironmentVariable("REBOTARM_VISION_PYTHON", default_value="python3"),
            ),
            # YOLO 模型路径与推理设备覆盖值;空串表示沿用配置文件.
            DeclareLaunchArgument("yolo_model_path", default_value=""),
            DeclareLaunchArgument("yolo_device", default_value=""),
            OpaqueFunction(function=_launch_setup),
        ]
    )

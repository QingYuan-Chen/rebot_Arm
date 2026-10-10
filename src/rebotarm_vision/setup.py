# 视觉包的 ament_python 安装脚本。
# 职责：安装 Python 包（含推理后端），将 launch、配置和模型安装到 share，
# 并注册可视化与抓取相关的控制台入口点。

from pathlib import Path

from setuptools import find_packages, setup

package_name = "rebotarm_vision"
# 模型权重存放在视觉包的 models/ 目录下。
# yolo26s-seg.pt 是 PyTorch 权重（通用回退），
# yolo26m-seg-fp16-b1-640-linux.engine 是 TensorRT 半精度引擎（批 1、640 输入，仅 Linux）。
vision_model_sources = [
    Path("models/yolo26s-seg.pt"),
    Path("models/yolo26m-seg-fp16-b1-640-linux.engine"),
]

# 运行期资产在构建期是可选的：权重可能尚未下载，此时不打包进 models/ 目录。
# 真正缺失模型会在检测/推理节点启动时报错（参数 yolo.model_path 必须指向存在的文件），
# 因此这里静默过滤不存在的权重是安全的，不会掩盖配置错误。
vision_model_sources = [
    path for path in vision_model_sources
    if (Path(__file__).resolve().parent / path).is_file()
]

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    # data_files 决定安装到 install/share/rebotarm_vision 下的目录结构；
    # 元组第一项是目标目录，第二项是待安装文件列表。
    data_files=[
        # ament 索引标记文件：ament/colcon 靠它识别本包
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        # 包清单
        (f"share/{package_name}", ["package.xml"]),
        # 启动文件：vision.launch.py 为通用入口，vision_ubuntu.launch.py 为 Ubuntu 原生相机入口
        (
            f"share/{package_name}/launch",
            ["launch/vision.launch.py", "launch/vision_ubuntu.launch.py"],
        ),
        # 参数配置：相机 profile、抓取候选姿态策略、GraspNet 策略、夹爪/重试/退避/视觉伺服/
        # 桌面安全等独立 YAML，供启动文件按 profile 选择加载
        (
            f"share/{package_name}/config",
            [
                "config/camera_ubuntu.yaml",
                "config/grasp_pose_policy.yaml",
                "config/graspnet_ubuntu.yaml",
                "config/gripper_policy.yaml",
                "config/handeye.yaml",
                "config/retry_policy.yaml",
                "config/retreat_policy.yaml",
                "config/single_bottle_grasp.yaml",
                "config/table_safety.yaml",
                "config/visual_servo.yaml",
                "config/visual_strategy_profile.yaml",
            ],
        ),
        # YOLO 权重（构建期缺失时为空列表）
        (
            f"share/{package_name}/models",
            [str(path) for path in vision_model_sources],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="reBotArm Maintainers",
    maintainer_email="support@example.com",
    description="ROS 2 Gemini2 vision node for reBotArm grasping.",
    license="Apache-2.0",
    # 控制台入口点。名称（字符串）是对外接口，脚本名与 launch 里的 executable= 必须一致，禁止改动。
    # 采集/检测：视觉主节点、GraspNet 候选节点、离线 YOLO 节点
    # 预览/可视化：抓取预览发送器、RViz 标记发布器
    # 执行/筛选：视觉抓取执行器、抓取候选 IK 过滤器
    # 工具/标定：抓取 TCP 坐标系广播、相机调试预览、深度探针、Open3D 查看器
    entry_points={
        "console_scripts": [
            "rebotarm_vision_node = rebotarm_vision.nodes.vision_node:main",
            "rebotarm_graspnet_baseline_node = rebotarm_vision.nodes.graspnet_baseline_node:main",
            "rebotarm_send_grasp_preview = rebotarm_vision.nodes.grasp_preview_sender_node:main",
            "rebotarm_visual_grasp_markers = rebotarm_vision.nodes.visual_grasp_marker_node:main",
            "rebotarm_grasp_candidate_markers = rebotarm_vision.nodes.grasp_candidate_marker_node:main",
            "rebotarm_single_bottle_grasp = rebotarm_vision.single_bottle_grasp:main",
            "rebotarm_visual_grasp_executor = rebotarm_vision.nodes.visual_grasp_executor_node:main",
            "rebotarm_grasp_candidate_ik_filter = rebotarm_vision.nodes.candidate_ik_filter_node:main",
            "rebotarm_grasp_tcp_frame = rebotarm_vision.nodes.grasp_tcp_frame_node:main",
            "rebotarm_debug_camera_preview = rebotarm_vision.diagnostics.debug_camera_preview:main",
            "rebotarm_grasp_depth_probe = rebotarm_vision.diagnostics.grasp_depth_probe_node:main",
            "rebotarm_graspnet_open3d_viewer = rebotarm_vision.diagnostics.graspnet_open3d_viewer:main",
            "rebotarm_offline_yolo_node = rebotarm_vision.nodes.offline_yolo_node:main",
        ],
    },
)

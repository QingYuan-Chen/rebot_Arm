"""仿真包（ament_python）的安装描述文件。

职责
    声明 Python 包、控制台入口以及需要随包安装到 ``share/`` 的模型与配置资源；
    本文件只描述安装布局，不会启动任何进程，也不开启硬件通道。

资源安装策略
    ``install_resources`` 把 ``models/`` 下的 XML 模型、STL 网格、
    ``config/*.yaml`` 与保留的 ``launch/*.launch.py`` 按"源目录 → 同名安装目录"归并后
    安装到 ``share/<包名>/<原相对目录>``。运行期通过 ament 资源索引按相对路径定位，
    因此模型/网格必须与代码同版本安装，否则会加载到旧模型或找不到网格。

控制台入口
    每个 ``console_scripts`` 项对应一个对外命令（健康检查、命令行驱动、查看器、
    ROS 节点、URDF→MJCF 转换、仿真轨迹控制器）。这些名字是外部调用契约（文档、
    启动文件与测试都直接引用），不可随意改名。

依赖约束
    物理引擎锁定在 3.x（``mujoco>=3.11,<4``）：模型语义与接触行为跨大版本不保证兼容，
    升级需要重新做模型校验。
"""

import os
from glob import glob

from setuptools import find_packages, setup

package_name = "rebotarm_simulation"


def install_resources(pattern):
    """把匹配 ``pattern`` 的资源按各自所在的相对目录归并为 data_files 条目。

    返回 ``[(安装目录, [文件列表])]``。之所以逐个源目录归并而不是统一平铺到一个
    目录：URDF/MuJoCo 模型用相对路径引用同目录的网格与子模型，运行时目录结构必须
    与源码树一致，否则相对引用解析失败。
    """
    files_by_destination = {}
    for path in glob(pattern, recursive=True):
        destination = os.path.join("share", package_name, os.path.dirname(path))
        files_by_destination.setdefault(destination, []).append(path)
    return [
        (destination, sorted(files_by_destination[destination]))
        for destination in sorted(files_by_destination)
    ]


launch_files = glob("launch/*.launch.py")


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    include_package_data=True,
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ]
    + install_resources("models/**/*.xml")
    + install_resources("models/**/*.[sS][tT][lL]")
    + install_resources("models/**/*.json")
    + install_resources("models/**/*.txt")
    + install_resources("models/**/*.step")
    + install_resources("config/*.yaml")
    + [(f"share/{package_name}/launch", sorted(launch_files))],
    install_requires=["setuptools", "mujoco>=3.11,<4", "numpy>=1.26", "PyYAML>=6"],
    zip_safe=True,
    maintainer="reBotArm Maintainers",
    maintainer_email="support@example.com",
    description="MuJoCo physics, simulated execution and diagnostics for reBotArm.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "rebotarm_mujoco_health = rebotarm_simulation.diagnostics.mujoco_health:main",
            "rebotarm_mujoco_cli = rebotarm_simulation.apps.mujoco_cli:main",
            "rebotarm_mujoco_viewer = rebotarm_simulation.apps.mujoco_viewer:main",
            "rebotarm_mujoco_teach_preview = rebotarm_simulation.apps.teach_preview:main",
            "rebotarm_mujoco_node = rebotarm_simulation.ros.mujoco_ros_node:main",
            "rebotarm_urdf_to_mjcf = rebotarm_simulation.model_tools.urdf_to_mjcf:main",
        ],
    },
)

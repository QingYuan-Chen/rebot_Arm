"""运动包的构建脚本（只在编译安装期执行，不参与运行时逻辑）。

要点
    1. ``packages=find_packages(exclude=["test"])``：把包目录整体安装，排除测试目录；
    2. ``data_files`` 负责把资源索引标记、``package.xml`` 与 ``config/*.yaml`` 安装到
       ``share/`` 下对应包的共享目录中，节点运行时才能按包共享目录找到参数文件；
    3. ``entry_points`` 声明两个可执行入口：位姿预览/执行节点与视觉就绪节点。这里注册的
       可执行文件名（以及模块路径）是上层启动文件与测试依赖的对外接口，改名即为破坏性变更。
"""

from glob import glob
from setuptools import find_packages, setup

# 包名同时决定共享目录、资源索引标记路径与入口点所属包，必须与 package.xml 的 name 一致。
package_name = "rebotarm_motion"

setup(
    name=package_name,
    version="0.1.0",
    # 只安装 Python 包目录；exclude 中的 "test" 是目录名，不会被当作运行时依赖。
    packages=find_packages(exclude=["test"]),
    data_files=[
        # 空标记文件：供 ament 的资源索引发现本包（缺它就查不到包共享目录）。
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        # 包清单随包安装，便于运行时查询依赖与版本。
        (f"share/{package_name}", ["package.xml"]),
        # 参数文件统一安装到 share/<包名>/config，与节点按包共享目录拼路径的读取方式对应。
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="reBotArm Maintainers",
    maintainer_email="support@example.com",
    description="Motion generation, retiming, validation, and safety utilities for reBotArm.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            # 可执行名 = 模块:函数；左侧名字被上层启动文件以 executable=... 引用。
            "PoseExecutionNode = rebotarm_motion.pose_execution_node:main",
            "rebotarm_visual_ready = rebotarm_motion.visual_ready_node:main",
            "rebotarm_mujoco_moveit_acceptance = rebotarm_motion.mujoco_moveit_acceptance:main",
        ],
    },
)

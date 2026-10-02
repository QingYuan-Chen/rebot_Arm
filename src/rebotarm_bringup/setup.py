"""启动组合包的安装脚本：只把 launch、config、rviz 三类资源装到共享目录。

本包没有 Python 运行时代码，因此 ``packages`` 为空；被安装的内容全部是启动文件与参数
文件，供 ``ros2 launch`` 与各节点在运行期按包共享目录（share）路径读取。
"""

from glob import glob
from pathlib import Path
from setuptools import setup

package_name = "rebotarm_bringup"


# Keep modular launch fragments under ``launch/includes`` installable while preserving
# the flat ``ros2 launch rebotarm_bringup <name>`` entry points.
launch_files = glob("launch/*.launch.py") + glob("launch/*.md")
launch_include_files = glob("launch/includes/*.launch.py")

setup(
    name=package_name,
    version="0.1.0",
    # 无 Python 包（本包不含可导入的模块），资源通过下面的 data_files 安装
    packages=[],
    data_files=[
        # ament 资源索引标记：让 ROS 2 能在环境中发现本包
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        # 包清单，随共享目录一起安装
        (f"share/{package_name}", ["package.xml"]),
        # 启动文件及同目录结构说明。
        (
            f"share/{package_name}/launch",
            launch_files,
        ),
        (
            f"share/{package_name}/launch/includes",
            launch_include_files,
        ),
        # 参数文件：所有 *.yaml（机械臂/夹爪/遥操作等）
        (f"share/{package_name}/config", glob("config/*.yaml")),
        # 可视化布局：所有 *.rviz
        (f"share/{package_name}/rviz", glob("rviz/*.rviz")),
    ],
    # 安装期依赖：仅需要 setuptools 本体
    install_requires=["setuptools"],
    # 纯资源包，可安全打包为 zip
    zip_safe=True,
    maintainer="reBotArm Maintainers",
    maintainer_email="support@example.com",
    description="Launch, configuration, and description files for reBotArm.",
    license="Apache-2.0",
)

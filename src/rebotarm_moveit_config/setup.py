# 构建脚本：把一个"纯数据 + 启动文件"的 ament_python 包按标准布局安装到 share/ 下。
#
# 本包没有 Python 模块，因此 packages=[]；所有内容通过 data_files 原样拷贝到
# share/<包名>/ 的对应子目录，供 MoveIt、RViz 与上层启动组合包按相对路径读取：
# - config/：URDF、SRDF、各种 YAML 规划/控制器配置（由 MoveItConfigsBuilder 按文件名加载）；
# - meshes/：URDF 里 package:// 引用的 STL 网格，缺失会导致模型加载失败；
# - launch/：演示启动文件（只收 *.launch.py，避免把 __pycache__ 之类的产物装进去）；
# - rviz/：MotionPlanning 面板配置。
# 注意：这里的安装子目录名与文件扩展名过滤器共同构成对外契约——启动文件与其它包按
# share/<包名>/<子目录>/<文件名> 定位资源，改动这些名字会直接导致运行期找不到文件。
from glob import glob
from setuptools import setup

package_name = "rebotarm_moveit_config"

setup(
    name=package_name,
    version="0.1.0",
    packages=[],
    data_files=[
        # ament 资源索引：声明本包已被安装，ros2/ament 靠它发现包（必须位于该固定路径）。
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        # 包清单：随包安装，供 rosdep/依赖检查读取。
        (f"share/{package_name}", ["package.xml"]),
        # 模型与规划配置（URDF/SRDF/YAML）。
        (f"share/{package_name}/config", glob("config/*")),
        # URDF 引用的 STL 网格文件。
        (f"share/{package_name}/meshes", glob("meshes/*")),
        # RViz MotionPlanning 面板配置。
        (f"share/{package_name}/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="reBotArm Maintainers",
    maintainer_email="support@example.com",
    description="Minimal MoveIt2 configuration package skeleton for reBotArm.",
    license="Apache-2.0",
)

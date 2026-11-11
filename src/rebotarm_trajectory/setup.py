"""共享轨迹库的 ament_python 打包入口.

文件职责:安装 Python 数值模块,包清单和 ament 资源索引.
对外接口:供 setuptools/colcon 构建使用,不注册 ROS 可执行节点.
安全约束:打包过程不导入数值实现或设备库;空 resource 标记须原样保留.
文件结构:setuptools 导入 → 包元数据和安装资源声明.
"""

from setuptools import find_packages, setup

setup(
    name="rebotarm_trajectory", version="0.1.0", packages=find_packages(),
    data_files=[("share/ament_index/resource_index/packages", ["resource/rebotarm_trajectory"]),
                ("share/rebotarm_trajectory", ["package.xml"])],
    install_requires=["setuptools"], zip_safe=True,
    maintainer="reBotArm Maintainers", maintainer_email="support@example.com",
    description="Pure numerical trajectory data and sampling for reBotArm.",
    license="Apache-2.0",
)

"""共享力控库的 ament_python 打包入口.

职责:安装数值模块,配置与包索引;接口:供 setuptools/colcon 构建.
安全约束:构建不导入控制算法或设备库,不注册硬件节点.
文件结构:构建工具导入,包元数据与安装资源声明.
"""

from setuptools import find_packages, setup

setup(
    name="rebotarm_force_control",
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/rebotarm_force_control"]),
        ("share/rebotarm_force_control", ["package.xml"]),
        ("share/rebotarm_force_control/config", ["config/gravity_compensation.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="reBotArm Maintainers",
    maintainer_email="support@example.com",
    description="Shared gravity and joint force control algorithms for reBotArm.",
    license="Apache-2.0",
)

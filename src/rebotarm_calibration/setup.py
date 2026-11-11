"""ament_python 打包脚本:安装标定包本体并注册两个命令行入口.

安装内容:Python 包代码(由 ``find_packages`` 收集),以及 ROS 2 资源索引所需的
``resource/<包名>`` 标记文件与 ``package.xml``-前者供 ``ros2 pkg`` 等工具按包名查找,
后者提供运行时依赖与元数据.

注册的控制台入口:

- ``rebotarm_handeye_residual``:手眼残差分析命令行(离线评估标定质量);
- ``rebotarm_tcp_calibration``:TCP 标定交互式节点(采集样本并解算 TCP 偏置).

这两个可执行文件名是对外接口(启动文件,文档与运维脚本按名字调用),不要改名.
"""

from setuptools import find_packages, setup


package_name = "rebotarm_calibration"

setup(
    name=package_name,
    version="0.1.0",
    # 只打包运行时代码,排除测试目录
    packages=find_packages(exclude=["test"]),
    data_files=[
        # ament 资源索引标记:文件名必须等于包名,否则 ros2 pkg 系列工具找不到本包
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        # 把 package.xml 装到 share/<包名>/,供运行时查询依赖与元数据
        (f"share/{package_name}", ["package.xml"]),
    ],
    # 运行期只需 setuptools;数值与视觉依赖由 package.xml 以 rosdep 键声明
    install_requires=["setuptools"],
    # 纯 Python 包,无依赖源码树相对路径的数据文件,允许 zip 形式安装
    zip_safe=True,
    maintainer="reBotArm Maintainers",
    maintainer_email="support@example.com",
    description="Hand-eye, TCP, and TF calibration validation tools for reBotArm.",
    license="Apache-2.0",
    entry_points={
        # console_scripts:脚本名 → 包内入口函数;安装位置由 setup.cfg 指定为 lib/<包名>
        "console_scripts": [
            "rebotarm_handeye_capture = rebotarm_calibration.handeye_capture_node:main",
            "rebotarm_handeye_calibration = rebotarm_calibration.handeye_calibration_cli:main",
            "rebotarm_handeye_residual = rebotarm_calibration.handeye_residual_cli:main",
            "rebotarm_tcp_calibration = rebotarm_calibration.tcp_calibration_node:main",
        ],
    },
)

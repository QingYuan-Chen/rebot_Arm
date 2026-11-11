"""ament_python 打包脚本:安装面板包本体,前端静态资源并注册一个控制台入口.

安装内容:Python 包代码(由 ``find_packages`` 收集),面板前端页面
``status_panel_assets/index.html``,以及 ROS 2 资源索引所需的 ``resource/<包名>``
标记文件与 ``package.xml``-前者供 ``ros2 pkg`` 等工具按包名查找,后者提供运行时
依赖与元数据.

注册的控制台入口 ``TeleopStatusPanelNode``:面板节点,负责聚合状态,提供本机
HTTP/SSE 服务并调用操作与示教层的服务.该可执行文件名是对外接口(启动文件,
文档与运维脚本按名字调用),不要改名.
"""

from setuptools import find_packages, setup

# 包名同时用于安装路径与资源索引标记文件名,必须与 package.xml 的 <name> 一致
package_name = "rebotarm_dashboard"

setup(
    name=package_name,
    version="0.1.0",
    # 只打包运行时代码,排除测试目录
    packages=find_packages(exclude=["test"]),
    # 非 .py 的前端资源不会被 find_packages 收集,需在此显式声明随包安装
    package_data={
        "rebotarm_dashboard.status_panel_assets": ["index.html", "calibration.html"],
    },
    data_files=[
        # ament 资源索引标记:文件名必须等于包名,否则 ros2 pkg 系列工具找不到本包
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        # 把 package.xml 装到 share/<包名>/,供运行时查询依赖与元数据
        (f"share/{package_name}", ["package.xml"]),
    ],
    # 运行期只需 setuptools;ROS 与第三方依赖由 package.xml 以 rosdep 键声明
    install_requires=["setuptools"],
    # 纯 Python 包,无依赖源码树相对路径的数据文件,允许 zip 形式安装
    zip_safe=True,
    maintainer="reBotArm Maintainers",
    maintainer_email="support@example.com",
    description="Web dashboard, SSE status, and API adapter for reBotArm.",
    license="Apache-2.0",
    entry_points={
        # console_scripts:脚本名 → 包内入口函数;安装位置由 setup.cfg 指定为 lib/<包名>
        "console_scripts": [
            "TeleopStatusPanelNode = rebotarm_dashboard.teleop_status_panel_node:main",
        ],
    },
)

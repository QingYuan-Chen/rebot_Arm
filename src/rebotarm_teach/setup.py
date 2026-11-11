from setuptools import find_packages, setup

# 包名：必须与 package.xml 的 <name> 和资源索引文件 resource/<包名> 完全一致。
package_name = "rebotarm_teach"

setup(
    name=package_name,
    version="0.1.0",
    # 只打包 Python 包目录，排除测试目录 test（不随安装发布）。
    packages=find_packages(exclude=["test"]),
    data_files=[
        # ament 资源索引：让 colcon/ros2 能通过包名定位到本包的 share 目录。
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        # 把 package.xml 安装到 share/<包名>，供运行时查询依赖与元数据。
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    # 纯 Python 包，安装产物与平台无关，允许以 zip 形式分发。
    zip_safe=True,
    maintainer="reBotArm Maintainers",
    maintainer_email="support@example.com",
    description="Teach recording, prepared trajectory, and replay workflow utilities for reBotArm.",
    license="Apache-2.0",
    entry_points={
        # 控制台入口：安装后可执行文件名 -> 模块:入口函数。
        # TeachRecorderNode：示教录制节点（重力补偿示教，写 JSONL 记录）。
        "console_scripts": [
            "TeachRecorderNode = rebotarm_teach.teach_recorder_node:main",
        ],
    },
)

"""rebotarmcontroller 的 ament_python 打包脚本。

只声明安装元数据与可执行入口，不含运行逻辑。六个 console_scripts 分别对应真机控制器
主节点与 examples 下的演示、验收客户端；入口名会出现在启动文件与测试中，改名或增删
入口会直接影响 launch 与验收流程。
"""

from setuptools import find_packages, setup

package_name = "rebotarmcontroller"  # 同时作为 ament 资源索引名与 share 目录名

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),  # 安装包内全部 Python 子包（含 examples）
    data_files=[
        # ament 资源索引标记：ros2 run 与启动文件按包名查找本包时依赖它
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        # 清单文件随包安装到 share 目录，供运行时按包名定位资源
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", ["config/gravity_compensation.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="reBotArm Maintainers",
    maintainer_email="support@example.com",
    description="ROS 2 controller node for reBotArm.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            # 真机硬件控制主节点：唯一访问电机 SDK/串口通道的进程
            "reBotArmController = rebotarmcontroller.rebotarm_controller:main",
            # 以下为可直接运行的演示客户端（只通过话题/服务/动作访问硬件）
            # 重力补偿（零重力手动拖拽）演示
            "GravityCompensation = rebotarmcontroller.examples.gravity_compensation:main",
            # 夹爪开合交互演示
            "GripperControl = rebotarmcontroller.examples.gripper_control:main",
            # 关节空间点位运动演示（走 FollowJointTrajectory 动作）
            "MoveTo = rebotarmcontroller.examples.move_to:main",
            # 末端位姿运动演示（走自定义 MoveToPose 动作）
            "MoveToPose = rebotarmcontroller.examples.move_to_pose:main",
        ],
    },
)

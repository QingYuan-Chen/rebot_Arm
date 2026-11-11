from setuptools import find_packages, setup

package_name = "rebotarm_teleop"

# 本包通过 ament_python 安装:节点以 console_scripts 形式注册,
# 由上层启动组合按可执行文件名拉起,因此入口名与入口函数都不能随意改名.
setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        # ROS 2 索引标记文件:ament 靠它发现本包,缺失会导致依赖解析失败
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        # 安装包清单到 share/<包名>/,供 ament 与运行时查询依赖信息
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="reBotArm Maintainers",
    maintainer_email="support@example.com",
    description="Keyboard, web, and gripper teleoperation command adapters for reBotArm.",
    license="Apache-2.0",
    entry_points={
        # 可执行文件名 → 模块函数:改左侧名字会让启动文件找不到节点
        "console_scripts": [
            "TeleopKeyboardNode = rebotarm_teleop.teleop_keyboard_node:main",
            "GripperVisualJointStateNode = rebotarm_teleop.gripper_visual_joint_state_node:main",
        ],
    },
)

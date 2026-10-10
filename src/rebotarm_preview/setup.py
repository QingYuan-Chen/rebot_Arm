from setuptools import find_packages, setup


package_name = "rebotarm_preview"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="reBotArm Maintainers",
    maintainer_email="support@example.com",
    description="Lightweight ROS 2 FollowJointTrajectory and RViz preview backend for reBotArm.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "rebotarm_sim_trajectory_controller = rebotarm_preview.rviz_preview_controller_node:main",
        ],
    },
)

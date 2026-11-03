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

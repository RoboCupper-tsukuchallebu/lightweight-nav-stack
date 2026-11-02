from setuptools import find_packages, setup

PACKAGE_NAME = "lightnav_pose_adapter"

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE_NAME]),
        ("share/" + PACKAGE_NAME, ["package.xml", "README.md"]),
        ("share/" + PACKAGE_NAME + "/config", ["config/pose_adapter.yaml"]),
    ],
    install_requires=["setuptools"],
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    maintainer="lightnav maintainers",
    maintainer_email="maintainers@example.com",
    description="Odometry to 2D pose adapter for lightnav",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "pose_adapter_node = lightnav_pose_adapter.pose_adapter_node:main",
        ],
    },
)

import glob

from setuptools import find_packages, setup

PACKAGE_NAME = "lightnav_sim"

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE_NAME]),
        ("share/" + PACKAGE_NAME, ["package.xml"]),
        ("share/" + PACKAGE_NAME + "/launch", glob.glob("launch/*.py")),
        ("share/" + PACKAGE_NAME + "/config", glob.glob("config/*.yaml")),
        ("share/" + PACKAGE_NAME + "/rviz", glob.glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lightnav maintainers",
    maintainer_email="maintainers@example.com",
    description="Kinematic simulator for lightnav closed-loop tests",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "kinematic_sim_node = lightnav_sim.kinematic_sim_node:main",
        ],
    },
)

from setuptools import find_packages, setup

PACKAGE_NAME = "lightnav_mission"

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE_NAME]),
        ("share/" + PACKAGE_NAME, ["package.xml"]),
    ],
    install_requires=["setuptools", "pyyaml"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="lightnav maintainers",
    maintainer_email="maintainers@example.com",
    description="Mission manager, state machine and waypoint CLIs for lightnav",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "mission_node = lightnav_mission.mission_node:main",
            "send_waypoints = lightnav_mission.send_waypoints:main",
            "record_waypoints = lightnav_mission.record_waypoints:main",
            "refine_waypoints = lightnav_mission.refine_waypoints:main",
        ],
    },
)

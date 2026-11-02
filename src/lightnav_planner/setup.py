from setuptools import find_packages, setup

PACKAGE_NAME = "lightnav_planner"

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE_NAME]),
        ("share/" + PACKAGE_NAME, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lightnav maintainers",
    maintainer_email="maintainers@example.com",
    description="Stateless straight-line waypoint path planner for lightnav",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "planner_node = lightnav_planner.planner_node:main",
        ],
    },
)

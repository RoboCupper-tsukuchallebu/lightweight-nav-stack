import glob

from setuptools import find_packages, setup

PACKAGE_NAME = "lightnav_bringup"

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE_NAME]),
        ("share/" + PACKAGE_NAME, ["package.xml", "README.md"]),
        ("share/" + PACKAGE_NAME + "/launch", glob.glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    maintainer="lightnav maintainers",
    maintainer_email="maintainers@example.com",
    description="Launch file that starts the lightnav nodes on the robot",
    license="Apache-2.0",
)

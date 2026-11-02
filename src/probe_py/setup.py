from setuptools import find_packages, setup
setup(name="probe_py", version="0.1.0", packages=find_packages(exclude=["test"]),
      data_files=[("share/ament_index/resource_index/packages", ["resource/probe_py"]), ("share/probe_py", ["package.xml"])],
      install_requires=["setuptools"], extras_require={"test": ["pytest"]}, zip_safe=True)

import os
from glob import glob
from setuptools import find_packages, setup

package_name = "humanoid_kinematics"

data_files = [
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ("share/" + package_name, ["package.xml"]),
]


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="HUMANOID-LAB contributors",
    maintainer_email="maintainers@users.noreply.github.com",
    description="Analytic 2-link leg forward/inverse kinematics library for the simplified biped (no ROS dependency).",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [

        ],
    },
)

import os
from glob import glob
from setuptools import find_packages, setup

package_name = "humanoid_description"

data_files = [
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ("share/" + package_name, ["package.xml"]),
]
data_files.append(("share/" + package_name + "/urdf", glob("urdf/**/*", recursive=True)))
data_files.append(("share/" + package_name + "/launch", glob("launch/**/*", recursive=True)))
data_files.append(("share/" + package_name + "/rviz", glob("rviz/**/*", recursive=True)))
data_files.append(("share/" + package_name + "/config", glob("config/**/*", recursive=True)))

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="HUMANOID-LAB contributors",
    maintainer_email="maintainers@users.noreply.github.com",
    description="URDF/xacro description, TF frames and RViz display launch for the HUMANOID-LAB simplified biped.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [

        ],
    },
)

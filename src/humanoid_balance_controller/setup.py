import os
from glob import glob
from setuptools import find_packages, setup

package_name = "humanoid_balance_controller"

data_files = [
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ("share/" + package_name, ["package.xml"]),
]
data_files.append(("share/" + package_name + "/launch", glob("launch/**/*", recursive=True)))
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
    description="LIP/capture-point push-recovery math and the ankle-strategy standing-balance PD controller.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "balance_controller = humanoid_balance_controller.balance_controller_node:main",
            "com_balance_controller = humanoid_balance_controller.com_balance_node:main",
        ],
    },
)

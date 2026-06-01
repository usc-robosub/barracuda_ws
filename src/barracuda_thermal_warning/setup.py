import os
from glob import glob
from setuptools import find_packages, setup

package_name = "barracuda_thermal_warning"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="root@todo.todo",
    description="Jetson thermal monitoring and overheating warning package for Barracuda onboard ROS 2 systems.",
    license="Apache-2.0",
    tests_require=[],
    entry_points={
        "console_scripts": [
            "jetson_thermal_warning = barracuda_thermal_warning.jetson_thermal_warning:main",
        ],
    },
)

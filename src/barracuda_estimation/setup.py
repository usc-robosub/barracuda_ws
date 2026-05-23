import os
from glob import glob
from setuptools import find_packages, setup

package_name = "barracuda_estimation"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="root@todo.todo",
    description="Direct-subscription estimator skeleton for Barracuda.",
    license="Apache-2.0",
    tests_require=[],
    entry_points={
        "console_scripts": [
            "estimator_node = barracuda_estimation.estimator_node:main",
        ],
    },
)

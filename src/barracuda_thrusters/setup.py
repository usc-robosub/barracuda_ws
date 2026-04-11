import os
from glob import glob
from setuptools import find_packages, setup

package_name = "barracuda_thrusters"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
    ],
    package_data={'barracuda_thrusters': ['*.csv'],}, # makes t200 thruster csv available to python files in this package
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mihir",
    maintainer_email="mihirsin@usc.edu",
    description="barracuda_thrusters package contains software that interfaces with the microcontrollers on the vehicle's thruster boards",
    license="Apache-2.0",
    tests_require=[],
    entry_points={
        "console_scripts": [
            "teensy_comms = barracuda_thrusters.teensy_comms_node:main",
        ],
    },
)

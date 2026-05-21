from setuptools import setup
import os
from glob import glob

package_name = "barracuda_navigation"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            f"goal_bridge = {package_name}.goal_bridge:main",
               f"hardcoded_nav2_publisher = {package_name}.hardcoded_nav2_publisher:main",
        ],
    },
)

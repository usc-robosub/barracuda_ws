from setuptools import setup


package_name = "barracuda_nvblox_launch"


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name + "/launch", ["launch/barracuda_nvblox.launch.py", "launch/barracuda_nvblox_launch.launch.py"]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Barracuda",
    maintainer_email="uscauv@gmail.com",
    description="Launch files to run nvblox against Barracuda ZED topics.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={},
)

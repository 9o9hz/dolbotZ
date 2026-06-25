import os
from glob import glob

from setuptools import find_packages, setup


package_name = "dolbotz"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=[
        "setuptools",
        "numpy",
    ],
    zip_safe=True,
    maintainer="j",
    maintainer_email="j@example.com",
    description="ROS2 nodes for terrain side-slope detection and related robot utilities.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "slope_decision = dolbotz.slope_decision:main",
        ],
    },
)

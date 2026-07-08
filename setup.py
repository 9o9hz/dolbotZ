import os
from glob import glob

from setuptools import find_packages, setup


package_name = "dolbotz"

setup(
    name=package_name,
    version="0.0.1",
    package_dir={"": "src"},
    packages=find_packages(where="src", exclude=["test"]),
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
            "flat_drive = dolbotz.flat_drive:main",
            "arm_pickup = dolbotz.arm_pickup:main",
            "gradient_map = dolbotz.gradient_map:main",
            "elevation_map = dolbotz.elevation_map:main",
            "arm_visualizer = dolbotz.arm_visualizer:main",
        ],
    },
)

import os

from setuptools import find_packages, setup


package_name = "dolbotz"


def _config_data_files():
    """config/ 전체를 재귀적으로 share/dolbotz/config/ 아래에 설치되도록
    (하위 디렉토리별 (dest, [files]) 튜플) 나열한다. .pt/.bin/.xml 등 모델
    가중치 파일도 확장자 구분 없이 전부 포함된다 — dolbotz.utils.paths가
    colcon install 환경에서 이 경로를 찾는다."""
    entries = []
    config_root = "config"
    for dirpath, _dirnames, filenames in os.walk(config_root):
        if not filenames:
            continue
        rel = os.path.relpath(dirpath, config_root)
        dest = os.path.join("share", package_name, "config") if rel == "." \
            else os.path.join("share", package_name, "config", rel)
        entries.append((dest, [os.path.join(dirpath, f) for f in filenames]))
    return entries


setup(
    name=package_name,
    version="0.0.1",
    package_dir={"": "src"},
    packages=find_packages(where="src", exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        *_config_data_files(),
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

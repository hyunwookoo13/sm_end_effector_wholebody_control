import os
from glob import glob
from setuptools import find_packages, setup

package_name = "sm_florence_2_vlm_ros2"

setup(
    name=package_name,
    version="0.1.12",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jdlee",
    maintainer_email="artofgene@kiro.re.kr",
    description="Florence-2 VLM 기반 RealSense RGB-D 객체 검출 및 3D 위치 추정 노드",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "sm_florence_2_vlm_node = sm_florence_2_vlm.florence_2_vlm_node:main",
        ],
    },
)

import os
from glob import glob
from setuptools import find_packages, setup

package_name = "sm_grasping_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kiro",
    maintainer_email="devnull@kiro.re.kr",
    description="GPD 기반 6DoF 그래스핑 추론 및 RViz 그리퍼 시각화 패키지",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "grasping_inference_node = sm_grasping_ros2.grasping_inference_node:main",
            "gripper_marker_node = sm_grasping_ros2.gripper_marker_node:main",
        ],
    },
)

from glob import glob

from setuptools import setup


package_name = "sm_navigation_nav2"


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kiro",
    maintainer_email="gunmin0525@gmail.com",
    description="Nav2 server integration for the pick-and-place system.",
    license="MIT",
    tests_require=["pytest"],
)

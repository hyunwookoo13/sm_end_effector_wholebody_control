from glob import glob

from setuptools import setup


package_name = "sm_bringup"


setup(
    name=package_name,
    version="0.0.0",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README_PICK_PLACE.md"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kiro",
    maintainer_email="gunmin0525@gmail.com",
    description="Full-system launch composition for SM mobile manipulation.",
    license="MIT",
    tests_require=["pytest"],
)

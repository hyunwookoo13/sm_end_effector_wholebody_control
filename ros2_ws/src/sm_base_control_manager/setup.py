from setuptools import setup


package_name = "sm_base_control_manager"


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kiro",
    maintainer_email="gunmin0525@gmail.com",
    description="Base control command mux for the pick-and-place system.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "navigation_cmd_mux = sm_base_control_manager.navigation_cmd_mux:main",
        ],
    },
)

from setuptools import setup


package_name = "sm_task_orchestrator"


setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kiro",
    maintainer_email="gunmin0525@gmail.com",
    description="Pick-and-place task orchestration and navigation handoff.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pick_place_task_manager = sm_task_orchestrator.pick_place_task_manager:main",
        ],
    },
)

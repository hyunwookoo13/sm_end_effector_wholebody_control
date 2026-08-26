from glob import glob
from setuptools import find_packages, setup


package_name = "sm_semantic_mvp"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kiro",
    maintainer_email="kiro@example.com",
    description=(
        "Semantic DB mission orchestration around the existing pick/place pipeline"
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "semantic_mission_orchestrator = "
            "sm_semantic_mvp.semantic_mission_orchestrator:main",
            "demo_task_sequencer = "
            "sm_semantic_mvp.demo_task_sequencer:main",
        ],
    },
)

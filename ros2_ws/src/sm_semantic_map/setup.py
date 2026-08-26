from setuptools import find_packages, setup


package_name = "sm_semantic_map"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", ["config/warehouse_objects.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kiro",
    maintainer_email="gunmin0525@gmail.com",
    description="Semantic object-map storage and calibration tools.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "collect_object_positions = sm_semantic_map.position_collector:main",
            "semantic_map_db = sm_semantic_map.semantic_db:main",
            "semantic_map_marker_publisher = "
            "sm_semantic_map.semantic_marker_publisher:main",
            "semantic_map_server = sm_semantic_map.semantic_map_server:main",
        ],
    },
)

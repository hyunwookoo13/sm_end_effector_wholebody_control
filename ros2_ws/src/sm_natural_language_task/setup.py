from setuptools import find_packages, setup

package_name = "sm_natural_language_task"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kiro",
    maintainer_email="gunmin0525@gmail.com",
    description="Natural-language to structured pick-and-place task adapter.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "natural_language_task_parser = sm_natural_language_task.natural_language_task_parser:main",
            "natural_language_task_console = sm_natural_language_task.natural_language_task_console:main",
        ],
    },
)

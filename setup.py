from glob import glob
import os

from setuptools import find_packages, setup


package_name = "dex3_pressure_tools"


def expand(patterns):
    files = []
    for pattern in patterns:
        files.extend(glob(pattern, recursive=True))
    return files


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "LICENSE", "README.md", "dependencies.repos"]),
        (f"share/{package_name}/launch", expand(["launch/*.launch.py"])),
        (f"share/{package_name}/config", expand(["config/*.yaml", "config/*.rviz"])),
        (f"share/{package_name}/description_files/urdf", expand(["description_files/urdf/*.urdf"])),
        (
            f"share/{package_name}/description_files/meshes",
            expand(["description_files/meshes/*.STL"]),
        ),
        (f"share/{package_name}/docs", expand(["docs/*.md"])),
        (f"share/{package_name}/docs/assets", expand(["docs/assets/*"])),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Lab Maintainers",
    maintainer_email="kanth042@example.com",
    description="Read-only ROS 2 tools for Unitree Dex3-1 tactile pressure.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dex3_pressure_visualizer = dex3_pressure_tools.pressure_visualizer:main",
            "dex3_pressure_calibrator = dex3_pressure_tools.pressure_calibrator:main",
            "dex3_pressure_raw_audit = dex3_pressure_tools.raw_audit:main",
            "dex3_pressure_session_recorder = dex3_pressure_tools.session_recorder:main",
        ],
    },
)

from glob import glob

from setuptools import setup

package_name = "landing_controller"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="UAV Landing Research Team",
    maintainer_email="research@example.com",
    description="PID baseline controller and NMPC-CBF-ready command interface.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "landing_controller_node = landing_controller.controller_node:main",
        ],
    },
)

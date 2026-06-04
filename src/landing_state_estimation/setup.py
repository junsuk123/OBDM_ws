from glob import glob

from setuptools import setup

package_name = "landing_state_estimation"

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
    description="Landing state aggregation node.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "state_estimator_node = landing_state_estimation.state_estimator_node:main",
        ],
    },
)

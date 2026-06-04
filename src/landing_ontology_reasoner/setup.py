from glob import glob

from setuptools import setup

package_name = "landing_ontology_reasoner"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/rules", glob("rules/*.yaml")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="UAV Landing Research Team",
    maintainer_email="research@example.com",
    description="Ontology-inspired rule reasoner for UAV landing.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "ontology_reasoner_node = landing_ontology_reasoner.ontology_reasoner_node:main",
        ],
    },
)

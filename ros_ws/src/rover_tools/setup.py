from setuptools import setup
from glob import glob
import os

package_name = "rover_tools"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    package_data={package_name: ["run_metadata.schema.json"]},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        ("share/" + package_name, ["README.md"]),
    ],
    install_requires=["setuptools", "jsonschema>=4.18,<5", "rfc3339-validator>=0.1.4,<1", "PyYAML>=6,<7"],
    zip_safe=True,
    maintainer="Elion Dynamics Engineering",
    maintainer_email="engineering@elion.com",
    description="Operations tooling: bag recording/export, metrics evaluation, dataset packaging.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "rover_tools = rover_tools.cli:main",
            "rover_metrics = rover_tools.metrics:main",
        ],
    },
)

from glob import glob
from setuptools import find_packages, setup

package_name = "ur_cbf_bringup"

setup(
    name=package_name,
    version="0.1.7",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Cayo Sousa",
    maintainer_email="cayoalmeida0@users.noreply.github.com",
    description="Bringup parametrizado para simulacao e hardware real Universal Robots.",
    license="Apache-2.0",
)

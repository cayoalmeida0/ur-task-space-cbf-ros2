from glob import glob
from setuptools import find_packages, setup

package_name = "ur_cbf_bringup"

setup(
    name=package_name,
    version="0.3.13",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/urdf", glob("urdf/*.xacro")),
    ],
    install_requires=["setuptools"],
    extras_require={
        "test": ["pytest"],
    },
    zip_safe=True,
    maintainer="Cayo Sousa",
    maintainer_email="cayoalmeida0@users.noreply.github.com",
    description="Bringup parametrizado para simulacao e hardware real Universal Robots.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "onrobot_width_adapter = "
            "ur_cbf_bringup.onrobot_width_adapter:main",
            "onrobot_real_adapter = "
            "ur_cbf_bringup.onrobot_real_adapter:main",
        ],
    },
)

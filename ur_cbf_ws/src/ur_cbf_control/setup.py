from glob import glob

from setuptools import find_packages, setup

package_name = "ur_cbf_control"

setup(
    name=package_name,
    version="0.6.5",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["osqp==1.1.3", "setuptools"],
    extras_require={
        "test": ["pytest"],
    },
    zip_safe=True,
    maintainer="Cayo Sousa",
    maintainer_email="cayoalmeida0@users.noreply.github.com",
    description="Controladores seguros no espaco de tarefa para Universal Robots.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "cartesian_position_test = "
            "ur_cbf_control.cartesian_position_test:main",
            "joint_velocity_pulse_test = "
            "ur_cbf_control.joint_velocity_pulse_test:main",
        ],
    },
)

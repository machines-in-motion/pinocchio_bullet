#!/usr/bin/env python

from os import path
from setuptools import setup, find_packages

old_package_name = "py_pinocchio_bullet"
package_name = "pinocchio_bullet"

with open("README.md", "r") as fh:
    long_description = fh.read()

setup(
    name=package_name,
    version="1.0.0",
    package_dir={
        package_name: path.join("src", package_name),
        old_package_name: path.join("src", old_package_name),
    },
    packages=[package_name, old_package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [path.join("resource", package_name)],
        ),
        (path.join("share", package_name), ["package.xml"]),
        (
            path.join("share", package_name, "urdf"),
            [
                path.join("resource", "ground.mtl"),
                path.join("resource", "ground.obj"),
                path.join("resource", "ground.urdf"),
                path.join("resource", "solo_12_base.stl"),
                path.join("resource", "solo12_hip_fe_fl.stl"),
                path.join("resource", "solo12_hip_fe_fr.stl"),
                path.join("resource", "solo12_hip_fe_hl.stl"),
                path.join("resource", "solo12_hip_fe_hr.stl"),
                path.join("resource", "solo12_lower_leg_left_side.stl"),
                path.join("resource", "solo12_lower_leg_right_side.stl"),
                path.join("resource", "solo12_upper_leg_left_side.stl"),
                path.join("resource", "solo12_upper_leg_right_side.stl"),
                path.join("resource", "solo12.urdf"),
                path.join("resource", "solo_foot.stl"),
                path.join("resource", "solo_lower_leg_left_side.stl"),
                path.join("resource", "solo_lower_leg_right_side.stl"),
                path.join("resource", "solo_upper_leg_left_side.stl"),
                path.join("resource", "solo_upper_leg_right_side.stl"),
            ],
        ),
    ],
    scripts=[path.join("demos", "demo_simulate_a_robot.py")],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mnaveau",
    maintainer_email="mnaveau@tuebingen.mpg.de",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/pypa/sampleproject",
    description="Wrapper around the pybullet interface using pinocchio.",
    license="BSD-3-clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: BSD-3-clause",
        "Operating System :: OS Independent",
    ],
    python_requires=">=3.6",
)

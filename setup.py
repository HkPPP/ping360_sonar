#!/usr/bin/env python3

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
     packages=['ping360_sonar'],
     package_dir={'': 'src'},
     install_requires=['bluerobotics-ping']
)

setup(**setup_args)

#!/usr/bin/env python3

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

# Setup configuration for catkin
setup_args = generate_distutils_setup(
    packages=['mpc_longitudinal_controller'],
    package_dir={'': 'src'}
)

setup(**setup_args)
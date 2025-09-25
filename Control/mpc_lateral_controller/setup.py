#!/usr/bin/env python3
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# Fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['mpc_lateral_controller'],
    package_dir={'': 'src'},
    requires=['rospy', 'numpy', 'cvxpy', 'pandas']
)

setup(**setup_args)
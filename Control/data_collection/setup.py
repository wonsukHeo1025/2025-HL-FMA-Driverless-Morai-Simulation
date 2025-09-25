#!/usr/bin/env python
from setuptools import setup, find_packages
from catkin_pkg.python_setup import generate_distutils_setup

# Setup for ROS1 Python package
d = generate_distutils_setup(
    packages=find_packages('src'),
    package_dir={'': 'src'},
    scripts=['scripts/data_collection_node.py'],
    install_requires=[]  # ROS dependencies handled by package.xml
)

setup(**d)
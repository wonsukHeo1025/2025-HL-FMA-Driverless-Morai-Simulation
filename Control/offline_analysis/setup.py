#!/usr/bin/env python
from setuptools import setup, find_packages

setup(
    name='offline_analysis',
    version='0.1.0',
    description='Offline CSV parsing and parameter estimation for vehicle data',
    packages=find_packages(),
    install_requires=[
        'numpy>=1.20',
        'pandas>=1.3',
        'matplotlib>=3.5'
    ],
    entry_points={
        'console_scripts': [
            'offline-analysis=offline_analysis.cli:main',
            'offline-analysis-montecarlo=offline_analysis.multi_run:main'
        ]
    },
    python_requires='>=3.8',
)
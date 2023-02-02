#!/usr/bin/env python3
from setuptools import setup, find_packages
import os

with open("README.md", "r") as fh:
    long_description = fh.read()

setup(
    name="k4r_data_provider",
    description="Map Description and format conversion for robotics applications.",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/knowledge4retail/k4r-data-provider",
    version="0.1",
    license="BSD-3",
    author="Andreas Bresser",
    packages=find_packages(),
    tests_require=[],
    include_package_data=True,
    install_requires=[
        'argcomplete',
        'requests_pkcs12'
    ],
    entry_points={
        'console_scripts': [
            'k4r_data_provider = k4r_data_provider.cli:main',
        ],
    },
)

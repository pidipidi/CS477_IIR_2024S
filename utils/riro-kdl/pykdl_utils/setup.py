#!/usr/bin/env python3
from setuptools import setup

package_name = 'pykdl_utils'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Daehyung Park',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)





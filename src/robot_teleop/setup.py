#!/usr/bin/env python3

from setuptools import find_packages, setup

package_name = 'robot_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='IB-Robot Team',
    maintainer_email='maintainer@example.com',
    description='Minimal serial-to-controller bridge for zero-latency teleoperation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_node = robot_teleop.teleop_node:main',
        ],
    },
)

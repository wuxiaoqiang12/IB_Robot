import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'so101_hw_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'resource'), glob(os.path.join('resource', '*.xml'))),
    ],
    install_requires=[
        'setuptools',
        'feetech-servo-sdk',
    ],
    zip_safe=True,
    maintainer='xqw',
    maintainer_email='wuxiaoqiang.rtos@huawei.com',

    description='SO101 hardware interface package',
    license='TODO: License declaration',
    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            'so101_motor_bridge = so101_hw_interface.motor_bridge:main',
            'so101_read_steps = so101_hw_interface.read_motor_steps:main',
            'so101_leader_pub = so101_hw_interface.leader_arm_pub:main',
            'so101_calibrate_arm = so101_hw_interface.so101_calibrate_arm:main',
            'so101_calibration_service = so101_hw_interface.calibration_service:main',
        ],
    },
)
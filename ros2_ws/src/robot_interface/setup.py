from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config', glob('config/**/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ch3cooh',
    maintainer_email='haoyida542@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "robot_interface = robot_interface.main:main",
        ],
    },
)

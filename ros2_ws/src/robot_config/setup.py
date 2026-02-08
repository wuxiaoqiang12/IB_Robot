from setuptools import setup
from setuptools import find_packages
from glob import glob

package_name = 'robot_config'

# Find all YAML config files
config_files = glob('config/robots/*.yaml')
world_files = glob('config/worlds/*.world')

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config/robots', config_files),
        ('share/' + package_name + '/config/worlds', world_files),
        ('share/' + package_name + '/launch', [
            'launch/robot.launch.py',
        ]),
    ],
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Unified robot configuration system for ros2_control and peripherals',
    license='Apache-2.0',
    tests_require=['pytest'],
)

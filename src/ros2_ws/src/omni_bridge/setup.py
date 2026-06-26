import os
from glob import glob
from setuptools import setup

package_name = 'omni_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'python-can>=4.6'],
    zip_safe=True,
    maintainer='Artlashmanov',
    maintainer_email='Artlashmanov@users.noreply.github.com',
    description='ROS2 CAN bridge for STM32 omni robot controller',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'can_bridge = omni_bridge.can_bridge_node:main',
            'wheel_odometry = omni_bridge.wheel_odometry_node:main',
        ],
    },
)

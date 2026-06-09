from setuptools import setup
import os
from glob import glob

package_name = 'uav_flight_controller_py'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bridge',
    maintainer_email='bridge@example.com',
    description='UAV flight controller for ROS2 + PX4 simulation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'flight_control_node = uav_flight_controller_py.flight_control_node:main',
        ],
    },
)

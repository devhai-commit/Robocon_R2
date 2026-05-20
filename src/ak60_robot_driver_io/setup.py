from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ak60_robot_driver_io'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        #launch file
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
        (os.path.join('share', package_name, 'vendor'), glob('vendor/*')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robocon',
    maintainer_email='robocon@todo.todo',
    description='Driver IO package for AK60 robot using CAN interface', 
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'can_node = ak60_robot_driver_io.can_bridge:main',
        'serial_node = ak60_robot_driver_io.steer_node:main',
        'imu_node = ak60_robot_driver_io.read_imu_node:main',
        'laser_node = ak60_robot_driver_io.laser_distance:main',
        ],
    },
)

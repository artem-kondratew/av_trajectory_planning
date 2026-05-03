import os

from glob import glob
from setuptools import find_packages, setup

package_name = 'carla_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='artem-kondratew',
    maintainer_email='artemkondratev5@gmail.com',
    description='Package with carla client functions',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "world_node = carla_ros.world_node:main",
            "vehicle_node = carla_ros.vehicle_node:main",
            "recorder_node = carla_ros.recorder_node:main",
        ],
    },
)

import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'sensor_fusion_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gowtham',
    maintainer_email='gowthame8me177@gmail.com',
    description='ROS2 Package estimates velocity using imu and depth data',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'fuse_data = sensor_fusion_pkg.fuse_data:main'
        ],
    },
)

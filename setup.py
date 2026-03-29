from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'ugv01_room_explore'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='teto',
    maintainer_email='teto@example.com',
    description='UGV01 room exploration with odom, SLAM and autonomous lidar exploration',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ugv_odom = ugv01_room_explore.ugv_odom:main',
            'auto_explore = ugv01_room_explore.auto_explore:main',
        ],
    },
)

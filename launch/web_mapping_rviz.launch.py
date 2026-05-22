from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_share = Path(get_package_share_directory('ugv01_room_explore'))
    base_launch = pkg_share / 'launch' / 'web_mapping.launch.py'

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(base_launch)),
            launch_arguments={
                'serial_port': '/dev/ttyUSB0',
                'lidar_model': 'LD19',
                'use_rviz': 'true',
            }.items()
        )
    ])

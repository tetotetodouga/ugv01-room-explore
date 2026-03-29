from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = Path(get_package_share_directory('ugv01_room_explore'))
    mapping_launch = pkg_share / 'launch' / 'mapping.launch.py'

    use_rviz = LaunchConfiguration('use_rviz')
    serial_port = LaunchConfiguration('serial_port')
    lidar_model = LaunchConfiguration('lidar_model')

    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(mapping_launch)),
        launch_arguments={
            'use_rviz': use_rviz,
            'serial_port': serial_port,
            'lidar_model': lidar_model,
        }.items()
    )

    auto_node = TimerAction(
        period=8.5,
        actions=[
            Node(
                package='ugv01_room_explore',
                executable='auto_explore',
                name='auto_explore',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('lidar_model', default_value='LD19'),

        mapping,
        auto_node,
    ])

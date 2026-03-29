from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = Path(get_package_share_directory('ugv01_room_explore'))
    urdf_path = pkg_share / 'urdf' / 'ugv01_box.urdf'
    slam_yaml = pkg_share / 'config' / 'slam_async.yaml'

    robot_description = urdf_path.read_text(encoding='utf-8')

    ldlidar_launch = Path(get_package_share_directory('ldlidar_node')) / 'launch' / 'ldlidar_bringup.launch.py'
    slam_launch = Path(get_package_share_directory('slam_toolbox')) / 'launch' / 'online_async_launch.py'

    use_rviz = LaunchConfiguration('use_rviz')
    serial_port = LaunchConfiguration('serial_port')
    lidar_model = LaunchConfiguration('lidar_model')

    odom_node = Node(
        package='ugv01_room_explore',
        executable='ugv_odom',
        name='ugv_odom',
        output='screen'
    )

    lidar_launch_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(ldlidar_launch)),
        launch_arguments={
            'serial_port': serial_port,
            'lidar_model': lidar_model,
        }.items()
    )

    lidar_configure = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/ldlidar_node', 'configure'],
                output='screen'
            )
        ]
    )

    lidar_activate = TimerAction(
        period=4.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/ldlidar_node', 'activate'],
                output='screen'
            )
        ]
    )

    robot_state_pub = TimerAction(
        period=4.5,
        actions=[
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'robot_description': robot_description}]
            )
        ]
    )

    slam_node = TimerAction(
        period=5.5,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(slam_launch)),
                launch_arguments={
                    'slam_params_file': str(slam_yaml),
                    'use_sim_time': 'false',
                }.items()
            )
        ]
    )

    rviz_node = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                condition=IfCondition(use_rviz)
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('lidar_model', default_value='LD19'),

        odom_node,
        lidar_launch_desc,
        lidar_configure,
        lidar_activate,
        robot_state_pub,
        slam_node,
        rviz_node,
    ])

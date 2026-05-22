from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction, LogInfo
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
        output='screen',
    )

    lidar_launch_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(ldlidar_launch)),
        launch_arguments={
            'serial_port': serial_port,
            'lidar_model': lidar_model,
        }.items()
    )

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )


    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[
            {'port': 9090},
            {'address': '0.0.0.0'}
        ]
    )

    lidar_lifecycle_helper = TimerAction(
        period=8.0,
        actions=[
            LogInfo(msg='[UGV01] Waiting for LD19 lifecycle node, then configure/activate...'),
            ExecuteProcess(
                cmd=[
                    'bash', '-lc',
                    '''
                    source /opt/ros/jazzy/setup.bash
                    source ~/ros2_ws/install/setup.bash

                    echo "[UGV01] waiting for /ldlidar_node discovery..."
                    STATE=""

                    for i in $(seq 1 80); do
                      STATE=$(ros2 lifecycle get /ldlidar_node 2>/dev/null || true)
                      if [ -n "$STATE" ]; then
                        echo "[UGV01] ldlidar state: $STATE"
                        break
                      fi
                      sleep 1
                    done

                    if echo "$STATE" | grep -q "unconfigured"; then
                      echo "[UGV01] configuring /ldlidar_node..."
                      ros2 lifecycle set /ldlidar_node configure || true
                      sleep 6
                    fi

                    STATE=$(ros2 lifecycle get /ldlidar_node 2>/dev/null || true)
                    echo "[UGV01] ldlidar state after configure: $STATE"

                    if echo "$STATE" | grep -q "inactive"; then
                      echo "[UGV01] activating /ldlidar_node..."
                      ros2 lifecycle set /ldlidar_node activate || true
                      sleep 6
                    fi

                    echo "[UGV01] final ldlidar state:"
                    ros2 lifecycle get /ldlidar_node || true
                    '''
                ],
                output='screen'
            )
        ]
    )

    slam_node = TimerAction(
        period=75.0,
        actions=[
            LogInfo(msg='[UGV01] Starting SLAM Toolbox after LiDAR lifecycle warmup...'),
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
        period=90.0,
        actions=[
            LogInfo(msg='[UGV01] Starting RViz after SLAM warmup...'),
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
        robot_state_pub,
        rosbridge_node,

        lidar_lifecycle_helper,
        slam_node,
        rviz_node,
    ])

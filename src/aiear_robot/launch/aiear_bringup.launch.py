"""Top-level deployment bringup: hardware + LiDAR + localization + Nav2.

Replaces the manual sequence:
  ros2 launch aiear_robot aiear_hardware.launch.py      (was aiear_bringup.launch.py)
  ros2 run rplidar_ros rplidar_composition --ros-args \
      -p serial_port:=/dev/rplidar -p serial_baudrate:=115200 \
      -p frame_id:=laser -p angle_compensate:=true
  ros2 launch aiear_robot aiear_nav2.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    aiear_share = get_package_share_directory('aiear_robot')
    launch_dir  = os.path.join(aiear_share, 'launch')

    default_map = os.path.join(aiear_share, 'maps', 'aiear_map_last_floor_hall.yaml')
    map_arg = DeclareLaunchArgument('map', default_value=default_map)

    # ── Hardware: robot_state_publisher + ros2_control + controllers ─────────
    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'aiear_hardware.launch.py')),
    )

    # ── RPLiDAR A1 ────────────────────────────────────────────────────────────
    rplidar = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port':      '/dev/rplidar',
            'serial_baudrate':  115200,
            'frame_id':         'laser',
            'angle_compensate': True,
        }],
        # Survives the stale-fd failure mode after USB re-enumeration
        respawn=True,
        respawn_delay=3.0,
    )

    # ── Localization + Nav2 (map_server, AMCL, planners, controllers) ────────
    # Delayed so controllers are spawned and odom→base_link TF is publishing
    # before AMCL and the costmaps come up. Nav2's own launch adds a further
    # 5 s delay between localization and the navigation servers.
    nav2 = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'aiear_nav2.launch.py')),
                launch_arguments={
                    'map': LaunchConfiguration('map'),
                }.items(),
            ),
        ],
    )

    return LaunchDescription([
        map_arg,
        hardware,
        rplidar,
        nav2,
    ])

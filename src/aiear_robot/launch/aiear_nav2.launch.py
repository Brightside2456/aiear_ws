import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory as pkg


def generate_launch_description():
    aiear_share = get_package_share_directory('aiear_robot')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    params_file = os.path.join(aiear_share, 'config', 'aiear_nav2.yaml')
    default_map = os.path.join(
        aiear_share, 'maps', 'aiear_map_last_floor_hall.yaml')

    map_arg = DeclareLaunchArgument('map', default_value=default_map)
    map_yaml = LaunchConfiguration('map')

    # Reuse Nav2's official bringup, fed our params + map.
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': 'false',
            'params_file': params_file,
            'autostart': 'true',
        }.items(),
    )

    return LaunchDescription([map_arg, nav2])

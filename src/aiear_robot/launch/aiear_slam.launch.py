import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    aiear_share = get_package_share_directory('aiear_robot')

    default_params = os.path.join(aiear_share, 'config', 'aiear_slam.yaml')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use Gazebo /clock instead of wall time')
    params_file_arg = DeclareLaunchArgument(
        'slam_params_file', default_value=default_params,
        description='Full path to the slam_toolbox parameter file')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file  = LaunchConfiguration('slam_params_file')

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time},  # overrides the value in the yaml
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        params_file_arg,
        slam_toolbox,
    ])

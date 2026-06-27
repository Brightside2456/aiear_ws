import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    aiear_share = get_package_share_directory('aiear_robot')
    nav2_share  = get_package_share_directory('nav2_bringup')

    params_file = os.path.join(aiear_share, 'config', 'aiear_nav2.yaml')
    default_map = os.path.join(aiear_share, 'maps', 'aiear_map_last_floor_hall.yaml')

    map_arg  = DeclareLaunchArgument('map', default_value=default_map)
    map_yaml = LaunchConfiguration('map')

    # ── Localization: map_server + amcl (autostart=true) ──────────────────────
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'localization_launch.py')),
        launch_arguments={
            'map':          map_yaml,
            'use_sim_time': 'false',
            'params_file':  params_file,
            'autostart':    'true',
        }.items(),
    )

    # ── Navigation nodes (individual, skips docking_server / collision_monitor)
    #
    # Velocity chain:
    #   controller_server ──┐
    #   behavior_server  ───┴─► /cmd_vel_nav  (TwistStamped)
    #                              │
    #                       velocity_smoother  → /diff_drive_controller/cmd_vel  (TwistStamped)
    #
    # twist_mux is bypassed: its Twist vs TwistStamped dual-publisher design
    # means the cmd_vel_out remapping only catches the Twist publisher, not the
    # TwistStamped one.  Re-add twist_mux once a twist_stamper conversion node
    # is in place (sudo apt install ros-jazzy-twist-stamper).

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[params_file],
        remappings=[('cmd_vel', 'cmd_vel_nav')],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        output='screen',
        parameters=[params_file],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        output='screen',
        parameters=[params_file],
        remappings=[('cmd_vel', 'cmd_vel_nav')],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        output='screen',
        parameters=[params_file],
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('cmd_vel',          'cmd_vel_nav'),                    # input from controller / behaviors
            ('cmd_vel_smoothed', '/diff_drive_controller/cmd_vel'), # output direct to controller
        ],
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        output='screen',
        parameters=[params_file],
    )

    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart':    True,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'velocity_smoother',
                'waypoint_follower',
            ],
        }],
    )

    # Delay navigation nodes so map_server + AMCL have time to fully activate
    # before controller_server tries to load the static costmap layer.
    # On a Pi, localization can take 3-5 s to configure + activate.
    navigation_nodes = TimerAction(
        period=5.0,
        actions=[
            controller_server,
            planner_server,
            behavior_server,
            bt_navigator,
            velocity_smoother,
            waypoint_follower,
            lifecycle_manager_navigation,
        ],
    )

    return LaunchDescription([
        map_arg,
        localization,
        navigation_nodes,
    ])

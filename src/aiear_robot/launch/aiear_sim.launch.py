"""
aiear_sim.launch.py  (fixed v3 – sensor topics)
────────────────────────────────────────────────────────────────────────────
Key fix in this version:
  The ros_gz_bridge now maps the real Gazebo Harmonic namespaced sensor
  topics to clean ROS topic names.

  In gz-sim 8, sensors always publish under:
    /world/<world>/model/<model>/link/<link>/sensor/<sensor>/<type>

  The bridge arguments below use the full Gz path on the left side of '@'
  and expose clean ROS names via remapping in the node's 'remappings' list.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_aiear  = get_package_share_directory('aiear_robot')
    pkg_ros_gz = get_package_share_directory('ros_gz_sim')

    # ── Launch arguments ──────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock',
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz2',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_rviz  = LaunchConfiguration('rviz')

    # ── 1. robot_state_publisher ──────────────────────────────────────
    xacro_file = os.path.join(pkg_aiear, 'urdf', 'aiear.urdf.xacro')

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', xacro_file]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    # ── 2. Gazebo Harmonic ────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-r empty.sdf',
            'gz_version': '8',
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # ── 3. Spawn robot ────────────────────────────────────────────────
    spawn_robot = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_aiear',
                output='screen',
                arguments=[
                    '-name',  'aiear',
                    '-topic', 'robot_description',
                    '-x', '0.0', '-y', '0.0', '-z', '0.1',
                    '-R', '0.0', '-P', '0.0', '-Y', '0.0',
                ],
            )
        ],
    )

    # ── 4. ros_gz_bridge ─────────────────────────────────────────────
    #
    # gz-sim 8 publishes sensors under a namespaced path:
    #   /world/<world>/model/<model>/link/<link>/sensor/<sensor>/<data>
    #
    # We bridge those long Gz paths and remap them to clean ROS names
    # using the node's 'remappings' parameter.
    #
    # HOW TO VERIFY THE EXACT PATH ON YOUR MACHINE:
    #   After spawning, run:  gz topic -l
    #   Look for lines containing 'scan' and 'image'.
    #   If your world name differs from 'empty', update the paths below.
    #
    GZ_MODEL = '/world/empty/model/aiear'
    GZ_LIDAR  = f'{GZ_MODEL}/link/lidar_link/sensor/rplidar_a1'
    GZ_CAM    = f'{GZ_MODEL}/link/camera_link/sensor/imx500_camera'

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            # Velocity command: ROS → Gz
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            # Odometry: Gz → ROS
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # Clock: Gz → ROS
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # TF: Gz → ROS
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # Joint states: Gz → ROS
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            # LiDAR — full Gz namespaced topic → ROS
            f'{GZ_LIDAR}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # Camera image — full Gz namespaced topic → ROS
            f'{GZ_CAM}/image@sensor_msgs/msg/Image[gz.msgs.Image',
            # Camera info — full Gz namespaced topic → ROS
            f'{GZ_CAM}/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        # Remap the long Gz paths to the clean ROS names your nodes expect
        remappings=[
            (f'{GZ_LIDAR}/scan',        '/scan'),
            (f'{GZ_CAM}/image',         '/camera/image_raw'),
            (f'{GZ_CAM}/camera_info',   '/camera/camera_info'),
        ],
    )

    # ── 5. RViz2 ─────────────────────────────────────────────────────
    rviz_config = os.path.join(pkg_aiear, 'config', 'aiear.rviz')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(launch_rviz),
    )

    return LaunchDescription([
        use_sim_time_arg,
        rviz_arg,
        robot_state_publisher,
        gazebo,
        spawn_robot,
        bridge,
        rviz,
    ])
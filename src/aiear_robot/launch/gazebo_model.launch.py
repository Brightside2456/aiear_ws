import os
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, FindExecutable
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # Specify the name of the package
    namePackage = "aiear_robot"
    pkg_aiear = get_package_share_directory(namePackage)
    
    # ── Launch Arguments ──────────────────────────────────────────────
    wheel_count_arg = DeclareLaunchArgument(
        'wheel_count', default_value='2',
        description='Number of wheels: 2 or 4'
    )
    use_ros2_control_arg = DeclareLaunchArgument(
        'use_ros2_control', default_value='false',
        description='Use ros2_control instead of simple gazebo plugins'
    )
    is_sim_arg = DeclareLaunchArgument(
        'is_sim', default_value='true',
        description='Whether we are in simulation or real hardware'
    )

    wheel_count = LaunchConfiguration('wheel_count')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    is_sim = LaunchConfiguration('is_sim')

    # ── Robot Description (Xacro) ─────────────────────────────────────
    xacro_file = os.path.join(pkg_aiear, 'model', 'robot.xacro')

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ', xacro_file, ' ',
            'wheel_count:=', wheel_count, ' ',
            'use_ros2_control:=', use_ros2_control, ' ',
            'is_sim:=', is_sim
        ]),
        value_type=str
    )

    # ── Nodes ─────────────────────────────────────────────────────────
    
    # 1. Robot State Publisher
    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': is_sim
        }]
    )

    # 2. Gazebo (only if is_sim is true)
    gazebo_pkg = get_package_share_directory('ros_gz_sim')
    gazeboLaunch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-r -v 4 empty.sdf',
            'on_exit_shutdown': 'true'
        }.items()
    )

    # 3. Spawn Robot in Gazebo
    spawnModelNodeGazebo = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'aiear_robot',
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    # 4. ROS-GZ Bridge
    bridge_params = os.path.join(pkg_aiear, 'parameters', 'bridge_parameters.yaml')
    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f"config_file:={bridge_params}"],
        output='screen'
    )

    # ── Launch Description Assembly ───────────────────────────────────
    ld = LaunchDescription()
    
    # Arguments
    ld.add_action(wheel_count_arg)
    ld.add_action(use_ros2_control_arg)
    ld.add_action(is_sim_arg)

    # Core Nodes
    ld.add_action(nodeRobotStatePublisher)
    ld.add_action(gazeboLaunch)
    ld.add_action(spawnModelNodeGazebo)
    ld.add_action(start_gazebo_ros_bridge_cmd)

    return ld

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # 1. Define the package name and the path to your .xacro file
    package_name = 'aiear_robot' # Change this if your package name is different!
    xacro_file = os.path.join(get_package_share_directory(package_name), 'urdf', 'aiear.urdf.xacro')

    # 2. Process the Xacro file into a standard XML string
    doc = xacro.process_file(xacro_file)
    robot_description_xml = doc.toxml()

    # 3. The Robot State Publisher Node
    # This takes your XML and publishes it to the ROS network so RViz knows what the robot looks like
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_xml, 'use_sim_time': True}]
    )

    # 4. Start Gazebo Harmonic
    # This boots up the physics engine with an empty world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
        )]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # 5. The Spawner Node
    # This tells Gazebo to look at the robot_description topic and physically inject the robot into the world
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'aiear',
                   '-z', '0.1'], # Spawns the robot 10cm slightly hovering so it drops safely
        output='screen'
    )

    # 6. The Bridge (Automating what you did in the terminal earlier!)
    # This connects the Gazebo /scan and /camera to ROS 2
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
        ],
        output='screen'
    )

    # 7. Return the LaunchDescription
    # This list tells ROS the exact order to execute everything
    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn_entity,
        bridge
    ])
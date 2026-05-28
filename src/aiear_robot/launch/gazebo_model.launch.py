import os
import xacro
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():

    # first store the xacro name (the one you specified in the robot tag)of the urdf file... you'll need it later
    # robotXacroName = "four_wheeled_diff_drive_robot" #From tutorial
    robotXacroName = "aiear" #custom

    # Specify the name of the pakage you used
    namePackage = "aiear_robot"

    # now define the relative (to aiear_robot package) path to the xacro file defining the model
    # modelFileRelativePath = 'model/robot.xacro'
    modelFileRelativePath = 'urdf/aiear.xacro'

    # If you want to define your own empty world model uncomment this
    # however,you then have to create empty_world.world
    # worldFileRelativePath = 'model/empty_world.world'

    # this is the absolute path to the model / urdf file
    pathModelFile = os.path.join(get_package_share_directory(namePackage), modelFileRelativePath)


    # If youre using your own world model,uncomment this
    # this is the absolute path to the world model
    # pathModelFile = os.path.join(get_package_share_directory(namePackage), worldFileRelativePath)

    # get the robot description from the xacro model file (By converting it to xml)
    robotDescription = xacro.process_file(pathModelFile).toxml()

    # This is the launch file from the gazebo_ros package
    # we are getting that , well use it later with some arguments
    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(launch_file_path=os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'))

    # this is the launch decription

    # if youre using your own world model
    # gazeboLaunch=IncludeLaunchDescription(gazebo_rosPackageLaunch, launch_arguments={
    #     'gz_args': ['-r -v -v4 ', pathWorldFile],
    #     'on_exit_shutdown' : 'true'
    # }.items()
    # )

    # if youre using an empty world model 
    gazeboLaunch = IncludeLaunchDescription(gazebo_rosPackageLaunch, launch_arguments={
        'gz_args' : ['-r -v -v4 empty.sdf'],
        'on_exit_shutdown': 'true'
    }.items()
    ) 

    #R 1. obot State Publisher node
    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description' : robotDescription,
                'use_sim_time': True
            }
        ]
    )

    # 2. Gazebo Node
    spawnModelNodeGazebo = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robotXacroName,
            '-topic', "robot_description"
        ],
        output='screen'
    )

    # this block is the one that allows  ros to comminicate with gazebo
    # by allowing us to send ky board commands as velociies to the robot in gazebo to control it
    bridge_params = os.path.join(
        get_package_share_directory(namePackage),
        'parameters',
        'bridge_parameters.yaml'
    )
    # 3. Bridge
    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f"config_file:={bridge_params}",

        ],
        output='screen'
    )

    launchDescriptionObject = LaunchDescription()
    
    # we add the gazeboLaunch
    launchDescriptionObject.add_action(gazeboLaunch)

    # we add the 3 nodes
    launchDescriptionObject.add_action(spawnModelNodeGazebo)
    launchDescriptionObject.add_action(nodeRobotStatePublisher)
    launchDescriptionObject.add_action(start_gazebo_ros_bridge_cmd)



    return launchDescriptionObject
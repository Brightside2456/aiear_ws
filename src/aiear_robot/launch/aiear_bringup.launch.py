import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # NOTE: adjust 'aiear_robot' if you place these files in another package
    pkg_share = get_package_share_directory('aiear_robot')

    urdf_xacro = os.path.join(pkg_share, 'urdf', 'aiear.urdf.xacro')
    controllers_yaml = os.path.join(pkg_share, 'config', 'aiear_controllers.yaml')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_xacro]), value_type=str)

    # Publishes /robot_description (topic) and TF from /joint_states
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='both',
    )

    # Jazzy pattern: controller_manager subscribes to /robot_description
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controllers_yaml],
        remappings=[('~/robot_description', '/robot_description')],
        output='both',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'],
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller',
                   '--controller-manager', '/controller_manager'],
    )

    # Start diff_drive only after joint_state_broadcaster is up
    delay_diff_drive = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_spawner],
        )
    )

    return LaunchDescription([
        robot_state_publisher,
        control_node,
        joint_state_broadcaster_spawner,
        delay_diff_drive,
    ])

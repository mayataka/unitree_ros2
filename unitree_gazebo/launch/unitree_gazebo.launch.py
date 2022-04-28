import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), 
            '/gazebo.launch.py']), 
        launch_arguments = {'pause': 'false'}.items(),
        # launch_arguments = {'pause': 'true'}.items(),
    )    

    a1_description_path = os.path.join(
        get_package_share_directory('a1_description'))
    xacro_file = os.path.join(a1_description_path, 'xacro', 'robot.xacro')
    robot_description = {'robot_description': Command(['xacro ', xacro_file, ' use_gazebo:=true DEBUG:=false'])}

    spawn_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )

    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py', 
        arguments=['-topic', 'robot_description', '-entity', 'a1', 
                   '-x', '0', '-y', '0', '-z', '0.5'],
                #    '-x', '0', '-y', '0', '-z', '0.8', '-unpause'],
        output='screen',
    )

    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )
    spawn_imu_sensor_broadcaster = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['imu_sensor_broadcaster', '--controller-manager', '/controller_manager'],
    )
    # spawn_FL_foot_force_sensor_broadcaster = Node(
    #     package='controller_manager',
    #     executable='spawner.py',
    #     arguments=['FL_foot_force_sensor_broadcaster', '--controller-manager', '/controller_manager'],
    # )
    # spawn_FR_foot_force_sensor_broadcaster = Node(
    #     package='controller_manager',
    #     executable='spawner.py',
    #     arguments=['FR_foot_force_sensor_broadcaster', '--controller-manager', '/controller_manager'],
    # )
    # spawn_RL_foot_force_sensor_broadcaster = Node(
    #     package='controller_manager',
    #     executable='spawner.py',
    #     arguments=['RL_foot_force_sensor_broadcaster', '--controller-manager', '/controller_manager'],
    # )
    # spawn_RR_foot_force_sensor_broadcaster = Node(
    #     package='controller_manager',
    #     executable='spawner.py',
    #     arguments=['RR_foot_force_sensor_broadcaster', '--controller-manager', '/controller_manager'],
    # )
    spawn_mpc = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['unitree_mpc', '--controller-manager', '/controller_manager'],
        # prefix=['xterm -e gdb -ex run --args'], # for debugging
        
    )

    return LaunchDescription([
        gazebo,
        spawn_robot_state_publisher,
        # spawn_entity,
        spawn_joint_state_broadcaster,
        spawn_imu_sensor_broadcaster,
        # spawn_FL_foot_force_sensor_broadcaster,
        # spawn_FR_foot_force_sensor_broadcaster,
        # spawn_RL_foot_force_sensor_broadcaster,
        # spawn_RR_foot_force_sensor_broadcaster,
        spawn_entity,
        spawn_mpc,
    ])
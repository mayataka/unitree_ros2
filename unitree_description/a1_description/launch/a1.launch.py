import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command


def generate_launch_description():
    a1_description_path = os.path.join(
        get_package_share_directory('a1_description'))
    xacro_file = os.path.join(a1_description_path, 'xacro', 'robot.xacro')
    params = {'robot_description': Command(['xacro ', xacro_file, ' use_gazebo:=true DEBUG:=false'])}

    # params = {'robot_description': robot_desc}
    rsp = Node(package='robot_state_publisher',
               executable='robot_state_publisher',
               output='both',
               parameters=[params])
    jsp = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
    )

    rviz_config_file = get_package_share_directory(
        'a1_description') + '/launch/a1.rviz'
    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     output='log',
                     arguments=['-d', rviz_config_file])

    return LaunchDescription([rsp, jsp, rviz_node])
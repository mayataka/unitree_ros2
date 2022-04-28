import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    controller_config = os.path.join(
        get_package_share_directory('unitree_gazebo'),
        'config', 'unitree_gazebo_controllers.yaml'
    )
    spawn_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('unitree_control'), 'launch'), 
            '/unitree_controller.launch.py']),
        parameters=[controller_config],
    )

    return LaunchDescription([
        spawn_controller,
    ])
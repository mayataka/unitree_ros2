from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='unitree_teleop',
            executable='unitree_teleop_twist',
            output='screen',
            prefix=['gnome-terminal --'],
        ),
        Node(
            package='unitree_teleop',
            executable='unitree_teleop_set_control_mode',
            output='screen',
            prefix=['gnome-terminal --'],
        )
    ])
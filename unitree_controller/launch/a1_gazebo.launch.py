from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_config_package",
            default_value="unitree_controller",
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_config_file",
            default_value="config/a1_controllers.yaml",
            description="Relative path from the config package to the YAML file with the controllers configuration .",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="a1_description",
            description="Description package with robot URDF/xacro files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="xacro/robot.xacro",
            description="Relative path from the description package to the URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_controller",
            default_value="unitree_controller",
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_rviz",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_package",
            default_value="a1_description",
            description="Package with the Rviz2\'s configuration folder. \
        Usually the argument is not set, it enables use of a custom setup.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value="launch/a1.rviz",
            description="Relative PATH from the rviz config package to the config file of the Rviz2\'"
        )
    )

    # Initialize Arguments
    controllers_config_package = LaunchConfiguration("controllers_config_package")
    controllers_config_file = LaunchConfiguration("controllers_config_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    robot_controller = LaunchConfiguration("robot_controller")
    start_rviz = LaunchConfiguration("start_rviz")
    rviz_config_package = LaunchConfiguration("rviz_config_package")
    rviz_config_file = LaunchConfiguration("rviz_config_file")

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), "/unitree_gazebo.launch.py"]),
        launch_arguments={
            "controllers_config_package": controllers_config_package,
            "controllers_config_file": controllers_config_file,
            "description_package": description_package,
            "description_file": description_file,
            "robot_controller": robot_controller,
            "start_rviz": start_rviz,
            "rviz_config_package": rviz_config_package,
            "rviz_config_file": rviz_config_file,
        }.items(),
    )

    return LaunchDescription(declared_arguments + [base_launch])
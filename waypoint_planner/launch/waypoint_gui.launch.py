from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Launch the Qt waypoint planner GUI."""

    config_file = os.path.join(
        get_package_share_directory('waypoint_planner'),
        'config',
        'waypoint_planner.yaml',
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level', default_value='info', description='rclpy log level'
    )

    gui_node = Node(
        package='waypoint_planner',
        executable='waypoint_gui',
        name='waypoint_gui',
        output='screen',
        parameters=[
            config_file,
            {
                'log_level': LaunchConfiguration('log_level'),
            },
        ],
    )

    return LaunchDescription([
        log_level_arg,
        gui_node,
    ])

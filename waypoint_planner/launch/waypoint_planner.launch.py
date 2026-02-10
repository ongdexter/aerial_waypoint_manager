from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('waypoint_planner')
    default_config = os.path.join(pkg_share, 'waypoint_planner.yaml')

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to the config YAML file'
    )

    waypoint_planner_node = Node(
        package='waypoint_planner',
        executable='waypoint_planner_node',
        name='waypoint_planner',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file')
        ],
        remappings=[
            # Add remappings here if needed
        ],
    )

    return LaunchDescription([
        config_file_arg,
        waypoint_planner_node
    ])

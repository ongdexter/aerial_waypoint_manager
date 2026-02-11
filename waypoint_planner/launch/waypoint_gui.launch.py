from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the Qt waypoint planner GUI."""

    log_level_arg = DeclareLaunchArgument(
        'log_level', default_value='info', description='rclpy log level'
    )

    satellite_map_arg = DeclareLaunchArgument(
        'satellite_map_file', default_value='/home/odexter/ros2_sim_ws/src/aerial_waypoint_manager/satellite_map.pkl',
        description='Path to satellite_map.pkl for map overlay'
    )

    gui_node = Node(
        package='waypoint_planner',
        executable='waypoint_gui',
        name='waypoint_gui',
        output='screen',
        parameters=[{
            'log_level': LaunchConfiguration('log_level'),
            'satellite_map_file': LaunchConfiguration('satellite_map_file'),
        }],
    )

    return LaunchDescription([
        log_level_arg,
        satellite_map_arg,
        gui_node,
    ])

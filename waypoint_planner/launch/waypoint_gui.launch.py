from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Launch the Qt waypoint planner GUI."""

    default_config = os.path.join(
        get_package_share_directory('waypoint_planner'),
        'waypoint_planner.yaml',
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to the shared waypoint planner config YAML file',
    )
    log_level_arg = DeclareLaunchArgument(
        'log_level', default_value='info', description='rclpy log level'
    )
    robot_ns_arg = DeclareLaunchArgument('robot_ns', default_value='uav')
    pkl_file_arg = DeclareLaunchArgument(
        'pkl_file', default_value='',
        description='Waypoint data pickle; empty disables the satellite map',
    )

    gui_node = Node(
        package='waypoint_planner',
        executable='waypoint_gui',
        namespace=LaunchConfiguration('robot_ns'),
        name='waypoint_gui',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'log_level': LaunchConfiguration('log_level'),
                'waypoint_data_file': ParameterValue(
                    LaunchConfiguration('pkl_file'), value_type=str,
                ),
            },
        ],
    )

    return LaunchDescription([
        config_file_arg,
        log_level_arg,
        robot_ns_arg,
        pkl_file_arg,
        gui_node,
    ])

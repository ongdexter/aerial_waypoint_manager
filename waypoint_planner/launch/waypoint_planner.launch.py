from launch import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
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
    robot_ns_arg = DeclareLaunchArgument('robot_ns', default_value='uav')

    def _launch_node(context):
        filename = LaunchConfiguration('pkl_file').perform(context)
        overrides = {
            'waypoint_data_file': ParameterValue(filename, value_type=str),
            'use_waypoint_graph': bool(filename),
        }
        waypoint_planner_node = Node(
            package='waypoint_planner',
            executable='waypoint_planner_node',
            namespace=LaunchConfiguration('robot_ns'),
            name='waypoint_planner',
            output='screen',
            parameters=[
                LaunchConfiguration('config_file'),
                overrides,
            ],
            remappings=[
                # Add remappings here if needed
            ],
        )
        return [waypoint_planner_node]

    return LaunchDescription([
        DeclareLaunchArgument('pkl_file', default_value='',
                              description='Waypoint graph pickle; empty disables graph routing'),
        config_file_arg,
        robot_ns_arg,
        OpaqueFunction(function=_launch_node)
    ])

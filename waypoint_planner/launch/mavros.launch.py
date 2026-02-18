#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Paths to MAVROS configs
    mavros_share = get_package_share_directory('mavros')
    pluginlists_yaml = os.path.join(mavros_share, 'share', 'mavros', 'launch', 'px4_pluginlists.yaml')
    config_yaml = os.path.join(mavros_share, 'share', 'mavros', 'launch', 'px4_config.yaml')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'fcu_url',
            default_value='/dev/ttyTHS1:921600',
            description='PX4 FCU URL (serial or UDP)'
        ),
        DeclareLaunchArgument('gcs_url', default_value=''),
        DeclareLaunchArgument('tgt_system', default_value='1'),
        DeclareLaunchArgument('tgt_component', default_value='1'),
        DeclareLaunchArgument('log_output', default_value='screen'),
        DeclareLaunchArgument('fcu_protocol', default_value='v2.0'),
        DeclareLaunchArgument('namespace', default_value='mavros'),

        # MAVROS Node
        Node(
            package='mavros',
            executable='mavros_node',
            namespace=LaunchConfiguration('namespace'),
            output=LaunchConfiguration('log_output'),
            parameters=[
                {'fcu_url': LaunchConfiguration('fcu_url')},
                {'gcs_url': LaunchConfiguration('gcs_url')},
                {'tgt_system': LaunchConfiguration('tgt_system')},
                {'tgt_component': LaunchConfiguration('tgt_component')},
                {'fcu_protocol': LaunchConfiguration('fcu_protocol')},
                pluginlists_yaml,
                config_yaml,
            ],
        ),
    ])

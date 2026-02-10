from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('waypoint_planner')
    default_config = os.path.join(pkg_share, 'waypoint_planner.yaml')

    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='/dev/ttyTHS1:921600',
    )
    
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        output='screen',
        parameters=[{
            'fcu_url': LaunchConfiguration('fcu_url')
        }]
    )

    return LaunchDescription([
        fcu_url_arg,
        mavros_node
    ])

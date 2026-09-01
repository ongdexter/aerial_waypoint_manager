from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ns_arg = DeclareLaunchArgument('robot_ns', default_value='uav')
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='/dev/ttyUSB0:921600',
    )

    pluginlists_yaml_arg = DeclareLaunchArgument(
        'pluginlists_yaml',
        default_value=PathJoinSubstitution([
            FindPackageShare('waypoint_planner'),
            'px4_pluginlists.yaml'
        ]),
    )

    config_yaml_arg = DeclareLaunchArgument(
        'config_yaml',
        default_value=PathJoinSubstitution([
            FindPackageShare('waypoint_planner'),
            'px4_config.yaml'
        ]),
    )

    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        output='screen',
        namespace=PathJoinSubstitution([
            LaunchConfiguration('robot_ns'), 'mavros'
        ]),
        parameters=[
            LaunchConfiguration('pluginlists_yaml'),
            LaunchConfiguration('config_yaml'),
            {
                'fcu_url': LaunchConfiguration('fcu_url'),
                'system_id': 21,
                'system_target_id': 121,
            }
        ]
    )

    return LaunchDescription([
        robot_ns_arg,
        fcu_url_arg,
        pluginlists_yaml_arg,
        config_yaml_arg,
        mavros_node
    ])
